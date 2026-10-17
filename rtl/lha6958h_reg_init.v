//////////////////////////////////////////////////////////////////////////////////
// Module : lha6958h_reg_init
// Author : Muse (for conleee43-arch)
// Date   : 2026-10-10
//
// Power-up register initialiser for the Legend Semi LHA6958H, PARALLEL
// register mode (Rev.B §7.4). Runs once after reset, then goes quiet.
//
// What it writes (defaults = power-on reset values, i.e. "change nothing"):
//   0x02 = 0x08  CONFIG: keep reset default (normal operation, internal
//             OS clock; the DOUT_FORMAT bits are don't-care in parallel mode)
//   0x03 = 0x33  RANGE_CH1_CH2: ch1/ch2 = +/-12.5V, low-bandwidth
//   0x04 = 0x33  RANGE_CH3_CH4: ch3/ch4 = +/-12.5V, low-bandwidth
//   0x05 = 0x33  RANGE_CH5_CH6: ch5/ch6 = +/-12.5V, low-bandwidth
//   0x06 = 0x33  RANGE_CH7_CH8: ch7/ch8 = +/-12.5V, low-bandwidth
//   0x08 = 0x00  OVERSAMPLING: no oversampling
//   0x00 = 0x00  (auto-appended) return to ADC read mode - mandatory,
//             because after any register access the device sits in register
//             mode and ADC data cannot be read (Rev.B §7.4).
//
// LHA range encoding reminder (NOT AD7606C-16 compatible - the porting trap):
//   per channel, 4 bits: 0000=+/-2.5V  0001=+/-5V  0010=+/-10V  0011=+/-12.5V
//   0100~0111 = same four ranges in HIGH-BANDWIDTH mode.
//   ADI's 0010 means +/-6.25V and 0101~0111 mean unipolar ranges - do not copy.
//
// Write-frame format (Rev.B §7.4, single frame):
//   DB15 = 0 (write command), DB[14:8] = ADD6..ADD0, DB[7:0] = write data.
//   Address + data are latched on the WR RISING edge.
// Timing (VDRIVE > 2.7V, Rev.B Table 3 + write table):
//   tLP_WR >= 88ns, tHP_WR >= 213ns, tCYC_WR >= 230ns,
//   tS_DB_WR >= 5ns (data setup), tH_WR_DB >= 5ns (data hold).
// Defaults below assume 50 MHz: WR low 5 clks (100ns), WR high 12 clks
// (240ns), frame cycle 17 clks (340ns >= 230ns).
//
// Sharing the bus with lha6958h_pro: this module drives the parallel bus
// only while init is running (init_done = 0). Sequencing with the read
// driver is handled by lha6958h_top.v:
//
//   pro.por_done  -> this.start_i      (reset pulse finished -> start init)
//   this.init_done -> pro.init_done_i  (init finished -> start conversions;
//                                       also selects the bus mux)
//
//   assign CS_N = init_done ? pro_cs_n : cfg_cs_n;
//   assign WR_N = init_done ? 1'b1      : cfg_wr_n;   // pro ties WR high
//   assign DB   = db_oe ? cfg_db : 16'hzzzz;          // pro never drives DB
//
// Do NOT issue register writes while the ADC RESET pin is asserted: the
// device ignores them. start_i exists exactly to enforce the order
// reset-pulse -> register writes -> ADC mode.
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module lha6958h_reg_init #(
  // --- system ---
  parameter CLK_FREQ_HZ     = 50_000_000,

  // --- register list (up to 8 user registers) ---
  parameter NUM_REGS        = 6,
  parameter [6:0] REG_ADDR0 = 7'h02, parameter [7:0] REG_DATA0 = 8'h08,
  parameter [6:0] REG_ADDR1 = 7'h03, parameter [7:0] REG_DATA1 = 8'h33,
  parameter [6:0] REG_ADDR2 = 7'h04, parameter [7:0] REG_DATA2 = 8'h33,
  parameter [6:0] REG_ADDR3 = 7'h05, parameter [7:0] REG_DATA3 = 8'h33,
  parameter [6:0] REG_ADDR4 = 7'h06, parameter [7:0] REG_DATA4 = 8'h33,
  parameter [6:0] REG_ADDR5 = 7'h08, parameter [7:0] REG_DATA5 = 8'h00,
  parameter [6:0] REG_ADDR6 = 7'h00, parameter [7:0] REG_DATA6 = 8'h00,
  parameter [6:0] REG_ADDR7 = 7'h00, parameter [7:0] REG_DATA7 = 8'h00,

  // --- write timing, in clk cycles (defaults @50MHz) ---
  parameter WR_LOW_CLKS     = 5,    // WR low  >= 88ns
  parameter WR_HIGH_CLKS    = 12,   // WR high >= 213ns
  parameter CS_SETUP_CLKS   = 2     // CS low before first WR
)(
  input  wire        clk,
  input  wire        rst_n,          // async, active-low system reset
  input  wire        start_i,        // from lha6958h_pro.por_done: begin the
                                    // sequence only after the ADC reset pulse
                                    // has finished (writes during RESET are
                                    // ignored by the device)
  input  wire        init_restart,   // pulse: re-run the init sequence

  // register-port pins (mux with lha6958h_pro at top level, see header)
  output reg         cfg_cs_n,
  output reg         cfg_wr_n,
  output reg [15:0]  cfg_db,         // {1'b0 cmd, addr[6:0], data[7:0]}
  output reg         db_oe,          // 1 while driving cfg_db

  output reg         init_done       // sticky 1 when sequence finished
);

  // ----------------------------------------------------------------
  // Simulation-only sanity checks (ignored by synthesis)
  // ----------------------------------------------------------------
  // synopsys translate_off
  initial begin
    if (NUM_REGS < 1 || NUM_REGS > 8)
      $display("WARNING lha6958h_reg_init: NUM_REGS=%0d out of range 1..8", NUM_REGS);
  end
  // synopsys translate_on

  localparam [2:0]
    R_IDLE   = 3'd0,
    R_SETUP  = 3'd1,   // CS low, bus driven, settle
    R_WR_LOW = 3'd2,   // WR low pulse
    R_WR_HI  = 3'd3,   // WR high (>= tHP_WR); WR rising edge latches
    R_DONE   = 3'd4;

  // reg_idx: 0..NUM_REGS-1 = user registers; 4'd8 = sentinel for the
  // auto-appended (0x00,0x00) "back to ADC read mode" write.
  localparam [3:0] IDX_EXIT = 4'd8;

  reg [2:0]  state;
  reg [3:0]  reg_idx;
  reg [15:0] t_cnt;
  reg        started;

  // current frame word: {write-cmd 0, addr[6:0], data[7:0]}
  function [15:0] frame_word;
    input [3:0] idx;
    begin
      case (idx)
        4'd0: frame_word = {1'b0, REG_ADDR0, REG_DATA0};
        4'd1: frame_word = {1'b0, REG_ADDR1, REG_DATA1};
        4'd2: frame_word = {1'b0, REG_ADDR2, REG_DATA2};
        4'd3: frame_word = {1'b0, REG_ADDR3, REG_DATA3};
        4'd4: frame_word = {1'b0, REG_ADDR4, REG_DATA4};
        4'd5: frame_word = {1'b0, REG_ADDR5, REG_DATA5};
        4'd6: frame_word = {1'b0, REG_ADDR6, REG_DATA6};
        4'd7: frame_word = {1'b0, REG_ADDR7, REG_DATA7};
        default: frame_word = 16'h0000;   // IDX_EXIT: (0x00, 0x00)
      endcase
    end
  endfunction

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state     <= R_IDLE;
      reg_idx   <= 4'd0;
      t_cnt     <= 16'd0;
      started   <= 1'b0;
      cfg_cs_n  <= 1'b1;
      cfg_wr_n  <= 1'b1;
      cfg_db    <= 16'd0;
      db_oe     <= 1'b0;
      init_done <= 1'b0;
    end else begin
      case (state)

        // --- auto-run once after the ADC reset pulse; re-runnable ---
        R_IDLE: begin
          init_done <= 1'b0;
          if ((init_restart || !started) && start_i) begin
            started  <= 1'b1;
            reg_idx  <= 4'd0;
            t_cnt    <= 16'd0;
            cfg_cs_n <= 1'b0;
            db_oe    <= 1'b1;
            cfg_db   <= frame_word(4'd0);
            state    <= R_SETUP;
          end
        end

        // --- CS low, drive the frame word, settle ---
        R_SETUP: begin
          cfg_wr_n <= 1'b1;
          if (t_cnt == CS_SETUP_CLKS - 1) begin
            t_cnt    <= 16'd0;
            cfg_wr_n <= 1'b0;
            state    <= R_WR_LOW;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        // --- WR low (>= tLP_WR = 88ns) ---
        R_WR_LOW: begin
          if (t_cnt == WR_LOW_CLKS - 1) begin
            t_cnt    <= 16'd0;
            cfg_wr_n <= 1'b1;          // rising edge: ADC latches addr+data
            state    <= R_WR_HI;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        // --- WR high (>= tHP_WR = 213ns), then advance ---
        R_WR_HI: begin
          if (t_cnt == WR_HIGH_CLKS - 1) begin
            t_cnt <= 16'd0;
            if (reg_idx == IDX_EXIT) begin
              state <= R_DONE;         // (0x00,0x00) write done
            end else if (reg_idx == NUM_REGS - 1) begin
              // user list done: append the mandatory (0x00,0x00) write
              // that returns the device to ADC read mode (Rev.B §7.4)
              reg_idx  <= IDX_EXIT;
              cfg_db   <= 16'h0000;
              cfg_wr_n <= 1'b0;
              state    <= R_WR_LOW;
            end else begin
              reg_idx  <= reg_idx + 1'b1;
              cfg_db   <= frame_word(reg_idx + 1'b1);
              cfg_wr_n <= 1'b0;
              state    <= R_WR_LOW;
            end
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        // --- all done: release the bus ---
        R_DONE: begin
          cfg_cs_n  <= 1'b1;
          cfg_wr_n  <= 1'b1;
          db_oe     <= 1'b0;
          init_done <= 1'b1;
          if (init_restart) begin
            started <= 1'b0;           // R_IDLE will auto-run again
            state   <= R_IDLE;
          end
        end

        default: state <= R_IDLE;
      endcase
    end
  end

endmodule
