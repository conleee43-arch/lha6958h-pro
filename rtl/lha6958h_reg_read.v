//////////////////////////////////////////////////////////////////////////////////
// Module : lha6958h_reg_read
// Author : Muse (for conleee43-arch)
// Date   : 2026-10-10
//
// Standalone register read controller for Legend Semi LHA6958H,
// implementing the TWO-FRAME PARALLEL READBACK PROTOCOL (Rev.B §7.4).
//
// Protocol sequence executed on start_i pulse:
//   1. Frame 1 (Write Read-Address Frame):
//      - Assert CS_N low.
//      - Drive DB[15:0] = {1'b1 (Read CMD), addr_i[6:0], 8'h00 (Don't care)}.
//      - Pulse WR_N low; rising edge of WR_N latches read address.
//      - Deassert CS_N high, float DB[15:0] (turnaround to input).
//
//   2. Frame 2 (Data Readback Frame):
//      - Assert CS_N low and pulse RD_N low.
//      - LHA6958H drives DB[15:0] = {1'b0, addr_i[6:0], DIN[7:0]}.
//      - FPGA latches DB[15:0] at RD_N low tail (meeting tD_RD_DB <= 27ns).
//      - Deassert RD_N and CS_N high.
//      - Verify DB[15] == 0 and DB[14:8] == addr_i; output DIN[7:0] to data_o.
//
//   3. Exit Register Mode:
//      - Assert CS_N low.
//      - Drive DB[15:0] = 16'h0000 (Write 0x00 to address 0x00).
//      - Pulse WR_N low; rising edge latches command.
//      - Deassert CS_N high, float DB[15:0]. Device returns to ADC read mode.
//      - Pulse done_o high (and err_o if readback format/address mismatched).
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module lha6958h_reg_read #(
  parameter CLK_FREQ_HZ     = 50_000_000,

  // Interface timing, in clock cycles (defaults @50MHz, 20ns period)
  // Rev.B §6.1.2 & §7.4:
  //   tLP_WR >= 88ns, tHP_WR >= 213ns
  //   tLP_RD >= 10ns, tHP_RD >= 10ns, tD_RD_DB <= 27ns (VDRIVE > 2.7V)
  parameter WR_LOW_CLKS     = 5,    // WR_N low pulse  >= 88ns  (100ns @50MHz)
  parameter WR_HIGH_CLKS    = 12,   // WR_N high gap   >= 213ns (240ns @50MHz)
  parameter RD_LOW_CLKS     = 4,    // RD_N low pulse  >= 27ns+margin (80ns @50MHz)
  parameter RD_HIGH_CLKS    = 4,    // RD_N high gap   (80ns @50MHz)
  parameter CS_SETUP_CLKS   = 2,    // CS_N low to strobe setup (40ns @50MHz)
  parameter CS_HOLD_CLKS    = 2     // Turnaround / hold gap (40ns @50MHz)
)(
  input  wire        clk,
  input  wire        rst_n,

  // User control interface
  input  wire        start_i,        // Single-cycle pulse: start register read
  input  wire [6:0]  addr_i,         // Target register address ADD[6:0]
  output reg  [7:0]  data_o,         // Latched register data DIN[7:0]
  output reg         busy_o,         // High while readback sequence is running
  output reg         done_o,         // Single-cycle pulse: sequence finished
  output reg         err_o,          // Single-cycle pulse: readback format/address mismatch

  // Parallel bus control interface (connects to bus mux or device pins)
  output reg         reg_cs_n,       // Chip select output
  output reg         reg_rd_n,       // Read strobe output
  output reg         reg_wr_n,       // Write strobe output
  output reg  [15:0] reg_db_out,     // FPGA data output to DB[15:0]
  output reg         reg_db_oe,      // FPGA DB output enable (1: drive, 0: Hi-Z)
  input  wire [15:0] reg_db_in       // FPGA data input from DB[15:0]
);

  // ----------------------------------------------------------------
  // State encoding
  // ----------------------------------------------------------------
  localparam [3:0]
    S_IDLE         = 4'd0,
    // Frame 1: Write Read-Address
    S_F1_SETUP     = 4'd1,   // CS low, drive DB = {1'b1, addr, 8'h00}
    S_F1_WR_LOW    = 4'd2,   // WR low pulse (>= 88ns)
    S_F1_WR_HI     = 4'd3,   // WR high, latched on rising edge (>= 213ns)
    S_F1_CS_HI     = 4'd4,   // CS high, float DB, turnaround gap
    // Frame 2: Read Data
    S_F2_SETUP     = 4'd5,   // CS low, settle
    S_F2_RD_LOW    = 4'd6,   // RD low pulse, sample DB at tail
    S_F2_RD_HI     = 4'd7,   // RD high
    S_F2_CS_HI     = 4'd8,   // CS high, release bus, check returned word
    // Exit: Write (0x00, 0x00) to return to ADC read mode
    S_EXIT_SETUP   = 4'd9,   // CS low, drive DB = 16'h0000
    S_EXIT_WR_LOW  = 4'd10,  // WR low pulse
    S_EXIT_WR_HI   = 4'd11,  // WR high, latched on rising edge
    S_EXIT_CS_HI   = 4'd12,  // CS high, float DB
    S_DONE         = 4'd13;  // Assert done/err pulses, return to IDLE

  reg [3:0]  state;
  reg [15:0] t_cnt;
  reg [6:0]  latch_addr;
  reg [15:0] latched_rdata;
  reg        err_lat;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state         <= S_IDLE;
      t_cnt         <= 16'd0;
      latch_addr    <= 7'd0;
      latched_rdata <= 16'd0;
      data_o        <= 8'd0;
      busy_o        <= 1'b0;
      done_o        <= 1'b0;
      err_o         <= 1'b0;
      err_lat       <= 1'b0;
      reg_cs_n      <= 1'b1;
      reg_rd_n      <= 1'b1;
      reg_wr_n      <= 1'b1;
      reg_db_out    <= 16'h0000;
      reg_db_oe     <= 1'b0;
    end else begin
      // Default single-cycle pulses
      done_o <= 1'b0;
      err_o  <= 1'b0;

      case (state)
        // ----------------------------------------------------------
        // Idle: wait for user request
        // ----------------------------------------------------------
        S_IDLE: begin
          busy_o     <= 1'b0;
          reg_cs_n   <= 1'b1;
          reg_rd_n   <= 1'b1;
          reg_wr_n   <= 1'b1;
          reg_db_oe  <= 1'b0;
          reg_db_out <= 16'h0000;
          t_cnt      <= 16'd0;
          err_lat    <= 1'b0;

          if (start_i) begin
            busy_o     <= 1'b1;
            latch_addr <= addr_i;
            // Frame 1 begins: assert CS_N, drive {1'b1, addr, 8'h00}
            reg_cs_n   <= 1'b0;
            reg_db_oe  <= 1'b1;
            reg_db_out <= {1'b1, addr_i, 8'h00}; // DB15=1 read cmd
            state      <= S_F1_SETUP;
          end
        end

        // ----------------------------------------------------------
        // Frame 1: Write Read-Address frame
        // ----------------------------------------------------------
        S_F1_SETUP: begin
          reg_wr_n <= 1'b1;
          if (t_cnt == CS_SETUP_CLKS - 1) begin
            t_cnt    <= 16'd0;
            reg_wr_n <= 1'b0;          // Pulse WR_N low
            state    <= S_F1_WR_LOW;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        S_F1_WR_LOW: begin
          if (t_cnt == WR_LOW_CLKS - 1) begin
            t_cnt    <= 16'd0;
            reg_wr_n <= 1'b1;          // Rising edge: ADC latches read address
            state    <= S_F1_WR_HI;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        S_F1_WR_HI: begin
          if (t_cnt == WR_HIGH_CLKS - 1) begin
            t_cnt     <= 16'd0;
            reg_cs_n  <= 1'b1;         // Deassert CS_N to finish Frame 1
            reg_db_oe <= 1'b0;         // Float DB bus (turnaround to input)
            state     <= S_F1_CS_HI;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        S_F1_CS_HI: begin
          if (t_cnt == CS_HOLD_CLKS - 1) begin
            t_cnt    <= 16'd0;
            reg_cs_n <= 1'b0;          // Frame 2 begins: assert CS_N low
            state    <= S_F2_SETUP;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        // ----------------------------------------------------------
        // Frame 2: Data Readback frame
        // ----------------------------------------------------------
        S_F2_SETUP: begin
          reg_rd_n <= 1'b1;
          if (t_cnt == CS_SETUP_CLKS - 1) begin
            t_cnt    <= 16'd0;
            reg_rd_n <= 1'b0;          // Assert RD_N low to enable ADC drive
            state    <= S_F2_RD_LOW;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        S_F2_RD_LOW: begin
          // Data is guaranteed valid <= 27ns after RD_N fall.
          // Latch DB bus near the end of RD_LOW pulse for maximal margin.
          if (t_cnt == RD_LOW_CLKS - 1) begin
            latched_rdata <= reg_db_in;
            t_cnt         <= 16'd0;
            reg_rd_n      <= 1'b1;     // Deassert RD_N high
            state         <= S_F2_RD_HI;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        S_F2_RD_HI: begin
          if (t_cnt == RD_HIGH_CLKS - 1) begin
            t_cnt    <= 16'd0;
            reg_cs_n <= 1'b1;          // Deassert CS_N to complete Frame 2
            state    <= S_F2_CS_HI;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        S_F2_CS_HI: begin
          // Check returned packet format: DB15 must be 0, DB[14:8] must match target address
          if (latched_rdata[15] == 1'b0 && latched_rdata[14:8] == latch_addr) begin
            data_o  <= latched_rdata[7:0];
            err_lat <= 1'b0;
          end else begin
            err_lat <= 1'b1;           // Address mismatch or invalid frame
          end

          if (t_cnt == CS_HOLD_CLKS - 1) begin
            t_cnt      <= 16'd0;
            // Begin Exit: assert CS_N low, drive 16'h0000 to return to ADC mode
            reg_cs_n   <= 1'b0;
            reg_db_oe  <= 1'b1;
            reg_db_out <= 16'h0000;
            state      <= S_EXIT_SETUP;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        // ----------------------------------------------------------
        // Exit: Write (0x00, 0x00) to return to ADC read mode
        // ----------------------------------------------------------
        S_EXIT_SETUP: begin
          reg_wr_n <= 1'b1;
          if (t_cnt == CS_SETUP_CLKS - 1) begin
            t_cnt    <= 16'd0;
            reg_wr_n <= 1'b0;          // Pulse WR_N low
            state    <= S_EXIT_WR_LOW;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        S_EXIT_WR_LOW: begin
          if (t_cnt == WR_LOW_CLKS - 1) begin
            t_cnt    <= 16'd0;
            reg_wr_n <= 1'b1;          // Rising edge: ADC exits register mode
            state    <= S_EXIT_WR_HI;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        S_EXIT_WR_HI: begin
          if (t_cnt == WR_HIGH_CLKS - 1) begin
            t_cnt     <= 16'd0;
            reg_cs_n  <= 1'b1;         // Deassert CS_N
            reg_db_oe <= 1'b0;         // Float DB bus
            state     <= S_EXIT_CS_HI;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        S_EXIT_CS_HI: begin
          if (t_cnt == CS_HOLD_CLKS - 1) begin
            t_cnt  <= 16'd0;
            done_o <= 1'b1;            // Report completion
            err_o  <= err_lat;         // Report error status
            busy_o <= 1'b0;
            state  <= S_DONE;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        // ----------------------------------------------------------
        // Done: pulse clears, return to IDLE
        // ----------------------------------------------------------
        S_DONE: begin
          state <= S_IDLE;
        end

        default: state <= S_IDLE;
      endcase
    end
  end

endmodule

