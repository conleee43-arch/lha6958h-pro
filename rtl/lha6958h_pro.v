//////////////////////////////////////////////////////////////////////////////////
// Module : lha6958h_pro
// Author : Muse (for conleee43-arch)
// Date   : 2026-10-10
//
// Professional-grade standalone driver for the Legend Semi LHA6958H
// 8-channel simultaneous-sampling 16-bit SAR ADC, PARALLEL interface,
// ADC read mode.
//
// Target: Altera Cyclone IV (EP4CE10), single clock domain, plain RTL,
// no AXI dependency. Spec source: LHA6958H Datasheet Rev.B (Chinese,
// 68 pages). Every timing number below is annotated with its manual
// section; do NOT copy AD7606 numbers here - several edges and the
// register map are deliberately incompatible.
//
// ---------------------------------------------------------------------------
// Hardware strapping assumed on the LHA6958H board (board level):
//   PAR/SER SEL = 0 (parallel interface)
//   OS2..OS0    = 111 -> software mode; analog range + OSR come from the
//                         register file, written at power-up by the
//                         companion module lha6958h_reg_init (see README).
//                         (OS2..OS0 != 111 also works = hardware mode; then
//                         RANGE pin selects +/-10V / +/-5V and the register
//                         port is unavailable. OS pins are latched on the
//                         BUSY falling edge, Rev.B §11.)
//   STBY        = 1 (ignored in software mode, Rev.B §9.2)
//   RANGE       = tie high or low (ignored in software mode, but must be
//                 terminated, Rev.B §8 pin 8)
//   REF SELECT  = per reference choice (high = internal 2.5V)
//   WR          = shared with lha6958h_reg_init: 1 while reading ADC data.
// ---------------------------------------------------------------------------
//
// KEY DIFFERENCES vs the AD7606 driver (ad7606_pro.v) - porting traps:
//   * Conversion starts on the RISING edge of CONVST (Rev.B §10.2), NOT the
//     falling edge as on the classic AD7606. This driver pulses CONVST high.
//   * LHA6958H has ONE CONVST pin (pin 9). There is no CONVST_A/CONVST_B.
//   * Parallel interface is 16-bit only; no 8-bit mode (Rev.B §13.1).
//   * Range register encoding (0x03~0x06) is NOT AD7606C-16 compatible:
//     LHA 0010 = +/-10V single-ended, 0100~0111 = high-bandwidth mode.
//     See lha6958h_reg_init.v defaults and README.
// ---------------------------------------------------------------------------
//
// Features (design ideas borrowed from ADI axi_ad7606x, re-implemented
// as plain RTL with no AXI dependency):
//   * Precise CONVST timer: sample rate set by parameter, no GPIO bit-bang.
//   * Conversion starts on CONVST RISING edge (LHA6958H convention).
//   * Read burst triggered by BUSY falling edge (2-stage synchronised),
//     not by blind cycle counting.
//   * FRSTDATA channel-alignment check: FRSTDATA must be high while the
//     first channel is latched, otherwise err_align is set (sticky).
//   * Watchdog on BUSY: err_busy_timeout (sticky) if BUSY does not assert
//     after CONVST, or stays high longer than BUSY_TIMEOUT_CLKS.
//   * Overrun detection: err_overrun (sticky) if the sample timer fires
//     while the previous frame is still being read.
//   * Power-up sequencing: tPOWER-UP delay (10 ms, supplies stable before
//     RESET), full RESET pulse, then por_done. The companion register-init
//     module (lha6958h_reg_init) starts on por_done and raises init_done;
//     conversions only begin after init_done_i. This ordering is mandatory:
//     register writes issued while RESET is asserted are ignored by the
//     device, so init must run AFTER the reset pulse, not during it.
//     Hardware mode (no register init): tie init_done_i = 1'b1 and skip
//     lha6958h_reg_init; see lha6958h_top.v for the reference integration.
//   * All interface timing is parameterised in clock cycles - no magic
//     numbers scattered through the state machine.
//
// User interface:
//   adc_data  : {ch8,ch7,...,ch1} packed, ch1 in [15:0], valid with adc_valid.
//   adc_valid : single-cycle pulse when a full 8-channel frame is latched.
//   por_done  : power-on reset finished; start signal for lha6958h_reg_init.
//   init_done_i: from lha6958h_reg_init.init_done (tie 1'b1 in hardware mode).
//   err_*     : sticky error flags, cleared by a pulse on err_clear.
//
// Timing defaults assume 50 MHz system clock and VDRIVE > 2.7V (3.3V logic).
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module lha6958h_pro #(
  // --- system ---
  parameter CLK_FREQ_HZ       = 50_000_000, // FPGA clock feeding this module
  parameter SAMPLE_RATE_HZ    = 500_000,    // CONVST rate (500kSPS)

  // --- LHA6958H interface timing, in clk cycles (defaults @50MHz) ---
  // Rev.B §6.1.1: tLP_CNV >= 80ns (CONVST low), tHP_CNV >= 80ns (CONVST high)
  parameter CONVST_HIGH_CLKS  = 4,          // CONVST high pulse width (>=80ns)
  parameter CONVST_LOW_CLKS   = 4,          // CONVST low gap      (>=80ns)
  // Rev.B §6.1.2: tLP_RD/tHP_RD >= 10ns, tCYC_RD >= 30ns,
  //               tD_RD_DB <= 27ns (VDRIVE>2.7V), tH_RD_DB >= 12ns
  parameter RD_LOW_CLKS       = 2,          // RD_N 低电平宽度 (满足 tLP_RD>=10ns，且覆盖出数延迟 tD_RD_DB<=27ns，40ns @50MHz)
  parameter RD_HIGH_CLKS      = 2,          // RD_N 高电平间隔 (满足 tHP_RD>=10ns，整周期 80ns >= tCYC_RD 30ns，40ns @50MHz)
  // Rev.B §9.1:
  // - full reset pulse >= 3us (RESET_HOLD_CLKS).
  // - tDEVICE_SETUP = 600us recovery delay after RESET falling edge
  //   before any subsequent operations (RESET_RECOVERY_CLKS).
  parameter RESET_HOLD_CLKS    = 500,       // power-on RESET high pulse (10us @50MHz >= 3us)
  parameter RESET_RECOVERY_CLKS= 30000,     // 600us @50MHz (tDEVICE_SETUP after RESET fall)
  // Rev.B §9.1: tPOWER-UP = 10 ms min between stable AVCC/VDRIVE and the
  // RESET assertion. The FPGA is usually configured later than that, but
  // the parameter keeps the sequencing datasheet-clean by construction.
  parameter POWERUP_DELAY_CLKS = 500000,    // 10 ms @50MHz
  // Watchdog bound: MUST exceed worst-case t_CONV of the configured OSR,
  // otherwise err_busy_timeout false-trips every frame. Rev.B Table 2
  // t_CONV typical: no OS 0.62us | x2 2.3us | x4 4.8us | x8 9.9us |
  // x16 20us | x32 40.2us | x64 80.8us | x128 161.9us | x256 324us.
  // Default 500 clks = 10us @50MHz: valid up to 8x oversampling.
  parameter BUSY_TIMEOUT_CLKS  = 500,

  // --- ADC configuration ---
  parameter [2:0] OS_SEL      = 3'b111      // 111 = software mode (registers);
                                           // others = hardware mode OSR
)(
  input  wire        clk,
  input  wire        rst_n,          // async, active-low system reset
  input  wire        init_done_i,    // from lha6958h_reg_init.init_done;
                                    // tie 1'b1 if no register init is used
                                    // (hardware mode)

  // LHA6958H parallel interface (see header for board strapping)
  output reg         convst,         // conversion start (RISING edge!)
  output reg         cs_n,           // chip select, low during read burst
  output reg         rd_n,           // read strobe
  output wire        wr_n,           // tied high: register writes are done
                                    // by lha6958h_reg_init, not here
  output reg         adc_reset,      // power-on reset to ADC (high active)
  output wire [2:0]  ad_os,          // oversampling pins OS2..OS0
  input  wire [15:0] db,             // parallel data bus (ADC -> FPGA)
  input  wire        busy,           // BUSY flag from ADC (async: synchronised inside)
  input  wire        first_data,     // FRSTDATA: high while CH1 is on the bus

  // user side
  output reg [127:0] adc_data,       // {ch8..ch1}, ch1 = [15:0]
  output reg         adc_valid,      // single-cycle frame-valid pulse
  output reg         por_done,       // power-on reset sequence finished
                                    // (start signal for lha6958h_reg_init)
  output reg         err_align,      // sticky: FRSTDATA mismatch on CH1
  output reg         err_busy_timeout,// sticky: BUSY watchdog expired
  output reg         err_overrun,    // sticky: sample tick during active frame
  input  wire        err_clear       // pulse to clear sticky error flags
);

  // ----------------------------------------------------------------
  // Simulation-only sanity checks (ignored by synthesis)
  // ----------------------------------------------------------------
  // synopsys translate_off
  initial begin
    if (SAMPLE_RATE_HZ > 1000000)
      $display("WARNING lha6958h_pro: SAMPLE_RATE_HZ=%0d exceeds LHA6958H tCYCLE (1us min, no-OS)",
               SAMPLE_RATE_HZ);
    if (CLK_FREQ_HZ / SAMPLE_RATE_HZ
        < (8*(RD_LOW_CLKS+RD_HIGH_CLKS) + 16))
      // physical floor: sample period must at least cover 8x RD burst + margin
      $display("WARNING lha6958h_pro: sample period shorter than readout burst can sustain");
  end
  // synopsys translate_on

  assign wr_n  = 1'b1;
  assign ad_os = OS_SEL;

  localparam CONVST_PERIOD_CLKS = CLK_FREQ_HZ / SAMPLE_RATE_HZ;

  // ----------------------------------------------------------------
  // BUSY synchroniser (async input from ADC clock domain)
  // ----------------------------------------------------------------
  reg busy_s0, busy_s1;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      busy_s0 <= 1'b0;
      busy_s1 <= 1'b0;
    end else begin
      busy_s0 <= busy;
      busy_s1 <= busy_s0;
    end
  end
  wire busy_fall = ~busy_s0 &  busy_s1;   // end of conversion

  // ----------------------------------------------------------------
  // CONVST sample timer (free-running) + pending flag.
  // A tick that arrives while a frame is in flight is NOT lost: it is
  // latched in sample_pending and serviced when the FSM returns to IDLE.
  // A second tick while one is still pending means the sample rate is
  // higher than conversion+readout can sustain -> err_overrun.
  // ----------------------------------------------------------------
  // The sample timer only runs once the register init has completed, so
  // no sample requests can pile up (and false-trip err_overrun) during
  // power-up / reset / register configuration.
  reg [31:0] tick_cnt;
  reg        tick_raw;        // single-cycle pulse every CONVST period
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tick_cnt <= 32'd0;
      tick_raw <= 1'b0;
    end else if (!init_done_i) begin
      tick_cnt <= 32'd0;
      tick_raw <= 1'b0;
    end else if (tick_cnt == CONVST_PERIOD_CLKS - 1) begin
      tick_cnt <= 32'd0;
      tick_raw <= 1'b1;
    end else begin
      tick_cnt <= tick_cnt + 1'b1;
      tick_raw <= 1'b0;
    end
  end

  // ----------------------------------------------------------------
  // Main state machine
  // ----------------------------------------------------------------
  localparam [3:0]
    S_POWERUP   = 4'd0,   // tPOWER-UP: supplies stable -> RESET assertion
    S_RST       = 4'd1,   // full RESET high pulse (>= 3us)
    S_RECOVERY  = 4'd2,   // tDEVICE_SETUP: wait 600us after RESET falling edge
    S_WAIT_INIT = 4'd3,   // wait for register init (init_done_i)
    S_IDLE      = 4'd4,
    S_CONVST_H  = 4'd5,   // CONVST high pulse (rising edge starts conversion)
    S_CONVST_L  = 4'd6,   // CONVST low gap
    S_WBUSY_H   = 4'd7,
    S_WBUSY_L   = 4'd8,
    S_READ      = 4'd9,
    S_DONE      = 4'd10;

  reg [3:0]  state;
  reg [31:0] t_cnt;          // generic timing counter
  reg [3:0]  ch_cnt;         // 0..7 channel index during readout
  reg [15:0] ch_data [0:7];  // per-channel holding registers
  reg        first_data_lat;  // FRSTDATA sampled with CH1
  reg [15:0] rd_phase;       // phase counter inside one RD_N pulse
  reg        sample_pending;  // a conversion request waiting for S_IDLE

  localparam RD_TOTAL_CLKS = RD_LOW_CLKS + RD_HIGH_CLKS;

  integer k;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state            <= S_POWERUP;
      convst           <= 1'b0;
      cs_n             <= 1'b1;
      rd_n             <= 1'b1;
      adc_reset        <= 1'b0;
      adc_valid        <= 1'b0;
      adc_data         <= 128'd0;
      por_done         <= 1'b0;
      err_align        <= 1'b0;
      err_busy_timeout <= 1'b0;
      err_overrun      <= 1'b0;
      sample_pending   <= 1'b0;
      t_cnt            <= 32'd0;
      ch_cnt           <= 4'd0;
      rd_phase         <= 16'd0;
      first_data_lat   <= 1'b0;
      for (k = 0; k < 8; k = k + 1)
        ch_data[k] <= 16'd0;
    end else begin
      // default: single-cycle pulses
      adc_valid   <= 1'b0;

      if (err_clear) begin
        err_align        <= 1'b0;
        err_busy_timeout <= 1'b0;
        err_overrun      <= 1'b0;
      end

      // latch sample requests; a second tick while one is pending = overrun
      if (tick_raw) begin
        if (sample_pending)
          err_overrun <= 1'b1;
        else
          sample_pending <= 1'b1;
      end

      // If the register-init sequencer (re)starts, init_done_i drops: abort
      // any in-flight frame and wait quietly. The top-level bus mux hands
      // CS/WR/DB to the init module while init_done_i is low, so the ADC
      // interface must release the bus instead of fighting it.
      if (!init_done_i && state != S_POWERUP && state != S_RST
          && state != S_RECOVERY && state != S_WAIT_INIT) begin
        cs_n           <= 1'b1;
        rd_n           <= 1'b1;
        convst         <= 1'b0;
        sample_pending <= 1'b0;
        state          <= S_WAIT_INIT;
      end else case (state)

        // --- tPOWER-UP: supplies stable -> RESET assertion, min 10 ms ---
        S_POWERUP: begin
          if (t_cnt == POWERUP_DELAY_CLKS - 1) begin
            t_cnt     <= 32'd0;
            adc_reset <= 1'b1;
            state     <= S_RST;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        // --- full reset pulse: high for >= 3us (default 10us) ---
        S_RST: begin
          adc_reset <= 1'b1;
          if (t_cnt == RESET_HOLD_CLKS - 1) begin
            t_cnt     <= 32'd0;
            adc_reset <= 1'b0;        // RESET falling edge (deasserted)
            state     <= S_RECOVERY;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        // --- tDEVICE_SETUP: wait 600 us after RESET falling edge ---
        S_RECOVERY: begin
          adc_reset <= 1'b0;
          if (t_cnt == RESET_RECOVERY_CLKS - 1) begin
            t_cnt    <= 32'd0;
            por_done <= 1'b1;         // 600us elapsed, chip ready for register writes
            state    <= S_WAIT_INIT;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        // --- wait for the register init sequencer (software mode) ---
        S_WAIT_INIT: begin
          if (init_done_i)
            state <= S_IDLE;
        end

        // --- wait for sample request ---
        S_IDLE: begin
          cs_n   <= 1'b1;
          rd_n   <= 1'b1;
          convst <= 1'b0;
          if (sample_pending) begin
            sample_pending <= 1'b0;
            state          <= S_CONVST_H;
            t_cnt          <= 32'd0;
            convst         <= 1'b1;        // RISING edge starts conversion
          end
        end

        // --- CONVST high pulse (>= tHP_CNV = 80ns) ---
        S_CONVST_H: begin
          if (t_cnt == CONVST_HIGH_CLKS - 1) begin
            t_cnt  <= 32'd0;
            convst <= 1'b0;
            state  <= S_CONVST_L;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        // --- CONVST low gap (>= tLP_CNV = 80ns) ---
        S_CONVST_L: begin
          if (t_cnt == CONVST_LOW_CLKS - 1) begin
            t_cnt <= 32'd0;
            state <= S_WBUSY_H;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        // --- wait for BUSY high (conversion started) ---
        // Level check, not edge: BUSY asserts <= 20ns after the CONVST
        // rising edge (tD_CNV_BSY, VDRIVE>2.7V), which can precede our
        // arrival in this state. BUSY is guaranteed low on entry.
        S_WBUSY_H: begin
          if (busy_s0) begin
            t_cnt <= 32'd0;
            state <= S_WBUSY_L;
          end else if (t_cnt == BUSY_TIMEOUT_CLKS - 1) begin
            err_busy_timeout <= 1'b1;      // BUSY never asserted: ADC/clock fault
            state            <= S_IDLE;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        // --- wait for BUSY falling (conversion done) ---
        S_WBUSY_L: begin
          if (busy_fall) begin
            t_cnt   <= 32'd0;
            ch_cnt  <= 4'd0;
            rd_phase<= 16'd0;
            cs_n    <= 1'b0;               // CS low for the whole burst
            state   <= S_READ;
          end else if (t_cnt == BUSY_TIMEOUT_CLKS - 1) begin
            err_busy_timeout <= 1'b1;      // BUSY stuck high
            state            <= S_IDLE;
          end else begin
            t_cnt <= t_cnt + 1'b1;
          end
        end

        // --- read 8 channels, one RD_N pulse each ---
        // Rev.B §13.1: a series of RD pulses outputs V1..V8 in ascending
        // order onto DB[15:0]. FRSTDATA is set high by the RD falling edge
        // for V1 and returns low after the next RD falling edge.
        S_READ: begin
          rd_n <= (rd_phase < RD_LOW_CLKS) ? 1'b0 : 1'b1;

          if (rd_phase == RD_LOW_CLKS) begin
            // data valid tD_RD_DB <= 27ns after RD_N falling; we latch at
            // RD_LOW_CLKS (=40ns @50MHz) so data and FRSTDATA are both settled.
            ch_data[ch_cnt] <= db;
            if (ch_cnt == 4'd0)
              first_data_lat <= first_data; // FRSTDATA must be high for CH1
          end

          if (rd_phase == RD_TOTAL_CLKS - 1) begin
            rd_phase <= 16'd0;
            if (ch_cnt == 4'd7) begin
              state <= S_DONE;
            end else begin
              ch_cnt <= ch_cnt + 1'b1;
            end
          end else begin
            rd_phase <= rd_phase + 1'b1;
          end
        end

        // --- frame complete ---
        S_DONE: begin
          cs_n      <= 1'b1;
          rd_n      <= 1'b1;
          adc_data  <= {ch_data[7], ch_data[6], ch_data[5], ch_data[4],
                        ch_data[3], ch_data[2], ch_data[1], ch_data[0]};
          adc_valid <= 1'b1;
          if (first_data_lat != 1'b1)
            err_align <= 1'b1;             // channel alignment lost
          state <= S_IDLE;
        end

        default: state <= S_POWERUP;
      endcase
    end
  end

endmodule
