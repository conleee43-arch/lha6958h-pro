//////////////////////////////////////////////////////////////////////////////////
// Module : lha6958h_top
// Author : Muse (for conleee43-arch)
// Date   : 2026-10-10
//
// Reference integration of the LHA6958H professional driver:
//   lha6958h_pro      - ADC read path (CONVST timer, BUSY-triggered burst)
//   lha6958h_reg_init - power-up software-mode register configuration
//
// Pin names follow workspace/lha6958h-pro/H1_pinmap.md (H1 connector).
// Instantiate this module directly in the FPGA top level; bind the ports
// with lha6958h_pin_assignments.qsf.
//
// Power-up order (enforced by the por_done/init_done handshake):
//   1. pro: tPOWER-UP delay -> full RESET pulse -> por_done = 1
//   2. reg_init (started by por_done): write CONFIG / RANGE 0x03..0x06 /
//      OVERSAMPLING, then (0x00,0x00) to return to ADC read mode
//      -> init_done = 1
//   3. pro (released by init_done): CONVST timer runs, frames stream out.
//
// While init_done = 0 the parallel bus (CS/WR/DB) belongs to reg_init;
// afterwards it belongs to pro. pro never drives DB (input only).
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module lha6958h_top #(
  // --- system ---
  parameter CLK_FREQ_HZ        = 50_000_000,
  parameter SAMPLE_RATE_HZ     = 500_000,

  // --- lha6958h_pro timing (clks) ---
  parameter CONVST_HIGH_CLKS   = 4,
  parameter CONVST_LOW_CLKS    = 4,
  parameter RD_LOW_CLKS        = 2,        // 40ns @50MHz (满足 tLP_RD>=10ns，且覆盖出数延迟 tD_RD_DB<=27ns)
  parameter RD_HIGH_CLKS       = 2,        // 40ns @50MHz (满足 tHP_RD>=10ns，整周期 80ns >= tCYC_RD 30ns)
  parameter POWERUP_DELAY_CLKS = 500000,   // tPOWER-UP 10 ms @50MHz
  parameter RESET_HOLD_CLKS    = 500,      // RESET high pulse (10us @50MHz >= 3us)
  parameter RESET_RECOVERY_CLKS= 30000,    // tDEVICE_SETUP = 600us @50MHz after RESET fall
  parameter BUSY_TIMEOUT_CLKS  = 500,      // must exceed tCONV of OS_RATIO
  parameter [2:0] OS_SEL       = 3'b111,   // 111 = software mode

  // --- lha6958h_reg_init: register list ---
  parameter NUM_REGS           = 6,
  parameter [6:0] REG_ADDR0 = 7'h02, parameter [7:0] REG_DATA0 = 8'h08,
  parameter [6:0] REG_ADDR1 = 7'h03, parameter [7:0] REG_DATA1 = 8'h33,
  parameter [6:0] REG_ADDR2 = 7'h04, parameter [7:0] REG_DATA2 = 8'h33,
  parameter [6:0] REG_ADDR3 = 7'h05, parameter [7:0] REG_DATA3 = 8'h33,
  parameter [6:0] REG_ADDR4 = 7'h06, parameter [7:0] REG_DATA4 = 8'h33,
  parameter [6:0] REG_ADDR5 = 7'h08, parameter [7:0] REG_DATA5 = 8'h00,
  parameter [6:0] REG_ADDR6 = 7'h00, parameter [7:0] REG_DATA6 = 8'h00,
  parameter [6:0] REG_ADDR7 = 7'h00, parameter [7:0] REG_DATA7 = 8'h00,

  // --- lha6958h_reg_init: write timing (clks) ---
  parameter WR_LOW_CLKS        = 5,        // >= tLP_WR 88 ns
  parameter WR_HIGH_CLKS       = 12,       // >= tHP_WR 213 ns
  parameter CS_SETUP_CLKS      = 2
)(
  input  wire         clk,
  input  wire         rst_n,          // async, active-low system reset
  input  wire         init_restart,   // pulse: re-run register init
  input  wire         err_clear,      // pulse: clear sticky error flags

  // LHA6958H parallel bus (H1 connector pin in comment)
  output wire         convst,         // H1_37
  output wire         cs_n,           // H1_14
  output wire         rd_n,           // H1_20
  output wire         wr_n,           // H1_35
  output wire         adc_reset,      // H1_36
  output wire [2:0]   ad_os,          // OS2..OS0, strapped 111 by default
  inout  wire [15:0]  db,             // H1 DB15..DB0
  input  wire         busy,           // H1_34
  input  wire         first_data,     // H1_32 FRSTDATA

  // user side
  output wire [127:0] adc_data,       // {ch8..ch1}, ch1 = [15:0]
  output wire         adc_valid,      // single-cycle frame-valid pulse
  output wire         init_done,      // register init complete, streaming
  output wire         por_done,       // power-on reset pulse finished
  output wire         err_align,      // sticky: FRSTDATA mismatch on CH1
  output wire         err_busy_timeout,// sticky: BUSY watchdog expired
  output wire         err_overrun     // sticky: sample tick during active frame
);

  wire pro_cs_n, pro_rd_n;
  wire cfg_cs_n, cfg_wr_n, db_oe;
  wire [15:0] cfg_db;

  lha6958h_pro #(
    .CLK_FREQ_HZ        (CLK_FREQ_HZ),
    .SAMPLE_RATE_HZ     (SAMPLE_RATE_HZ),
    .CONVST_HIGH_CLKS   (CONVST_HIGH_CLKS),
    .CONVST_LOW_CLKS    (CONVST_LOW_CLKS),
    .RD_LOW_CLKS        (RD_LOW_CLKS),
    .RD_HIGH_CLKS       (RD_HIGH_CLKS),
    .POWERUP_DELAY_CLKS (POWERUP_DELAY_CLKS),
    .RESET_HOLD_CLKS    (RESET_HOLD_CLKS),
    .RESET_RECOVERY_CLKS(RESET_RECOVERY_CLKS),
    .BUSY_TIMEOUT_CLKS  (BUSY_TIMEOUT_CLKS),
    .OS_SEL             (OS_SEL)
  ) pro_inst (
    .clk        (clk),
    .rst_n      (rst_n),
    .init_done_i(init_done),
    .por_done   (por_done),
    .convst     (convst),
    .cs_n       (pro_cs_n),
    .rd_n       (pro_rd_n),
    .wr_n       (),              // pro ties WR high; init module drives it
    .adc_reset  (adc_reset),
    .ad_os      (ad_os),
    .db         (db),            // pro is read-only on DB
    .busy       (busy),
    .first_data (first_data),
    .adc_data   (adc_data),
    .adc_valid  (adc_valid),
    .err_align  (err_align),
    .err_busy_timeout (err_busy_timeout),
    .err_overrun(err_overrun),
    .err_clear  (err_clear)
  );

  lha6958h_reg_init #(
    .CLK_FREQ_HZ   (CLK_FREQ_HZ),
    .NUM_REGS      (NUM_REGS),
    .REG_ADDR0 (REG_ADDR0), .REG_DATA0 (REG_DATA0),
    .REG_ADDR1 (REG_ADDR1), .REG_DATA1 (REG_DATA1),
    .REG_ADDR2 (REG_ADDR2), .REG_DATA2 (REG_DATA2),
    .REG_ADDR3 (REG_ADDR3), .REG_DATA3 (REG_DATA3),
    .REG_ADDR4 (REG_ADDR4), .REG_DATA4 (REG_DATA4),
    .REG_ADDR5 (REG_ADDR5), .REG_DATA5 (REG_DATA5),
    .REG_ADDR6 (REG_ADDR6), .REG_DATA6 (REG_DATA6),
    .REG_ADDR7 (REG_ADDR7), .REG_DATA7 (REG_DATA7),
    .WR_LOW_CLKS   (WR_LOW_CLKS),
    .WR_HIGH_CLKS  (WR_HIGH_CLKS),
    .CS_SETUP_CLKS (CS_SETUP_CLKS)
  ) init_inst (
    .clk         (clk),
    .rst_n       (rst_n),
    .start_i     (por_done),
    .init_restart(init_restart),
    .cfg_cs_n    (cfg_cs_n),
    .cfg_wr_n    (cfg_wr_n),
    .cfg_db      (cfg_db),
    .db_oe       (db_oe),
    .init_done   (init_done)
  );

  // Bus ownership: reg_init while configuring, pro while streaming.
  // (RD is only ever driven by pro; reg_init uses WR, not RD.)
  assign cs_n = init_done ? pro_cs_n : cfg_cs_n;
  assign wr_n = init_done ? 1'b1     : cfg_wr_n;
  assign rd_n = pro_rd_n;
  assign db   = db_oe ? cfg_db : 16'hzzzz;

endmodule
