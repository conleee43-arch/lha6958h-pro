//////////////////////////////////////////////////////////////////////////////////
// Testbench : tb_lha6958h_pro
//
// Behavioral model of the LHA6958H (parallel interface, software mode)
// plus the integrated lha6958h_top, and five test phases:
//   Phase 0 : power-up order - RESET pulse must complete BEFORE the first
//             register write (writes during RESET are ignored by silicon).
//   Phase 1 : register init - 6 user regs (with NON-default values to prove
//             the writes really land) + auto-appended (0x00,0x00) exit write.
//   Phase 2 : normal operation - 3 frames, data pattern, adc_valid period,
//             no error flags.
//   Phase 3 : FRSTDATA stuck low -> err_align must assert; clear & recover.
//   Phase 4 : BUSY stuck high  -> err_busy_timeout must assert; clear/recover.
//
// Model notes (Rev.B facts): conversion starts on CONVST RISING edge;
// BUSY asserts <= 20ns after CONVST rising; FRSTDATA goes high on the RD
// falling edge for V1 and low after the next RD falling edge; WR rising
// edge latches {cmd,addr,data} with DB15=0 for writes; register writes
// while the RESET pin is high are IGNORED (this is what Phase 0 guards).
//
// Run: iverilog -o tb_lha6958h_pro tb_lha6958h_pro.v \
//          ../rtl/lha6958h_top.v ../rtl/lha6958h_pro.v \
//          ../rtl/lha6958h_reg_init.v && ./tb_lha6958h_pro
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module tb_lha6958h_pro;

  // Use a fast sample rate so the sim finishes quickly:
  // 50MHz / 50kHz = 1000 clocks per frame.
  localparam CLK_PERIOD_NS  = 20;
  localparam SAMPLE_RATE_HZ = 50_000;
  localparam FRAME_CLKS     = 50_000_000 / SAMPLE_RATE_HZ; // 1000

  reg clk = 0;
  always #(CLK_PERIOD_NS/2) clk = ~clk;

  reg rst_n = 0;
  reg err_clear = 0;

  // ----------------------------------------------------------------
  // DUT: integrated top
  // ----------------------------------------------------------------
  wire        convst, cs_n, rd_n, wr_n, adc_reset;
  wire [2:0]  ad_os;
  wire [15:0] db_bus;
  wire        busy, first_data;
  wire [127:0] adc_data;
  wire        adc_valid, init_done, por_done;
  wire        err_align, err_busy_timeout, err_overrun;

  lha6958h_top #(
    .CLK_FREQ_HZ        (50_000_000),
    .SAMPLE_RATE_HZ     (SAMPLE_RATE_HZ),
    .POWERUP_DELAY_CLKS (100),   // short for sim (10 ms in real use)
    .RESET_HOLD_CLKS    (256),   // short for sim (>= 3 us full-reset in real use)
    .RESET_RECOVERY_CLKS(300),   // short for sim (600 us tDEVICE_SETUP in real use)
    .BUSY_TIMEOUT_CLKS  (500),
    .OS_SEL             (3'b111),
    // non-default register values: prove the writes really take effect
    // (reset defaults are 0x33, so 0x70 must come from our write)
    .REG_DATA0          (8'h08), // CONFIG: reset default
    .REG_DATA1          (8'h70), // RANGE 0x03: ch2=+/-12.5V HBW, ch1=+/-2.5V
    .REG_DATA2          (8'h33),
    .REG_DATA3          (8'h33),
    .REG_DATA4          (8'h33),
    .REG_DATA5          (8'h00)   // OVERSAMPLING: none
  ) top_inst (
    .clk(clk), .rst_n(rst_n),
    .init_restart(1'b0), .err_clear(err_clear),
    .convst(convst), .cs_n(cs_n), .rd_n(rd_n), .wr_n(wr_n),
    .adc_reset(adc_reset), .ad_os(ad_os),
    .db(db_bus), .busy(busy), .first_data(first_data),
    .adc_data(adc_data), .adc_valid(adc_valid),
    .init_done(init_done), .por_done(por_done),
    .err_align(err_align), .err_busy_timeout(err_busy_timeout),
    .err_overrun(err_overrun)
  );

  // ----------------------------------------------------------------
  // Behavioral LHA6958H model
  // ----------------------------------------------------------------
  reg [7:0]  regfile [0:127];
  reg [15:0] db_drv = 16'hzzzz;
  reg        busy_r = 0;
  reg        first_data_r = 0;
  reg [3:0]  rd_idx = 0;
  reg [15:0] ch_mem [0:7];
  integer    write_count = 0;

  // Phase-0 ordering probes
  integer t_reset_fall = 0;
  integer t_first_write = 0;

  // fault injection controls
  reg fault_align_stuck_low = 0;  // FRSTDATA never asserts
  reg fault_busy_stuck_high = 0;  // BUSY never deasserts

  assign db_bus     = db_drv;
  assign busy       = fault_busy_stuck_high ? 1'b1 : busy_r;
  assign first_data = fault_align_stuck_low ? 1'b0 : first_data_r;

  integer i;
  // power-on defaults (Rev.B reset values where known)
  task model_reset;
    begin
      for (i = 0; i < 128; i = i + 1) regfile[i] = 8'h00;
      regfile[8'h02] = 8'h08;
      regfile[8'h03] = 8'h33; regfile[8'h04] = 8'h33;
      regfile[8'h05] = 8'h33; regfile[8'h06] = 8'h33;
      write_count  = 0;
      busy_r       = 1'b0;
      rd_idx       = 0;
      db_drv       = 16'hzzzz;
      first_data_r = 1'b0;
      for (i = 0; i < 8; i = i + 1)
        ch_mem[i] = 16'h0101 * (i + 1);   // ch1=0x0101 ... ch8=0x0808
    end
  endtask

  initial model_reset();
  always @(posedge adc_reset) model_reset();
  always @(negedge adc_reset) t_reset_fall = $time;

  // register writes: WR rising edge latches {cmd,addr[6:0],data[7:0]}.
  // Silicon ignores writes while RESET is high - the model does too.
  always @(posedge wr_n) begin
    if (!cs_n && !adc_reset) begin
      if (db_bus[15] == 1'b0) begin   // write command
        regfile[db_bus[14:8]] <= db_bus[7:0];
        if (write_count == 0) t_first_write <= $time;
        write_count <= write_count + 1;
      end
    end
  end

  // conversion: CONVST RISING edge -> BUSY high for T_CONV_CLKS
  localparam T_CONV_CLKS = 40;   // 0.8us, >= 0.62us typical no-OS tCONV
  reg [15:0] conv_cnt = 0;
  reg        converting = 0;

  always @(posedge convst) begin
    converting <= 1'b1;
    conv_cnt   <= 0;
  end

  always @(posedge clk) begin
    if (converting) begin
      busy_r <= 1'b1;
      if (conv_cnt == T_CONV_CLKS - 1) begin
        busy_r     <= 1'b0;
        converting <= 1'b0;
      end else begin
        conv_cnt <= conv_cnt + 1'b1;
      end
    end
  end

  // parallel read: RD_N falling drives channel data; FRSTDATA for CH1
  always @(negedge rd_n) begin
    if (!cs_n) begin
      db_drv       <= ch_mem[rd_idx];
      first_data_r <= (rd_idx == 0);
    end
  end
  always @(posedge rd_n) begin
    if (!cs_n && rd_idx < 7)
      rd_idx <= rd_idx + 1'b1;
  end
  always @(posedge cs_n) begin
    rd_idx       <= 0;
    db_drv       <= 16'hzzzz;
    first_data_r <= 1'b0;
  end

  // ----------------------------------------------------------------
  // Test procedure
  // ----------------------------------------------------------------
  integer errors = 0;
  integer last_valid_time = 0;
  integer period_clks;
  integer w;
  reg [127:0] expected_data;

  initial begin
    expected_data = {16'h0808, 16'h0707, 16'h0606, 16'h0505,
                     16'h0404, 16'h0303, 16'h0202, 16'h0101};

    $display("=== tb_lha6958h_pro start ===");
    #(CLK_PERIOD_NS*10);
    rst_n = 1;
    $display("[%0t] reset released", $time);

    // ---------------- Phase 0: reset-before-write ordering ----------------
    $display("--- Phase 0: reset pulse must precede first register write ---");
    w = 0;
    while (!por_done && w < 100000) begin
      #(CLK_PERIOD_NS);
      w = w + 1;
    end
    if (!por_done) begin
      $display("FAIL: por_done never asserted"); errors = errors + 1;
    end else begin
      $display("PASS: por_done asserted after %0d clks", w);
    end

    // ---------------- Phase 1: register init ----------------
    $display("--- Phase 1: register init ---");
    w = 0;
    while (!init_done && w < 100000) begin
      #(CLK_PERIOD_NS);
      w = w + 1;
    end
    if (!init_done) begin
      $display("FAIL: init_done never asserted"); errors = errors + 1;
    end else begin
      $display("PASS: init_done asserted after %0d clks", w);
    end
    if (t_first_write == 0 || t_first_write < t_reset_fall) begin
      $display("FAIL: first write at %0t not after RESET fall at %0t",
               t_first_write, t_reset_fall);
      errors = errors + 1;
    end else if (t_first_write - t_reset_fall < 300 * CLK_PERIOD_NS) begin
      $display("FAIL: recovery delay violated: elapsed %0t ns, want >= %0t ns",
               t_first_write - t_reset_fall, 300 * CLK_PERIOD_NS);
      errors = errors + 1;
    end else begin
      $display("PASS: tDEVICE_SETUP recovery delay verified (elapsed %0t ns >= %0t ns)",
               t_first_write - t_reset_fall, 300 * CLK_PERIOD_NS);
    end
    // 6 user writes + 1 auto-appended (0x00,0x00)
    if (write_count !== 7) begin
      $display("FAIL: write_count=%0d, want 7 (6 regs + exit write)", write_count);
      errors = errors + 1;
    end else begin
      $display("PASS: 7 register writes seen (incl. 0x00 exit write)");
    end
    if (regfile[8'h02] !== 8'h08) begin
      $display("FAIL: CONFIG(0x02)=%h want 08", regfile[8'h02]); errors = errors + 1;
    end
    // 0x70 is NOT the reset default (0x33): proves the write really landed
    if (regfile[8'h03] !== 8'h70) begin
      $display("FAIL: RANGE(0x03)=%h want 70 (custom value)", regfile[8'h03]);
      errors = errors + 1;
    end else begin
      $display("PASS: RANGE 0x03 = 0x70 (custom value took effect)");
    end
    if (regfile[8'h04] !== 8'h33 || regfile[8'h05] !== 8'h33 ||
        regfile[8'h06] !== 8'h33) begin
      $display("FAIL: RANGE regs 0x04..0x06 not 0x33"); errors = errors + 1;
    end
    if (regfile[8'h08] !== 8'h00) begin
      $display("FAIL: OVERSAMPLING(0x08)=%h want 00", regfile[8'h08]); errors = errors + 1;
    end else begin
      $display("PASS: OVERSAMPLING(0x08) = 0x00 (no oversampling)");
    end

    // ---------------- Phase 2: normal frames ----------------
    $display("--- Phase 2: normal frames ---");
    repeat (3) @(posedge adc_valid);
    @(posedge adc_valid); last_valid_time = $time;
    @(posedge adc_valid);
    period_clks = ($time - last_valid_time) / CLK_PERIOD_NS;
    $display("frame period = %0d clks (expect %0d)", period_clks, FRAME_CLKS);
    if (period_clks != FRAME_CLKS) begin
      $display("FAIL: frame period mismatch"); errors = errors + 1;
    end
    if (adc_data !== expected_data) begin
      $display("FAIL: data mismatch: got %h want %h", adc_data, expected_data);
      errors = errors + 1;
    end else begin
      $display("PASS: 8-channel data pattern correct");
    end
    if (err_align || err_busy_timeout || err_overrun) begin
      $display("FAIL: unexpected error flags a=%b t=%b o=%b",
               err_align, err_busy_timeout, err_overrun);
      errors = errors + 1;
    end else begin
      $display("PASS: no error flags in normal operation");
    end

    // ---------------- Phase 3: FRSTDATA fault ----------------
    $display("--- Phase 3: FRSTDATA stuck low ---");
    fault_align_stuck_low = 1;
    @(posedge adc_valid);
    #(CLK_PERIOD_NS*5);
    if (!err_align) begin
      $display("FAIL: err_align not asserted"); errors = errors + 1;
    end else begin
      $display("PASS: err_align asserted on FRSTDATA fault");
    end
    err_clear = 1; #(CLK_PERIOD_NS*2); err_clear = 0;
    fault_align_stuck_low = 0;
    @(posedge adc_valid); #(CLK_PERIOD_NS*5);
    if (err_align) begin
      $display("FAIL: err_align not cleared"); errors = errors + 1;
    end else begin
      $display("PASS: err_align cleared, frames resume");
    end
    if (adc_data !== expected_data) begin
      $display("FAIL: data wrong after recovery"); errors = errors + 1;
    end

    // ---------------- Phase 4: BUSY stuck high ----------------
    $display("--- Phase 4: BUSY stuck high ---");
    fault_busy_stuck_high = 1;
    w = 0;
    while (!err_busy_timeout && w < 3000) begin
      #(CLK_PERIOD_NS);
      w = w + 1;
    end
    if (!err_busy_timeout) begin
      $display("FAIL: err_busy_timeout not asserted"); errors = errors + 1;
    end else begin
      $display("PASS: err_busy_timeout asserted on BUSY stuck");
    end
    err_clear = 1; #(CLK_PERIOD_NS*2); err_clear = 0;
    fault_busy_stuck_high = 0;
    @(posedge adc_valid); #(CLK_PERIOD_NS*5);
    if (err_busy_timeout) begin
      $display("FAIL: err_busy_timeout not cleared"); errors = errors + 1;
    end else begin
      $display("PASS: watchdog cleared, frames resume");
    end
    if (adc_data !== expected_data) begin
      $display("FAIL: data wrong after watchdog recovery"); errors = errors + 1;
    end

    // ---------------- summary ----------------
    #(CLK_PERIOD_NS*100);
    if (errors == 0)
      $display("=== ALL TESTS PASSED ===");
    else
      $display("=== %0d TEST(S) FAILED ===", errors);
    $finish;
  end

  // safety timeout
  initial begin
    #(CLK_PERIOD_NS*500000);
    $display("FAIL: simulation timeout");
    $finish;
  end

endmodule
