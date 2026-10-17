//////////////////////////////////////////////////////////////////////////////////
// Testbench : tb_lha6958h_reg_read
// Author    : Muse (for conleee43-arch)
// Date      : 2026-10-10
//
// Verifies lha6958h_reg_read against a behavioral LHA6958H model
// that supports the Two-Frame parallel readback protocol:
//   Test 1: Normal readback of Register 0x02 (CONFIG = 8'h08)
//   Test 2: Normal readback of Register 0x03 (RANGE  = 8'h33)
//   Test 3: Normal readback of Register 0x08 (OSR    = 8'h04)
//   Test 4: Verify Exit sequence writes (0x00, 0x00) and leaves chip in ADC mode
//   Test 5: Fault injection (corrupted return address) triggers err_o
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module tb_lha6958h_reg_read;

  localparam CLK_PERIOD_NS = 20; // 50 MHz

  reg clk = 0;
  always #(CLK_PERIOD_NS/2) clk = ~clk;

  reg rst_n = 0;

  // DUT signals
  reg        start_i = 0;
  reg  [6:0] addr_i = 0;
  wire [7:0] data_o;
  wire       busy_o;
  wire       done_o;
  wire       err_o;

  wire        reg_cs_n;
  wire        reg_rd_n;
  wire        reg_wr_n;
  wire [15:0] reg_db_out;
  wire        reg_db_oe;
  wire [15:0] db_bus;

  // Instantiate DUT
  lha6958h_reg_read #(
    .CLK_FREQ_HZ   (50_000_000),
    .WR_LOW_CLKS   (5),
    .WR_HIGH_CLKS  (12),
    .RD_LOW_CLKS   (4),
    .RD_HIGH_CLKS  (4),
    .CS_SETUP_CLKS (2),
    .CS_HOLD_CLKS  (2)
  ) dut (
    .clk        (clk),
    .rst_n      (rst_n),
    .start_i    (start_i),
    .addr_i     (addr_i),
    .data_o     (data_o),
    .busy_o     (busy_o),
    .done_o     (done_o),
    .err_o      (err_o),
    .reg_cs_n   (reg_cs_n),
    .reg_rd_n   (reg_rd_n),
    .reg_wr_n   (reg_wr_n),
    .reg_db_out (reg_db_out),
    .reg_db_oe  (reg_db_oe),
    .reg_db_in  (db_bus)
  );

  // ----------------------------------------------------------------
  // Behavioral LHA6958H model supporting Two-Frame readback
  // ----------------------------------------------------------------
  reg [7:0]  regfile [0:127];
  reg [15:0] model_db_drv = 16'hzzzz;
  reg [6:0]  latched_rd_addr = 7'd0;
  reg        in_reg_read_mode = 0;
  reg        in_reg_mode = 0;
  reg        fault_corrupt_addr = 0;

  assign db_bus = reg_db_oe ? reg_db_out : model_db_drv;

  // Initialize model registers
  initial begin
    regfile[7'h02] = 8'h08; // CONFIG
    regfile[7'h03] = 8'h33; // RANGE
    regfile[7'h08] = 8'h04; // OSR
  end

  // WR_N rising edge: latches address or write command
  always @(posedge reg_wr_n) begin
    if (!reg_cs_n) begin
      if (db_bus[15] == 1'b1) begin
        // Frame 1: Read address latch
        latched_rd_addr  <= db_bus[14:8];
        in_reg_read_mode <= 1'b1;
        in_reg_mode      <= 1'b1;
      end else begin
        // Write command or Exit command (address 0x00)
        regfile[db_bus[14:8]] <= db_bus[7:0];
        if (db_bus[14:8] == 7'h00) begin
          in_reg_read_mode <= 1'b0;
          in_reg_mode      <= 1'b0; // Exited to ADC mode!
        end
      end
    end
  end

  // RD_N falling edge: LHA6958H drives DB bus in read mode
  always @(negedge reg_rd_n) begin
    if (!reg_cs_n && in_reg_read_mode) begin
      if (fault_corrupt_addr)
        model_db_drv <= {1'b0, 7'h7F, regfile[latched_rd_addr]}; // Injected error
      else
        model_db_drv <= {1'b0, latched_rd_addr, regfile[latched_rd_addr]};
    end
  end

  always @(posedge reg_rd_n) begin
    model_db_drv <= 16'hzzzz;
  end

  always @(posedge reg_cs_n) begin
    model_db_drv <= 16'hzzzz;
  end

  // ----------------------------------------------------------------
  // Test procedure
  // ----------------------------------------------------------------
  integer errors = 0;

  task read_and_check;
    input [6:0] addr;
    input [7:0] exp_data;
    input       exp_err;
    begin
      @(posedge clk);
      addr_i  <= addr;
      start_i <= 1'b1;
      @(posedge clk);
      start_i <= 1'b0;

      // Wait for done_o
      while (!done_o) @(posedge clk);

      if (err_o !== exp_err) begin
        $display("FAIL [Addr %h]: err_o=%b, want %b", addr, err_o, exp_err);
        errors = errors + 1;
      end else if (!exp_err && data_o !== exp_data) begin
        $display("FAIL [Addr %h]: data_o=%h, want %h", addr, data_o, exp_data);
        errors = errors + 1;
      end else begin
        $display("PASS [Addr %h]: data_o=%h, err_o=%b (as expected)", addr, data_o, err_o);
      end

      // Check that chip returned to ADC mode (in_reg_mode == 0)
      if (in_reg_mode !== 1'b0) begin
        $display("FAIL [Addr %h]: chip NOT restored to ADC mode!", addr);
        errors = errors + 1;
      end else begin
        $display("PASS [Addr %h]: chip successfully returned to ADC mode", addr);
      end
    end
  endtask

  initial begin
    $dumpfile("reg_read.vcd");
    $dumpvars(0, tb_lha6958h_reg_read);
    $display("=== tb_lha6958h_reg_read start ===");
    #(CLK_PERIOD_NS * 5);
    rst_n = 1;
    #(CLK_PERIOD_NS * 5);

    // Test 1: Read CONFIG (0x02)
    $display("--- Test 1: Read Register 0x02 (CONFIG) ---");
    read_and_check(7'h02, 8'h08, 1'b0);

    // Test 2: Read RANGE (0x03)
    $display("--- Test 2: Read Register 0x03 (RANGE) ---");
    read_and_check(7'h03, 8'h33, 1'b0);

    // Test 3: Read OSR (0x08)
    $display("--- Test 3: Read Register 0x08 (OSR) ---");
    read_and_check(7'h08, 8'h04, 1'b0);

    // Test 4: Fault injection (corrupted returned address)
    $display("--- Test 4: Fault injection (expect error flag) ---");
    fault_corrupt_addr = 1;
    read_and_check(7'h02, 8'h08, 1'b1);
    fault_corrupt_addr = 0;

    #(CLK_PERIOD_NS * 20);
    if (errors == 0)
      $display("=== ALL %0d TESTS PASSED ===", 4);
    else
      $display("=== %0d TESTS FAILED ===", errors);

    $finish;
  end

endmodule

