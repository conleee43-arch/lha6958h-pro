//////////////////////////////////////////////////////////////////////////////////
// Testbench : tb_lha6958h_overrun
//
// Overrun check: run lha6958h_pro at a sample rate the conversion+readout
// path cannot sustain, and verify err_overrun asserts.
//
// Math @50MHz: conversion 40 clks (0.8us model) + 8x RD burst (8x8=64 clks)
// + FSM overhead ~10 clks ~= 114 clks per frame. SAMPLE_RATE_HZ = 500kHz
// gives a 100-clk period < 114, so the timer must overrun.
//
// Run: iverilog -o tb_lha6958h_overrun tb_lha6958h_overrun.v \
//          ../rtl/lha6958h_pro.v && ./tb_lha6958h_overrun
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module tb_lha6958h_overrun;

  localparam CLK_PERIOD_NS = 20;

  reg clk = 0;
  always #(CLK_PERIOD_NS/2) clk = ~clk;

  reg rst_n = 0;
  reg err_clear = 0;

  wire convst, cs_n, rd_n, wr_n, adc_reset;
  wire [2:0] ad_os;
  wire [15:0] db = 16'hzzzz;
  reg  busy_r = 0;
  wire busy = busy_r;
  wire first_data = 1'b0;

  wire [127:0] adc_data;
  wire adc_valid;
  wire err_align, err_busy_timeout, err_overrun;

  lha6958h_pro #(
    .CLK_FREQ_HZ       (50_000_000),
    .SAMPLE_RATE_HZ    (500_000),   // 100-clk period: unsustainable on purpose
    .POWERUP_DELAY_CLKS(64),        // short for sim (10 ms in real use)
    .RESET_HOLD_CLKS   (64),
    .BUSY_TIMEOUT_CLKS (500)
  ) dut (
    .clk(clk), .rst_n(rst_n),
    .init_done_i(1'b1),             // standalone: no register init (HW-mode style)
    .por_done(),
    .convst(convst), .cs_n(cs_n), .rd_n(rd_n), .wr_n(wr_n),
    .adc_reset(adc_reset), .ad_os(ad_os),
    .db(db), .busy(busy), .first_data(first_data),
    .adc_data(adc_data), .adc_valid(adc_valid),
    .err_align(err_align), .err_busy_timeout(err_busy_timeout),
    .err_overrun(err_overrun), .err_clear(err_clear)
  );

  // minimal ADC model: BUSY high for 40 clks after each CONVST rising edge
  reg [15:0] conv_cnt = 0;
  reg converting = 0;
  always @(posedge convst) begin converting <= 1'b1; conv_cnt <= 0; end
  always @(posedge clk) begin
    if (converting) begin
      busy_r <= 1'b1;
      if (conv_cnt == 40 - 1) begin busy_r <= 1'b0; converting <= 1'b0; end
      else conv_cnt <= conv_cnt + 1'b1;
    end
  end

  integer w;
  initial begin
    $display("=== tb_lha6958h_overrun start ===");
    #(CLK_PERIOD_NS*10);
    rst_n = 1;
    w = 0;
    while (!err_overrun && w < 20000) begin
      #(CLK_PERIOD_NS);
      w = w + 1;
    end
    if (!err_overrun) begin
      $display("FAIL: err_overrun not asserted");
    end else begin
      $display("PASS: err_overrun asserted after %0d clks at 500kSPS", w);
    end
    err_clear = 1; #(CLK_PERIOD_NS*2); err_clear = 0;
    if (err_overrun)
      $display("FAIL: err_overrun not cleared");
    else
      $display("PASS: err_overrun cleared");
    $display("=== done ===");
    $finish;
  end

  initial begin
    #(CLK_PERIOD_NS*100000);
    $display("FAIL: simulation timeout");
    $finish;
  end

endmodule
