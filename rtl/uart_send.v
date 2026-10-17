// =============================================================================
// Module Name:  uart_send
// Description:  UART 串口发送模块（并转串）
//               直接引用自参考工程 D:\FPGA\SoftWare-Project\LHA5278H_ADC_Acquisition\rtl\uart_send.v
//               标准 8-N-1 格式: 1 起始位(0), 8 数据位(LSB在前), 1 停止位(1)
// =============================================================================

`timescale 1ns / 1ps

module uart_send(
    input   wire        sys_clk,       // 系统时钟 50MHz
    input   wire        sys_rst_n,     // 低电平复位
    input   wire        uart_en,       // 发送使能脉冲
    input   wire [7:0]  uart_din,      // 待发送字节数据
    output  wire        uart_tx_busy,  // 发送忙标志
    output  reg         uart_txd       // 串口发送引脚
);

    parameter CLK_FREQ = 50000000;
    parameter UART_BPS = 1500000;
    localparam BPS_CNT = CLK_FREQ / UART_BPS;

    reg        uart_en_d0, uart_en_d1;
    reg [15:0] clk_cnt;
    reg [ 3:0] tx_cnt;
    reg        tx_flag;
    reg [ 7:0] tx_data;

    wire en_flag;

    assign uart_tx_busy = tx_flag; // 发送状态即为 busy
    assign en_flag = (~uart_en_d1) & uart_en_d0; // 使能上升沿检测

    // 打拍检测使能脉冲
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            uart_en_d0 <= 1'b0;
            uart_en_d1 <= 1'b0;
        end else begin
            uart_en_d0 <= uart_en;
            uart_en_d1 <= uart_en_d0;
        end
    end

    // 发送过程控制
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            tx_flag <= 1'b0;
            tx_data <= 8'd0;
        end else if (en_flag) begin
            tx_flag <= 1'b1;
            tx_data <= uart_din;
        end else if ((tx_cnt == 4'd9) && (clk_cnt == BPS_CNT - 1)) begin
            tx_flag <= 1'b0;
            tx_data <= 8'd0;
        end
    end

    // 时钟与位数计数
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            clk_cnt <= 16'd0;
        else if (tx_flag) begin
            if (clk_cnt < BPS_CNT - 1)
                clk_cnt <= clk_cnt + 1'b1;
            else
                clk_cnt <= 16'd0;
        end else
            clk_cnt <= 16'd0;
    end

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            tx_cnt <= 4'd0;
        else if (tx_flag) begin
            if (clk_cnt == BPS_CNT - 1)
                tx_cnt <= tx_cnt + 1'b1;
        end else
            tx_cnt <= 4'd0;
    end

    // 串行驱动输出 (LSB 优先)
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            uart_txd <= 1'b1;
        else if (tx_flag) begin
            case (tx_cnt)
                4'd0: uart_txd <= 1'b0;       // 起始位 (低电平)
                4'd1: uart_txd <= tx_data[0]; // 数据位 LSB 到 MSB
                4'd2: uart_txd <= tx_data[1];
                4'd3: uart_txd <= tx_data[2];
                4'd4: uart_txd <= tx_data[3];
                4'd5: uart_txd <= tx_data[4];
                4'd6: uart_txd <= tx_data[5];
                4'd7: uart_txd <= tx_data[6];
                4'd8: uart_txd <= tx_data[7];
                4'd9: uart_txd <= 1'b1;       // 停止位 (高电平)
                default: uart_txd <= 1'b1;
            endcase
        end else
            uart_txd <= 1'b1; // 空闲保持高电平
    end

endmodule
