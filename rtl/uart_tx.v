//==============================================================================
// Module Name:  uart_tx
// Description:  高精度 UART 串口数据发送模块 (参考 D:\FPGA\SoftWare-Project\LHA5278H_ADC_Acquisition\rtl\uart_tx.v)
//               支持 1.5Mbps (50MHz / 33分频，误差仅 1.01%)
//               标准 8-N-1 格式: 1 起始位(0), 8 数据位(LSB在前), 1 停止位(1)
// Standard:     Verilog-2001
//==============================================================================

`timescale 1ns / 1ps

module uart_tx #(
    parameter integer SYS_CLK_FREQ = 50_000_000, // 系统时钟 50MHz
    parameter integer BAUD_RATE    = 1_500_000    // 目标波特率 1.5Mbps
)(
    //--------------------------------------------------------------------------
    // 系统时钟与复位
    //--------------------------------------------------------------------------
    input  wire       sys_clk,       // 外部 50MHz 时钟
    input  wire       sys_rst_n,     // 低电平复位

    //--------------------------------------------------------------------------
    // 用户发送接口
    //--------------------------------------------------------------------------
    input  wire       tx_en,         // 发送使能脉冲 (高电平脉冲触发)
    input  wire [7:0] tx_data,       // 待发送 8 位数据
    output wire       tx_busy,       // 发送忙标志 (发送过程中持续为 1)
    output reg        tx_done,       // 单字节发送完成脉冲 (完成时刻单周期高电平)

    //--------------------------------------------------------------------------
    // 物理发送引脚
    //--------------------------------------------------------------------------
    output reg        uart_txd       // 串口发送引脚 (空闲为高电平 1)
);

    localparam integer BPS_CNT = SYS_CLK_FREQ / BAUD_RATE;

    reg [15:0] clk_cnt;
    reg [3:0]  tx_cnt;
    reg        tx_flag;
    reg [7:0]  data_reg;

    assign tx_busy = tx_flag;

    // 发送状态控制
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            tx_flag  <= 1'b0;
            data_reg <= 8'h00;
            tx_done  <= 1'b0;
        end else begin
            tx_done <= 1'b0; // 默认拉低完成脉冲

            if (tx_en && !tx_flag) begin
                tx_flag  <= 1'b1;
                data_reg <= tx_data;
            end else if ((tx_cnt == 4'd9) && (clk_cnt == BPS_CNT - 1)) begin
                tx_flag  <= 1'b0;
                data_reg <= 8'h00;
                tx_done  <= 1'b1; // 单字节发送完成单周期脉冲
            end
        end
    end

    // 波特率时钟计数器
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            clk_cnt <= 16'd0;
        end else if (tx_flag) begin
            if (clk_cnt < BPS_CNT - 1) begin
                clk_cnt <= clk_cnt + 1'b1;
            end else begin
                clk_cnt <= 16'd0;
            end
        end else begin
            clk_cnt <= 16'd0;
        end
    end

    // 位计数器 (0=起始位, 1~8=数据位, 9=停止位)
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            tx_cnt <= 4'd0;
        end else if (tx_flag) begin
            if (clk_cnt == BPS_CNT - 1) begin
                tx_cnt <= tx_cnt + 1'b1;
            end
        end else begin
            tx_cnt <= 4'd0;
        end
    end

    // 串行数据发送驱动 (LSB 优先)
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            uart_txd <= 1'b1;
        end else if (tx_flag) begin
            case (tx_cnt)
                4'd0: uart_txd <= 1'b0;        // 起始位 (0)
                4'd1: uart_txd <= data_reg[0]; // 数据位 D0 ~ D7
                4'd2: uart_txd <= data_reg[1];
                4'd3: uart_txd <= data_reg[2];
                4'd4: uart_txd <= data_reg[3];
                4'd5: uart_txd <= data_reg[4];
                4'd6: uart_txd <= data_reg[5];
                4'd7: uart_txd <= data_reg[6];
                4'd8: uart_txd <= data_reg[7];
                4'd9: uart_txd <= 1'b1;        // 停止位 (1)
                default: uart_txd <= 1'b1;
            endcase
        end else begin
            uart_txd <= 1'b1; // 空闲保持高电平
        end
    end

endmodule
