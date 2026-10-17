//==============================================================================
// Module Name: lha6958h_data_pack_buf
// Description: LHA6958H 1024 组 (8通道 16-bit，每组16字节) 数据缓存与极简通信协议打包发送控制器
//              1. 内部构建 1024 x 128-bit 简单双口 RAM (推断为片上 M9K Block RAM)
//              2. 接收端: 计满 1024 组后输出 pack_cap_done 脉冲通知控制层
//              3. 发送端: 接收 tx_start 后按极简协议依次发送:
//                 - 帧头 (4B): 0x5A, 0xA5, 0x55, 0xAA
//                 - 包序号 (1B): pkt_cnt (0x00 ~ 0xFF 循环自增)
//                 - 状态标志 (1B): status_in (硬件报警/系统状态)
//                 - 数据载荷 (16384B): 1024 组 x 16 字节 (每组 CH1~CH8 高字节在前 MSB first)
//                 - 帧尾 (2B): 0x0D, 0x0A (\r\n)
//                 整包共 16,392 字节，发送完成后输出 pack_tx_done 脉冲通知控制层进入下一轮采集
// Standard:    Verilog-2001
//==============================================================================

`timescale 1ns / 1ps

module lha6958h_data_pack_buf #(
    parameter integer PACK_GROUP_NUM = 1024, // 每次打包传输的组数 (默认 1024)
    parameter integer RAM_DEPTH      = 1024, // RAM 存储深度
    parameter integer ADDR_WIDTH     = 10    // RAM 地址位宽 (2^10 = 1024)
)(
    //--------------------------------------------------------------------------
    // 系统时钟与复位
    //--------------------------------------------------------------------------
    input  wire                   sys_clk,       // 50MHz 系统时钟
    input  wire                   sys_rst_n,     // 全局异步复位，低有效

    //--------------------------------------------------------------------------
    // 采集端数据写入接口 (与 lha6958h_top 连接)
    //--------------------------------------------------------------------------
    input  wire                   cap_en,        // 采集使能 (处于采集状态时有效)
    input  wire                   data_valid,    // 8 通道 16-bit 转换数据有效脉冲
    input  wire [127:0]           data_in,       // 8 通道拼接数据 {CH1[15:0], ..., CH8[15:0]}
    output reg                    pack_cap_done, // 采满 1024 组指示脉冲 (持续单周期)

    //--------------------------------------------------------------------------
    // 传输控制与 UART 发送器交互接口
    //--------------------------------------------------------------------------
    input  wire                   tx_start,      // 启动整包 UART 传输触发脉冲
    input  wire [7:0]             status_in,     // 状态/错误字 (注入到包头第6字节)
    output reg                    uart_tx_en,    // UART 发送使能脉冲 (单周期)
    output reg  [7:0]             uart_tx_data,  // 送往 UART 发送器的字节数据
    input  wire                   uart_tx_busy,  // UART 发送器繁忙指示 (接 uart_send 的 uart_tx_busy)
    output reg                    pack_tx_done,  // 全部 1024 组数据发送完毕指示脉冲
    output reg                    tx_active      // 正在进行 UART 发送过程指示
);

    //==========================================================================
    // 1024 x 128-bit 简单双口 RAM (Simple Dual-Port RAM)
    // Quartus II 会自动推断并综合为片上 M9K Block RAM
    //==========================================================================
    reg [127:0] mem [0:RAM_DEPTH-1];
    reg [127:0] ram_rd_data;

    // RAM 写逻辑 (采集写入)
    reg [ADDR_WIDTH-1:0] wr_addr;
    wire ram_wr_en = cap_en && data_valid;

    always @(posedge sys_clk) begin
        if (ram_wr_en) begin
            mem[wr_addr] <= data_in;
        end
    end

    // RAM 读逻辑 (UART 读出)
    reg [ADDR_WIDTH-1:0] rd_addr;
    reg                  ram_rd_en;

    always @(posedge sys_clk) begin
        if (ram_rd_en) begin
            ram_rd_data <= mem[rd_addr];
        end
    end

    //==========================================================================
    // 采集写入计数与满包控制
    //==========================================================================
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            wr_addr       <= {ADDR_WIDTH{1'b0}};
            pack_cap_done <= 1'b0;
        end else begin
            pack_cap_done <= 1'b0; // 默认拉低单周期脉冲

            if (!cap_en) begin
                wr_addr <= {ADDR_WIDTH{1'b0}}; // 停止采集时复位写指针
            end else if (data_valid) begin
                if (wr_addr == PACK_GROUP_NUM - 1) begin
                    wr_addr       <= {ADDR_WIDTH{1'b0}};
                    pack_cap_done <= 1'b1; // 采满 1024 组，触发采集完成
                end else begin
                    wr_addr <= wr_addr + 1'b1;
                end
            end
        end
    end

    //==========================================================================
    // UART 打包读取与极简协议发送状态机
    // 协议时序: 
    // 1. 响应 tx_start
    // 2. 发送 6 字节包头 (0x5A, 0xA5, 0x55, 0xAA, pkt_cnt, status_reg)
    // 3. 依次读取 0 ~ 1023 组数据，每组 16 字节逐字节发送 (共 16384 字节)
    // 4. 发送 2 字节包尾 (0x0D, 0x0A)
    // 5. 输出 pack_tx_done 脉冲，pkt_cnt 自增 1
    //==========================================================================
    localparam [3:0]
        ST_TX_IDLE      = 4'd0,  // 空闲状态，等待 tx_start
        ST_TX_HEAD      = 4'd1,  // 发送 6 字节包头 (4B 帧头 + 1B 序号 + 1B 状态)
        ST_TX_HEAD_WAIT = 4'd2,  // 等待包头单字节发送完成
        ST_TX_RD_RAM    = 4'd3,  // 发出 RAM 读地址与使能
        ST_TX_LATCH     = 4'd4,  // 等待 1 拍 RAM 读延迟并锁存 128 位组数据
        ST_TX_DATA      = 4'd5,  // 触发 UART 发送当前数据字节
        ST_TX_DATA_WAIT = 4'd6,  // 等待数据单字节发送完成
        ST_TX_TAIL      = 4'd7,  // 发送 2 字节包尾 (0x0D, 0x0A)
        ST_TX_TAIL_WAIT = 4'd8,  // 等待包尾单字节发送完成
        ST_TX_FINISH    = 4'd9;  // 整包发送完成状态

    reg [3:0]   tx_fsm;
    reg [7:0]   pkt_cnt;        // 0 ~ 255 循环递增包序号
    reg [7:0]   status_reg;     // 锁存本次大包的状态字节
    reg [2:0]   head_idx;       // 0 ~ 5 包头索引
    reg         tail_idx;       // 0 ~ 1 包尾索引
    reg [4:0]   byte_idx;       // 0 ~ 15 每组 16 字节计数器
    reg [127:0] group_shift_reg;// 128-bit 组数据移位寄存器

    // 检测 uart_tx_busy 下降沿 (当前字节完全发送完毕时刻)
    reg uart_tx_busy_d;
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            uart_tx_busy_d <= 1'b0;
        else
            uart_tx_busy_d <= uart_tx_busy;
    end
    wire uart_byte_done = uart_tx_busy_d & (~uart_tx_busy);

    // 获取包头字节
    function [7:0] get_header_byte;
        input [2:0] idx;
        input [7:0] count;
        input [7:0] stat;
        case (idx)
            3'd0: get_header_byte = 8'h5A; // 帧同步头 1
            3'd1: get_header_byte = 8'hA5; // 帧同步头 2
            3'd2: get_header_byte = 8'h55; // 帧同步头 3
            3'd3: get_header_byte = 8'hAA; // 帧同步头 4
            3'd4: get_header_byte = count; // 包序号
            3'd5: get_header_byte = stat;  // 状态标志
            default: get_header_byte = 8'h00;
        endcase
    endfunction

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            tx_fsm          <= ST_TX_IDLE;
            pkt_cnt         <= 8'h00;
            status_reg      <= 8'h00;
            head_idx        <= 3'd0;
            tail_idx        <= 1'b0;
            rd_addr         <= {ADDR_WIDTH{1'b0}};
            ram_rd_en       <= 1'b0;
            byte_idx        <= 5'd0;
            group_shift_reg <= 128'd0;
            uart_tx_en      <= 1'b0;
            uart_tx_data    <= 8'h00;
            pack_tx_done    <= 1'b0;
            tx_active       <= 1'b0;
        end else begin
            pack_tx_done <= 1'b0; // 默认拉低单周期脉冲
            uart_tx_en   <= 1'b0; // 默认拉低单周期脉冲
            ram_rd_en    <= 1'b0; // 默认拉低读使能

            case (tx_fsm)
                //--------------------------------------------------------------
                // 1. 空闲状态: 等待启动传输脉冲
                //--------------------------------------------------------------
                ST_TX_IDLE: begin
                    tx_active <= 1'b0;
                    rd_addr   <= {ADDR_WIDTH{1'b0}};
                    head_idx  <= 3'd0;
                    tail_idx  <= 1'b0;
                    byte_idx  <= 5'd0;
                    if (tx_start) begin
                        tx_active  <= 1'b1;
                        status_reg <= status_in;
                        tx_fsm     <= ST_TX_HEAD; // 首先发送 6 字节包头
                    end
                end

                //--------------------------------------------------------------
                // 2. 发送 6 字节包头 (0x5A, 0xA5, 0x55, 0xAA, pkt_cnt, status)
                //--------------------------------------------------------------
                ST_TX_HEAD: begin
                    if (!uart_tx_busy) begin
                        uart_tx_en   <= 1'b1;
                        uart_tx_data <= get_header_byte(head_idx, pkt_cnt, status_reg);
                        tx_fsm       <= ST_TX_HEAD_WAIT;
                    end
                end

                ST_TX_HEAD_WAIT: begin
                    if (uart_byte_done) begin
                        if (head_idx < 3'd5) begin
                            head_idx <= head_idx + 1'b1;
                            tx_fsm   <= ST_TX_HEAD; // 继续发送下一个包头字节
                        end else begin
                            // 包头 6 字节已发完，启动读取第 0 组 RAM 数据
                            rd_addr   <= {ADDR_WIDTH{1'b0}};
                            ram_rd_en <= 1'b1;
                            tx_fsm    <= ST_TX_LATCH;
                        end
                    end
                end

                //--------------------------------------------------------------
                // 3. 发起 RAM 读使能
                //--------------------------------------------------------------
                ST_TX_RD_RAM: begin
                    ram_rd_en <= 1'b1;
                    tx_fsm    <= ST_TX_LATCH;
                end

                //--------------------------------------------------------------
                // 4. 锁存从 RAM 读出的 128-bit 组数据
                //--------------------------------------------------------------
                ST_TX_LATCH: begin
                    group_shift_reg <= ram_rd_data;
                    byte_idx        <= 5'd0;
                    tx_fsm          <= ST_TX_DATA;
                end

                //--------------------------------------------------------------
                // 5. 发送当前数据字节: 取出最高字节送给 uart_send
                //--------------------------------------------------------------
                ST_TX_DATA: begin
                    if (!uart_tx_busy) begin
                        uart_tx_en   <= 1'b1;
                        uart_tx_data <= group_shift_reg[127:120];
                        tx_fsm       <= ST_TX_DATA_WAIT;
                    end
                end

                //--------------------------------------------------------------
                // 6. 等待单数据字节发送完成
                //--------------------------------------------------------------
                ST_TX_DATA_WAIT: begin
                    if (uart_byte_done) begin
                        group_shift_reg <= {group_shift_reg[119:0], 8'h00}; // 移出已发送的 8 位
                        if (byte_idx < 5'd15) begin
                            byte_idx <= byte_idx + 1'b1;
                            tx_fsm   <= ST_TX_DATA; // 继续发送当前组的下一个字节 (共 16 字节)
                        end else begin
                            // 当前组 16 字节已全部发送完毕
                            if (rd_addr < PACK_GROUP_NUM - 1) begin
                                rd_addr <= rd_addr + 1'b1;
                                tx_fsm  <= ST_TX_RD_RAM; // 读取下一组数据
                            end else begin
                                // 1024 组数据全部发完，进入包尾发送阶段
                                tail_idx <= 1'b0;
                                tx_fsm   <= ST_TX_TAIL;
                            end
                        end
                    end
                end

                //--------------------------------------------------------------
                // 7. 发送 2 字节包尾 (0x0D, 0x0A)
                //--------------------------------------------------------------
                ST_TX_TAIL: begin
                    if (!uart_tx_busy) begin
                        uart_tx_en   <= 1'b1;
                        uart_tx_data <= (tail_idx == 1'b0) ? 8'h0D : 8'h0A;
                        tx_fsm       <= ST_TX_TAIL_WAIT;
                    end
                end

                ST_TX_TAIL_WAIT: begin
                    if (uart_byte_done) begin
                        if (tail_idx == 1'b0) begin
                            tail_idx <= 1'b1;
                            tx_fsm   <= ST_TX_TAIL; // 发送第 2 个包尾字节 0x0A
                        end else begin
                            // 2 字节包尾发送完毕，整包完成
                            tx_fsm <= ST_TX_FINISH;
                        end
                    end
                end

                //--------------------------------------------------------------
                // 8. 全部数据发送完成: 产生 pack_tx_done 脉冲，序号自增，回到 IDLE
                //--------------------------------------------------------------
                ST_TX_FINISH: begin
                    pack_tx_done <= 1'b1;
                    pkt_cnt      <= pkt_cnt + 1'b1; // 包序号自增
                    tx_active    <= 1'b0;
                    tx_fsm       <= ST_TX_IDLE;
                end

                default: begin
                    tx_fsm <= ST_TX_IDLE;
                end
            endcase
        end
    end

endmodule
