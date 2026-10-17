//==============================================================================
// Module Name: lha6958h_board_top
// Description: LHA6958H 8 通道 16 位 1MSPS ADC FPGA 板级工程顶层
//              目标芯片: Altera Cyclone IV E (EP4CE10F17C8)
//              集成:
//              1. LHA6958H 并行总线驱动及寄存器初始化
//              2. 1024 组 (8通道 16-bit, 128-bit/组) 片上 RAM 缓存与打包模块
//              3. UART 高速串口发送 (参考 LHA5278H 架构，支持 1.5Mbps / 115200)
//              4. 上电/心跳问候字符串 ("Hello! LHA6958H ADC UART OK!\r\n")
//              5. 循环突发采集与串口大包传输调度
//              6. 板载 LED 状态指示 (LED0: 初始化完成, LED1: 心跳闪烁/错误常亮)
// Standard:    Verilog-2001
//==============================================================================

`timescale 1ns / 1ps

module lha6958h_board_top #(
    parameter CLK_FREQ_HZ    = 50_000_000, // 板载 50MHz 时钟
    parameter SAMPLE_RATE_HZ = 500_000,    // 采样率 (配置为 500kSPS，转换完成后读数无溢出)
    parameter UART_BPS       = 1_500_000,  // UART 串口波特率 (默认 1.5Mbps，可改为 115200)
    parameter PACK_GROUP_NUM = 1024,       // 每次打包传输的组数 (默认 1024 组)
    parameter POR_WAIT_CYCLES= 500_000     // 上电首次测试字符等待周期 (10ms @50MHz)
)(
    //--------------------------------------------------------------------------
    // 板载基础时钟与控制
    //--------------------------------------------------------------------------
    input  wire        clk,            // PIN_E1, 板载 50MHz 主时钟
    input  wire        rst_n,          // PIN_M1, 板载复位按键 (低有效)

    //--------------------------------------------------------------------------
    // LHA6958H 并行总线及控制线 (H1 接口，也是 Logic Pro 8 逻辑分析仪 CH0~CH6 监测引脚)
    //--------------------------------------------------------------------------
    output wire        RESET,          // PIN_P15, H1_36 (Logic CH0: 复位，高有效)
    output wire        WR,             // PIN_R14, H1_35 (Logic CH1: 寄存器写选通)
    output wire        CS,             // PIN_T9,  H1_14 (Logic CH2: 片选，低有效)
    output wire        CONVST,         // PIN_T12, H1_37 (Logic CH3: 启动转换，上升沿)
    input  wire        BUSY,           // PIN_N13, H1_34 (Logic CH4: 忙指示，高有效)
    output wire        RD,             // PIN_N6,  H1_20 (Logic CH5: 读选通，低有效)
    input  wire        FRSTDATA,       // PIN_M11, H1_32 (Logic CH6: 首通道指示)

    inout  wire [15:0] DB,             // H1 并行数据总线 DB[15:0]

    //--------------------------------------------------------------------------
    // 板载 LED 指示灯
    //--------------------------------------------------------------------------
    output wire        led_init_done,  // PIN_A4, LED0 (低电平点亮: 上电寄存器初始化完成)
    output wire        led_heartbeat,  // PIN_B5, LED1 (心跳闪烁: 指示系统运行与数据采集中)

    //--------------------------------------------------------------------------
    // 板载 UART 串口接口 (参考 LHA5278H 架构)
    //--------------------------------------------------------------------------
    output wire        uart_txd        // PIN_A13, UART 串口发送端口 (1.5Mbps / 115200)
);

    //==========================================================================
    // 异步复位同步释放处理 (与参考工程一致)
    //==========================================================================
    reg [1:0] rst_sync = 2'b00;
    wire sys_rst_n_sync = rst_sync[1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rst_sync <= 2'b00;
        else
            rst_sync <= {rst_sync[0], 1'b1};
    end

    //==========================================================================
    // 1 秒心跳脉冲与 LED1 闪烁发生器 (50MHz 下 50,000,000 周期)
    //==========================================================================
    reg [25:0] sec_cnt       = 26'd0;
    reg        sec_pulse     = 1'b0;
    reg        heartbeat_reg = 1'b1; // 复位时默认熄灭 (低电平点亮)

    always @(posedge clk or negedge sys_rst_n_sync) begin
        if (!sys_rst_n_sync) begin
            sec_cnt       <= 26'd0;
            sec_pulse     <= 1'b0;
            heartbeat_reg <= 1'b1;
        end else begin
            if (sec_cnt < 26'd49_999_999) begin
                sec_cnt   <= sec_cnt + 1'b1;
                sec_pulse <= 1'b0;
            end else begin
                sec_cnt       <= 26'd0;
                sec_pulse     <= 1'b1;           // 产生 1 秒单周期心跳脉冲
                heartbeat_reg <= ~heartbeat_reg; // 1 秒翻转一次
            end
        end
    end

    //==========================================================================
    // 内部信号线网声明
    //==========================================================================
    wire [127:0] adc_data;
    wire         adc_valid;
    wire         init_done;
    wire         por_done;
    wire         err_align;
    wire         err_busy_timeout;
    wire         err_overrun;

    // 锁存最新一组 8 通道 ADC 数据（防止综合被优化）
    reg [127:0]  adc_data_latched;
    always @(posedge clk or negedge sys_rst_n_sync) begin
        if (!sys_rst_n_sync)
            adc_data_latched <= 128'd0;
        else if (adc_valid)
            adc_data_latched <= adc_data;
    end

    // LED0: 初始化完成后常亮 (低电平点亮)
    assign led_init_done = ~init_done;

    // LED1: 正常采集时 1Hz 闪烁；若发生错误则常亮报警 (低电平点亮)
    wire has_err = err_align | err_busy_timeout | err_overrun;
    assign led_heartbeat = has_err ? 1'b0 : heartbeat_reg;

    //==========================================================================
    // 例化 LHA6958H 驱动核心顶层
    //==========================================================================
    lha6958h_top #(
        .CLK_FREQ_HZ        (CLK_FREQ_HZ),
        .SAMPLE_RATE_HZ     (SAMPLE_RATE_HZ),
        .CONVST_HIGH_CLKS   (4),     // 80ns @50MHz (>= tHP_CNV 80ns)
        .CONVST_LOW_CLKS    (4),     // 80ns @50MHz (>= tLP_CNV 80ns)
        .RD_LOW_CLKS        (2),     // 40ns @50MHz (>= tD_RD_DB 27ns)
        .RD_HIGH_CLKS       (2),     // 40ns @50MHz (周期 80ns >= 30ns)
        .REG_DATA1          (8'h22), // CH2/CH1: ±10V 量程
        .REG_DATA2          (8'h22), // CH4/CH3: ±10V 量程
        .REG_DATA3          (8'h22), // CH6/CH5: ±10V 量程
        .REG_DATA4          (8'h22), // CH8/CH7: ±10V 量程
        .REG_DATA5          (8'h00)  // 无过采样 (1MSPS capability)
    ) u_lha6958h_top (
        .clk              (clk),
        .rst_n            (sys_rst_n_sync),
        .init_restart     (1'b0),
        .err_clear        (1'b0),

        // LHA6958H 接口
        .convst           (CONVST),
        .cs_n             (CS),
        .rd_n             (RD),
        .wr_n             (WR),
        .adc_reset        (RESET),
        .ad_os            (),
        .db               (DB),
        .busy             (BUSY),
        .first_data       (FRSTDATA),

        // 内部接口
        .adc_data         (adc_data),
        .adc_valid        (adc_valid),
        .init_done        (init_done),
        .por_done         (por_done),
        .err_align        (err_align),
        .err_busy_timeout (err_busy_timeout),
        .err_overrun      (err_overrun)
    );

    //==========================================================================
    // 组装 8 通道 16-bit 数据为大端序格式送入打包 RAM (CH1 -> CH8)
    // adc_data 映射:
    // adc_data[ 15:  0] = CH1
    // adc_data[ 31: 16] = CH2
    // adc_data[ 47: 32] = CH3
    // adc_data[ 63: 48] = CH4
    // adc_data[ 79: 64] = CH5
    // adc_data[ 95: 80] = CH6
    // adc_data[111: 96] = CH7
    // adc_data[127:112] = CH8
    //==========================================================================
    wire [127:0] pack_data_in = {
        adc_data[ 15:  0], // CH1 (MSB first 发送时先发 CH1[15:8], CH1[7:0])
        adc_data[ 31: 16], // CH2
        adc_data[ 47: 32], // CH3
        adc_data[ 63: 48], // CH4
        adc_data[ 79: 64], // CH5
        adc_data[ 95: 80], // CH6
        adc_data[111: 96], // CH7
        adc_data[127:112]  // CH8
    };

    // 硬件状态字注入: 包括错误标志与初始化状态
    wire [7:0] pack_status = {4'd0, err_overrun, err_busy_timeout, err_align, init_done};

    //==========================================================================
    // 1024 组数据缓存与打包传输控制模块
    //==========================================================================
    reg         cap_en;
    wire        pack_cap_done;
    reg         tx_start;
    wire        buf_uart_tx_en;
    wire [7:0]  buf_uart_tx_data;
    wire        pack_tx_done;
    wire        pack_tx_active;
    wire        uart_tx_busy;

    lha6958h_data_pack_buf #(
        .PACK_GROUP_NUM(PACK_GROUP_NUM),
        .RAM_DEPTH     (1024),
        .ADDR_WIDTH    (10)
    ) u_pack_buf (
        .sys_clk      (clk),
        .sys_rst_n    (sys_rst_n_sync),
        .cap_en       (cap_en),
        .data_valid   (adc_valid),
        .data_in      (pack_data_in),
        .pack_cap_done(pack_cap_done),
        .tx_start     (tx_start),
        .status_in    (pack_status),
        .uart_tx_en   (buf_uart_tx_en),
        .uart_tx_data (buf_uart_tx_data),
        .uart_tx_busy (uart_tx_busy),
        .pack_tx_done (pack_tx_done),
        .tx_active    (pack_tx_active)
    );

    //==========================================================================
    // 串口发送模块 (引用自参考工程 uart_send.v)
    //==========================================================================
    wire       uart_send_en;
    wire [7:0] uart_send_din;

    uart_send #(
        .CLK_FREQ(CLK_FREQ_HZ),
        .UART_BPS(UART_BPS)
    ) u_uart_send (
        .sys_clk      (clk),
        .sys_rst_n    (sys_rst_n_sync),
        .uart_en      (uart_send_en),
        .uart_din     (uart_send_din),
        .uart_tx_busy (uart_tx_busy),
        .uart_txd     (uart_txd)
    );

    // 检测 uart_tx_busy 下降沿 (单字节发送彻底完成瞬间)
    reg uart_busy_d;
    always @(posedge clk or negedge sys_rst_n_sync) begin
        if (!sys_rst_n_sync)
            uart_busy_d <= 1'b0;
        else
            uart_busy_d <= uart_tx_busy;
    end
    wire uart_byte_done = uart_busy_d & (~uart_tx_busy);

    //==========================================================================
    // 测试字符/心跳发送状态机 (参考 LHA5278H)
    // 字符串: "Hello! LHA6958H ADC UART OK!\r\n" (共 30 字节)
    // 触发策略:
    // 1. 上电后延时约 10ms 立即发送第 1 次 (确保开机或按复位键即刻有输出)
    // 2. 当没有进行 ADC 大包传输时，每 1 秒心跳周期发送 1 次，方便串口调试助手测试
    //==========================================================================
    localparam BANNER_LEN = 30;
    reg [4:0] banner_idx;
    reg       banner_tx_en;
    reg [7:0] banner_tx_data;
    reg       banner_active;

    function [7:0] get_banner_char;
        input [4:0] idx;
        case (idx)
            5'd0:  get_banner_char = "H";
            5'd1:  get_banner_char = "e";
            5'd2:  get_banner_char = "l";
            5'd3:  get_banner_char = "l";
            5'd4:  get_banner_char = "o";
            5'd5:  get_banner_char = "!";
            5'd6:  get_banner_char = " ";
            5'd7:  get_banner_char = "L";
            5'd8:  get_banner_char = "H";
            5'd9:  get_banner_char = "A";
            5'd10: get_banner_char = "6";
            5'd11: get_banner_char = "9";
            5'd12: get_banner_char = "5";
            5'd13: get_banner_char = "8";
            5'd14: get_banner_char = "H";
            5'd15: get_banner_char = " ";
            5'd16: get_banner_char = "A";
            5'd17: get_banner_char = "D";
            5'd18: get_banner_char = "C";
            5'd19: get_banner_char = " ";
            5'd20: get_banner_char = "U";
            5'd21: get_banner_char = "A";
            5'd22: get_banner_char = "R";
            5'd23: get_banner_char = "T";
            5'd24: get_banner_char = " ";
            5'd25: get_banner_char = "O";
            5'd26: get_banner_char = "K";
            5'd27: get_banner_char = "!";
            5'd28: get_banner_char = 8'h0D; // \r
            5'd29: get_banner_char = 8'h0A; // \n
            default: get_banner_char = 8'h00;
        endcase
    endfunction

    // 开机 10ms (500,000 个 50MHz 时钟) 初始启动脉冲
    reg [19:0] por_timer;
    reg        por_trig;

    always @(posedge clk or negedge sys_rst_n_sync) begin
        if (!sys_rst_n_sync) begin
            por_timer <= 20'd0;
            por_trig  <= 1'b0;
        end else if (por_timer < POR_WAIT_CYCLES) begin
            por_timer <= por_timer + 1'b1;
            if (por_timer == POR_WAIT_CYCLES - 1)
                por_trig <= 1'b1;
            else
                por_trig <= 1'b0;
        end else begin
            por_trig <= 1'b0;
        end
    end

    // 顶层采集与传输状态定义
    localparam [1:0]
        APP_INIT     = 2'd0,  // 等待芯片寄存器初始化完成 (init_done)
        APP_SAMPLING = 2'd1,  // 采集中: 开启 cap_en，满 1024 组进入发送
        APP_TX_START = 2'd2,  // 触发 tx_start
        APP_TX_WAIT  = 2'd3;  // 等待 1024 组 (16392 字节) 发送完毕

    reg [1:0] app_state;

    // 触发 Banner 条件: 上电 10ms 触发 或 系统未进入正式采集循环时每 1 秒心跳脉冲
    wire banner_trig = por_trig | (sec_pulse && (app_state == APP_INIT));

    localparam [1:0]
        ST_B_IDLE = 2'd0,
        ST_B_SEND = 2'd1,
        ST_B_WAIT = 2'd2;

    reg [1:0] b_state;

    always @(posedge clk or negedge sys_rst_n_sync) begin
        if (!sys_rst_n_sync) begin
            b_state        <= ST_B_IDLE;
            banner_idx     <= 5'd0;
            banner_tx_en   <= 1'b0;
            banner_tx_data <= 8'h00;
            banner_active  <= 1'b0;
        end else begin
            banner_tx_en <= 1'b0;

            case (b_state)
                ST_B_IDLE: begin
                    banner_active <= 1'b0;
                    banner_idx    <= 5'd0;
                    if (banner_trig && !pack_tx_active) begin
                        banner_active <= 1'b1;
                        b_state       <= ST_B_SEND;
                    end
                end

                ST_B_SEND: begin
                    if (!uart_tx_busy) begin
                        banner_tx_en   <= 1'b1;
                        banner_tx_data <= get_banner_char(banner_idx);
                        b_state        <= ST_B_WAIT;
                    end
                end

                ST_B_WAIT: begin
                    if (uart_byte_done) begin
                        if (banner_idx < BANNER_LEN - 1) begin
                            banner_idx <= banner_idx + 1'b1;
                            b_state    <= ST_B_SEND;
                        end else begin
                            banner_active <= 1'b0;
                            b_state       <= ST_B_IDLE;
                        end
                    end
                end
            endcase
        end
    end

    //==========================================================================
    // UART 发送多路复用 MUX (测试字符串 vs 1024 组 ADC 采样数据)
    //==========================================================================
    assign uart_send_en  = banner_active ? banner_tx_en   : buf_uart_tx_en;
    assign uart_send_din = banner_active ? banner_tx_data : buf_uart_tx_data;

    //==========================================================================
    // 连续采集与 UART 传输调度状态机
    //==========================================================================
    always @(posedge clk or negedge sys_rst_n_sync) begin
        if (!sys_rst_n_sync) begin
            app_state <= APP_INIT;
            cap_en    <= 1'b0;
            tx_start  <= 1'b0;
        end else begin
            tx_start <= 1'b0;

            case (app_state)
                // 1. 等待芯片上电配置完成
                APP_INIT: begin
                    cap_en <= 1'b0;
                    if (init_done && !banner_active) begin
                        cap_en    <= 1'b1; // 启动采集
                        app_state <= APP_SAMPLING;
                    end
                end

                // 2. 正常采样并写入 RAM，采满 1024 组后切换至发送
                APP_SAMPLING: begin
                    cap_en <= 1'b1;
                    if (pack_cap_done) begin
                        cap_en    <= 1'b0; // 暂停写入，避免冲刷 RAM
                        app_state <= APP_TX_START;
                    end
                end

                // 3. 启动 UART 发送 1024 组整包
                APP_TX_START: begin
                    cap_en <= 1'b0;
                    if (!banner_active && !uart_tx_busy) begin
                        tx_start  <= 1'b1;
                        app_state <= APP_TX_WAIT;
                    end
                end

                // 4. 等待 1024 组数据全部发完
                APP_TX_WAIT: begin
                    cap_en <= 1'b0;
                    if (pack_tx_done) begin
                        cap_en    <= 1'b1; // 发送完毕，重新开启采集
                        app_state <= APP_SAMPLING;
                    end
                end

                default: begin
                    app_state <= APP_INIT;
                end
            endcase
        end
    end

endmodule
