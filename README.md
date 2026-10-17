# LHA6958H 专业级 FPGA 驱动与板级采集系统

Legend Semi LHA6958H（8 通道 16 位 1MSPS 同步采样 SAR ADC）并行接口驱动与完整板级采集系统，
目标芯片 Altera Cyclone IV E（`EP4CE10F17C8`），单时钟域纯 RTL，无 AXI 依赖。
规格依据：`LHA6958H_Datasheet_Rev.B.pdf`（68 页中文）。

---

## 1. 工程文件结构

```
lha6958h-pro/
├── rtl/
│   ├── lha6958h_board_top.v    # 【工程板级顶层】Quartus 实际顶层实体 (TOP_LEVEL_ENTITY)，
│   │                           # 集成 ADC 驱动、1024 组 RAM 打包、1.5M UART 发送、心跳与 LED
│   ├── lha6958h_top.v          # 【ADC 驱动子系统顶层】集成 pro 读数 + reg_init 初始化，可直接复用到其他工程
│   ├── lha6958h_pro.v          # ADC 读数核心：CONVST 定时器、BUSY 下降沿触发读突发、
│   │                           # FRSTDATA 通道对齐校验、BUSY 看门狗、采样 overrun 检测、
│   │                           # tPOWER-UP 延迟 + 上电复位时序（por_done 握手）
│   ├── lha6958h_reg_init.v     # 上电寄存器初始化：并行 WR 写 CONFIG/量程/过采样，
│   │                           # 最后自动追加 (0x00,0x00) 切回 ADC 读数模式
│   ├── lha6958h_reg_read.v     # 独立寄存器读回控制器：双帧并行读回协议（Rev.B §7.4）
│   ├── lha6958h_data_pack_buf.v# 1024 组 (16KB) 片上 M9K Block RAM 缓存与通信协议大包组装
│   ├── uart_send.v             # 串口发送底层驱动（支持 1.5Mbps / 115200 等波特率）
│   └── uart_tx.v               # 备用高精度串口发送模块
├── sim/
│   ├── tb_lha6958h_pro.v       # 行为模型 + 5 组核心测试（复位、初始化、正常帧、FRSTDATA、超时）
│   ├── tb_lha6958h_overrun.v   # 采样率超限 → err_overrun 异常检测
│   ├── tb_lha6958h_reg_read.v  # 寄存器双帧读回时序仿真
│   ├── run_gui.do              # ModelSim 自动化仿真脚本
│   ├── run_modelsim_gui.bat    # ModelSim 图形化一键启动脚本
│   └── render_waveform.py      # 波形解析与渲染脚本 (生成 wave_reg_read.png)
├── gui/                        # PySide6 8 通道实时示波器上位机
│   ├── main.py                 # 上位机主入口
│   ├── main_window.py          # 主界面与绘图逻辑
│   ├── protocol.py             # 16,392 字节极简大包协议解析器
│   ├── serial_worker.py        # 独立串口异步接收与缓存线程
│   └── virtual_source.py       # 离线模拟信号生成器
├── output_files/
│   └── lha6958h.sof            # Quartus 编译生成的 FPGA 固件
├── download_sof.bat            # USB-Blaster 固件一键烧录脚本
├── run_gui.bat                 # 上位机一键启动脚本
├── uart_terminal.py            # 纯终端动态 ANSI 看板与实时监控器
├── H1_pinmap.md                # H1 连接器信号→H1 引脚→FPGA 引脚对照表 (50-pin)
├── lha6958h.qpf                # Quartus II 工程文件
├── lha6958h.qsf                # 【工程核心约束】28 个物理管脚、3.3-V LVCMOS 电平及综合设置
└── lha6958h_pin_assignments.qsf# 23 个 ADC 引脚参考约束片段
```

---

## 2. 顶层架构与工程实体对应关系

工程中存在两级顶层抽象，以 **`lha6958h.qsf` 工程设置为准**：

| 实体名称 | 文件 | 角色说明 |
|:---|:---|:---|
| **`lha6958h_board_top`** | `rtl/lha6958h_board_top.v` | **Quartus 实际物理板级顶层**（`TOP_LEVEL_ENTITY`）。直接连接开发板时钟、复位、LED、UART 串口和 H1 接口 ADC 信号。 |
| **`lha6958h_top`** | `rtl/lha6958h_top.v` | **ADC 驱动核心子系统**。仅负责 LHA6958H 驱动及上电寄存器初始化，输出 128 位并行数据 `adc_data` 和 `adc_valid`，供嵌入到其他系统使用。 |

### 板级顶层引脚分配（共 28 Pins，见 `lha6958h.qsf`）

- **系统时钟/复位**:
  - `clk` (PIN_E1, 50MHz)
  - `rst_n` (PIN_M1, 低有效复位按键)
- **状态指示与通信**:
  - `led_init_done` (PIN_A4, LED0, 低电平点亮：寄存器配置成功)
  - `led_heartbeat` (PIN_B5, LED1, 1Hz 闪烁：系统正常采集中；常亮：硬件报警)
  - `uart_txd` (PIN_A13, UART 串口发送端口，默认 1.5Mbps)
- **LHA6958H 控制引脚 (H1 接口，Logic Pro 8 CH0~CH6 监测信号)**:
  - `RESET` (PIN_P15, H1_36 / CH0): ADC 上电复位，高有效
  - `WR` (PIN_R14, H1_35 / CH1): 软件模式寄存器写选通，上升沿锁存
  - `CS` (PIN_T9, H1_14 / CH2): 片选，低有效
  - `CONVST` (PIN_T12, H1_37 / CH3): 转换启动脉冲，**上升沿**有效
  - `BUSY` (PIN_N13, H1_34 / CH4): 忙状态指示输入（内部两级同步）
  - `RD` (PIN_N6, H1_20 / CH5): 读选通，低有效
  - `FRSTDATA` (PIN_M11, H1_32 / CH6): 首通道指示输入（通道对齐校验）
- **LHA6958H 并行总线**:
  - `DB[15:0]`: 双向总线（H1 接口打散引脚，软件模式写寄存器输出配置帧，采集中输入 16 位补码数据）

---

## 3. 上电与初始化时序（por_done / init_done 强制握手）

```
lha6958h_pro: tPOWER-UP(10ms) → RESET 脉冲(≥3µs) → 恢复等待(600µs) → por_done=1
                                                                     ↓
lha6958h_reg_init: 写 0x02/0x03~0x06/0x08 → 写 0x00 回 ADC 模式 → init_done=1
                                                                     ↓
lha6958h_pro: CONVST 定时器启动，连续突发出数
```

> **强制设计原则**：**寄存器写操作必须在 RESET 脉冲及 tDEVICE_SETUP 之后执行**。芯片在 RESET=1 期间会忽略寄存器写操作。工程采用硬件单向信号握手，确保时序 100% 可靠。

---

## 4. 驱动子系统例化方式 (`lha6958h_top.v`)

若需要在自己的 SOC/FPGA 工程中嵌入 ADC 采集核心，可直接例化 `lha6958h_top`：

```verilog
lha6958h_top #(
  .CLK_FREQ_HZ        (50_000_000), // 系统时钟频率
  .SAMPLE_RATE_HZ     (500_000),    // 500kSPS 采样率 (无溢出，信噪比最佳)
  .BUSY_TIMEOUT_CLKS  (500),        // 必须大于所配过采样档的最坏 tCONV
  .REG_DATA1          (8'h22),      // CH2/CH1: ±10V 低带宽量程 (0010_0010)
  .REG_DATA2          (8'h22),      // CH4/CH3: ±10V 低带宽量程
  .REG_DATA3          (8'h22),      // CH6/CH5: ±10V 低带宽量程
  .REG_DATA4          (8'h22),      // CH8/CH7: ±10V 低带宽量程
  .REG_DATA5          (8'h00)       // 0x08 寄存器过采样: 0x00=无过采样 (1MSPS)
) u_adc_core (
  .clk                (clk),
  .rst_n              (rst_n),
  .init_restart       (1'b0),
  .err_clear          (err_clear),

  // 连接到 ADC 芯片物理引脚
  .convst             (CONVST),
  .cs_n               (CS),
  .rd_n               (RD),
  .wr_n               (WR),
  .adc_reset          (RESET),
  .ad_os              (),           // OS 在板上已焊死为 111，可悬空
  .db                 (DB),
  .busy               (BUSY),
  .first_data         (FRSTDATA),

  // 用户侧数据输出接口
  .adc_data           (adc_data),   // 128 位: {CH8[15:0], ..., CH1[15:0]}
  .adc_valid          (adc_valid),  // 单周期数据有效脉冲
  .init_done          (init_done),  // 初始化完成标志
  .por_done           (por_done),
  .err_align          (err_align),
  .err_busy_timeout   (err_busy_timeout),
  .err_overrun        (err_overrun)
);
```

---

## 5. 仿真与验证

仿真验证均已通过，支持 Icarus Verilog 与 ModelSim：

```bash
# 1. 核心读数与寄存器配置时序仿真 (Icarus Verilog)
cd sim
iverilog -o tb_lha6958h_pro tb_lha6958h_pro.v ../rtl/lha6958h_top.v ../rtl/lha6958h_pro.v ../rtl/lha6958h_reg_init.v && ./tb_lha6958h_pro

# 2. 采样率超限保护仿真
iverilog -o tb_lha6958h_overrun tb_lha6958h_overrun.v ../rtl/lha6958h_pro.v && ./tb_lha6958h_overrun

# 3. 寄存器双帧读回时序仿真
iverilog -o tb_lha6958h_reg_read tb_lha6958h_reg_read.v ../rtl/lha6958h_reg_read.v && ./tb_lha6958h_reg_read

# 4. ModelSim 图形化仿真 (Windows)
双击 sim/run_modelsim_gui.bat 一键载入并查看波形
```

---

## 6. UART 串口传输与数据打包协议 (参考 LHA5278H 架构)

### 1. 硬件引脚分配
- **FPGA 芯片**: Cyclone IV E (`EP4CE10F17C8`)
- **UART 发送端口**: `PIN_A13`（`uart_txd`，3.3-V LVCMOS）
- **默认波特率**: `1,500,000 bps`（1.5Mbps，8-N-1，误差仅 1.01%），支持参数化修改为 115200 等。

### 2. 通信协议大包格式 (整包 16,392 字节)
每包包含 1024 组完整的 8 通道 16 位 ADC 采样数据：
- **帧头** (4 字节): `0x5A, 0xA5, 0x55, 0xAA`
- **包序号** (1 字节): `0x00 ~ 0xFF` 循环自增
- **状态字** (1 字节): `status`（D0: init_done, D1: err_align, D2: err_busy_timeout, D3: err_overrun）
- **数据载荷** (16,384 字节): 1024 组 × 8 通道 × 2 字节（16-bit 补码，高字节在先 MSB first，CH1 ~ CH8 排列）
- **帧尾** (2 字节): `0x0D, 0x0A` (`\r\n`)

### 3. 上电心跳与运行
- **心跳与问候**：上电完成或空闲时，每 1 秒心跳自动发送字符串：`"Hello! LHA6958H ADC UART OK!\r\n"`。
- **固件一键下载**：开发板连接 USB-Blaster 后，双击根目录下的 `download_sof.bat` 即可一键烧录 [output_files/lha6958h.sof](file:///d:/FPGA/SoftWare-Project/lha6958h-pro/output_files/lha6958h.sof)。
- **实时上位机**：双击 `run_gui.bat` 或运行 `python gui/main.py` 启动示波器图形界面。
- **终端监控**：运行 `python uart_terminal.py` 可在命令行直接查看 ANSI 动态波形与各通道数值。
