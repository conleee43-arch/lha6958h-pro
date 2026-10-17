# LHA6958H 专业级 FPGA 驱动

Legend Semi LHA6958H（8 通道 16 位 1MSPS 同步采样 SAR ADC）并行接口驱动，
目标 Altera Cyclone IV（EP4CE10）单时钟域纯 RTL，无 AXI 依赖。
规格依据：`LHA6958H_Datasheet_Rev.B.pdf`（68 页中文），时序逐项提取见
`workspace/datasheets/sar-adc/lha6958h_timing_extract.md`。

## 文件

```
lha6958h-pro/
├── rtl/
│   ├── lha6958h_top.v        # 顶层集成：pro + reg_init，上电时序与总线二选一
│   ├── lha6958h_pro.v        # ADC 读数驱动：CONVST 定时器、BUSY 下降沿触发读突发、
│   │                         # FRSTDATA 通道对齐校验、BUSY 看门狗、采样 overrun 检测、
│   │                         # tPOWER-UP 延迟 + 上电复位时序（por_done 握手）
│   └── lha6958h_reg_init.v   # 上电寄存器初始化：并行 WR 写 CONFIG/量程/过采样，
│                             # 最后自动追加 (0x00,0x00) 切回 ADC 读数模式
├── sim/
│   ├── tb_lha6958h_pro.v     # 行为模型 + 5 组测试：复位先于写时序 / 寄存器初始化
│   │                         # （非常规值验证写入生效）/ 正常帧 / FRSTDATA 故障 /
│   │                         # BUSY 卡死
│   └── tb_lha6958h_overrun.v # 采样率超限 → err_overrun 检测
├── H1_pinmap.md              # H1 连接器信号→H1 引脚→FPGA 引脚对照表
└── lha6958h_pin_assignments.qsf  # Quartus 引脚约束（已核对）
```

仿真（已验证全过）：

```
cd sim
iverilog -o tb_lha6958h_pro tb_lha6958h_pro.v ../rtl/lha6958h_top.v ../rtl/lha6958h_pro.v ../rtl/lha6958h_reg_init.v && ./tb_lha6958h_pro
iverilog -o tb_lha6958h_overrun tb_lha6958h_overrun.v ../rtl/lha6958h_pro.v && ./tb_lha6958h_overrun
```

## 上电时序（por_done / init_done 握手，强制顺序）

```
pro: tPOWER-UP(10ms) → RESET 脉冲(≥3µs) → por_done=1
                                            ↓
reg_init: 写 0x02/0x03~0x06/0x08 → 写 0x00 回 ADC 模式 → init_done=1
                                            ↓
pro: CONVST 定时器启动，开始出数
```

这个顺序是强制的：**寄存器写必须在 RESET 脉冲之后**——芯片在 RESET=1
期间会忽略寄存器写。如果把 pro 按住复位等 init 完成（如早期版本），
写操作会全部丢失（且之后 pro 自己的复位还会清掉寄存器）。不要改这个顺序。

## 板级 strapping（LQFP-64 引脚号见手册 §8 表7）

| 引脚 | 接法 | 说明 |
|---|---|---|
| PAR/SER SEL（6） | 0（ADC 板上焊死） | 并行接口（本驱动只支持并行）；**不在 H1 上** |
| OS2~OS0（3~5） | 111（ADC 板上焊死） | 软件模式：量程/过采样走寄存器，由 reg_init 上电配置；**不在 H1 上**，驱动的 `ad_os` 输出可悬空 |
| STBY（7） | 1 | 软件模式下被忽略，接高即可 |
| RANGE（8） | 高或低 | 软件模式下被忽略，但必须端接 |
| REF SELECT（34） | 按基准方案 | 高=内部 2.5V；低=外部基准接 REFIN/REFOUT（42） |
| WR（10） | 顶层 `wr_n` | 初始化阶段由 reg_init 驱动，之后保持高 |
| RESET（11） | 顶层 `adc_reset` | 驱动上电给完全复位（≥3µs，默认 1.3ms） |
| CONVST（9） | 顶层 `convst` | **上升沿**启动转换（注意：与经典 AD7606 的下降沿相反） |
| AVCC | 5V | 用户已确认 |
| VDRIVE（23） | 3.3V | 用户已确认（>2.7V 档：tD_RD_DB 取 27ns） |

注意：LHA6958H 只有**一个** CONVST 引脚，没有 CONVST_A/B。
H1 引脚对照见 `H1_pinmap.md`，Quartus 约束见 `lha6958h_pin_assignments.qsf`。

## 顶层例化（直接用）

```verilog
lha6958h_top #(
  .SAMPLE_RATE_HZ    (500_000),   // 500kSPS 采样率 (无溢出，信噪比最佳)
  .BUSY_TIMEOUT_CLKS (500),       // 必须大于所配过采样档的最坏 tCONV
  .REG_DATA1         (8'h22),     // 例：0x03 = ch2/ch1 均为 ±10V 低带宽
  .REG_DATA2         (8'h22),
  .REG_DATA3         (8'h22),
  .REG_DATA4         (8'h22)
  // REG_DATA5(0x08) 过采样：0x00=无，0x01=2x，0x02=4x … 0x08=256x
) adc_top (
  .clk(clk), .rst_n(rst_n),
  .init_restart(1'b0), .err_clear(err_clear),
  .convst(CONVST), .cs_n(CS_N), .rd_n(RD_N), .wr_n(WR_N),
  .adc_reset(RESET), .ad_os(),            // OS 在 ADC 板上已焊死可悬空
  .db(DB), .busy(BUSY), .first_data(FRSTDATA),
  .adc_data(adc_data), .adc_valid(adc_valid),
  .init_done(init_done), .por_done(),
  .err_align(err_align), .err_busy_timeout(err_busy_timeout),
  .err_overrun(err_overrun)
);
// init_done=1 之后 adc_valid 才开始出数；此前的 adc_data 无效。
```

## 参数速查

- `SAMPLE_RATE_HZ`：上限 1MSPS（tCYCLE=1µs，无过采样时）。过采样下两次 CONVST
  间隔必须大于该档 tCONV（手册表2：x2=2.3µs … x256=324µs 典型值）。
- `BUSY_TIMEOUT_CLKS`：必须大于所配过采样档的最坏 tCONV，否则每帧误报。
  默认 500（@50MHz=10µs）覆盖到 8x 档；16x 及以上请按表放大。
- `RESET_HOLD_CLKS`：默认 500（@50MHz=10µs），完全复位高脉冲宽度（手册 §9.1 规定 ≥3µs）。
- `RESET_RECOVERY_CLKS`：默认 30000（@50MHz=600µs），RESET 下降沿后 tDEVICE_SETUP 设备准备时间，必须等满 600µs 才能发起首个写操作。
- `POWERUP_DELAY_CLKS`：默认 500000（@50MHz=10ms），tPOWER-UP：电源稳定到
  RESET 置位的最短时间。
- 量程/过采样改法：顶层的 `REG_DATA0~5`（默认 CONFIG=0x08、全通道 ±12.5V
  低带宽、无过采样——即复位默认值）。量程编码见 reg_init 模块头注释——
  **LHA 的 0010=±10V，0100~0111=高带宽模式**，与 AD7606C-16 不兼容，
  不要直接搬 ADI 的配置代码。

## 从 ad7606_pro.v 移植时的三个坑（已在本驱动中处理）

1. **CONVST 边沿反了**：LHA6958H 是上升沿启动转换（手册 §10.2），AD7606 是
   下降沿。本驱动脉冲 CONVST 为高。
2. **只有一个 CONVST**：LHA6958H 引脚 9 是唯一的 CONVST，无 A/B 之分。
3. **量程寄存器编码不同**：地址都是 0x03~0x06、复位值都是 0x33，但位定义
   不兼容（见上）。并行接口只有 16 位模式，无 8 位模式。

## 上板前待确认（需用户给）

- [x] H1 引脚表 → 已整理 `H1_pinmap.md` + `lha6958h_pin_assignments.qsf`（23/23 核对一致）
- [x] VDRIVE=3.3V（>2.7V 档，时序按 27ns 取）——用户已确认；AVCC=5V
- [x] H1_48 脚：原理图打 X，用户引脚表写 3.3V——用户已确认不需要理会（板上 LDO 由 5V 转出 3.3V，非信号脚）
- [ ] ADC 板上 OS2~OS0 是否焊死为 111（软件模式）、PAR/SER SEL 是否为 0（并行）——H1 上没有这两个信号，请确认 ADC 板
- [ ] REF SELECT 的接法（内部 2.5V 基准还是外部基准）

## UART 串口传输与数据打包协议 (参考 LHA5278H 架构)

### 1. 硬件引脚分配
- **FPGA 芯片**: Cyclone IV E (EP4CE10F17C8)
- **UART 发送端口**: `PIN_A13`（`uart_txd`，3.3-V LVCMOS）
- **默认波特率**: `1,500,000 bps`（1.5Mbps，8-N-1，误差仅 1.01%），支持参数化修改为 115200 等。

### 2. 通信协议大包格式 (整包 16,392 字节)
每包包含 1024 组完整的 8 通道 16 位 ADC 采样数据：
- **帧头** (4 字节): `0x5A, 0xA5, 0x55, 0xAA`
- **包序号** (1 字节): `0x00 ~ 0xFF` 循环自增
- **状态字** (1 字节): `status`（D0: init_done, D1: err_align, D2: err_busy_timeout, D3: err_overrun）
- **数据载荷** (16,384 字节): 1024 组 × 8 通道 × 2 字节（16-bit 补码，高字节在先 MSB first，CH1 ~ CH8 排列）
- **帧尾** (2 字节): `0x0D, 0x0A` (`\r\n`)

### 3. 上电与心跳机制
- 上电 10ms 或系统处于空闲时，每 1 秒心跳周期自动发送问候测试字符串：`"Hello! LHA6958H ADC UART OK!\r\n"`。
- 进入连续采集循环后，串口自动切换为极简大包高速流传输。

### 4. 纯终端数据实时监视 (无需弹窗，直接在 Terminal 刷新)
直接在终端中运行独立终端监视器：
```bash
# 自动检测并连接串口 (如 COM12)，以 ANSI 动态看板实时刷新 8 通道数据与幅度条
python uart_terminal.py

# 滚动数据流模式 (每包打印一行)
python uart_terminal.py --stream

# 指定串口与波特率
python uart_terminal.py --port COM12 --baud 1500000

# 离线模拟数据演示 (无硬件时验证)
python uart_terminal.py --virtual
```

### 5. 图形界面上位机运行
项目内置 PySide6 实时图形上位机：
```bash
python gui/main.py
```
或直接双击根目录下的 `run_gui.bat` 启动。在图形界面运行期间，终端也会同步实时打印接收到的各通道最新数据摘要。
