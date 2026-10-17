# -*- coding: utf-8 -*-
"""
LHA6958H 8 通道 16-bit 1MSPS ADC 终端实时数据监视器 (CLI Terminal Monitor)
直接在命令行终端中实时高亮显示 8 通道 ADC 转换数据、电压、原始十六进制码值与幅度条。

用法:
    python uart_terminal.py                     # 自动探测并连接可用串口 (如 COM12)
    python uart_terminal.py --port COM12        # 指定串口号
    python uart_terminal.py --baud 1500000      # 指定波特率 (默认 1500000, 可设 115200)
    python uart_terminal.py --stream            # 滚动输出模式 (每包打印一行)
    python uart_terminal.py --virtual           # 离线模拟数据演示 (无硬件时测试)
"""

import os
import sys
import time
import argparse
import numpy as np

# Windows 控制台编码安全兼容与 ANSI 转义序列支持
if sys.platform.startswith("win"):
    try:
        import ctypes
        kernel32 = ctypes.windll.kernel32
        kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 7)
    except Exception:
        pass
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

# 确保能加载 gui 模块中的协议与数据源
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from gui.protocol import (
    NUM_CHANNELS,
    DEFAULT_VREF,
    FrameStreamParser,
    parse_payload_to_matrix,
)


def get_available_ports():
    """检测当前系统所有串口"""
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        return [(p.device, p.description) for p in sorted(ports, key=lambda x: x.device)]
    except ImportError:
        return []


def render_bar(volts, vref=10.0, width=20):
    """生成带正负极性的 ASCII 电平指示条"""
    half = width // 2
    norm = max(-1.0, min(1.0, volts / vref))
    if norm >= 0:
        fill = int(norm * half)
        bar = " " * half + "|" + "=" * fill + " " * (half - fill)
    else:
        fill = int(-norm * half)
        bar = " " * (half - fill) + "=" * fill + "|" + " " * half
    pct = (volts / vref) * 100.0
    return f"[{bar}] {pct:+6.1f}%"


def run_terminal_monitor(port=None, baudrate=1500000, stream_mode=False, virtual_mode=False, vref=DEFAULT_VREF):
    print("=" * 78)
    print("   LHA6958H 8-Channel 16-bit 1MSPS ADC Real-Time Terminal Monitor")
    print("   [配置] 采样率: 500 kSPS (1024 组/包, 单包捕获窗口: 2.048 ms)")
    print("=" * 78)

    # 1. 串口配置或模拟源选择
    if virtual_mode:
        print("[模式] 正在运行离线模拟数据源 (Virtual Generator)")
    else:
        ports = get_available_ports()
        if not ports:
            print("[错误] 未检测到任何可用串口！请确认 USB 转串口线已插入电脑。")
            print("       (提示: 您可使用 `python uart_terminal.py --virtual` 进行脱机模拟演示)")
            return

        if not port:
            if len(ports) == 1:
                port = ports[0][0]
                print(f"[自动选择] 检测到唯一串口: {port} ({ports[0][1]})")
            else:
                print("检测到多个串口端口:")
                for idx, (p_dev, p_desc) in enumerate(ports):
                    print(f"  [{idx + 1}] {p_dev}: {p_desc}")
                # 优先寻找包含 CH340 / USB / Serial 的端口
                suggested_port = None
                for p_dev, p_desc in ports:
                    if any(k in p_desc.upper() for k in ["CH340", "USB", "CP210", "FTDI"]):
                        suggested_port = p_dev
                        break
                if suggested_port:
                    port = suggested_port
                    print(f"[推荐选择] 优先匹配 FPGA 常用端口: {port}")
                else:
                    port = ports[0][0]
                    print(f"[默认选择] 选择首个端口: {port}")

        print(f"[连接] 正在打开串口: {port} @ {baudrate} bps ...")

    # 2. 初始化工作流
    parser = FrameStreamParser()
    running = True

    ser = None
    virtual_worker = None

    virt_phase = 0.0
    virt_pkt_id = 0
    if not virtual_mode:
        import serial
        try:
            ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,
            )
            if hasattr(ser, "set_buffer_size"):
                try:
                    ser.set_buffer_size(rx_size=1048576, tx_size=65536)
                except Exception:
                    pass
            ser.reset_input_buffer()
            print(f"[成功] 串口 {port} 已打开，正在持续接收 FPGA 转换数据...")
            print("       按 Ctrl+C 可随时退出监视。\n", flush=True)
        except Exception as e:
            print(f"[错误] 打开串口失败: {e}", flush=True)
            return

    # 清屏准备
    if not stream_mode:
        os.system("cls" if os.name == "nt" else "clear")

    total_pkts = 0
    last_stat_time = time.time()
    bytes_in_sec = 0
    fps_val = 0.0
    kb_val = 0.0

    try:
        while running:
            valid_frames = []

            if virtual_mode:
                time.sleep(0.15)  # 约 6.6 FPS
                t = np.linspace(virt_phase, virt_phase + 0.02, 1024, endpoint=False)
                virt_phase += 0.02

                v = np.zeros((1024, NUM_CHANNELS), dtype=np.float64)
                v[:, 0] = 7.5 * np.sin(2 * np.pi * 10 * t)
                v[:, 1] = 5.0 * np.sin(2 * np.pi * 25 * t)
                v[:, 2] = 4.5 * (2 * np.abs(2 * (15 * t - np.floor(15 * t + 0.5))) - 1)
                v[:, 3] = 3.3
                v[:, 4] = -3.3
                v[:, 5] = 4.0 * np.sign(np.sin(2 * np.pi * 50 * t))
                v[:, 6] = 8.0 * (t % 0.05) / 0.05 - 4.0
                v[:, 7] = np.random.uniform(-0.5, 0.5, 1024)

                c = np.round((v / vref) * 32768.0).astype(np.int32)
                c = np.clip(c, -32768, 32767)

                valid_frames.append({
                    "pkt_id": virt_pkt_id,
                    "raw_codes": c,
                    "volts": v,
                    "status": 0x01,
                })
                virt_pkt_id = (virt_pkt_id + 1) & 0xFF
                bytes_in_sec += 16392

            else:
                waiting = ser.in_waiting
                to_read = max(waiting, 1024)
                chunk = ser.read(to_read)
                if not chunk:
                    time.sleep(0.005)
                    continue

                bytes_in_sec += len(chunk)
                parsed = parser.push_data(chunk)
                for item in parsed:
                    p_id = item["pkt_id"]
                    stat = item["status"]
                    payload = item["payload"]
                    raw_codes, volts = parse_payload_to_matrix(payload, vref)
                    valid_frames.append({
                        "pkt_id": p_id,
                        "raw_codes": raw_codes,
                        "volts": volts,
                        "status": stat,
                    })

            now = time.time()
            if now - last_stat_time >= 1.0:
                dt = now - last_stat_time
                kb_val = (bytes_in_sec / dt) / 1024.0
                fps_val = total_pkts / dt
                bytes_in_sec = 0
                last_stat_time = now

            for f in valid_frames:
                total_pkts += 1
                pkt_id = f["pkt_id"]
                raw_codes = f["raw_codes"]
                volts = f["volts"]
                status = f["status"]

                last_codes = raw_codes[-1, :]
                last_volts = volts[-1, :]
                min_v = np.min(volts, axis=0)
                max_v = np.max(volts, axis=0)

                init_ok = bool(status & 0x01)
                err_align = bool(status & 0x02)
                err_busy = bool(status & 0x04)
                err_overrun = bool(status & 0x08)

                stat_text = "正常 (Init OK)" if init_ok else "初始化中"
                if err_align:
                    stat_text = "[警告:对齐错误]"
                elif err_busy:
                    stat_text = "[警告:BUSY超时]"
                elif err_overrun:
                    stat_text = "[警告:采样溢出]"

                if stream_mode:
                    # 滚动单行模式: [Pkt #XX] CH1: +X.XXXV | CH2: ...
                    summary = " | ".join([f"CH{i+1}:{last_volts[i]:+6.3f}V" for i in range(8)])
                    print(f"[Pkt #{pkt_id:03d} | {stat_text}] {summary}", flush=True)
                else:
                    # 看板刷新模式: 回到首行重绘完整表格
                    buf = []
                    buf.append("\033[H")  # ANSI 光标移至左上角
                    buf.append("=" * 82)
                    mode_label = "[虚拟演示模式]" if virtual_mode else f"[串口: {port} @ {baudrate} bps]"
                    buf.append(f" LHA6958H 8-Channel 16-bit 1MSPS ADC 终端监视器  {mode_label}")
                    buf.append(f" 包序号: #{pkt_id:03d} | 吞吐: {kb_val:5.1f} kB/s | 状态: {stat_text} | 累计: {total_pkts} 包")
                    buf.append("=" * 82)
                    buf.append(" 通道 |  实时电压 (V) | 16-bit 码值 | 有符号十进制 | 极值范围 (V) |      电平幅度指示")
                    buf.append("------+---------------+-------------+--------------+--------------+-------------------------")
                    for ch in range(NUM_CHANNELS):
                        c_val = int(last_codes[ch])
                        v_val = float(last_volts[ch])
                        hex_str = f"0x{(c_val & 0xFFFF):04X}"
                        dec_str = f"{c_val:+6d}"
                        bar_str = render_bar(v_val, vref=vref, width=16)
                        rng_str = f"[{min_v[ch]:+5.2f}, {max_v[ch]:+5.2f}]"
                        buf.append(f"  CH{ch+1} |   {v_val:+8.4f} V  |   {hex_str}    |   {dec_str}   | {rng_str} | {bar_str}")
                    buf.append("=" * 82)
                    buf.append(" 提示: 终端正持续实时刷新 ADC 转换数据，按 Ctrl+C 可退出监视。")
                    sys.stdout.write("\n".join(buf) + "\n")
                    sys.stdout.flush()

    except KeyboardInterrupt:
        print("\n\n[用户中止] 已退出终端监视器。")
    finally:
        if ser and ser.is_open:
            ser.close()
        if virtual_worker:
            virtual_worker.stop()


def main():
    parser = argparse.ArgumentParser(description="LHA6958H 8通道 16-bit ADC 终端实时数据监视器")
    parser.add_argument("--port", "-p", type=str, default=None, help="串口端口号 (如 COM12, 默认自动探测)")
    parser.add_argument("--baud", "-b", type=int, default=1500000, help="波特率 (默认 1500000, 可设 115200)")
    parser.add_argument("--stream", "-s", action="store_true", help="单行滚动流模式 (默认看板模式)")
    parser.add_argument("--virtual", "-v", action="store_true", help="启用离线模拟演示数据流")
    parser.add_argument("--vref", type=float, default=DEFAULT_VREF, help="满量程参考电压 (默认 10.0V)")

    args = parser.parse_args()
    run_terminal_monitor(
        port=args.port,
        baudrate=args.baud,
        stream_mode=args.stream,
        virtual_mode=args.virtual,
        vref=args.vref,
    )


if __name__ == "__main__":
    main()
