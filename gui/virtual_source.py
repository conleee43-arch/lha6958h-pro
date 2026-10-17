# -*- coding: utf-8 -*-
"""
虚拟数据发生源 (Virtual Data Generator)
用于在脱机或硬件未连接时，高保真模拟 FPGA 按照极简协议持续发出 8 通道 16-bit 1024 组数据包
"""

import time
import numpy as np
from PySide6.QtCore import QThread, Signal
from .protocol import (
    PACK_GROUPS,
    NUM_CHANNELS,
    DEFAULT_VREF,
)


class VirtualDataWorker(QThread):
    """
    虚拟数据发生器后台线程
    信号:
        sig_frame_ready: 发射已解析的 (pkt_id, raw_codes, voltages, status)
        sig_status_info: 发射状态信息 (帧率, 字节率等)
        sig_error: 异常错误信息
    """

    sig_frame_ready = Signal(int, np.ndarray, np.ndarray, int)
    sig_status_info = Signal(dict)
    sig_error = Signal(str)

    def __init__(self, fps: float = 6.0, parent=None):
        """
        fps: 模拟大包发送频率 (默认 6.0 帧/秒，匹配 1.5Mbps 下约 164ms 一包的节拍)
        """
        super().__init__(parent)
        self._running = False
        self._fps = fps
        self._pkt_id = 0
        self._phase = 0.0
        self.vref = DEFAULT_VREF

    def run(self):
        self._running = True
        interval = 1.0 / self._fps
        last_stat_time = time.time()
        frames_in_sec = 0
        bytes_in_sec = 0

        while self._running:
            start_t = time.time()

            # 生成 1024 组 x 8 通道的模拟波形 (以伏特为单位, ±10V 量程)
            # 在 500 kSPS 采样率下，每包 1024 组对应的时间跨度为 1024 / 500,000 = 2.048 ms
            t_span = PACK_GROUPS / 500_000.0  # 2.048 ms
            t = np.linspace(self._phase, self._phase + t_span, PACK_GROUPS, endpoint=False)
            self._phase += t_span

            # 8 路不同形态的特征信号 (匹配 500kSPS 下适合示波与表格观测的音频与中频信号)
            volts = np.zeros((PACK_GROUPS, NUM_CHANNELS), dtype=np.float64)
            volts[:, 0] = 8.0 * np.sin(2 * np.pi * 500 * t)            # CH1: 500Hz 正弦波 (±8V, 单包 2.048ms 呈现 ~1 个完整周期)
            volts[:, 1] = 6.0 * np.sin(2 * np.pi * 1000 * t)           # CH2: 1kHz 正弦波 (±6V, ~2 个周期)
            volts[:, 2] = 5.0 * (2 * np.abs(2 * (800 * t - np.floor(800 * t + 0.5))) - 1)  # CH3: 800Hz 三角波
            volts[:, 3] = 3.3                                          # CH4: 直流 +3.3V
            volts[:, 4] = -3.3                                         # CH5: 直流 -3.3V
            volts[:, 5] = 4.0 * np.sign(np.sin(2 * np.pi * 1000 * t))   # CH6: 1kHz 方波 (±4V)
            volts[:, 6] = 8.0 * ((t * 500) % 1.0) - 4.0                # CH7: 500Hz 锯齿波
            volts[:, 7] = np.random.uniform(-0.5, 0.5, PACK_GROUPS)    # CH8: 随机白噪声

            # 裁剪防止超量程
            volts = np.clip(volts, -self.vref, self.vref * 32767.0 / 32768.0)

            # 换算回 16-bit 补码整数
            raw_codes = np.round((volts / self.vref) * 32768.0).astype(np.int32)
            raw_codes = np.clip(raw_codes, -32768, 32767)

            # 模拟硬件状态: init_done = 1
            status = 0x01

            frames_in_sec += 1
            bytes_in_sec += (PACK_GROUPS * NUM_CHANNELS * 2 + 8)

            self.sig_frame_ready.emit(self._pkt_id, raw_codes, volts, status)
            self._pkt_id = (self._pkt_id + 1) & 0xFF

            now = time.time()
            if now - last_stat_time >= 1.0:
                dt = now - last_stat_time
                fps_val = frames_in_sec / dt
                kb_val = (bytes_in_sec / dt) / 1024.0
                self.sig_status_info.emit({
                    "fps": fps_val,
                    "kbps": kb_val,
                    "total_frames": self._pkt_id,
                    "lost_frames": 0,
                    "is_virtual": True,
                })
                frames_in_sec = 0
                bytes_in_sec = 0
                last_stat_time = now

            # 节拍休眠
            elapsed = time.time() - start_t
            remain = interval - elapsed
            if remain > 0:
                time.sleep(remain)

    def stop(self):
        self._running = False
        self.wait(2000)
