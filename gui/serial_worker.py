# -*- coding: utf-8 -*-
"""
真实串口异步读取与协议帧拼包工作线程 (SerialWorker)
"""

import time
import serial
import serial.tools.list_ports
import numpy as np
from PySide6.QtCore import QThread, Signal
from .protocol import (
    FrameStreamParser,
    parse_payload_to_matrix,
    DEFAULT_VREF,
)


class SerialWorker(QThread):
    """
    串口后台数据采集线程
    信号:
        sig_frame_ready: 发射已成功解析的整包 (pkt_id, raw_codes, voltages, status)
        sig_status_info: 发射传输吞吐指标字典
        sig_connected:   连接成功指示 (port, baudrate)
        sig_disconnected:已断开指示
        sig_error:       错误报告
    """

    sig_frame_ready = Signal(int, np.ndarray, np.ndarray, int)
    sig_status_info = Signal(dict)
    sig_connected = Signal(str, int)
    sig_disconnected = Signal()
    sig_error = Signal(str)

    def __init__(self, port: str = "COM1", baudrate: int = 1500000, parent=None):
        super().__init__(parent)
        self.port = port
        self.baudrate = baudrate
        self.vref = DEFAULT_VREF
        self._running = False
        self._ser: serial.Serial = None
        self._parser = FrameStreamParser()

    @staticmethod
    def get_available_ports():
        """枚举系统当前所有可用串口号及描述"""
        ports = serial.tools.list_ports.comports()
        return [(p.device, p.description) for p in sorted(ports, key=lambda x: x.device)]

    def run(self):
        try:
            # 打开串口 (Windows 下 1.5M 波特率高吞吐必须加大读缓冲区)
            self._ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,
            )
            # 设置驱动接收缓冲区大小为 1MB
            if hasattr(self._ser, "set_buffer_size"):
                try:
                    self._ser.set_buffer_size(rx_size=1048576, tx_size=65536)
                except Exception:
                    pass

            self._ser.reset_input_buffer()
            self._parser.reset()
            self._running = True
            self.sig_connected.emit(self.port, self.baudrate)

        except Exception as e:
            self.sig_error.emit(f"无法打开串口 {self.port} (波特率 {self.baudrate}): {str(e)}")
            return

        last_stat_time = time.time()
        bytes_in_sec = 0
        frames_in_sec = 0

        while self._running:
            try:
                waiting = self._ser.in_waiting
                to_read = max(waiting, 1024)
                chunk = self._ser.read(to_read)
                if not chunk:
                    continue

                bytes_in_sec += len(chunk)
                valid_frames = self._parser.push_data(chunk)

                for frame in valid_frames:
                    frames_in_sec += 1
                    pkt_id = frame["pkt_id"]
                    status = frame["status"]
                    payload = frame["payload"]
                    raw_codes, volts = parse_payload_to_matrix(payload, self.vref)
                    self.sig_frame_ready.emit(pkt_id, raw_codes, volts, status)

                now = time.time()
                if now - last_stat_time >= 1.0:
                    dt = now - last_stat_time
                    fps_val = frames_in_sec / dt
                    kb_val = (bytes_in_sec / dt) / 1024.0
                    self.sig_status_info.emit({
                        "fps": fps_val,
                        "kbps": kb_val,
                        "total_frames": self._parser.total_frames_received,
                        "lost_frames": self._parser.lost_frames_count,
                        "is_virtual": False,
                    })
                    frames_in_sec = 0
                    bytes_in_sec = 0
                    last_stat_time = now

            except Exception as e:
                if self._running:
                    self.sig_error.emit(f"串口读取异常中断: {str(e)}")
                break

        # 关闭清理
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass
        self._running = False
        self.sig_disconnected.emit()

    def stop(self):
        self._running = False
        self.wait(2000)
