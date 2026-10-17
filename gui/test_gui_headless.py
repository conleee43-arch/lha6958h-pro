# -*- coding: utf-8 -*-
"""
自动化测试 GUI 主窗口创建、虚拟数据流接收与界面刷新
"""

import os
import sys

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer
from gui.main_window import MainWindow
from gui.virtual_source import VirtualDataWorker


def test_gui():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()

    frames_received = []

    def on_frame(pkt_id, raw_codes, volts, status):
        frames_received.append((pkt_id, raw_codes.shape, volts.shape, status))

    worker = VirtualDataWorker(fps=6.0)
    worker.sig_frame_ready.connect(win._on_frame_ready)
    worker.sig_frame_ready.connect(on_frame)
    worker.sig_status_info.connect(win._on_status_info)
    worker.start()

    # 运行事件循环 1.5 秒
    timer = QTimer()
    timer.setSingleShot(True)
    timer.timeout.connect(app.quit)
    timer.start(1500)

    app.exec()

    worker.stop()
    win.close()

    print(f"GUI 自动化测试完成! 1.5 秒内成功接收处理 16-bit 大包数: {len(frames_received)}")
    assert len(frames_received) >= 4, f"接收大包数过少: {len(frames_received)}"
    print("最新一包码值矩阵尺寸:", frames_received[-1][1], "电压矩阵尺寸:", frames_received[-1][2])
    print("CH1 ~ CH8 最新电压值与 16-bit 十六进制码值:")
    for ch in range(8):
        v_str = win.table_channels.item(ch, 1).text()
        hex_str = win.table_channels.item(ch, 2).text()
        print(f"  CH{ch+1}: {v_str} V, Code={hex_str}")


if __name__ == "__main__":
    test_gui()
