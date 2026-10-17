# -*- coding: utf-8 -*-
"""
LHA6958H 8 通道 16-bit 1MSPS ADC 数据采集展示上位机入口
"""

import os
import sys
import traceback

# 确保项目根目录在 Python 模块检索路径中
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

print("=" * 65, flush=True)
print("  [LHA6958H] 8 通道 16-bit 1MSPS ADC 采集系统上位机正在启动...", flush=True)
print("=" * 65, flush=True)

try:
    from PySide6.QtWidgets import QApplication
    from PySide6.QtCore import Qt
    from gui.main_window import MainWindow

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    print("[提示] 正在创建并显示主窗口...", flush=True)
    window = MainWindow()
    window.show()

    # 强行置前台并激活窗口（防止被 IDE 或其他全屏窗口遮挡）
    window.raise_()
    window.activateWindow()

    print("[成功] 上位机界面已启动！如果未在前台看到，请检查 Windows 任务栏。", flush=True)
    print("=" * 65, flush=True)

    sys.exit(app.exec())

except Exception as e:
    print(f"\n[启动失败异常]: {e}", flush=True)
    traceback.print_exc()
    input("\n按回车键退出...")
    sys.exit(1)
