# -*- coding: utf-8 -*-
"""
LHA6958H 8 通道 16-bit ADC 数据展示上位机主窗口 (MainWindow)
基于 PySide6 实现
"""

import sys
import numpy as np
from PySide6.QtCore import Qt, Slot
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QComboBox,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QProgressBar,
    QTextEdit,
    QMessageBox,
)

from .protocol import NUM_CHANNELS, DEFAULT_VREF
from .serial_worker import SerialWorker
from .virtual_source import VirtualDataWorker


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LHA6958H 8通道 16bit 1MSPS ADC 数据采集系统")
        self.resize(1120, 760)
        self.setMinimumSize(960, 600)

        self._serial_worker: SerialWorker = None
        self._virtual_worker: VirtualDataWorker = None
        self._vref = DEFAULT_VREF

        self._init_ui()
        self._refresh_ports()

        self.log("系统就绪: LHA6958H 8 通道 16-bit ADC 采集系统 (采样率: 500 kSPS, 单包窗口: 2.048 ms)")
        self.log("硬件提示: 每次 FPGA 重新上电需下载固件 (可双击根目录下 download_sof.bat)")

        # 智能优先匹配到有效 FPGA 串口 (如 CH340) 时，启动后自动建立连接
        if self.combo_port.count() > 0 and self.combo_port.currentData():
            dev = self.combo_port.currentData()
            self.log(f"已智能优选 FPGA 串口: {dev} @ 1.5Mbps，正在自动连接...")
            self._start_serial()


    def _init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(10)

        # 1. 顶部控制栏
        top_ctrl_group = QGroupBox("串口通信与数据源配置")
        top_layout = QHBoxLayout(top_ctrl_group)
        top_layout.setContentsMargins(12, 10, 12, 10)
        top_layout.setSpacing(10)

        # 串口配置
        top_layout.addWidget(QLabel("串口号:"))
        self.combo_port = QComboBox()
        self.combo_port.setMinimumWidth(150)
        top_layout.addWidget(self.combo_port)

        btn_refresh = QPushButton("刷新")
        btn_refresh.clicked.connect(self._refresh_ports)
        top_layout.addWidget(btn_refresh)

        top_layout.addSpacing(10)
        top_layout.addWidget(QLabel("波特率:"))
        self.combo_baud = QComboBox()
        for baud in [1500000, 115200, 921600, 2000000, 460800]:
            self.combo_baud.addItem(str(baud), baud)
        top_layout.addWidget(self.combo_baud)

        self.btn_serial_toggle = QPushButton("打开串口")
        self.btn_serial_toggle.setStyleSheet("font-weight: bold; background-color: #e1f5fe;")
        self.btn_serial_toggle.clicked.connect(self._toggle_serial)
        top_layout.addWidget(self.btn_serial_toggle)

        top_layout.addSpacing(15)

        # 离线模拟数据演示按钮
        self.btn_virtual_toggle = QPushButton("启动模拟演示")
        self.btn_virtual_toggle.setStyleSheet("font-weight: bold; background-color: #ede7f6;")
        self.btn_virtual_toggle.clicked.connect(self._toggle_virtual)
        top_layout.addWidget(self.btn_virtual_toggle)

        top_layout.addSpacing(20)

        # COM 口连接状态
        self.lbl_conn_status = QLabel("状态: 待机空闲")
        self.lbl_conn_status.setStyleSheet("color: gray; font-weight: bold;")
        top_layout.addWidget(self.lbl_conn_status)

        top_layout.addStretch()

        # 硬件状态指示 (FPGA 寄存器初始化 / 错误警告)
        self.lbl_hw_status = QLabel("FPGA 状态: --")
        self.lbl_hw_status.setStyleSheet("color: #0277bd; font-weight: bold;")
        top_layout.addWidget(self.lbl_hw_status)

        main_layout.addWidget(top_ctrl_group, 0)

        # 2. 8 通道实时数据表格 (主体核心区域)
        table_group = QGroupBox("LHA6958H 8 通道实时采集数据 (每包 1024 组 @ 500kSPS, 满量程 ±10V, 观测窗口 2.048 ms)")
        table_layout = QVBoxLayout(table_group)

        self.table_channels = QTableWidget(NUM_CHANNELS, 7)
        self.table_channels.setHorizontalHeaderLabels([
            "通道", "实时电压 (V)", "原始 16-bit 码值", "有符号十进制", "最小值 (V)", "最大值 (V)", "电平幅度指示"
        ])
        self.table_channels.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_channels.horizontalHeader().setSectionResizeMode(6, QHeaderView.ResizeToContents)
        self.table_channels.verticalHeader().setVisible(False)
        self.table_channels.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_channels.setEditTriggers(QTableWidget.NoEditTriggers)

        for ch in range(NUM_CHANNELS):
            item_name = QTableWidgetItem(f"CH{ch + 1}")
            item_name.setTextAlignment(Qt.AlignCenter)
            item_name.setFont(QFont("Arial", 10, QFont.Bold))
            self.table_channels.setItem(ch, 0, item_name)

            for col in range(1, 6):
                it = QTableWidgetItem("--")
                it.setTextAlignment(Qt.AlignCenter)
                self.table_channels.setItem(ch, col, it)

            # 电平指示条 (-100% ~ +100%)
            bar = QProgressBar()
            bar.setRange(-100, 100)
            bar.setValue(0)
            bar.setFormat("%v %")
            bar.setAlignment(Qt.AlignCenter)
            bar.setFixedWidth(160)
            self.table_channels.setCellWidget(ch, 6, bar)

        table_layout.addWidget(self.table_channels)
        main_layout.addWidget(table_group, 1)

        # 3. 操作日志
        log_group = QGroupBox("系统运行与通信日志")
        log_group.setMaximumHeight(140)
        log_layout = QVBoxLayout(log_group)
        log_layout.setContentsMargins(8, 12, 8, 8)
        self.text_log = QTextEdit()
        self.text_log.setReadOnly(True)
        log_layout.addWidget(self.text_log)
        main_layout.addWidget(log_group, 0)

    def log(self, msg: str):
        self.text_log.append(f"[{sys.platform}] {msg}")

    def _refresh_ports(self):
        self.combo_port.clear()
        ports = SerialWorker.get_available_ports()
        default_idx = 0
        for idx, (dev, desc) in enumerate(ports):
            self.combo_port.addItem(f"{dev} ({desc})", dev)
            if any(k in desc.upper() for k in ["CH340", "USB", "CP210", "FTDI"]):
                default_idx = idx
        if not ports:
            self.combo_port.addItem("无可用串口", "")
            self.btn_serial_toggle.setEnabled(False)
        else:
            self.combo_port.setCurrentIndex(default_idx)
            self.btn_serial_toggle.setEnabled(True)


    # --------------------------------------------------------------------------
    # 真实串口控制逻辑
    # --------------------------------------------------------------------------
    def _toggle_serial(self):
        if self._serial_worker and self._serial_worker.isRunning():
            self._stop_serial()
        else:
            self._start_serial()

    def _start_serial(self):
        if self._virtual_worker and self._virtual_worker.isRunning():
            self._stop_virtual()

        port = self.combo_port.currentData()
        if not port:
            QMessageBox.warning(self, "警告", "请先选择有效的串口端口！")
            return

        baud = self.combo_baud.currentData()
        self._serial_worker = SerialWorker(port=port, baudrate=baud, parent=self)
        self._serial_worker.vref = self._vref
        self._serial_worker.sig_connected.connect(self._on_serial_connected)
        self._serial_worker.sig_disconnected.connect(self._on_serial_disconnected)
        self._serial_worker.sig_frame_ready.connect(self._on_frame_ready)
        self._serial_worker.sig_status_info.connect(self._on_status_info)
        self._serial_worker.sig_error.connect(self._on_error)

        self._serial_worker.start()
        self.btn_serial_toggle.setText("关闭串口")
        self.btn_serial_toggle.setStyleSheet("font-weight: bold; background-color: #ffcdd2;")
        self.combo_port.setEnabled(False)
        self.combo_baud.setEnabled(False)
        self.btn_virtual_toggle.setEnabled(False)

    def _stop_serial(self):
        if self._serial_worker:
            self._serial_worker.stop()
            self._serial_worker = None
        self.btn_serial_toggle.setText("打开串口")
        self.btn_serial_toggle.setStyleSheet("font-weight: bold; background-color: #e1f5fe;")
        self.combo_port.setEnabled(True)
        self.combo_baud.setEnabled(True)
        self.btn_virtual_toggle.setEnabled(True)
        self.lbl_conn_status.setText("状态: 串口已关闭")
        self.lbl_conn_status.setStyleSheet("color: gray; font-weight: bold;")

    # --------------------------------------------------------------------------
    # 模拟数据演示控制逻辑
    # --------------------------------------------------------------------------
    def _toggle_virtual(self):
        if self._virtual_worker and self._virtual_worker.isRunning():
            self._stop_virtual()
        else:
            self._start_virtual()

    def _start_virtual(self):
        if self._serial_worker and self._serial_worker.isRunning():
            self._stop_serial()

        self._virtual_worker = VirtualDataWorker(fps=6.0, parent=self)
        self._virtual_worker.vref = self._vref
        self._virtual_worker.sig_frame_ready.connect(self._on_frame_ready)
        self._virtual_worker.sig_status_info.connect(self._on_status_info)
        self._virtual_worker.sig_error.connect(self._on_error)
        self._virtual_worker.start()

        self.btn_virtual_toggle.setText("停止模拟")
        self.btn_virtual_toggle.setStyleSheet("font-weight: bold; background-color: #ffcdd2;")
        self.btn_serial_toggle.setEnabled(False)
        self.combo_port.setEnabled(False)
        self.combo_baud.setEnabled(False)
        self.lbl_conn_status.setText("状态: 虚拟信号发生器运行中")
        self.lbl_conn_status.setStyleSheet("color: #7b1fa2; font-weight: bold;")
        self.log("已启动虚拟模拟数据流: 8通道正弦/方波/三角波/直流特征信号已注入")

    def _stop_virtual(self):
        if self._virtual_worker:
            self._virtual_worker.stop()
            self._virtual_worker = None
        self.btn_virtual_toggle.setText("启动模拟演示")
        self.btn_virtual_toggle.setStyleSheet("font-weight: bold; background-color: #ede7f6;")
        self.btn_serial_toggle.setEnabled(True)
        self.combo_port.setEnabled(True)
        self.combo_baud.setEnabled(True)
        self.lbl_conn_status.setText("状态: 待机空闲")
        self.lbl_conn_status.setStyleSheet("color: gray; font-weight: bold;")
        self.log("虚拟模拟数据流已停止")

    @Slot(str, int)
    def _on_serial_connected(self, port: str, baud: int):
        self.lbl_conn_status.setText(f"状态: 已连接 {port} @ {baud} bps")
        self.lbl_conn_status.setStyleSheet("color: green; font-weight: bold;")
        self.log(f"已成功打开串口: {port}, 波特率: {baud}")

    @Slot()
    def _on_serial_disconnected(self):
        self._stop_serial()
        self.log("串口连接已断开")

    @Slot(int, np.ndarray, np.ndarray, int)
    def _on_frame_ready(self, pkt_id: int, raw_codes: np.ndarray, voltages: np.ndarray, status: int):
        last_codes = raw_codes[-1, :]
        last_volts = voltages[-1, :]

        min_in_pack = np.min(voltages, axis=0)
        max_in_pack = np.max(voltages, axis=0)

        # 硬件状态解析
        init_ok = bool(status & 0x01)
        err_align = bool(status & 0x02)
        err_busy_timeout = bool(status & 0x04)
        err_overrun = bool(status & 0x08)

        err_list = []
        if err_align:
            err_list.append("对齐失步")
        if err_busy_timeout:
            err_list.append("BUSY超时")
        if err_overrun:
            err_list.append("采样溢出")

        if err_list:
            self.lbl_hw_status.setText(f"FPGA 状态: 警告 [{', '.join(err_list)}]")
            self.lbl_hw_status.setStyleSheet("color: red; font-weight: bold;")
        elif init_ok:
            self.lbl_hw_status.setText("FPGA 状态: 正常运行 (Init OK)")
            self.lbl_hw_status.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.lbl_hw_status.setText("FPGA 状态: 初始化中...")
            self.lbl_hw_status.setStyleSheet("color: orange; font-weight: bold;")

        # 终端控制台同步实时输出当前包的 8 通道数据
        ch_summary = " | ".join([f"CH{i+1}:{last_volts[i]:+6.3f}V" for i in range(NUM_CHANNELS)])
        print(f"[UART Rx Pkt #{pkt_id:03d}] {ch_summary}", flush=True)

        for ch in range(NUM_CHANNELS):
            code_val = int(last_codes[ch])
            v_val = float(last_volts[ch])

            hex_str = f"0x{(code_val & 0xFFFF):04X}"

            self.table_channels.item(ch, 1).setText(f"{v_val:+.4f}")
            self.table_channels.item(ch, 2).setText(hex_str)
            self.table_channels.item(ch, 3).setText(f"{code_val:+d}")
            self.table_channels.item(ch, 4).setText(f"{min_in_pack[ch]:+.4f}")
            self.table_channels.item(ch, 5).setText(f"{max_in_pack[ch]:+.4f}")

            # 电平幅度百分比 (-100% ~ +100%)
            pct = int((v_val / self._vref) * 100)
            pct = max(-100, min(100, pct))
            bar: QProgressBar = self.table_channels.cellWidget(ch, 6)
            if bar:
                bar.setValue(pct)
                bar.setFormat(f"{pct:+d}%")

    @Slot(dict)
    def _on_status_info(self, info: dict):
        fps = info.get("fps", 0.0)
        kbps = info.get("kbps", 0.0)
        lost = info.get("lost_frames", 0)
        total = info.get("total_frames", 0)
        is_virt = info.get("is_virtual", False)
        mode_str = "[模拟演示] " if is_virt else ""
        stat_msg = f"{mode_str}已接收 {total} 包 | 速率: {kbps:.1f} kB/s ({fps:.1f} FPS) | 丢包: {lost}"
        self.statusBar().showMessage(stat_msg)
        if lost > 0:
            self.log(f"警告: 累计丢包 {lost} 包")

    @Slot(str)
    def _on_error(self, err_msg: str):
        self._stop_serial()
        self.log(f"错误: {err_msg}")
        QMessageBox.critical(self, "通信异常", err_msg)


    def closeEvent(self, event):
        self._stop_serial()
        self._stop_virtual()
        event.accept()
