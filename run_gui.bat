@echo off
chcp 65001 >nul
title LHA6958H 8通道 16-bit ADC 数据采集上位机
cd /d "%~dp0"
echo 正在启动 LHA6958H 数据采集上位机...
python gui\main.py
if %errorlevel% neq 0 (
    echo.
    echo 启动出现异常，请确认已安装 PySide6, pyserial, numpy
    pause
)
