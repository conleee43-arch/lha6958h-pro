@echo off
chcp 65001 >nul
title LHA6958H FPGA 固件烧录工具
echo ======================================================================
echo   [LHA6958H] 正在通过 USB-Blaster 向 FPGA 下载固件 (lha6958h.sof)...
echo ======================================================================

set SOF_PATH=output_files\lha6958h.sof
if not exist "%SOF_PATH%" (
    echo [错误] 未在 output_files\ 目录下找到 lha6958h.sof 文件！
    pause
    exit /b 1
)

set PGM_EXE=C:\altera\13.1\quartus\bin64\quartus_pgm.exe
if not exist "%PGM_EXE%" (
    set PGM_EXE=quartus_pgm.exe
)

"%PGM_EXE%" -c "USB-Blaster [USB-0]" -m JTAG -o "p;%SOF_PATH%"

if %errorlevel% equ 0 (
    echo.
    echo ======================================================================
    echo   [成功] FPGA 固件已顺利烧录！
    echo          板载状态:
    echo            - LED0 常亮: LHA6958H 上电寄存器初始化成功 (Init OK)
    echo            - LED1 闪烁: 系统正常采集中
    echo          现在可启动 python gui/main.py 或双击 run_gui.bat 查看波形数据。
    echo ======================================================================
) else (
    echo.
    echo [失败] 固件烧录失败，请检查:
    echo   1. USB-Blaster 下载线是否已插紧电脑和 FPGA 板
    echo   2. FPGA 开发板电源是否已开启 (5V/3.3V)
    echo   3. 设备管理器中 Altera USB-Blaster 驱动是否正常
)
echo.
pause

