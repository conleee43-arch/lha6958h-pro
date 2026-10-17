@echo off
cd /d %~dp0
echo Launching ModelSim GUI with waveform...
start C:\modeltech64_2019.2\win64\vsim.exe -do "do run_gui.do"

