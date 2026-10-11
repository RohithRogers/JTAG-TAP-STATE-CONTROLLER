@echo off
REM run.bat — compile + simulate with Icarus Verilog (Windows)
REM Double-click or run from project root: sim\run.bat
setlocal
cd /d %~dp0..
if not exist sim mkdir sim
iverilog -g2012 -Wall -o sim\tb.vvp rtl\tap_fsm.sv rtl\apb_jtag_tap_top.sv tb\tb_top.sv
if errorlevel 1 (
  echo COMPILE FAILED
  exit /b 1
)
vvp sim\tb.vvp
