# Makefile for APB-compliant JTAG TAP project (Icarus Verilog)
# Usage (Windows with iverilog in PATH, or Git Bash / nmake):
#   mingw32-make sim      - compile + run
#   mingw32-make waves    - same (produces sim/wave.vcd, view with GTKWave)
#   mingw32-make clean    - remove build artifacts

IVERILOG = iverilog
VVP      = vvp
FLAGS    = -g2012 -Wall -o sim/tb.vvp
SRC      = rtl/tap_fsm.sv rtl/apb_jtag_tap_top.sv tb/tb_top.sv

sim: $(SRC)
	$(IVERILOG) $(FLAGS) $(SRC)
	$(VVP) sim/tb.vvp

waves: sim

clean:
	del /Q sim\tb.vvp sim\wave.vcd 2>NUL || rm -f sim/tb.vvp sim/wave.vcd
