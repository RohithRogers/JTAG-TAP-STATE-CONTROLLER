# APB-Compliant JTAG TAP State Controller

SystemVerilog RTL + plain-Verilog verification environment (Icarus-safe, no UVM),
where an APB slave bit-bangs the IEEE 1149.1 JTAG TAP controller with an
internally generated TCK (one tick per APB write to CTRL with `TCK_EN=1`).
DR scan supports the standard bank: **BSR (8b, EXTEST/SAMPLE)**,
**BYPASS (1b)** and **32-bit IDCODE**, selected by the latched IR
(power-up instruction = IDCODE; unimplemented opcodes ⇒ BYPASS).

## Structure

```
rtl/tap_fsm.sv           16-state TAP FSM (IEEE 1149.1 next-state function)
rtl/apb_jtag_tap_top.sv  APB slave regs + tick pipeline + IR hold + BSR/BYPASS/IDCODE + TDO
tb/tb_top.sv             APB driver, JTAG stepper, ref model, scoreboard,
                         manual coverage (states/arcs/DR banks), directed + random tests
sim/run.bat, sim/Makefile  Icarus build/run
docs/spec_report.md      APB + JTAG + DR-bank spec + transactions report
```

## Run (Icarus Verilog, Windows)

```
sim\run.bat
REM or: iverilog -g2012 -Wall -o sim\tb.vvp rtl\tap_fsm.sv rtl\apb_jtag_tap_top.sv tb\tb_top.sv
REM      vvp sim\tb.vvp
```

Waveform: `sim/wave.vcd` (open in GTKWave).

## Current result

```
States visited : 16/16, Arcs covered : 32/32, DR banks : 3/3
RESULT: PASS=373 FAIL=0 — ALL TESTS PASSED
```

See `docs/spec_report.md` for the full APB/JTAG specification and transaction walkthrough.
