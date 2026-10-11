# APB-Compliant JTAG TAP State Controller — Specification & Transaction Report

Project: `APB compliant JTAG tap state controller`
Simulator: Icarus Verilog (`iverilog -g2012` + `vvp`)
Testbench: plain SystemVerilog/Verilog (Icarus-safe, no UVM/classes/covergroups)
TCK model: generated internally — one TCK tick per APB write to CTRL with `TCK_EN=1`
RTL: `rtl/tap_fsm.sv`, `rtl/apb_jtag_tap_top.sv` · TB: `tb/tb_top.sv`

---

## 1. APB (Advanced Peripheral Bus) Specification

APB is the ARM AMBA low-bandwidth peripheral bus. This DUT implements an **APB4 slave**
(subset: no `PSTRB`, no protection bits — documented as a simplification).

### 1.1 Signals used

| Signal    | Dir (slave view) | Width | Meaning |
|-----------|------------------|-------|---------|
| `PCLK`    | in  | 1 | Bus clock. All transfers synchronous to rising edge. |
| `PRESETn` | in  | 1 | Active-low reset. Resets APB regs + TAP FSM. |
| `PADDR`   | in  | 8 | Byte address. Only `0x00/0x04/0x08/0x0C` valid. |
| `PSEL`    | in  | 1 | Slave select. Marks start of transfer. |
| `PENABLE` | in  | 1 | Enable. Distinguishes SETUP vs ACCESS phase. |
| `PWRITE`  | in  | 1 | 1 = write, 0 = read. |
| `PWDATA`  | in  | 32 | Write data (valid during ACCESS). |
| `PRDATA`  | out | 32 | Read data (valid during ACCESS). |
| `PREADY`  | out | 1 | Slave ready. **Always 1** in this design (zero wait states). |
| `PSLVERR` | out | 1 | Error. 1 when address is invalid (`!= 0x00/04/08/0C`). |

### 1.2 The two-phase transfer

Every APB transaction has exactly two phases (with `PREADY=1`, each lasts 1 `PCLK`):

1. **SETUP** — master drives `PADDR/PWRITE/PWDATA`, asserts `PSEL=1`, `PENABLE=0`.
2. **ACCESS** — master asserts `PENABLE=1` (keeping `PSEL=1`). Slave samples
   inputs / drives `PRDATA/PSLVERR/PREADY` on the rising edge ending this phase.
   Master then deasserts to idle (`PSEL=0, PENABLE=0`).

Back-to-back transfers are allowed (new SETUP immediately after ACCESS).
`PENABLE` without a preceding `PSEL` is a protocol violation — the RTL contains
a simulation-only checker that `$display`s on this condition.

ASCII timing — write then read (`PREADY=1`):

```
PCLK     __/^\__/^\__/^\__/^\__/^\__/^\__
PSEL     _____/^^^^^^^^^^^^^^\___________
PENABLE  __________/^^^^^\_______________   <- ACCESS = 1 cycle
PWRITE   ====== WRITE=1 ==================
PADDR    ====== ADDR =====================
PWDATA   ====== WDATA ====================
PREADY   ^^^^^^^^^^^1^^^^^^^^^^^^^^^^^^^^   (always 1)
PSLVERR  _______________0/1_______________   (1 only on bad addr)
```

### 1.3 What this DUT simplifies (vs full AMBA APB4)

- `PREADY` fixed 1 → no wait states, no slave-stall testing needed.
- No `PSTRB` (all writes are full 32-bit), no `PPROT`.
- Single slave, no arbiter/decoder/multiplexer.
- `PSLVERR` only for illegal address; no timeout errors.

These are stated explicitly so the simplifications are not mistaken for bugs.

---

## 2. JTAG TAP State Controller Specification (IEEE 1149.1)

### 2.1 The 16 states

| # | Name | Abbrev | What happens here |
|---|------|--------|-------------------|
| 0 | Test-Logic-Reset | TLR | Reset. Entered by `TRST_N=0`, `PRESETn=0`, or 5× `TMS=1`. |
| 1 | Run-Test/Idle | RTI | Idle / run test. Stays with `TMS=0`. |
| 2 | Select-DR-Scan | SELDR | Chooses DR vs IR branch. |
| 3 | Capture-DR | CAPDR | Loads the *selected* DR: BSR←`8'hA5`, BYPASS←`1'b0`, IDCODE←`IDCODE_VAL`. |
| 4 | Shift-DR | SHIFTDR | Shifts selected DR LSB-first; `TDO` = shifted-out bit (registered post-tick). |
| 5 | Exit1-DR | E1DR | Exit or pause decision. |
| 6 | Pause-DR | PAUSEDR | Wait state, holds DR. |
| 7 | Exit2-DR | E2DR | Second exit; can return to shift or update. |
| 8 | Update-DR | UPDR | Latch/commit: BSR shift→`BSR_HOLD` (EXTEST/SAMPLE); BYPASS/IDCODE have no update action. |
| 9 | Select-IR-Scan | SELIR | Second-level select. |
| 10 | Capture-IR | CAPIR | Loads IR with fixed `4'b0101` (LSBs `01` per standard). |
| 11 | Shift-IR | SHIFTIR | Shifts IR: `TDO=IR[0]`, `IR={TDI,IR[3:1]}` per TCK. |
| 12 | Exit1-IR | E1IR | Exit or pause decision. |
| 13 | Pause-IR | PAUSEIR | Wait state, holds IR. |
| 14 | Exit2-IR | E2IR | Second exit. |
| 15 | Update-IR | UPIR | Latches `IR shift → IR hold` (the instruction takes effect here). |

Encoding in this project: `TLR=0 … UPIR=15` (`tap_state_o[3:0]`).

### 2.2 Transition table (next state on rising TCK as a function of TMS)

| Current | TMS=0 | TMS=1 |
|---------|-------|-------|
| TLR | RTI | TLR |
| RTI | RTI | SELDR |
| SELDR | CAPDR | SELIR |
| CAPDR | SHIFTDR | E1DR |
| SHIFTDR | SHIFTDR | E1DR |
| E1DR | PAUSEDR | UPDR |
| PAUSEDR | PAUSEDR | E2DR |
| E2DR | SHIFTDR | UPDR |
| UPDR | RTI | SELDR |
| SELIR | CAPIR | TLR |
| CAPIR | SHIFTIR | E1IR |
| SHIFTIR | SHIFTIR | E1IR |
| E1IR | PAUSEIR | UPIR |
| PAUSEIR | PAUSEIR | E2IR |
| E2IR | SHIFTIR | UPIR |
| UPIR | RTI | SELDR |

State diagram (text):

```
        TLR <--SELIR(1)-- ... every "1" path trends up-right toward TLR
         |0                (5 consecutive 1s from anywhere => TLR)
        RTI --1--> SELDR --1--> SELIR --1--> TLR
                    |0            |0
                  CAPDR         CAPIR
                    |             |
               SHIFTDR/E1IR... SHIFTIR/E1IR... (symmetric DR/IR branches)
```

Key facts verified by the testbench:

- **Forced reset:** any state + 5× `TMS=1` ⇒ `TLR` (test `test_tms_reset`).
- **Asynchronous reset:** `TRST_N=0` (APB-programmed) or `PRESETn=0` ⇒ `TLR` immediately.
- **Capture/Shift/Update discipline:** CAP loads a known pattern, SHIFT moves
  LSB-first with registered `TDO` = shifted-out bit, UPDATE commits
  (`UPDATE_IR` latches the instruction, `UPDATE_DR` latches BSR→hold).
  Pause states hold data.
- **IR=`4b`, BSR=`8b`, BYPASS=`1b`, IDCODE=`32b`** in this project
  (parameters `IR_WIDTH/DR_WIDTH/IDCODE_VAL`).

### 2.3 DR bank: BSR / BYPASS / IDCODE (selected by latched IR)

| Instruction | Encoding | Selects | Capture value | Shift behavior | Update action |
|-------------|----------|---------|---------------|----------------|---------------|
| EXTEST | `4'h0` | BSR (8b) | `8'hA5` | `BSR={TDI,BSR[7:1]}`, `TDO=BSR[0]` | `BSR_HOLD<=BSR` |
| SAMPLE/PRELOAD | `4'h1` | BSR (8b) | `8'hA5` | same as above | `BSR_HOLD<=BSR` |
| IDCODE | `4'h2` | IDCODE (32b) | `IDCODE_VAL=32'h1495_1093` (ver / part / mfr=`0x093`, LSB=1) | `IDCSR={TDI,IDCSR[31:1]}` | none |
| BYPASS | `4'hF` | BYPASS (1b) | `1'b0` | `BYP<=TDI`, `TDO`=old `BYP` (1-TCK delay) | none |
| any other | — | BYPASS (1b) | `1'b0` | same as BYPASS (per 1149.1, unimplemented ⇒ BYPASS) | none |

Power-up instruction (after `PRESETn`/`TRST_N`) is **IDCODE**, per IEEE 1149.1.
`STATUS[11:10]` reports the active bank (`00`=BSR, `01`=BYPASS, `10`=IDCODE).

---

## 3. DUT Register Map & Internal TCK Model

### 3.1 Register map

| Addr | Name | Access | Bits | Description |
|------|------|--------|------|-------------|
| `0x00` | CTRL | RW | `[0]` `TRST_N` (1=run, 0=reset TAP; reset default 1) | JTAG control + TCK strobe |
| | | | `[1]` `TMS_BIT` | TMS applied on next internal TCK |
| | | | `[2]` `TDI_BIT` | TDI shifted on next internal TCK (SHIFT states) |
| | | | `[3]` `TCK_EN` | 1 = this write generates one internal TCK tick |
| `0x04` | STATUS | RO | `[0]` `TDO` | Registered shifted-out bit (post-tick), 0 outside SHIFT-after-tick |
| | | | `[4:1]` `TAP_STATE` | Current FSM encoding 0–15 |
| | | | `[5]` `TCK` | Tick pulse (debug) |
| | | | `[9:6]` `IR_HOLD` | Latched instruction (takes effect at UPDATE_IR) |
| | | | `[11:10]` `DR_SEL` | Active DR bank: `00`=BSR `01`=BYPASS `10`=IDCODE |
| `0x08` | IR_REG | RW | `[3:0]` | IR shift register. Preload by write; CAP overwrites `0101`; SHIFT shifts; UPDATE_IR latches to `IR_HOLD`. |
| `0x0C` | BSR_REG | RW | `[7:0]` | Boundary Scan Register shift. Preload by write (SAMPLE/EXTEST); CAP overwrites `A5`; SHIFT shifts. |
| `0x10` | IDCODE_REG | RO | `[31:0]` | IDCODE shift. `0` until first CAPDR with IDCODE selected, then `IDCODE_VAL`; SHIFT shifts; writes ⇒ `PSLVERR`. |
| `0x14` | BYPASS_REG | RW | `[0]` | BYPASS bit. Preload by write; CAP overwrites `0`; SHIFT = 1-TCK delay. |
| `0x18` | BSR_HOLD | RO | `[7:0]` | BSR update (latch) — copied from `BSR_REG` at UPDATE_DR; writes ⇒ `PSLVERR`. |
| `0x1C` | INSTR | RO | `[3:0]` | Latched instruction (`IR_HOLD`); writes ⇒ `PSLVERR`. |

Writes to any RO register (`STATUS/IDCODE/BSR_HOLD/INSTR`) ⇒ `PSLVERR=1`.
Illegal address ⇒ `PSLVERR=1`, `PRDATA=0`.

### 3.2 Internal-TCK pipeline (the heart of “APB-compliant JTAG”)

`TCK` is **not** a free-running pin. It is a one-`PCLK`-wide pulse (`tck_o`)
produced by the DUT **two PCLK cycles after** an APB write to `CTRL` with `TCK_EN=1`:

```
PCLK        /\/\/\/\/\/\/\/\/\
APB ACCESS  ___/^^^\___________  (write CTRL, TCK_EN=1, TMS/TDI captured)
tick_d      _______/^\_________  (armed next cycle)
tick(tck_o) _________/__\______  (FSM advances here using stored TMS/TDI)
tap_state   =====OLD=====X=NEW=
```

Consequences for the testbench (and any software driver):

1. Write `CTRL = {28'b0, TCK_EN, TDI, TMS, TRST_N}`.
2. Wait **2× `PCLK`**.
3. Read `STATUS` to observe the new `TAP_STATE`/`TDO`.
4. `TRST_N=0` beats everything (async reset, no tick needed).

`TDO` timing: registered `tdo_q` captures the pre-shift LSB on each tick,
so the value read from `STATUS` **after** the tick is the bit just shifted
out (real JTAG behavior — `TDO` valid after `TCK`). The scoreboard therefore
samples expected `TDO` from its reference shift register **before** advancing
its reference state and compares it with the DUT value read **after** the tick.

---

## 4. Transaction Descriptions (worked examples)

### 4.1 APB write — preload BSR with `0x55`

```
SETUP:  PSEL=1 PENABLE=0 PWRITE=1 PADDR=0x0C PWDATA=0x55
ACCESS: PSEL=1 PENABLE=1 ... PREADY=1 PSLVERR=0 -> BSR <= 0x55
IDLE:   PSEL=0 PENABLE=0
```

No internal TCK is generated (writes to `IR/BSR/BYPASS` never tick; only `CTRL` writes do).
Writes to RO (`0x04/0x10/0x18/0x1C`) return `PSLVERR=1` and change nothing.

### 4.2 APB read — poll TAP state

```
SETUP:  PSEL=1 PENABLE=0 PWRITE=0 PADDR=0x04
ACCESS: PSEL=1 PENABLE=1 ... PRDATA={20'b0,dr_sel,ir_hold,tick,state,tdo} PSLVERR=0
```

### 4.3 APB error — write to `0xFF`

```
ACCESS: PREADY=1 PSLVERR=1 PRDATA=0 (no register updated, no tick)
```

Covered by `test_apb_sanity`.

### 4.4 One JTAG step — `RTI → SELDR` (`TMS=1`)

```
1. APB WRITE 0x00 <= 0x0B  (binary 1011: TCK_EN=1, TDI=0, TMS=1, TRST_N=1)
2. wait 2*PCLK            (tick fires, FSM RTI->SELDR)
3. APB READ  0x04 => STATUS[4:1]=2 (SELDR), TDO=0
```

`tb.jtag_step(1,0)` automates exactly this, plus reference-model comparison,
`TDO` check (when leaving a SHIFT state), `tms_o/tdi_o` pin checks, and
coverage updates. Every JTAG movement in every test goes through this task,
so each is simultaneously an APB transaction test and a TAP test.

### 4.5 Full IR shift — load instruction `1010` (LSB-first `0,1,0,1`)

Assumes starting in `RTI`:

| # | TMS | TDI | From → To | Notes |
|---|-----|-----|-----------|-------|
| 1 | 1 | 0 | RTI→SELDR | |
| 2 | 1 | 0 | SELDR→SELIR | |
| 3 | 0 | 0 | SELIR→CAPIR | capture `0101` into IR |
| 4 | 0 | 0 | CAPIR→SHIFTIR | |
| 5 | 0 | 0 | SHIFTIR→SHIFTIR | TDO checked = old `IR[0]` |
| 6 | 0 | 1 | SHIFTIR→SHIFTIR | TDO checked |
| 7 | 0 | 0 | SHIFTIR→SHIFTIR | TDO checked |
| 8 | 1 | 1 | SHIFTIR→E1IR | last bit + exit, TDO checked |
| 9 | 1 | 0 | E1IR→UPIR | |
| 10 | 0 | 0 | UPIR→RTI | then `APB READ 0x08` returns `1010` |

Covered by `test_shift_ir` (`test_shift_bsr` is the BSR analog with `8'h3C`;
`test_bypass` checks the 1-TCK delay; `test_idcode` shifts out all 32 bits).

### 4.5b DR scans — BSR / BYPASS / IDCODE (all assume start in RTI)

Instruction load (e.g. SAMPLE=`0001`, LSB-first) is itself an IR scan (§4.5),
ending with `UPDATE_IR` latching `INSTR` and `APB READ 0x1C` confirming it.

* **BSR** (`test_shift_bsr`): load `SAMPLE`, `RTI→SELDR→CAPDR` (BSR captures
  `A5`) `→SHIFTDR`, shift `8'h3C` LSB-first exiting with `TMS=1` on the last
  bit, `E1DR→UPDR→RTI`. `APB READ 0x0C` returns `3C`, `APB READ 0x18`
  (`BSR_HOLD`) returns `3C` — proving capture/shift **and** update.
* **BYPASS** (`test_bypass`): load `4'hF`, `→CAPDR` (captures `0`) `→SHIFTDR`.
  First shift-out is `0`; afterwards `TDO(n) = TDI(n-1)` (single-flop delay),
  verified bit-by-bit for pattern `101100`.
* **IDCODE** (`test_idcode`): load `4'h2` (also the power-up default),
  `→CAPDR` (loads `32'h1495_1093`) `→SHIFTDR`, shift 32 bits with `TDI=0`.
  Collected `TDO[31:0]` must equal `1495_1093` (LSB-first, LSB=1 verified
  explicitly). Afterwards `APB READ 0x10` reads `0` (register drained by zeros).

### 4.6 Pause/exit walk (DR branch)

`SHIFTDR --1--> E1DR --0--> PAUSEDR --0--> PAUSEDR(loop) --1--> E2DR --0--> SHIFTDR`
vs `--1--> UPDR --0--> RTI`. Both forks are exercised in `test_fsm_walk`,
which visits **all 16 states and all 32 arcs**.

### 4.7 Reset transactions

- **Power/APB reset:** `PRESETn=0` for 4 PCLK → everything (regs + FSM) to reset values.
- **TAP reset via APB:** `APB WRITE 0x00 <= 0x08` (`TRST_N=0`) → FSM drops to `TLR`
  asynchronously; then `WRITE 0x00 <= 0x09` releases (`TRST_N=1`, one tick `TLR→RTI`).
- **TAP reset via TMS:** five `jtag_step(1,·)` from anywhere → `TLR`.

---

## 5. Verification Environment & Test List

`tb_top` contains (all plain tasks, Icarus-safe):

- **APB master driver** (`apb_write/apb_read`): correct SETUP/ACCESS sequencing,
  `PREADY` check, `PSLVERR` return, transaction counting.
- **JTAG stepper** (`jtag_step`): the single procedural bridge between APB and
  JTAG — every TAP advance is an APB write + wait + APB read.
- **Reference model** (`ref_state/ref_ir/ref_ir_hold/ref_bsr/ref_bsr_hold/ref_bypass/ref_idcode_sr`, `ref_next/ref_advance`): independent
  IEEE-1149.1 next-state function + IR/DR-bank mirror used for predictions.
- **Scoreboard**: state equality + `TDO` equality per active bank (SHIFT states) + `tms_o/tdi_o`
  pin checks + `INSTR`/`DR_SEL`/`BSR_HOLD`/IDCODE-shift-out checks; `pass_cnt/fail_cnt`.
- **Manual functional coverage**: `state_visited[15:0]`, `arc0_seen/arc1_seen[15:0]`,
  `dr_seen[2:0]` (BSR/BYPASS/IDCODE)
  (covergroups are unsupported in Icarus, hence manual bits), plus APB/JTAG counters.
- **Protocol checker**: `PENABLE`-without-`PSEL` message in RTL (sim-only).

| Test | What it proves |
|------|----------------|
| `test_reset` | `TRST_N`/`PRESETn` force `TLR`, power-up `INSTR=IDCODE`, `DR_SEL=IDCODE` |
| `test_apb_sanity` | IR/BSR/BYPASS loopback, IDCODE pre-capture=0, `PSLVERR` on RO-write + bad addr (R+W) |
| `test_tms_reset` | 5×`TMS=1` ⇒ `TLR` |
| `test_fsm_walk` | all 16 states + all 32 TMS arcs vs reference |
| `test_shift_ir` | 4-bit IR shift + `TDO` per-bit + readback + hold; unimplemented op ⇒ BYPASS bank |
| `test_shift_bsr` | SAMPLE load, 8-bit BSR capture/shift (`A5`→`3C`) + `BSR_HOLD` update proof |
| `test_bypass` | BYPASS capture-0 + 1-TCK `TDO=TDI(n-1)` delay chain + reg readback |
| `test_idcode` | full 32-bit IDCODE shift-out = `1495_1093`, LSB=1, drain check |
| `test_random_tms` | 200 constrained-random `(TMS,TDI)` steps vs reference (all banks) |
| coverage report | `States 16/16`, `Arcs 32/32`, `DR banks 3/3` expected; warns otherwise |

Run: `sim\run.bat` (or `mingw32-make -C sim sim`). Waveform: `sim/wave.vcd` (GTKWave).
Success: `RESULT: PASS=… FAIL=0` + `ALL TESTS PASSED`.

---

## 6. File Map

```
rtl/tap_fsm.sv        — 16-state FSM + next-state function
rtl/apb_jtag_tap_top.sv — APB slave regs, tick pipeline, IR hold, BSR/BYPASS/IDCODE bank, TDO
tb/tb_top.sv          — driver, stepper, ref model, scoreboard, coverage, tests
sim/Makefile, sim/run.bat — Icarus build/run
docs/spec_report.md   — this file
sim/wave.vcd          — generated VCD (after run)
```

*End of report.*
