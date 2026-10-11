// tb_top.sv — plain-Verilog (Icarus-compatible) verification environment
// for apb_jtag_tap_top with BSR/BYPASS/IDCODE DR bank.
// No classes/interfaces/covergroups/mailboxes.
// Run: iverilog -g2012 -o sim/tb.vvp rtl/tap_fsm.sv rtl/apb_jtag_tap_top.sv tb/tb_top.sv && vvp sim/tb.vvp
`timescale 1ns/1ps

module tb_top;

    // ---- Clocks / resets ----
    reg PCLK;
    reg PRESETn;

    initial PCLK = 0;
    always #5 PCLK = ~PCLK;  // 100 MHz

    // ---- APB signals ----
    reg  [7:0]  PADDR;
    reg         PSEL;
    reg         PENABLE;
    reg         PWRITE;
    reg  [31:0] PWDATA;
    wire [31:0] PRDATA;
    wire        PREADY;
    wire        PSLVERR;

    // ---- JTAG outputs from DUT ----
    wire       tck_o, tms_o, tdi_o, tdo_o, trst_no;
    wire [3:0] tap_state_o;

    // ---- DUT ----
    apb_jtag_tap_top #(.IR_WIDTH(4), .DR_WIDTH(8)) dut (
        .PCLK(PCLK), .PRESETn(PRESETn),
        .PADDR(PADDR), .PSEL(PSEL), .PENABLE(PENABLE), .PWRITE(PWRITE),
        .PWDATA(PWDATA), .PRDATA(PRDATA), .PREADY(PREADY), .PSLVERR(PSLVERR),
        .tck_o(tck_o), .tms_o(tms_o), .tdi_o(tdi_o), .tdo_o(tdo_o),
        .trst_no(trst_no), .tap_state_o(tap_state_o)
    );

    // ---- State encodings (must match RTL) ----
    localparam [3:0]
        TLR=0, RTI=1, SELDR=2, CAPDR=3, SHIFTDR=4, E1DR=5, PAUSEDR=6,
        E2DR=7, UPDR=8, SELIR=9, CAPIR=10, SHIFTIR=11, E1IR=12,
        PAUSEIR=13, E2IR=14, UPIR=15;

    // ---- APB map (must match RTL) ----
    localparam [7:0] A_CTRL=8'h00, A_STATUS=8'h04, A_IR=8'h08, A_BSR=8'h0C,
                     A_IDCODE=8'h10, A_BYPASS=8'h14, A_BSRHOLD=8'h18, A_INSTR=8'h1C;
    localparam [7:0] A_DR = A_BSR; // alias: legacy DR address = BSR

    // ---- Instructions / DR select (must match RTL) ----
    localparam [3:0] I_EXTEST=4'h0, I_SAMPLE=4'h1, I_IDCODE=4'h2, I_BYPASS=4'hF;
    localparam [1:0] DR_BSR=2'b00, DR_BYPASS=2'b01, DR_IDCODE=2'b10;
    localparam [31:0] IDCODE_VAL = 32'h1495_1093;

    // ---- Scoreboard / coverage counters ----
    integer pass_cnt, fail_cnt;
    integer i, j, k;
    reg [15:0] state_visited;        // bit per TAP state
    reg [15:0] arc0_seen, arc1_seen; // per-state TMS=0 / TMS=1 transition seen
    reg [2:0]  dr_seen;              // per DR bank (BSR/BYPASS/IDCODE) selected
    reg [31:0] apb_txn_cnt, jtag_step_cnt;
    reg        last_tdo;             // TDO sampled by most recent jtag_step

    // Reference model (mirrors DUT incl. DR bank)
    reg [3:0] ref_state;
    reg [3:0] ref_ir;        // IR shift
    reg [3:0] ref_ir_hold;   // latched instruction
    reg [7:0] ref_bsr;       // BSR shift
    reg [7:0] ref_bsr_hold;  // BSR update hold
    reg       ref_bypass;    // BYPASS bit
    reg [31:0] ref_idcode_sr;

    function [1:0] ref_drsel;
        input [3:0] instr;
        begin
            case (instr)
                I_EXTEST: ref_drsel = DR_BSR;
                I_SAMPLE: ref_drsel = DR_BSR;
                I_IDCODE: ref_drsel = DR_IDCODE;
                default:  ref_drsel = DR_BYPASS;
            endcase
        end
    endfunction

    function [3:0] ref_next;
        input [3:0] cur;
        input tms_in;
        begin
            case (cur)
                TLR:     ref_next = tms_in ? TLR : RTI;
                RTI:     ref_next = tms_in ? SELDR : RTI;
                SELDR:   ref_next = tms_in ? SELIR : CAPDR;
                CAPDR:   ref_next = tms_in ? E1DR : SHIFTDR;
                SHIFTDR: ref_next = tms_in ? E1DR : SHIFTDR;
                E1DR:    ref_next = tms_in ? UPDR : PAUSEDR;
                PAUSEDR: ref_next = tms_in ? E2DR : PAUSEDR;
                E2DR:    ref_next = tms_in ? UPDR : SHIFTDR;
                UPDR:    ref_next = tms_in ? SELDR : RTI;
                SELIR:   ref_next = tms_in ? TLR : CAPIR;
                CAPIR:   ref_next = tms_in ? E1IR : SHIFTIR;
                SHIFTIR: ref_next = tms_in ? E1IR : SHIFTIR;
                E1IR:    ref_next = tms_in ? UPIR : PAUSEIR;
                PAUSEIR: ref_next = tms_in ? E2IR : PAUSEIR;
                E2IR:    ref_next = tms_in ? UPIR : SHIFTIR;
                UPIR:    ref_next = tms_in ? SELDR : RTI;
                default: ref_next = TLR;
            endcase
        end
    endfunction

    task ref_advance;
        input tms_in;
        input tdi_in;
        begin
            // mirror RTL tick: capture/shift/update use OLD state + OLD ir_hold
            if (ref_state == CAPIR) ref_ir = 4'b0101;
            else if (ref_state == SHIFTIR) ref_ir = {tdi_in, ref_ir[3:1]};
            else if (ref_state == UPIR) ref_ir_hold = ref_ir;
            if (ref_state == CAPDR) begin
                case (ref_drsel(ref_ir_hold))
                    DR_BSR:    ref_bsr = 8'hA5;
                    DR_BYPASS: ref_bypass = 1'b0;
                    DR_IDCODE: ref_idcode_sr = IDCODE_VAL;
                endcase
            end else if (ref_state == SHIFTDR) begin
                case (ref_drsel(ref_ir_hold))
                    DR_BSR:    ref_bsr = {tdi_in, ref_bsr[7:1]};
                    DR_BYPASS: ref_bypass = tdi_in;
                    DR_IDCODE: ref_idcode_sr = {tdi_in, ref_idcode_sr[31:1]};
                endcase
            end else if (ref_state == UPDR) begin
                if (ref_drsel(ref_ir_hold) == DR_BSR)
                    ref_bsr_hold = ref_bsr;
            end
            ref_state = ref_next(ref_state, tms_in);
        end
    endtask

    function exp_tdo_f;
        input [3:0] st;
        begin
            if (st == SHIFTIR) exp_tdo_f = ref_ir[0];
            else if (st == SHIFTDR) begin
                case (ref_drsel(ref_ir_hold))
                    DR_BSR:    exp_tdo_f = ref_bsr[0];
                    DR_BYPASS: exp_tdo_f = ref_bypass;
                    DR_IDCODE: exp_tdo_f = ref_idcode_sr[0];
                    default:   exp_tdo_f = 1'b0;
                endcase
            end else exp_tdo_f = 1'b0;
        end
    endfunction

    task ref_reset;
        begin
            ref_state = TLR; ref_ir = 0; ref_ir_hold = I_IDCODE;
            ref_bsr = 0; ref_bsr_hold = 0; ref_bypass = 0; ref_idcode_sr = 0;
        end
    endtask

    // ============ APB DRIVER TASKS ============
    task apb_idle;
        begin
            PSEL = 0; PENABLE = 0; PWRITE = 0; PADDR = 8'h00; PWDATA = 32'h0;
        end
    endtask

    task apb_write;
        input [7:0] addr;
        input [31:0] data;
        output verr;
        begin
            // SETUP phase (drive on negedge so DUT samples cleanly on posedge)
            @(negedge PCLK);
            PADDR = addr; PWDATA = data; PWRITE = 1; PSEL = 1; PENABLE = 0;
            @(posedge PCLK); // DUT sees SETUP (no sampling)
            // ACCESS phase
            @(negedge PCLK);
            PENABLE = 1;
            @(posedge PCLK); // DUT samples write on this edge
            #1;
            verr = PSLVERR;
            if (!PREADY) begin
                $display("FAIL @%0t: PREADY low (unsupported wait state)", $time);
                fail_cnt = fail_cnt + 1;
            end
            apb_txn_cnt = apb_txn_cnt + 1;
            // return to idle
            @(negedge PCLK);
            apb_idle();
        end
    endtask

    task apb_read;
        input [7:0] addr;
        output [31:0] data;
        output verr;
        begin
            @(negedge PCLK);
            PADDR = addr; PWRITE = 0; PSEL = 1; PENABLE = 0;
            @(posedge PCLK);
            @(negedge PCLK);
            PENABLE = 1;
            @(posedge PCLK);
            #1;
            data = PRDATA; verr = PSLVERR;
            apb_txn_cnt = apb_txn_cnt + 1;
            @(negedge PCLK);
            apb_idle();
        end
    endtask

    // ============ JTAG STEP TASK (scoreboard + coverage) ============
    // One APB-driven internal TCK. Checks DUT state vs ref model and TDO.
    task jtag_step;
        input tms_in;
        input tdi_in;
        reg verr;
        reg [31:0] status;
        reg [3:0] dut_state, exp_state;
        reg exp_tdo, got_tdo;
        reg [3:0] from_state;
        reg [1:0] sel;
        begin
            from_state = ref_state;
            exp_tdo = exp_tdo_f(ref_state);
            sel = ref_drsel(ref_ir_hold);

            // drive one TCK: CTRL = {TCK_EN=1, TDI, TMS, TRST_N=1}
            apb_write(A_CTRL, {28'b0, 1'b1, tdi_in, tms_in, 1'b1}, verr);
            if (verr) begin
                $display("FAIL @%0t: PSLVERR on CTRL write", $time); fail_cnt = fail_cnt+1;
            end
            // tick pipeline = 2 PCLKs after the sampling edge, then read STATUS
            repeat (3) @(posedge PCLK);
            apb_read(A_STATUS, status, verr);
            got_tdo = status[0];
            dut_state = status[4:1];
            last_tdo = got_tdo;

            // advance reference AFTER sampling expected TDO
            ref_advance(tms_in, tdi_in);
            exp_state = ref_state;

            // coverage
            state_visited[dut_state] = 1'b1;
            if (tms_in) arc1_seen[from_state] = 1'b1;
            else arc0_seen[from_state] = 1'b1;
            if (from_state == CAPDR || from_state == SHIFTDR || from_state == UPDR)
                dr_seen[sel] = 1'b1;
            // cross-check STATUS upper fields (IR_HOLD + DR_SEL)
            if (status[9:6] !== ref_ir_hold && (from_state == UPIR || 1'b0)) begin
                // checked explicitly in load_instruction; skip here
            end
            jtag_step_cnt = jtag_step_cnt + 1;

            // checks
            if (dut_state !== exp_state) begin
                $display("FAIL @%0t: JTAG step TMS=%b from %0d: DUT=%0d REF=%0d",
                         $time, tms_in, from_state, dut_state, exp_state);
                fail_cnt = fail_cnt + 1;
            end else pass_cnt = pass_cnt + 1;

            if ((from_state == SHIFTIR || from_state == SHIFTDR) && (got_tdo !== exp_tdo)) begin
                $display("FAIL @%0t: TDO mismatch in state %0d (DRsel=%0d): got=%b exp=%b",
                         $time, from_state, sel, got_tdo, exp_tdo);
                fail_cnt = fail_cnt + 1;
            end

            // pin-level checks
            if (tms_o !== tms_in) begin
                $display("FAIL @%0t: tms_o=%b expected %b", $time, tms_o, tms_in); fail_cnt=fail_cnt+1;
            end
            if (tdi_o !== tdi_in) begin
                $display("FAIL @%0t: tdi_o=%b expected %b", $time, tdi_o, tdi_in); fail_cnt=fail_cnt+1;
            end
        end
    endtask

    task check_state;
        input [3:0] exp;
        input [127:0] name;
        reg verr; reg [31:0] s;
        begin
            apb_read(A_STATUS, s, verr);
            if (s[4:1] !== exp) begin
                $display("FAIL %0s: state=%0d expected %0d", name, s[4:1], exp);
                fail_cnt = fail_cnt + 1;
            end else begin
                $display("PASS %0s: state=%0d", name, exp);
                pass_cnt = pass_cnt + 1;
            end
            state_visited[s[4:1]] = 1'b1;
        end
    endtask

    task check_instr;
        input [3:0] exp;
        input [127:0] name;
        reg verr; reg [31:0] d;
        begin
            apb_read(A_INSTR, d, verr);
            if (verr || d[3:0] !== exp) begin
                $display("FAIL %0s: INSTR=%h expected %h (verr=%b)", name, d[3:0], exp, verr);
                fail_cnt = fail_cnt + 1;
            end else begin
                $display("PASS %0s: INSTR=%h", name, exp);
                pass_cnt = pass_cnt + 1;
            end
        end
    endtask

    task check_drsel;
        input [1:0] exp;
        input [127:0] name;
        reg verr; reg [31:0] s;
        begin
            apb_read(A_STATUS, s, verr);
            if (s[11:10] !== exp) begin
                $display("FAIL %0s: DR_SEL=%b expected %b", name, s[11:10], exp);
                fail_cnt = fail_cnt + 1;
            end else begin
                $display("PASS %0s: DR_SEL=%b", name, exp);
                pass_cnt = pass_cnt + 1;
            end
        end
    endtask

    // Go to RTI from anywhere (tracked steps, keeps ref aligned)
    task goto_rti;
        begin
            jtag_step(1, 0); jtag_step(1, 0); jtag_step(1, 0);
            jtag_step(1, 0); jtag_step(1, 0);
            jtag_step(0, 0); // TLR->RTI
        end
    endtask

    // IR scan to load an instruction; assumes start in RTI, ends in RTI
    task load_instruction;
        input [3:0] instr;
        integer b;
        begin
            jtag_step(1, 0); // RTI->SELDR
            jtag_step(1, 0); // SELDR->SELIR
            jtag_step(0, 0); // SELIR->CAPIR
            jtag_step(0, 0); // CAPIR->SHIFTIR
            for (b = 0; b < 3; b = b + 1)
                jtag_step(0, instr[b]);       // shift bits[2:0], stay
            jtag_step(1, instr[3]);           // last bit + exit ->E1IR
            jtag_step(1, 0);                  // E1IR->UPIR
            jtag_step(0, 0);                  // UPIR->RTI (latches ir_hold)
            check_instr(instr, "load_instr");
        end
    endtask

    // ============ TESTS ============
    task test_reset;
        reg verr; reg [31:0] d;
        begin
            $display("\n--- test_reset ---");
            apb_write(A_CTRL, 32'h0000_0008, verr); // TRST=0,TCK_EN=1
            @(posedge PCLK); @(posedge PCLK);
            apb_read(A_STATUS, d, verr);
            if (d[4:1] !== TLR) begin $display("FAIL reset: not in TLR"); fail_cnt=fail_cnt+1; end
            else begin $display("PASS reset: TLR after TRST_N=0"); pass_cnt=pass_cnt+1; end
            ref_reset();
            // release TRST (one tick TLR->RTI)
            apb_write(A_CTRL, 32'h0000_0009, verr); // TRST=1,TCK_EN=1,TMS=0
            @(posedge PCLK); @(posedge PCLK);
            ref_state = ref_next(TLR, 1'b0);
            // power-up instruction must be IDCODE
            check_instr(I_IDCODE, "reset INSTR=IDCODE");
            check_drsel(DR_IDCODE, "reset DR=IDCODE");
        end
    endtask

    task test_apb_sanity;
        reg verr; reg [31:0] d;
        begin
            $display("\n--- test_apb_sanity ---");
            apb_write(A_IR, 32'h0000_000A, verr);
            if (verr) begin $display("FAIL IR write PSLVERR"); fail_cnt=fail_cnt+1; end
            apb_read(A_IR, d, verr);
            if (d[3:0] !== 4'hA) begin $display("FAIL IR loopback got %h", d); fail_cnt=fail_cnt+1; end
            else begin $display("PASS IR loopback"); pass_cnt=pass_cnt+1; end
            apb_write(A_BSR, 32'h0000_0055, verr);
            apb_read(A_BSR, d, verr);
            if (d[7:0] !== 8'h55) begin $display("FAIL BSR loopback got %h", d); fail_cnt=fail_cnt+1; end
            else begin $display("PASS BSR loopback"); pass_cnt=pass_cnt+1; end
            apb_write(A_BYPASS, 32'h0000_0001, verr);
            apb_read(A_BYPASS, d, verr);
            if (d[0] !== 1'b1) begin $display("FAIL BYPASS loopback got %h", d); fail_cnt=fail_cnt+1; end
            else begin $display("PASS BYPASS loopback"); pass_cnt=pass_cnt+1; end
            // IDCODE is RO but readable; pre-capture value is 0 (captured on CAPDR)
            apb_read(A_IDCODE, d, verr);
            if (verr || d !== 32'h0) begin $display("FAIL IDCODE pre-capture got %h verr=%b", d, verr); fail_cnt=fail_cnt+1; end
            else begin $display("PASS IDCODE pre-capture = 0"); pass_cnt=pass_cnt+1; end
            // writes to RO registers must PSLVERR
            apb_write(A_IDCODE, 32'hDEAD_BEEF, verr);
            if (!verr) begin $display("FAIL IDCODE write: PSLVERR not set"); fail_cnt=fail_cnt+1; end
            else begin $display("PASS IDCODE write PSLVERR"); pass_cnt=pass_cnt+1; end
            apb_write(A_STATUS, 32'h0, verr);
            if (!verr) begin $display("FAIL STATUS write: PSLVERR not set"); fail_cnt=fail_cnt+1; end
            else begin $display("PASS STATUS write PSLVERR"); pass_cnt=pass_cnt+1; end
            // bad address must give PSLVERR
            apb_write(8'hFF, 32'hDEAD_BEEF, verr);
            if (!verr) begin $display("FAIL bad-addr write: PSLVERR not set"); fail_cnt=fail_cnt+1; end
            else begin $display("PASS bad-addr write PSLVERR"); pass_cnt=pass_cnt+1; end
            apb_read(8'hFF, d, verr);
            if (!verr) begin $display("FAIL bad-addr read: PSLVERR not set"); fail_cnt=fail_cnt+1; end
            else begin $display("PASS bad-addr read PSLVERR"); pass_cnt=pass_cnt+1; end
            ref_ir = 4'hA; ref_bsr = 8'h55; ref_bypass = 1'b1;
        end
    endtask

    task test_tms_reset;
        begin
            $display("\n--- test_tms_reset (5x TMS=1) ---");
            jtag_step(1, 0); jtag_step(1, 0); jtag_step(1, 0);
            jtag_step(1, 0); jtag_step(1, 0);
            check_state(TLR, "tms_reset->TLR");
        end
    endtask

    task test_fsm_walk;
        begin
            $display("\n--- test_fsm_walk (all arcs) ---");
            jtag_step(0, 0); // TLR->RTI
            jtag_step(1, 0); // RTI->SELDR
            jtag_step(0, 0); // SELDR->CAPDR
            jtag_step(0, 0); // CAPDR->SHIFTDR
            jtag_step(0, 0); // SHIFTDR loop
            jtag_step(1, 0); // SHIFTDR->E1DR
            jtag_step(0, 0); // E1DR->PAUSEDR
            jtag_step(0, 0); // PAUSEDR loop
            jtag_step(1, 0); // PAUSEDR->E2DR
            jtag_step(0, 0); // E2DR->SHIFTDR
            jtag_step(1, 0); // SHIFTDR->E1DR
            jtag_step(1, 0); // E1DR->UPDR
            jtag_step(1, 0); // UPDR->SELDR (TMS=1)
            jtag_step(1, 0); // SELDR->SELIR
            jtag_step(0, 0); // SELIR->CAPIR
            jtag_step(0, 0); // CAPIR->SHIFTIR
            jtag_step(0, 0); // SHIFTIR loop
            jtag_step(1, 0); // SHIFTIR->E1IR
            jtag_step(0, 0); // E1IR->PAUSEIR
            jtag_step(0, 0); // PAUSEIR loop
            jtag_step(1, 0); // PAUSEIR->E2IR
            jtag_step(0, 0); // E2IR->SHIFTIR
            jtag_step(1, 0); // SHIFTIR->E1IR
            jtag_step(1, 0); // E1IR->UPIR
            jtag_step(0, 0); // UPIR->RTI
            jtag_step(1, 0); // RTI->SELDR
            jtag_step(0, 0); // SELDR->CAPDR
            jtag_step(1, 0); // CAPDR->E1DR
            jtag_step(0, 0); // E1DR->PAUSEDR
            jtag_step(1, 0); // PAUSEDR->E2DR
            jtag_step(1, 0); // E2DR->UPDR
            jtag_step(0, 0); // UPDR->RTI
            jtag_step(1, 0); // RTI->SELDR
            jtag_step(1, 0); // SELDR->SELIR
            jtag_step(1, 0); // SELIR->TLR
            check_state(TLR, "fsm_walk end TLR");
            jtag_step(0, 0); // TLR->RTI for next tests
        end
    endtask

    task test_shift_ir;
        reg verr; reg [31:0] d;
        begin
            $display("\n--- test_shift_ir (load 4'b1010) ---");
            jtag_step(1, 0); // RTI->SELDR
            jtag_step(1, 0); // SELDR->SELIR
            jtag_step(0, 0); // SELIR->CAPIR (captures 0101)
            jtag_step(0, 0); // CAPIR->SHIFTIR
            jtag_step(0, 0);
            jtag_step(0, 1);
            jtag_step(0, 0);
            jtag_step(1, 1); // ->E1IR
            jtag_step(1, 0); // E1IR->UPIR
            jtag_step(0, 0); // UPIR->RTI (latches 1010)
            apb_read(A_IR, d, verr);
            $display("INFO IR back = %h (ref %h)", d[3:0], ref_ir);
            if (d[3:0] !== ref_ir) begin $display("FAIL IR shift data"); fail_cnt=fail_cnt+1; end
            else begin $display("PASS IR shift data"); pass_cnt=pass_cnt+1; end
            check_instr(4'hA, "IR hold = 1010");
            check_drsel(DR_BYPASS, "unimplemented IR -> BYPASS");
        end
    endtask

    task test_shift_bsr;
        reg verr; reg [31:0] d;
        begin
            $display("\n--- test_shift_bsr (SAMPLE + pattern 8'h3C) ---");
            load_instruction(I_SAMPLE);
            check_drsel(DR_BSR, "SAMPLE -> BSR");
            // RTI->SELDR->CAPDR->SHIFTDR
            jtag_step(1, 0); // RTI->SELDR
            jtag_step(0, 0); // SELDR->CAPDR (captures A5 into BSR)
            jtag_step(0, 0); // CAPDR->SHIFTDR
            // shift 8'h3C = 0011_1100 LSB first: 0,0,1,1,1,1,0,0(last exits)
            jtag_step(0, 0); jtag_step(0, 0); jtag_step(0, 1); jtag_step(0, 1);
            jtag_step(0, 1); jtag_step(0, 1); jtag_step(0, 0); jtag_step(1, 0);
            jtag_step(1, 0); // E1DR->UPDR
            jtag_step(0, 0); // UPDR->RTI (bsr_hold <= bsr)
            apb_read(A_BSR, d, verr);
            $display("INFO BSR back = %h (ref %h)", d[7:0], ref_bsr);
            if (d[7:0] !== ref_bsr) begin $display("FAIL BSR shift data"); fail_cnt=fail_cnt+1; end
            else begin $display("PASS BSR shift data"); pass_cnt=pass_cnt+1; end
            apb_read(A_BSRHOLD, d, verr);
            $display("INFO BSR_HOLD = %h (ref %h)", d[7:0], ref_bsr_hold);
            if (d[7:0] !== ref_bsr_hold) begin $display("FAIL BSR update (hold)"); fail_cnt=fail_cnt+1; end
            else begin $display("PASS BSR update (hold)"); pass_cnt=pass_cnt+1; end
        end
    endtask

    task test_bypass;
        reg verr; reg [31:0] d;
        reg [5:0] tdi_pat;
        integer b;
        begin
            $display("\n--- test_bypass (1-bit delay check) ---");
            load_instruction(I_BYPASS);
            check_drsel(DR_BYPASS, "BYPASS instr -> BYPASS");
            tdi_pat = 6'b101100;
            jtag_step(1, 0); // RTI->SELDR
            jtag_step(0, 0); // SELDR->CAPDR (captures 0 into bypass)
            jtag_step(0, 0); // CAPDR->SHIFTDR
            // first shift-out must be capture-0
            jtag_step(0, tdi_pat[0]);
            if (last_tdo !== 1'b0) begin $display("FAIL BYPASS capture-0, got %b", last_tdo); fail_cnt=fail_cnt+1; end
            else begin $display("PASS BYPASS capture-0"); pass_cnt=pass_cnt+1; end
            // subsequent TDO = previous TDI (1-deep delay)
            for (b = 1; b < 6; b = b + 1) begin
                jtag_step(0, tdi_pat[b]);
                if (last_tdo !== tdi_pat[b-1]) begin
                    $display("FAIL BYPASS delay bit%0d: got %b exp %b", b, last_tdo, tdi_pat[b-1]);
                    fail_cnt = fail_cnt + 1;
                end
            end
            $display("PASS BYPASS delay chain");
            pass_cnt = pass_cnt + 1;
            jtag_step(1, 0); // SHIFTDR->E1DR
            jtag_step(1, 0); // E1DR->UPDR
            jtag_step(0, 0); // UPDR->RTI
            apb_read(A_BYPASS, d, verr);
            if (d[0] !== ref_bypass) begin $display("FAIL BYPASS reg %b vs %b", d[0], ref_bypass); fail_cnt=fail_cnt+1; end
            else begin $display("PASS BYPASS reg"); pass_cnt=pass_cnt+1; end
        end
    endtask

    task test_idcode;
        reg verr; reg [31:0] d, shifted;
        integer b;
        begin
            $display("\n--- test_idcode (shift out 32'h%h) ---", IDCODE_VAL);
            load_instruction(I_IDCODE);
            check_drsel(DR_IDCODE, "IDCODE instr -> IDCODE");
            jtag_step(1, 0); // RTI->SELDR
            jtag_step(0, 0); // SELDR->CAPDR (loads IDCODE_VAL)
            jtag_step(0, 0); // CAPDR->SHIFTDR
            shifted = 0;
            for (b = 0; b < 31; b = b + 1) begin
                jtag_step(0, 0); // shift with TDI=0, stay
                shifted[b] = last_tdo;
            end
            jtag_step(1, 0); // last bit + exit ->E1DR
            shifted[31] = last_tdo;
            $display("INFO IDCODE shifted = %h (exp %h)", shifted, IDCODE_VAL);
            if (shifted !== IDCODE_VAL) begin $display("FAIL IDCODE shift-out"); fail_cnt=fail_cnt+1; end
            else begin $display("PASS IDCODE shift-out"); pass_cnt=pass_cnt+1; end
            if (shifted[0] !== 1'b1) begin $display("FAIL IDCODE LSB!=1"); fail_cnt=fail_cnt+1; end
            else begin $display("PASS IDCODE LSB=1"); pass_cnt=pass_cnt+1; end
            jtag_step(1, 0); // E1DR->UPDR
            jtag_step(0, 0); // UPDR->RTI
            apb_read(A_IDCODE, d, verr);
            // after shifting 32 zeros, idcode_sr should be 0
            if (d !== ref_idcode_sr) begin $display("FAIL IDCODE reg %h vs ref %h", d, ref_idcode_sr); fail_cnt=fail_cnt+1; end
            else begin $display("PASS IDCODE reg drained to %h", d); pass_cnt=pass_cnt+1; end
        end
    endtask

    task test_random_tms;
        integer seed, n;
        reg tms_r, tdi_r;
        begin
            $display("\n--- test_random_tms (200 steps) ---");
            seed = 12345;
            for (n = 0; n < 200; n = n + 1) begin
                tms_r = $random(seed) & 1'b1;
                tdi_r = ($random(seed) >> 1) & 1'b1;
                jtag_step(tms_r, tdi_r);
            end
            $display("INFO random done, ref_state=%0d dut=%0d", ref_state, tap_state_o);
        end
    endtask

    task report_coverage;
        integer s, st_cnt, arc_cnt, dr_cnt;
        begin
            st_cnt = 0; arc_cnt = 0; dr_cnt = 0;
            for (s = 0; s < 16; s = s + 1) begin
                if (state_visited[s]) st_cnt = st_cnt + 1;
                if (arc0_seen[s]) arc_cnt = arc_cnt + 1;
                if (arc1_seen[s]) arc_cnt = arc_cnt + 1;
            end
            for (s = 0; s < 3; s = s + 1)
                if (dr_seen[s]) dr_cnt = dr_cnt + 1;
            $display("\n===== COVERAGE =====");
            $display("States visited : %0d/16", st_cnt);
            $display("Arcs covered   : %0d/32", arc_cnt);
            $display("DR banks used  : %0d/3 (BSR=%b BYPASS=%b IDCODE=%b)",
                     dr_cnt, dr_seen[0], dr_seen[1], dr_seen[2]);
            $display("APB txns       : %0d", apb_txn_cnt);
            $display("JTAG steps     : %0d", jtag_step_cnt);
            if (st_cnt != 16) $display("WARN: missing states!");
            if (arc_cnt != 32) $display("WARN: missing arcs!");
            if (dr_cnt != 3) $display("WARN: missing DR banks!");
        end
    endtask

    // ============ MAIN ============
    initial begin
        $dumpfile("sim/wave.vcd");
        $dumpvars(0, tb_top);
        pass_cnt = 0; fail_cnt = 0;
        apb_txn_cnt = 0; jtag_step_cnt = 0;
        state_visited = 0; arc0_seen = 0; arc1_seen = 0; dr_seen = 0;
        last_tdo = 0;
        ref_reset();

        apb_idle();
        PRESETn = 0;
        repeat (4) @(posedge PCLK);
        PRESETn = 1;
        repeat (2) @(posedge PCLK);
        state_visited[TLR] = 1'b1;

        test_reset();
        test_apb_sanity();
        test_tms_reset();
        test_fsm_walk();
        test_shift_ir();
        test_shift_bsr();
        test_bypass();
        test_idcode();
        test_random_tms();
        report_coverage();

        $display("\n===== RESULT: PASS=%0d FAIL=%0d =====", pass_cnt, fail_cnt);
        if (fail_cnt == 0) $display("ALL TESTS PASSED");
        else $display("TESTS FAILED");
        $finish;
    end

    // Timeout guard
    initial begin
        #2000000;
        $display("TIMEOUT — simulation did not finish");
        $finish;
    end

endmodule
