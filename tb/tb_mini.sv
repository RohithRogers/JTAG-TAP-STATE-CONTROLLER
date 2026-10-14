// tb_mini.sv — compact report-friendly testbench for apb_jtag_tap_top.
// Same DUT, same APB->TCK bridge, same BSR/BYPASS/IDCODE checks, ~180 lines.
// Run: iverilog -g2012 -o sim\mini.vvp rtl\tap_fsm.sv rtl\apb_jtag_tap_top.sv tb\tb_mini.sv && vvp sim\mini.vvp
`timescale 1ns/1ps

module tb_mini;
    reg PCLK = 0; always #5 PCLK = ~PCLK;           // 100 MHz
    reg PRESETn, PSEL, PENABLE, PWRITE;
    reg  [7:0] PADDR; reg [31:0] PWDATA;
    wire [31:0] PRDATA; wire PREADY, PSLVERR;
    wire tck_o, tms_o, tdi_o, tdo_o, trst_no; wire [3:0] tap_state_o;

    apb_jtag_tap_top dut (.PCLK(PCLK), .PRESETn(PRESETn), .PADDR(PADDR),
        .PSEL(PSEL), .PENABLE(PENABLE), .PWRITE(PWRITE), .PWDATA(PWDATA),
        .PRDATA(PRDATA), .PREADY(PREADY), .PSLVERR(PSLVERR), .tck_o(tck_o),
        .tms_o(tms_o), .tdi_o(tdi_o), .tdo_o(tdo_o), .trst_no(trst_no),
        .tap_state_o(tap_state_o));

    // States / addresses / instructions (must match RTL)
    localparam TLR=0, RTI=1, SELDR=2, CAPDR=3, SHIFTDR=4, E1DR=5, PAUSEDR=6,
               E2DR=7, UPDR=8, SELIR=9, CAPIR=10, SHIFTIR=11, E1IR=12,
               PAUSEIR=13, E2IR=14, UPIR=15;
    localparam C_CTRL=8'h00, C_STAT=8'h04, C_IR=8'h08, C_BSR=8'h0C,
               C_ID=8'h10, C_BYP=8'h14, C_HOLD=8'h18, C_INSTR=8'h1C;
    localparam [31:0] IDVAL = 32'h1495_1093;

    integer pass = 0, fail = 0, b;
    reg verr; reg [31:0] d, got_id;
    reg last_tdo;

    // --- reference model (mirrors RTL) ---
    reg [3:0] rs = TLR, ri = 0, rh = 4'h2;
    reg [7:0] rb = 0, rbh = 0; reg byp = 0; reg [31:0] id = 0;

    function [1:0] sel(input [3:0] ir);  // 0=BSR 1=BYP 2=IDCODE
        sel = (ir == 4'h0 || ir == 4'h1) ? 2'b00 : (ir == 4'h2) ? 2'b10 : 2'b01;
    endfunction

    function [3:0] nxt(input [3:0] c, input t);
        case (c)
            TLR:     nxt = t ? TLR : RTI;     RTI: nxt = t ? SELDR : RTI;
            SELDR:   nxt = t ? SELIR : CAPDR; CAPDR: nxt = t ? E1DR : SHIFTDR;
            SHIFTDR: nxt = t ? E1DR : SHIFTDR; E1DR: nxt = t ? UPDR : PAUSEDR;
            PAUSEDR: nxt = t ? E2DR : PAUSEDR; E2DR: nxt = t ? UPDR : SHIFTDR;
            UPDR:    nxt = t ? SELDR : RTI;   SELIR: nxt = t ? TLR : CAPIR;
            CAPIR:   nxt = t ? E1IR : SHIFTIR; SHIFTIR: nxt = t ? E1IR : SHIFTIR;
            E1IR:    nxt = t ? UPIR : PAUSEIR; PAUSEIR: nxt = t ? E2IR : PAUSEIR;
            E2IR:    nxt = t ? UPIR : SHIFTIR; UPIR: nxt = t ? SELDR : RTI;
            default: nxt = TLR;
        endcase
    endfunction

    function exp_tdo;
        if (rs == SHIFTIR) exp_tdo = ri[0];
        else if (rs == SHIFTDR)
            exp_tdo = (sel(rh) == 0) ? rb[0] : (sel(rh) == 1) ? byp : id[0];
        else exp_tdo = 0;
    endfunction

    task adv(input tms_i, input tdi_i);  // advance ref exactly like RTL tick
        begin
            if (rs == CAPIR) ri = 4'b0101;
            else if (rs == SHIFTIR) ri = {tdi_i, ri[3:1]};
            else if (rs == UPIR) rh = ri;
            if (rs == CAPDR) begin
                if (sel(rh) == 0) rb = 8'hA5;
                else if (sel(rh) == 1) byp = 0;
                else id = IDVAL;
            end else if (rs == SHIFTDR) begin
                if (sel(rh) == 0) rb = {tdi_i, rb[7:1]};
                else if (sel(rh) == 1) byp = tdi_i;
                else id = {tdi_i, id[31:1]};
            end else if (rs == UPDR && sel(rh) == 0) rbh = rb;
            rs = nxt(rs, tms_i);
        end
    endtask

    // --- APB driver (SETUP on negedge, sample on posedge: race-free) ---
    task idle; begin PSEL = 0; PENABLE = 0; PWRITE = 0; PADDR = 0; PWDATA = 0; end endtask
    task w(input [7:0] a, input [31:0] wdata, output v);
        begin @(negedge PCLK); PADDR = a; PWDATA = wdata; PWRITE = 1; PSEL = 1; PENABLE = 0;
              @(posedge PCLK); @(negedge PCLK); PENABLE = 1;
              @(posedge PCLK); #1; v = PSLVERR; @(negedge PCLK); idle(); end
    endtask
    task r(input [7:0] a, output [31:0] rdata, output v);
        begin @(negedge PCLK); PADDR = a; PWRITE = 0; PSEL = 1; PENABLE = 0;
              @(posedge PCLK); @(negedge PCLK); PENABLE = 1;
              @(posedge PCLK); #1; rdata = PRDATA; v = PSLVERR;
              @(negedge PCLK); idle(); end
    endtask

    // --- one APB-driven JTAG TCK with scoreboard ---
    task step(input tms_i, input tdi_i);
        reg [3:0] from; reg et;
        begin
            from = rs; et = exp_tdo();
            w(C_CTRL, {28'b0, 1'b1, tdi_i, tms_i, 1'b1}, verr);
            repeat (3) @(posedge PCLK);
            r(C_STAT, d, verr);
            last_tdo = d[0];
            adv(tms_i, tdi_i);
            if (d[4:1] !== rs) begin $display("FAIL step TMS=%b %0d->DUT%0d REF%0d", tms_i, from, d[4:1], rs); fail = fail + 1; end
            else pass = pass + 1;
            if ((from == SHIFTIR || from == SHIFTDR) && d[0] !== et) begin
                $display("FAIL TDO st=%0d got=%b exp=%b", from, d[0], et); fail = fail + 1; end
        end
    endtask

    task load(input [3:0] ins);  // IR scan RTI->RTI, then check INSTR
        begin
            step(1, 0); step(1, 0); step(0, 0); step(0, 0);
            step(0, ins[0]); step(0, ins[1]); step(0, ins[2]); step(1, ins[3]);
            step(1, 0); step(0, 0);
            r(C_INSTR, d, verr);
            if (d[3:0] !== ins) begin $display("FAIL INSTR %h!=%h", d[3:0], ins); fail = fail + 1; end
            else begin $display("PASS INSTR=%h", ins); pass = pass + 1; end
        end
    endtask

    task check(input [3:0] exp_st, input [127:0] name);
        begin r(C_STAT, d, verr);
            if (d[4:1] !== exp_st) begin $display("FAIL %0s st=%0d exp=%0d", name, d[4:1], exp_st); fail = fail + 1; end
            else begin $display("PASS %0s", name); pass = pass + 1; end end
    endtask

    initial begin
        $dumpfile("sim/mini.vcd"); $dumpvars(0, tb_mini);
        idle(); PRESETn = 0; repeat (4) @(posedge PCLK); PRESETn = 1; repeat (2) @(posedge PCLK);

        // 1. reset: TRST_N=0 -> TLR, release -> RTI, power-up INSTR=IDCODE
        w(C_CTRL, 32'h08, verr); repeat (2) @(posedge PCLK); check(TLR, "reset->TLR");
        w(C_CTRL, 32'h09, verr); repeat (2) @(posedge PCLK);
        rs = nxt(TLR, 0); r(C_INSTR, d, verr);
        if (d[3:0] !== 4'h2) begin $display("FAIL INSTR!=IDCODE"); fail = fail + 1; end
        else begin $display("PASS INSTR=IDCODE"); pass = pass + 1; end

        // 2. APB sanity: BSR loopback + PSLVERR on bad addr
        w(C_BSR, 32'h55, verr); r(C_BSR, d, verr);
        if (d[7:0] !== 8'h55) begin $display("FAIL BSR %h", d); fail = fail + 1; end
        else begin $display("PASS BSR loopback"); pass = pass + 1; end
        rb = 8'h55;
        w(8'hFF, 32'hDEAD, verr);
        if (!verr) begin $display("FAIL no PSLVERR"); fail = fail + 1; end
        else begin $display("PASS PSLVERR"); pass = pass + 1; end

        // 3. BSR: load SAMPLE, capture A5, shift 0x3C, prove update via HOLD
        load(4'h1);
        step(1, 0); step(0, 0); step(0, 0);
        step(0, 0); step(0, 0); step(0, 1); step(0, 1);
        step(0, 1); step(0, 1); step(0, 0); step(1, 0);
        step(1, 0); step(0, 0);
        r(C_BSR, d, verr);
        if (d[7:0] !== 8'h3C) begin $display("FAIL BSR=%h", d[7:0]); fail = fail + 1; end
        else begin $display("PASS BSR=3C"); pass = pass + 1; end
        r(C_HOLD, d, verr);
        if (d[7:0] !== rbh) begin $display("FAIL HOLD=%h ref=%h", d[7:0], rbh); fail = fail + 1; end
        else begin $display("PASS BSR_HOLD=%h", d[7:0]); pass = pass + 1; end

        // 4. BYPASS: capture-0 then 1-deep delay
        load(4'hF);
        step(1, 0); step(0, 0); step(0, 0); step(0, 1);
        if (last_tdo !== 0) begin $display("FAIL BYP cap"); fail = fail + 1; end
        else begin $display("PASS BYPASS cap-0 + delay"); pass = pass + 1; end
        step(0, 0); step(0, 1); step(1, 1); step(1, 0); step(0, 0);

        // 5. IDCODE: shift out 32 bits, must equal IDVAL
        load(4'h2);
        step(1, 0); step(0, 0); step(0, 0); got_id = 0;
        for (b = 0; b < 31; b = b + 1) begin step(0, 0); got_id[b] = last_tdo; end
        step(1, 0); got_id[31] = last_tdo;
        if (got_id !== IDVAL) begin $display("FAIL ID %h!=%h", got_id, IDVAL); fail = fail + 1; end
        else begin $display("PASS IDCODE=%h", got_id); pass = pass + 1; end
        step(1, 0); step(0, 0);

        $display("\nMINI RESULT: PASS=%0d FAIL=%0d", pass, fail);
        $display("%0s", (fail == 0) ? "ALL MINI TESTS PASSED" : "MINI TESTS FAILED");
        $finish;
    end
    initial begin #2000000; $display("TIMEOUT"); $finish; end
endmodule
