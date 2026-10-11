// apb_jtag_tap_top.sv — APB-compliant JTAG TAP state controller.
// Bit-bang model: each APB write to CTRL with TCK_EN=1 generates ONE internal
// TCK tick (one PCLK-wide pulse, FSM advances 2 PCLKs after the APB sample edge).
// DR bank (selected by latched IR): BSR (8b) / BYPASS (1b) / IDCODE (32b).
// Icarus-safe RTL (no SVA/properties, plain always blocks).
`timescale 1ns/1ps

module apb_jtag_tap_top #(
    parameter IR_WIDTH  = 4,
    parameter DR_WIDTH  = 8,          // BSR width (kept for compat)
    parameter [31:0] IDCODE_VAL = 32'h1495_1093 // ver=1, part=0x4951, mfr=0x093, LSb=1
)(
    // APB4 slave (PCLK domain, PREADY always 1, no wait states)
    input  wire        PCLK,
    input  wire        PRESETn,   // active-low APB reset
    input  wire [7:0]  PADDR,
    input  wire        PSEL,
    input  wire        PENABLE,
    input  wire        PWRITE,
    input  wire [31:0] PWDATA,
    output reg  [31:0] PRDATA,
    output wire        PREADY,    // always 1
    output reg         PSLVERR,

    // JTAG port (TCK generated internally)
    output wire        tck_o,     // 1-PCLK pulse per APB-driven step
    output wire        tms_o,
    output wire        tdi_o,
    output wire        tdo_o,
    output wire        trst_no,   // TRST_N value programmed via APB (also readable)
    output wire [3:0]  tap_state_o
);

    // Register map (byte addresses, word aligned)
    localparam [7:0]
        ADDR_CTRL    = 8'h00,  // RW: [0]=TRST_N [1]=TMS [2]=TDI [3]=TCK_EN
        ADDR_STATUS  = 8'h04,  // RO: [0]=TDO [4:1]=STATE [5]=TCK [9:6]=IR_HOLD [11:10]=DR_SEL
        ADDR_IR      = 8'h08,  // RW: IR shift register (IR_WIDTH bits)
        ADDR_BSR     = 8'h0C,  // RW: BSR shift register (DR_WIDTH bits)
        ADDR_IDCODE  = 8'h10,  // RO: IDCODE shift register (32 bits)
        ADDR_BYPASS  = 8'h14,  // RW: BYPASS bit (bit0 only)
        ADDR_BSRHOLD = 8'h18,  // RO: BSR update/hold register (DR_WIDTH bits)
        ADDR_INSTR   = 8'h1C;  // RO: latched instruction (bits[3:0])

    // Instructions (IR_WIDTH=4)
    localparam [3:0]
        INSTR_EXTEST = 4'h0,
        INSTR_SAMPLE = 4'h1,
        INSTR_IDCODE = 4'h2,
        INSTR_BYPASS = 4'hF;

    // DR select encoding (STATUS[11:10])
    localparam [1:0]
        DR_BSR    = 2'b00,
        DR_BYPASS = 2'b01,
        DR_IDCODE = 2'b10;

    localparam [3:0]
        S_CAPTURE_DR = 4'd3,
        S_SHIFT_DR   = 4'd4,
        S_UPDATE_DR  = 4'd8,
        S_CAPTURE_IR = 4'd10,
        S_SHIFT_IR   = 4'd11,
        S_UPDATE_IR  = 4'd15;

    // DR select function from latched instruction
    function [1:0] dr_sel_of;
        input [IR_WIDTH-1:0] instr;
        begin
            case (instr)
                INSTR_EXTEST: dr_sel_of = DR_BSR;
                INSTR_SAMPLE: dr_sel_of = DR_BSR;
                INSTR_IDCODE: dr_sel_of = DR_IDCODE;
                default:      dr_sel_of = DR_BYPASS; // BYPASS + all unimplemented
            endcase
        end
    endfunction

    // Control registers (APB domain)
    reg        trst_n_q;   // default 1 (not in reset)
    reg        tms_q;
    reg        tdi_q;

    // TCK tick pipeline: write -> tick_d -> tick (FSM advances on tick)
    reg        tick_d;
    reg        tick_q;

    // JTAG registers
    reg [IR_WIDTH-1:0] ir_sr;     // IR shift
    reg [IR_WIDTH-1:0] ir_hold;   // latched instruction (UPDATE_IR)
    reg [DR_WIDTH-1:0] bsr;       // Boundary Scan Register shift
    reg [DR_WIDTH-1:0] bsr_hold;  // BSR update/hold (UPDATE_DR)
    reg                bypass;    // BYPASS 1-bit shift
    reg [31:0]         idcode_sr; // IDCODE shift
    reg                tdo_q;     // registered shifted-out bit
    wire [1:0]         dr_sel;

    assign dr_sel = dr_sel_of(ir_hold);

    // TAP FSM
    wire       tap_rst_n;
    wire [3:0] tap_state;

    assign tap_rst_n = PRESETn & trst_n_q;

    tap_fsm u_fsm (
        .clk     (PCLK),
        .rst_n   (tap_rst_n),
        .tms     (tms_q),
        .tick    (tick_q),
        .state_o (tap_state)
    );

    // JTAG register update on internal TCK tick.
    // tdo_q captures the pre-shift LSB so STATUS/TDO read AFTER the tick
    // returns the bit just shifted out.
    always @(posedge PCLK or negedge tap_rst_n) begin
        if (!tap_rst_n) begin
            ir_sr     <= {IR_WIDTH{1'b0}};
            ir_hold   <= INSTR_IDCODE; // power-up instruction = IDCODE (IEEE 1149.1)
            bsr       <= {DR_WIDTH{1'b0}};
            bsr_hold  <= {DR_WIDTH{1'b0}};
            bypass    <= 1'b0;
            idcode_sr <= 32'h0000_0000;
            tdo_q     <= 1'b0;
        end else if (tick_q) begin
            // TDO capture (pre-shift)
            if (tap_state == S_SHIFT_IR)
                tdo_q <= ir_sr[0];
            else if (tap_state == S_SHIFT_DR) begin
                case (dr_sel)
                    DR_BSR:    tdo_q <= bsr[0];
                    DR_BYPASS: tdo_q <= bypass;
                    DR_IDCODE: tdo_q <= idcode_sr[0];
                    default:   tdo_q <= 1'b0;
                endcase
            end else
                tdo_q <= 1'b0;
            // register updates
            case (tap_state)
                S_CAPTURE_IR: ir_sr <= 4'b0101; // fixed capture pattern (LSBs = 01)
                S_SHIFT_IR:   ir_sr <= {tdi_q, ir_sr[IR_WIDTH-1:1]};
                S_UPDATE_IR:  ir_hold <= ir_sr;
                S_CAPTURE_DR: begin
                    case (dr_sel)
                        DR_BSR:    bsr <= 8'hA5;
                        DR_BYPASS: bypass <= 1'b0; // BYPASS captures 0
                        DR_IDCODE: idcode_sr <= IDCODE_VAL;
                    endcase
                end
                S_SHIFT_DR: begin
                    case (dr_sel)
                        DR_BSR:    bsr <= {tdi_q, bsr[DR_WIDTH-1:1]};
                        DR_BYPASS: bypass <= tdi_q;
                        DR_IDCODE: idcode_sr <= {tdi_q, idcode_sr[31:1]};
                    endcase
                end
                S_UPDATE_DR: begin
                    if (dr_sel == DR_BSR)
                        bsr_hold <= bsr; // EXTEST/SAMPLE update latches BSR
                end
                default: begin
                    ir_sr     <= ir_sr;
                    bsr       <= bsr;
                    idcode_sr <= idcode_sr;
                end
            endcase
        end
    end

    // APB write detection (access phase, PREADY=1 so single-cycle access)
    wire apb_access = PSEL & PENABLE;
    wire apb_write  = apb_access & PWRITE;
    wire addr_valid = (PADDR == ADDR_CTRL)   || (PADDR == ADDR_STATUS) ||
                      (PADDR == ADDR_IR)     || (PADDR == ADDR_BSR)    ||
                      (PADDR == ADDR_IDCODE) || (PADDR == ADDR_BYPASS) ||
                      (PADDR == ADDR_BSRHOLD)|| (PADDR == ADDR_INSTR);
    // Read-only addrs: STATUS, IDCODE, BSRHOLD, INSTR (writes -> PSLVERR)
    wire addr_ro = (PADDR == ADDR_STATUS) || (PADDR == ADDR_IDCODE) ||
                   (PADDR == ADDR_BSRHOLD) || (PADDR == ADDR_INSTR);
    wire wr_ctrl   = apb_write & (PADDR == ADDR_CTRL);
    wire wr_ir     = apb_write & (PADDR == ADDR_IR);
    wire wr_bsr    = apb_write & (PADDR == ADDR_BSR);
    wire wr_bypass = apb_write & (PADDR == ADDR_BYPASS);

    assign PREADY  = 1'b1;
    assign tck_o   = tick_q;
    assign tms_o   = tms_q;
    assign tdi_o   = tdi_q;
    assign trst_no = trst_n_q;
    assign tap_state_o = tap_state;
    assign tdo_o = tdo_q;

    // Control regs + tick pipeline + APB preloads
    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
            trst_n_q <= 1'b1;
            tms_q    <= 1'b0;
            tdi_q    <= 1'b0;
            tick_d   <= 1'b0;
            tick_q   <= 1'b0;
        end else begin
            tick_q <= tick_d;
            tick_d <= 1'b0;
            if (wr_ctrl) begin
                trst_n_q <= PWDATA[0];
                tms_q    <= PWDATA[1];
                tdi_q    <= PWDATA[2];
                if (PWDATA[3])
                    tick_d <= 1'b1;  // one internal TCK on next-next edge
            end
            if (wr_ir)
                ir_sr <= PWDATA[IR_WIDTH-1:0];
            if (wr_bsr)
                bsr <= PWDATA[DR_WIDTH-1:0];
            if (wr_bypass)
                bypass <= PWDATA[0];
        end
    end

    // APB read mux (combinational) + PSLVERR
    always @(*) begin
        PRDATA  = 32'h0000_0000;
        PSLVERR = 1'b0;
        if (PSEL && PENABLE) begin
            if (!addr_valid) begin
                PSLVERR = 1'b1;
                PRDATA  = 32'h0000_0000;
            end else if (apb_write && addr_ro) begin
                PSLVERR = 1'b1; // write to read-only register
                PRDATA  = 32'h0000_0000;
            end else begin
                PSLVERR = 1'b0;
                case (PADDR)
                    ADDR_CTRL:   PRDATA = {28'b0, 1'b0, tdi_q, tms_q, trst_n_q};
                    ADDR_STATUS: PRDATA = {20'b0, dr_sel, ir_hold, tick_q, tap_state, tdo_o};
                    ADDR_IR:     PRDATA = {{(32-IR_WIDTH){1'b0}}, ir_sr};
                    ADDR_BSR:    PRDATA = {{(32-DR_WIDTH){1'b0}}, bsr};
                    ADDR_IDCODE: PRDATA = idcode_sr;
                    ADDR_BYPASS: PRDATA = {31'b0, bypass};
                    ADDR_BSRHOLD:PRDATA = {{(32-DR_WIDTH){1'b0}}, bsr_hold};
                    ADDR_INSTR:  PRDATA = {28'b0, ir_hold};
                    default:     PRDATA = 32'h0000_0000;
                endcase
            end
        end
    end

    // Simple protocol checker (simulation only): PSEL must precede PENABLE.
    // synthesis translate_off
    reg psel_prev;
    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn)
            psel_prev <= 1'b0;
        else begin
            if (PENABLE && !PSEL && !psel_prev)
                $display("APB PROTOCOL VIOLATION @%0t: PENABLE without PSEL", $time);
            psel_prev <= PSEL;
        end
    end
    // synthesis translate_on

endmodule
