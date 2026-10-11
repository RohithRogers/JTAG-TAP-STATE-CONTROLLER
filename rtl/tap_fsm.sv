// tap_fsm.sv — IEEE 1149.1 16-state JTAG TAP controller FSM.
// Icarus-safe: plain Verilog-2001 style, no always_ff, no interfaces.
`timescale 1ns/1ps

module tap_fsm (
    input  wire       clk,      // PCLK (internal TCK ticks are enables on this clock)
    input  wire       rst_n,    // combined reset: PRESETn & TRST_N (active low)
    input  wire       tms,      // TMS sampled for this TCK tick
    input  wire       tick,     // 1-PCLK-wide internal TCK strobe: advance FSM once
    output reg  [3:0] state_o   // current TAP state encoding
);

    // State encoding
    localparam [3:0]
        TEST_LOGIC_RESET = 4'd0,
        RUN_TEST_IDLE    = 4'd1,
        SELECT_DR        = 4'd2,
        CAPTURE_DR       = 4'd3,
        SHIFT_DR         = 4'd4,
        EXIT1_DR         = 4'd5,
        PAUSE_DR         = 4'd6,
        EXIT2_DR         = 4'd7,
        UPDATE_DR        = 4'd8,
        SELECT_IR        = 4'd9,
        CAPTURE_IR       = 4'd10,
        SHIFT_IR         = 4'd11,
        EXIT1_IR         = 4'd12,
        PAUSE_IR         = 4'd13,
        EXIT2_IR         = 4'd14,
        UPDATE_IR        = 4'd15;

    // Next-state function (IEEE 1149.1)
    function [3:0] tap_next;
        input [3:0] cur;
        input       tms_in;
        begin
            case (cur)
                TEST_LOGIC_RESET: tap_next = tms_in ? TEST_LOGIC_RESET : RUN_TEST_IDLE;
                RUN_TEST_IDLE:    tap_next = tms_in ? SELECT_DR        : RUN_TEST_IDLE;
                SELECT_DR:        tap_next = tms_in ? SELECT_IR        : CAPTURE_DR;
                CAPTURE_DR:       tap_next = tms_in ? EXIT1_DR         : SHIFT_DR;
                SHIFT_DR:         tap_next = tms_in ? EXIT1_DR         : SHIFT_DR;
                EXIT1_DR:         tap_next = tms_in ? UPDATE_DR        : PAUSE_DR;
                PAUSE_DR:         tap_next = tms_in ? EXIT2_DR         : PAUSE_DR;
                EXIT2_DR:         tap_next = tms_in ? UPDATE_DR        : SHIFT_DR;
                UPDATE_DR:        tap_next = tms_in ? SELECT_DR        : RUN_TEST_IDLE;
                SELECT_IR:        tap_next = tms_in ? TEST_LOGIC_RESET : CAPTURE_IR;
                CAPTURE_IR:       tap_next = tms_in ? EXIT1_IR         : SHIFT_IR;
                SHIFT_IR:         tap_next = tms_in ? EXIT1_IR         : SHIFT_IR;
                EXIT1_IR:         tap_next = tms_in ? UPDATE_IR        : PAUSE_IR;
                PAUSE_IR:         tap_next = tms_in ? EXIT2_IR         : PAUSE_IR;
                EXIT2_IR:         tap_next = tms_in ? UPDATE_IR        : SHIFT_IR;
                UPDATE_IR:        tap_next = tms_in ? SELECT_DR        : RUN_TEST_IDLE;
                default:          tap_next = TEST_LOGIC_RESET;
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state_o <= TEST_LOGIC_RESET;
        else if (tick)
            state_o <= tap_next(state_o, tms);
    end

endmodule
