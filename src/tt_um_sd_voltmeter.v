// ============================================================================
//  tt_um_kalnozolsm_tt_um_sd_voltmeter.v  —  Sigma-delta bitstream voltmeter, 3-digit LED display
//  Tiny Tapeout project (rename the top module to tt_um_<your_project_name>)
//
//  WHAT IT DOES
//    The other chip (sigma-delta modulator) sends a fast stream of 1s and 0s
//    on ONE pin. The fraction of time the pin is 1 equals Vin / 3.3 V.
//    This chip:
//      1. synchronises the pin to its own 32 MHz clock          (sync2)
//      2. counts how many clock ticks the pin is 1 during 0.1 s,
//         and turns that count into a 9-bit code 0..511         (sd_decimator)
//      3. converts the code to hundredths of a volt 0..329      (top level)
//      4. splits that into 3 decimal digits                     (bin2bcd)
//      5. drives 3 seven-segment digits one at a time, 1 kHz    (display_mux)
//    Result: the display shows X.XX volts and updates 10 times per second.
//
//  PINS (change here if the team picks other pins)
//    ui_in[0]      sigma-delta bitstream input
//    uo_out[7:0]   segments {dp, g, f, e, d, c, b, a}
//    uio_out[2:0]  digit enables: [2] = left digit (volts, with dot),
//                                 [1] = middle (0.1 V), [0] = right (0.01 V)
//
//  EVERYTHING ADJUSTABLE IS A PARAMETER IN THE TOP MODULE:
//    CLK_HZ, UPDATE_HZ, SWITCH_HZ, CODE_BITS, VREF_CV, SEG_ON, DIG_ON
//
//  The bit rate of the other chip does NOT need to be known: the decimator
//  measures how long the pin is high, not individual bits. It only has to be
//  clearly slower than CLK_HZ (a few MHz or less is comfortable at 32 MHz).
//
//  Plain Verilog-2005.
// ============================================================================
`timescale 1ns/1ps
`default_nettype none
/* verilator lint_off DECLFILENAME */   // several modules in one file on purpose

module tt_um_kalnozolsm_tt_um_sd_voltmeter #(
    parameter CLK_HZ    = 32_000_000,  // chip clock
    parameter UPDATE_HZ = 10,          // new readings per second
    parameter SWITCH_HZ = 1_000,       // display digit-switching rate
    parameter CODE_BITS = 9,           // ADC resolution: 9 bits -> 0..511
    parameter VREF_CV   = 330,         // full scale in hundredths of a volt (3.30 V), max 999
    parameter SEG_ON    = 1,           // level that lights a segment (1 or 0)
    parameter DIG_ON    = 1            // level that enables a digit (1 or 0)
)(
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n            // active-low reset (Tiny Tapeout standard)
);

    // ---- 1) bring the asynchronous input into our clock domain ----------
    wire bit_s;
    sync2 u_sync (.clk(clk), .rst_n(rst_n), .d(ui_in[0]), .q(bit_s));

    // ---- 2) count for 0.1 s -> 9-bit code -------------------------------
    wire [CODE_BITS-1:0] code;
    wire                 code_valid;
    sd_decimator #(
        .WINDOW   (CLK_HZ / UPDATE_HZ),
        .CODE_BITS(CODE_BITS)
    ) u_dec (
        .clk(clk), .rst_n(rst_n), .bit_in(bit_s),
        .code(code), .code_valid(code_valid)
    );

    // ---- 3) code -> hundredths of a volt, rounded -----------------------
    //      cv = round(code * VREF_CV / 2^CODE_BITS)
    wire [31:0] cv_full = (code * VREF_CV + (1 << (CODE_BITS - 1))) >> CODE_BITS;
    wire [9:0]  cv      = cv_full[9:0];

    // ---- 4) binary -> 3 BCD digits, stored when a new reading is ready --
    wire [11:0] bcd_new;
    bin2bcd u_bcd (.bin(cv), .bcd(bcd_new));

    reg [11:0] bcd;                    // {volts, tenths, hundredths}
    always @(posedge clk) begin
        if (!rst_n)          bcd <= 12'd0;
        else if (code_valid) bcd <= bcd_new;
    end

    // ---- 5) multiplexed display -----------------------------------------
    wire [7:0] seg;
    wire [2:0] dig;
    display_mux #(
        .DIV   (CLK_HZ / SWITCH_HZ),
        .SEG_ON(SEG_ON),
        .DIG_ON(DIG_ON)
    ) u_disp (
        .clk(clk), .rst_n(rst_n), .bcd(bcd),
        .seg(seg), .dig(dig)
    );

    assign uo_out  = seg;
    assign uio_out = {5'b00000, dig};
    assign uio_oe  = 8'b0000_0111;     // uio[2:0] are outputs, the rest inputs

    // keep the linter quiet about pins we do not use
    // (also cv_full[31:10]: the result never exceeds 999, so those bits are always 0)
    wire _unused = &{ena, ui_in[7:1], uio_in, cv_full[31:10], 1'b0};

endmodule


// ============================================================================
//  sync2 — two flip-flops in series. The input comes from another chip with
//  its own timing, so it can change at any moment, even right at our clock
//  edge. The second flip-flop gives the first one time to settle
//  (avoids metastability).
// ============================================================================
module sync2 (
    input  wire clk,
    input  wire rst_n,
    input  wire d,
    output wire q
);
    reg s1, s2;
    always @(posedge clk) begin
        if (!rst_n) begin s1 <= 1'b0; s2 <= 1'b0; end
        else        begin s1 <= d;    s2 <= s1;   end
    end
    assign q = s2;
endmodule


// ============================================================================
//  sd_decimator — turns the bitstream into a number.
//
//  For WINDOW clock ticks (0.1 s) it counts the ticks where the input is 1.
//      code = count / STEP,   STEP = WINDOW / 2^CODE_BITS
//  With 32 MHz and 10 Hz: WINDOW = 3 200 000, STEP = 6250.
//
//  Dividing by 6250 is done without a divider: a small counter (sub_cnt)
//  counts 1s up to 6250; every time it wraps, acc goes up by one. At the end
//  of the window acc IS the 9-bit code. Then both counters restart.
//  sub_cnt starts at STEP/2 instead of 0, so the result is rounded to the
//  nearest code instead of always rounded down.
//
//  Input always 0 -> code 0; input always 1 -> code 511 (clamped).
//  If WINDOW is not divisible by 2^CODE_BITS the result is slightly off in
//  gain (rounded STEP); with the default numbers it is exact.
// ============================================================================
module sd_decimator #(
    parameter WINDOW    = 3_200_000,
    parameter CODE_BITS = 9
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 bit_in,
    output reg  [CODE_BITS-1:0] code,
    output reg                  code_valid      // 1-clock pulse per new code
);
    localparam integer STEP = WINDOW >> CODE_BITS;
    localparam integer WW   = $clog2(WINDOW + 1);       // window counter width
    localparam integer SW   = $clog2(STEP + 1);         // step counter width

    // constants, first as 32-bit integers, then cut to the exact counter width
    localparam integer WIN_LAST_I  = WINDOW - 1;
    localparam integer STEP_LAST_I = STEP - 1;
    localparam integer SUB_START_I = STEP / 2;          // start halfway -> rounds to nearest
    localparam integer CODE_MAX_I  = (1 << CODE_BITS) - 1;
    localparam [WW-1:0]      WIN_LAST  = WIN_LAST_I[WW-1:0];
    localparam [SW-1:0]      STEP_LAST = STEP_LAST_I[SW-1:0];
    localparam [SW-1:0]      SUB_START = SUB_START_I[SW-1:0];
    localparam [CODE_BITS:0] CODE_MAX  = CODE_MAX_I[CODE_BITS:0];

    reg [WW-1:0]      win_cnt;                    // position in the window
    reg [SW-1:0]      sub_cnt;                    // 1s counted toward next step
    reg [CODE_BITS:0] acc;                        // code being built (1 spare bit)

    always @(posedge clk) begin
        if (!rst_n) begin
            win_cnt    <= {WW{1'b0}};
            sub_cnt    <= SUB_START;
            acc        <= {(CODE_BITS+1){1'b0}};
            code       <= {CODE_BITS{1'b0}};
            code_valid <= 1'b0;
        end else if (win_cnt == WIN_LAST) begin
            // end of window: publish result, start again
            win_cnt    <= {WW{1'b0}};
            sub_cnt    <= SUB_START;
            acc        <= {(CODE_BITS+1){1'b0}};
            code       <= (acc > CODE_MAX) ? CODE_MAX[CODE_BITS-1:0] : acc[CODE_BITS-1:0];
            code_valid <= 1'b1;
        end else begin
            win_cnt    <= win_cnt + 1'b1;
            code_valid <= 1'b0;
            if (bit_in) begin
                if (sub_cnt == STEP_LAST) begin
                    sub_cnt <= {SW{1'b0}};
                    acc     <= acc + 1'b1;
                end else begin
                    sub_cnt <= sub_cnt + 1'b1;
                end
            end
        end
    end
endmodule


// ============================================================================
//  bin2bcd — binary 0..999 to three decimal digits (double dabble):
//  shift left one bit at a time; before each shift, add 3 to any digit
//  that is 5 or more.
// ============================================================================
module bin2bcd (
    input  wire [9:0]  bin,
    output reg  [11:0] bcd                       // {hundreds, tens, ones}
);
    reg [21:0] s;                                // [21:10] BCD, [9:0] binary
    integer i;
    always @* begin
        s = {12'd0, bin};
        for (i = 0; i < 10; i = i + 1) begin
            if (s[13:10] >= 4'd5) s[13:10] = s[13:10] + 4'd3;
            if (s[17:14] >= 4'd5) s[17:14] = s[17:14] + 4'd3;
            if (s[21:18] >= 4'd5) s[21:18] = s[21:18] + 4'd3;
            s = s << 1;
        end
        bcd = s[21:10];
    end
endmodule


// ============================================================================
//  display_mux — lights one digit at a time, switching every DIV clocks
//  (32 MHz / 32 000 = 1 kHz, so each digit flashes ~333 times per second).
//  A short blank gap (DIV/32) at each switch prevents faint "ghost" digits
//  while the transistors turn off. Outputs are registered (clean pins).
// ============================================================================
module display_mux #(
    parameter DIV    = 32_000,
    parameter SEG_ON = 1,
    parameter DIG_ON = 1
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [11:0] bcd,                      // {volts, tenths, hundredths}
    output reg  [7:0]  seg,                      // {dp,g,f,e,d,c,b,a}
    output reg  [2:0]  dig                       // one-hot digit enable
);
    localparam integer DW      = $clog2(DIV + 1);
    localparam integer DIV_LAST_I = DIV - 1;
    localparam integer BLANK_I    = (DIV / 32 < 1) ? 1 : DIV / 32;
    localparam [DW-1:0] DIV_LAST  = DIV_LAST_I[DW-1:0];
    localparam [DW-1:0] BLANK     = BLANK_I[DW-1:0];

    reg [DW-1:0] div_cnt;
    reg [1:0]    sel;                            // 2 = left, 1 = middle, 0 = right

    always @(posedge clk) begin
        if (!rst_n) begin
            div_cnt <= {DW{1'b0}};
            sel     <= 2'd0;
        end else if (div_cnt == DIV_LAST) begin
            div_cnt <= {DW{1'b0}};
            sel     <= (sel == 2'd2) ? 2'd0 : sel + 2'd1;
        end else begin
            div_cnt <= div_cnt + 1'b1;
        end
    end

    // pick the digit for the current position
    reg [3:0] digit;
    reg       dp;
    always @* begin
        case (sel)
            2'd2:    begin digit = bcd[11:8]; dp = 1'b1; end   // volts + dot
            2'd1:    begin digit = bcd[7:4];  dp = 1'b0; end
            default: begin digit = bcd[3:0];  dp = 1'b0; end
        endcase
    end

    // seven-segment pattern {g,f,e,d,c,b,a}, 1 = segment lit
    reg [6:0] pat;
    always @* begin
        case (digit)
            4'd0: pat = 7'b0111111;
            4'd1: pat = 7'b0000110;
            4'd2: pat = 7'b1011011;
            4'd3: pat = 7'b1001111;
            4'd4: pat = 7'b1100110;
            4'd5: pat = 7'b1101101;
            4'd6: pat = 7'b1111101;
            4'd7: pat = 7'b0000111;
            4'd8: pat = 7'b1111111;
            4'd9: pat = 7'b1101111;
            default: pat = 7'b1000000;           // "-" (should never appear)
        endcase
    end

    wire       blank  = (div_cnt < BLANK);
    wire [7:0] seg_hi = blank ? 8'b0 : {dp, pat};
    wire [2:0] dig_hi = blank ? 3'b0 : (3'b001 << sel);

    always @(posedge clk) begin
        if (!rst_n) begin
            seg <= SEG_ON ? 8'h00 : 8'hFF;
            dig <= DIG_ON ? 3'b000 : 3'b111;
        end else begin
            seg <= SEG_ON ? seg_hi : ~seg_hi;
            dig <= DIG_ON ? dig_hi : ~dig_hi;
        end
    end
endmodule

`default_nettype wire
