`timescale 1ps/1ps

module dfe_tap (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable_adapt,      // HIGH to enable adaptation
    input  wire signed [15:0] error_val,         // 16-bit signed error from top level
    input  wire signed [15:0] prev_ideal_target, // Previous cycle's noiseless target voltage
    output wire signed [17:0] dfe_out            // 18-bit feedback correction signal
);

reg signed [15:0] dfe_acc; // Q8.8 accumulator: [15:8] integer, [7:0] fractional

// LMS sign extraction - both use the 2's complement MSB so the
// conventions agree (1 = negative). NOTE: do NOT use the PAM4
// decision_code MSB here: code 2'b11/2'b10 are the POSITIVE symbols,
// so code[1] is 1 for positive - the opposite polarity of a sign bit.
wire error_sign = error_val[15];
wire prev_sign  = prev_ideal_target[15];

// Saturation bounds so the accumulator clamps instead of wrapping
localparam signed [15:0] DFE_MAX = 16'sh7FFF;
localparam signed [15:0] DFE_MIN = 16'sh8000;

// SS-LMS for DFE: w <= w + mu * sign(error) * sign(prev_decision)
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // initialize DFE weight to 0.0, matching MATLAB dfe_weight = 0.0
        dfe_acc <= 16'sh0000;
    end
    else if (enable_adapt) begin
        if (error_sign == prev_sign) begin
            // same sign -> sign product is (+) -> step up
            if (dfe_acc != DFE_MAX)
                dfe_acc <= dfe_acc + 16'sd1;
        end else begin
            // opposite sign -> sign product is (-) -> step down
            if (dfe_acc != DFE_MIN)
                dfe_acc <= dfe_acc - 16'sd1;
        end
    end
end

// Multiply with the FULL Q8.8 weight, then realign by >>> 8, same as the
// FFE taps. S16(Q8.8) * S16 = S32(Q24.8); [25:8] is the integer part,
// bounded well inside 18 bits (|weight| < 128, |target| <= 93).
wire signed [31:0] dfe_out_full = dfe_acc * prev_ideal_target;
assign dfe_out = dfe_out_full[25:8];

endmodule
