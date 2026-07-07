`timescale 1ps/1ps

module ffe_tap #(
    // Reset value of the coefficient in Q8.8 fixed point.
    // Each instance gets its own starting weight (tap0 = 1.0, others = 0.0),
    // matching the MATLAB init ffe_weights = [1.0; 0.0; 0.0]
    parameter signed [15:0] INIT_COEF = 16'sh0000
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable_adapt, // HIGH to enable SS-LMS
    input  wire signed [7:0]  rx_data,      // 8-bit quantized data from ADC
    input  wire signed [15:0] error_val,    // Signed error from slicer
    output wire signed [15:0] tap_out       // data * coefficient, integer LSB units
);

    // 16-bit accumulator in Q8.8: [15:8] is integer, [7:0] is fractional
    reg signed [15:0] coef_acc;

    // Multiply with the FULL Q8.8 coefficient, then shift right by 8 to
    // drop the fractional alignment. Taking only coef_acc[15:8] would throw
    // away the fraction entirely - the channel needs fractional weights
    // (~0.3, ~0.1), which an integer-only weight can never represent.
    // S8 * S16(Q8.8) = S24(Q16.8); [23:8] is the integer part = >>> 8
    wire signed [23:0] product = rx_data * coef_acc;
    assign tap_out = product[23:8];

    // Sign bits (MSB in 2's complement): 1 = negative
    wire data_sign  = rx_data[7];
    wire error_sign = error_val[15];

    // Saturation bounds so the accumulator clamps instead of wrapping
    localparam signed [15:0] COEF_MAX = 16'sh7FFF; // +127.996
    localparam signed [15:0] COEF_MIN = 16'sh8000; // -128.0

    // SS-LMS: w <= w - mu * sign(error) * sign(data), mu = 1 LSB = 1/256
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            coef_acc <= INIT_COEF;
        end else if (enable_adapt) begin
            if (data_sign == error_sign) begin
                // Same sign -> sign product is (+) -> step down
                if (coef_acc != COEF_MIN)
                    coef_acc <= coef_acc - 16'sd1;
            end else begin
                // Opposite sign -> sign product is (-) -> step up
                if (coef_acc != COEF_MAX)
                    coef_acc <= coef_acc + 16'sd1;
            end
        end
    end

endmodule
