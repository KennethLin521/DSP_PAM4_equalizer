`timescale 1ns / 1ps 

module ffe_tap ( 
    input  wire               clk,
    input  wire               rst_n, 
    input  wire               enable_adapt, // HIGH to enable SS-LMS
    input  wire signed [7:0]  rx_data,      // 8-bit quantized data from ADC 
    input  wire signed [15:0] error_val,    // Signed error from slicer 
    output wire signed [15:0] tap_out       // Product of data and top 8 bits of coefficients 
); 

    // 16-bit accumulator: [15:8] is integer, [7:0] is fractional
    reg signed [15:0] coef_acc; 

    // Slice off top 8 bits of accumulator to use as filter weight
    wire signed [7:0] active_weight = coef_acc[15:8]; 

    // 8-bit data * 8-bit weight = 16-bit output 
    assign tap_out = rx_data * active_weight; 

    // Extract sign bits (MSB in 2's complement)
    wire data_sign  = rx_data[7];
    wire error_sign = error_val[15];

    // SS-LMS logic 
    always @(posedge clk or negedge rst_n) begin 
        if (!rst_n) begin 
            // Initialize coeff accumulator to 1.0 in 8.8 fixed point (0x0100)
            coef_acc <= 16'h0100;
        end else if (enable_adapt) begin
            // Sign match = product of sign is (+) = subtract 1 LSB from accumulator
            // Sign mismatch = product of sign is (-) = add 1 LSB to accumulator
            if (data_sign == error_sign) begin 
                coef_acc <= coef_acc - 16'd1;
            end else begin 
                coef_acc <= coef_acc + 16'd1;
            end 
        end
    end 
    
endmodule
