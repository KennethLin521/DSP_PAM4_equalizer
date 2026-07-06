`timescale 1ps/1ps

module dfe_tap (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable_adapt,      // HIGH to enable adaptation
    input  wire signed [15:0] error_val,         // 16-bit signed error from top level
    input  wire        [1:0]  prev_decision_code,// 2-bit decision code from previous cycle
    input  wire signed [15:0] prev_ideal_target, // Previous cycle's noiseless target voltage
    output wire signed [17:0] dfe_out            // 18-bit feedback correction signal
);

reg signed [15:0] dfe_acc; // 16-bit accumulator, [15:8] is integer weight, [7:0] is fractional

wire signed [7:0] active_dfe_weight = dfe_acc[15:8]; // extract first 8 bits as active integer weight

// LMS sign extraction 
wire error_sign = error_val[15];
// sign of previous decision 
// given 2'b11 (+3) and 2'b10 (+1) are positive, 2'b01 (-1) and 2'b00 (-3) are negative
// --> MSB of decision code directly represents sign 
wire prev_sign = prev_decision_code[1];

// sequential weight update 
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        //initialize DFE weight to 0.0
        dfe_acc <= 16'sh0000;
    end
    else if(enable_adapt) begin
        // SS-LMS for DFE
        // if sign matches (+/+, -/-), product is (+) -> add 1 LSB to cancel ISI
        // is sign mismatch (+/-), product is (-) -> subtract 1 LSB 
        if (error_sign == prev_sign) begin
            dfe_acc <= dfe_acc + 16'sd1;
        end else begin 
            dfe_acc <= dfe_acc - 16'sd1;
        end
    end
end

// combinational multiplication
// 8-bit weight * 16-bit target = 24-bit raw signed product 
wire signed [23:0] dfe_out_full = active_dfe_weight * prev_ideal_target;

// slice bottom 18 bits to match top level FFE adder tree
assign dfe_out = dfe_out_full[17:0];

endmodule