`timescale 1ps/1ps

module pam4_slicer (
    input wire signed [17:0] eq_signal, // from FFE adder tree
    output reg [1:0] decision_code, // 2-bit digital representation from PAM4 symbol
    output reg signed [15:0] ideal_target // noiseless ref. volage in ADC LSB
);

    // define the slicing thresholds based on 31 LSBs/Volt
    localparam signed [17:0] TH_POS  = 18'sd62;   //  2.0 Volts
    localparam signed [17:0] TH_ZERO = 18'sd0;    //  0.0 Volts
    localparam signed [17:0] TH_NEG  = -18'sd62;  // -2.0 Volts

    // define ideal noiseless target output code
    localparam signed [15:0] REF_P3  = 16'sd93;   //  3.0 Volts
    localparam signed [15:0] REF_P1  = 16'sd31;   //  1.0 Volts
    localparam signed [15:0] REF_N1  = -16'sd31;  // -1.0 Volts
    localparam signed [15:0] REF_N3  = -16'sd93;  // -3.0 Volts

    // Combinational evaluation block
    always @(*) begin
        if (eq_signal >= TH_POS) begin
            decision_code = 2'b11;  // Represents +3
            ideal_target  = REF_P3;
        end 
        else if (eq_signal >= TH_ZERO && eq_signal < TH_POS) begin
            decision_code = 2'b10;  // Represents +1
            ideal_target  = REF_P1;
        end 
        else if (eq_signal >= TH_NEG && eq_signal < TH_ZERO) begin
            decision_code = 2'b01;  // Represents -1
            ideal_target  = REF_N1;
        end 
        else begin
            decision_code = 2'b00;  // Represents -3
            ideal_target  = REF_N3;
        end
    end
endmodule