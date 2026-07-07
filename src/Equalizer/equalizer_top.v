`timescale 1ps/1ps

module equalizer_top (
    input wire clk,
    input wire rst_n,
    input wire enable_adapt,
    input wire signed [7:0] rx_data,
    output wire signed [15:0] eq_output
);

// FFE delay line
// holding 2 previous 8-bit samples for 3-tap FFE
reg signed [7:0] rx_data_d1;
reg signed [7:0] rx_data_d2;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        //clear taps on reset
        rx_data_d1 <= 8'sd0;
        rx_data_d2 <= 8'sd0;
    end else begin
        // shift samples down the taps
        rx_data_d1 <= rx_data;
        rx_data_d2 <= rx_data_d1;
    end
end

// instantiate 3 FFE taps
wire signed [15:0] tap0_out;
wire signed [15:0] tap1_out;
wire signed [15:0] tap2_out;

wire signed [15:0] error_val; // feedback wire for slicer

    // FFE tap 0 - main cursor, boots at weight 1.0 (Q8.8 0x0100)
    // matching MATLAB ffe_weights = [1.0; 0.0; 0.0]
    ffe_tap #(.INIT_COEF(16'sh0100)) u_ffe_tap0 (
        .clk (clk),
        .rst_n (rst_n),
        .enable_adapt (enable_adapt),
        .rx_data (rx_data),       // Current sample
        .error_val (error_val),     // Shared feedback error wire
        .tap_out (tap0_out)
    );
    // FFE tap 1 - post-cursor, boots at 0.0
    ffe_tap #(.INIT_COEF(16'sh0000)) u_ffe_tap1 (
        .clk (clk),
        .rst_n (rst_n),
        .enable_adapt (enable_adapt),
        .rx_data (rx_data_d1),    // 1-cycle old sample
        .error_val (error_val),
        .tap_out (tap1_out)
    );
    // FFE tap 2 - post-cursor, boots at 0.0
    ffe_tap #(.INIT_COEF(16'sh0000)) u_ffe_tap2 (
        .clk (clk),
        .rst_n (rst_n),
        .enable_adapt (enable_adapt),
        .rx_data (rx_data_d2),    // 2-cycles old sample
        .error_val (error_val),
        .tap_out(tap2_out)
    );

   wire signed [17:0] ffe_out; // 18 bits to prevent overflow
   assign ffe_out = tap0_out + tap1_out + tap2_out; // summation

    // DFE loop subtraction
    wire signed [17:0] dfe_out;
    wire signed [17:0] eq_signal;

    // equalized signal = FFE output - DFE feedback
    assign eq_signal = ffe_out - dfe_out;

    // connect top level pin - saturate 18 -> 16 bits instead of
    // truncating, so overrange values clip rather than wrap sign
    assign eq_output = (eq_signal > 18'sd32767)  ? 16'sd32767 :
                       (eq_signal < -18'sd32768) ? -16'sd32768 :
                       eq_signal[15:0];

    // PAM4 slicer
    wire [1:0] decision_code; // recovered PAM4 symbol (for BER checks later)
    wire signed [15:0] ideal_target;

    pam4_slicer u_slicer (
        .eq_signal (eq_signal),
        .decision_code (decision_code),
        .ideal_target (ideal_target)
    );

    // error calculation for LMS
    // error = actual equalized signal - ideal target signal
    // first sign extend ideal_target from 16 to 18 bits to match
    wire signed [17:0] raw_error = eq_signal - {{2{ideal_target[15]}}, ideal_target};

    // saturate error into 16 bits (wrapping here would flip the sign
    // the SS-LMS update relies on)
    assign error_val = (raw_error > 18'sd32767)  ? 16'sd32767 :
                       (raw_error < -18'sd32768) ? -16'sd32768 :
                       raw_error[15:0];

    // DFE pipeline
    // need to capture previous decision --> capture slicer output in register
    reg signed [15:0] prev_ideal_target;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_ideal_target <= 16'sd0;
        end else begin
            prev_ideal_target <= ideal_target;
        end
    end

    // instantiate DFE module
    dfe_tap u_dfe_tap (
        .clk (clk),
        .rst_n (rst_n),
        .enable_adapt (enable_adapt),
        .error_val (error_val),
        .prev_ideal_target (prev_ideal_target),
        .dfe_out (dfe_out)
    );


endmodule
