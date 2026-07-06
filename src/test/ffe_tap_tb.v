`timescale 1ns / 1ps

module ffe_tap_tb;

    // Inputs to the Device Under Test (DUT)
    reg         clk;
    reg         rst_n;
    reg         enable_adapt;
    reg  [7:0]  rx_data;
    reg  [15:0] error_val;

    // Outputs from the DUT
    wire [15:0] tap_out;

    // Instantiate the Device Under Test
    ffe_tap u_ffe_tap (
        .clk(clk),
        .rst_n(rst_n),
        .enable_adapt(enable_adapt),
        .rx_data(rx_data),
        .error_val(error_val),
        .tap_out(tap_out)
    );

    // 1. Clock Generation (50MHz for easy viewing)
    always #10 clk = ~clk;

    initial begin
        // Initialize signals
        clk = 0;
        rst_n = 0;
        enable_adapt = 0;
        rx_data = 8'd0;
        error_val = 16'd0;

        // Setup waveform dumping for GTKWave
        $dumpfile("ffe_tap_wave.vcd");
        $dumpvars(0, ffe_tap_tb);

        // 2. Apply Reset
        #25;
        rst_n = 1; // Release reset
        #20;
        
        // 3. Test Case 1: Enable adaptation, apply MATCHING signs
        // Both rx_data and error_val are positive (Sign bits = 0)
        // Expectation: Accumulator should subtract 1 LSB every clock edge
        $display("--- Test Case 1: Matching Positive Signs ---");
        enable_adapt = 1;
        rx_data   = 8'sb0000_1010;   // +10
        error_val = 16'sb0000_0000_0000_0101; // +5
        #40; // Let it run for 2 clock cycles

        // 4. Test Case 2: Apply MISMATCHING signs
        // rx_data is positive (0), error_val is negative (1)
        // Expectation: Accumulator should add 1 LSB every clock edge
        $display("--- Test Case 2: Mismatching Signs ---");
        rx_data   = 8'sb0000_1010;   // +10 (Positive)
        error_val = 16'sb1000_0000_0000_0101; // Negative
        #40; // Let it run for 2 clock cycles

        // 5. Test Case 3: Disable adaptation
        // Expectation: Accumulator frozen, output remains completely static
        $display("--- Test Case 3: Disable Adaptation ---");
        enable_adapt = 0;
        rx_data   = 8'sb0000_0001;   // Change data to check multiplier
        #40;

        $display("Simulation complete.");
        $finish;
    end

endmodule
