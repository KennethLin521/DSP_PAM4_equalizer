`timescale 1ps/1ps

module tb_equalizer;
    parameter NUM_SAMPLES = 5000; // Matching your MATLAB config
    parameter CLK_PERIOD  = 10;   // 10ns cycle time

    // inputs to DUT
    reg clk;
    reg rst_n;
    reg enable_adapt;
    reg signed [7:0] rx_data;

    // outputs from DUT 
    wire signed [15:0] eq_output;

    // testbench internal memory for MATLAB stimuli
    reg [7:0] rx_mem [0:NUM_SAMPLES - 1];
    integer out_file;
    integer i;

    // instantiate top-level module 
    equalizer_top u_dut (
        .clk(clk),
        .rst_n(rst_n),
        .enable_adapt(enable_adapt),
        .rx_data(rx_data),
        .eq_output(eq_output)
    );

    // clock gen
    initial begin
        clk = 0;
        forever #(CLK_PERIOD / 2) clk = ~clk; // 50% duty cycle
    end

    // for dumping to GTKWave
    initial begin
        $dumpfile("equalizer_sim.vcd"); // Creates the waveform file
        $dumpvars(0, tb_equalizer);     // Dumps ALL signals in the testbench and DUT
    end

    // main co-simulation flow 
    initial begin
        // 1. read MATLAB .hex dump
        $readmemh("rx_stimulus.hex", rx_mem);

        // 2. open output file to dump output
        out_file = $fopen("eq_output_dump.hex", "w");

        // 3. initial reset state 
        rst_n = 0;
        enable_adapt = 0; // keep off to check static behavior 
        rx_data = 8'sd0;

        #(CLK_PERIOD * 5);
        @(negedge clk);
        rst_n = 1; // release reset

        #(CLK_PERIOD * 5);

        // 4. feed stimuli 
        $display(" ----- starting RTL co-sim -----");
        for(i = 0; i < NUM_SAMPLES; i = i + 1) begin
            rx_data = rx_mem[i];

            @(posedge clk); // sample occurs on clk edge

            // log output to file 
            $fdisplay(out_file, "%h", eq_output);

            // turning adaptation on
            if (i == 500) begin 
                $display("Enabling adaptation loop at sample %d", i);
                enable_adapt = 1;
            end 
        end

        // 5. wrap up
        #(CLK_PERIOD * 10);
        $fclose(out_file);
        $display("----- co-sim complete -----");
        $finish;
    end
    
endmodule