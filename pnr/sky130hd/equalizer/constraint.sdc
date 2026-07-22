# Timing constraints for equalizer_top on sky130hd.
# The RTL sim used a "10 unit" clock; sky130 liberty is in ns, so 10.0 ns
# = 100 MHz. Deliberately relaxed so timing closes on the first pass;
# tighten later once the flow is understood.

set clk_name   clk
# 20 ns / 50 MHz: the single-cycle LMS loop (multiply -> adder tree ->
# slicer -> error -> coefficient update) measures ~16.6 ns in sky130hd,
# so 10 ns was structurally impossible without pipelining the loop
# (delayed-LMS). Relaxed for clean closure; pipelining is a future task.
set clk_period 20.0

create_clock -name $clk_name -period $clk_period [get_ports $clk_name]

# Async reset release is guaranteed by usage (deasserted at a negedge,
# far from any rising clock edge), not by timing the giant rst_n fanout
# net. Without this, recovery checks on ~90 flop reset pins dominate wns.
set_false_path -from [get_ports rst_n]

# A little pessimism so the clock tree is built with margin.
set_clock_uncertainty 0.10 [get_clocks $clk_name]

# Model board/pad delays on the data pins (everything except the clock).
# NOTE: OpenSTA (OpenROAD's timing engine) does NOT implement the Synopsys
# command remove_from_collection. Use the Tcl lsearch idiom that ORFS's own
# example SDCs use to drop the clock port from the input list.
set clk_port [get_ports $clk_name]
set_input_delay  2.0 -clock $clk_name [lsearch -inline -all -not -exact [all_inputs] $clk_port]
set_output_delay 2.0 -clock $clk_name [all_outputs]
