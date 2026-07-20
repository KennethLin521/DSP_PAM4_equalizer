# ============================================================
#  ORFS design config: 3-tap FFE + 1-tap DFE PAM4 equalizer
#  Platform: sky130hd (SkyWater 130nm high-density std cells)
#
#  Assumes this repo is cloned to ~/DSP_PAM4_equalizer inside the
#  OpenROAD-flow-scripts Codespace. Run from ORFS flow/ dir:
#    make DESIGN_CONFIG=$HOME/DSP_PAM4_equalizer/pnr/sky130hd/equalizer/config.mk <stage>
# ============================================================

export DESIGN_NICKNAME = equalizer
export DESIGN_NAME     = equalizer_top
export PLATFORM        = sky130hd

# The 4 synthesizable DUT modules (NOT the *_tb testbenches).
# ORFS runs its OWN Yosys synthesis on these sources.
export VERILOG_FILES = $(HOME)/DSP_PAM4_equalizer/src/Equalizer/ffe_tap.v \
                       $(HOME)/DSP_PAM4_equalizer/src/Equalizer/dfe_tap.v \
                       $(HOME)/DSP_PAM4_equalizer/src/Equalizer/pam4_slicer.v \
                       $(HOME)/DSP_PAM4_equalizer/src/Equalizer/equalizer_top.v

export SDC_FILE = $(HOME)/DSP_PAM4_equalizer/pnr/sky130hd/equalizer/constraint.sdc

# First-pass floorplan sized from a utilization target (easier + robust
# than fixed coordinates). ~28000 um^2 of cells / 0.40 = ~70000 um^2 core,
# a ~265 x 265 um square. 40% util leaves comfortable routing room.
export CORE_UTILIZATION = 40
export PLACE_DENSITY    = 0.55
