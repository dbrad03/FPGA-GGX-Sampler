# Create a Vivado PROJECT (.xpr) for GUI floorplanning of the single-lane GGX
# datapath. Same source set as sim/phase0_enum.tcl (which synthesizes the FOLD
# cleanly), out-of-context (axis_ggx_control has no board pins), 5 ns clock.
# Open with:  vivado vivado/ggx_floorplan/ggx_floorplan.xpr
set origin /home/darchb/Projects/FPGA-GGX-Sampler
create_project ggx_floorplan $origin/vivado/ggx_floorplan -part xc7z020clg400-1 -force

source $origin/sim/rtl_sources.tcl
add_files -norecurse [concat $RTL_SOURCES(axis_ggx_control) \
                             [list $origin/sim/ggx_trig_rom.mem]]

# axis_pre_ggx_sampler keeps its .v extension but imports ggx_latency_pkg, which
# is SystemVerilog. Vivado infers Verilog-2001 from the extension and would
# reject the package scope resolution, so set the language explicitly.
set_property file_type SystemVerilog [get_files $origin/hdl/axis_pre_ggx_sampler.v]

set_property top axis_ggx_control [current_fileset]
update_compile_order -fileset sources_1

add_files -fileset constrs_1 -norecurse $origin/vivado/ggx_clk.xdc

# Out-of-context synthesis (no I/O buffer insertion -> matches the OOC P&R flow
# that produced the WNS -1.454 fold baseline; the design is a module, not a pinned top).
set_property -name {STEPS.SYNTH_DESIGN.ARGS.MORE OPTIONS} -value {-mode out_of_context} \
  -objects [get_runs synth_1]

puts "PROJECT_CREATED"
