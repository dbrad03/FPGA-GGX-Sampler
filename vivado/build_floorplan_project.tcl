# Create a Vivado PROJECT (.xpr) for GUI floorplanning of the single-lane GGX
# datapath. Same source set as sim/phase0_enum.tcl (which synthesizes the FOLD
# cleanly), out-of-context (axis_ggx_control has no board pins), 5 ns clock.
# Open with:  vivado vivado/ggx_floorplan/ggx_floorplan.xpr
set origin /home/darchb/Projects/FPGA-GGX-Sampler
create_project ggx_floorplan $origin/vivado/ggx_floorplan -part xc7z020clg400-1 -force

add_files -norecurse [list \
  $origin/hdl/axis_ggx_control.sv \
  $origin/hdl/axis_skid_buffer.sv \
  $origin/hdl/axis_fifo_2deep.sv \
  $origin/hdl/axis_ggx_event_basis.sv \
  $origin/hdl/axis_top_lvl_sampler.sv \
  $origin/hdl/axis_ggx_projected_area.sv \
  $origin/hdl/axis_ggx_reproject_normalize.sv \
  $origin/hdl/axis_fixed_norm3.sv \
  $origin/hdl/axis_fixed_sqrt.sv \
  $origin/hdl/axis_fixed_div.sv \
  $origin/hdl/axis_fixed_inv_sqrt_nodsp.sv \
  $origin/hdl/axis_fixed_inv_sqrt_folded.sv \
  $origin/hdl/axis_sobol2d_stateless.sv \
  $origin/hdl/axis_nested_uniform_scramble.sv \
  $origin/hdl/axis_hash_combine_2d.sv \
  $origin/hdl/axis_trig_lut.sv \
  $origin/hdl/axis_pre_ggx_sampler.v \
  $origin/sim/ggx_trig_rom.mem ]

set_property top axis_ggx_control [current_fileset]
update_compile_order -fileset sources_1

add_files -fileset constrs_1 -norecurse $origin/vivado/ggx_clk.xdc

# Out-of-context synthesis (no I/O buffer insertion -> matches the OOC P&R flow
# that produced the WNS -1.454 fold baseline; the design is a module, not a pinned top).
set_property -name {STEPS.SYNTH_DESIGN.ARGS.MORE OPTIONS} -value {-mode out_of_context} \
  -objects [get_runs synth_1]

puts "PROJECT_CREATED"
