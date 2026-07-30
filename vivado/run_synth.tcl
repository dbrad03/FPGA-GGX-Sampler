open_project /home/darchb/Projects/FPGA-GGX-Sampler/vivado/ggx_floorplan/ggx_floorplan.xpr
reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1
puts "SYNTH_STATUS [get_property STATUS [get_runs synth_1]]"
puts "SYNTH_PROGRESS [get_property PROGRESS [get_runs synth_1]]"
