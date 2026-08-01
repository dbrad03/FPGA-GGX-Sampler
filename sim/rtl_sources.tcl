# GENERATED FILE -- DO NOT EDIT.
#
# Regenerate:  python sim/sources.py --emit-tcl
# Verify:      python sim/sources.py --check-tcl
# Source of truth: sim/sources.py
#
# Paths resolve relative to THIS file, so it can be sourced from sim/ or
# vivado/ alike. Every list leads with ggx_latency_pkg.sv, because a
# SystemVerilog package must be analyzed before the modules importing it.
#
# Read these with `read_verilog -sv`, including axis_pre_ggx_sampler.v:
# it keeps a .v extension but imports ggx_latency_pkg, and Vivado would
# otherwise infer Verilog-2001 from the extension and reject the package
# scope resolution.

set _rtl_dir [file normalize [file join [file dirname [info script]] .. hdl]]

# The trig LUT's ROM is gitignored, so a fresh clone or worktree lacks it.
# Vivado does NOT error on a missing $readmem file -- it warns, leaves the
# ROM uninitialized and constant-folds the LUT away, then reports timing for
# a design 12 DSPs lighter than the real one. Fail loudly instead.
set _trig_rom [file join [file dirname [info script]] ggx_trig_rom.mem]
if {![file exists $_trig_rom]} {
  error "missing $_trig_rom -- run: python sim/gen_roms.py"
}

set RTL_SOURCES(axis_cordic_normalize) [list \
  $_rtl_dir/ggx_latency_pkg.sv \
  $_rtl_dir/axis_cordic_normalize.sv \
]

set RTL_SOURCES(axis_fixed_div) [list \
  $_rtl_dir/ggx_latency_pkg.sv \
  $_rtl_dir/axis_fixed_div.sv \
]

set RTL_SOURCES(axis_fixed_inv_sqrt) [list \
  $_rtl_dir/ggx_latency_pkg.sv \
  $_rtl_dir/axis_fixed_inv_sqrt.sv \
]

set RTL_SOURCES(axis_fixed_inv_sqrt_folded) [list \
  $_rtl_dir/ggx_latency_pkg.sv \
  $_rtl_dir/axis_fixed_inv_sqrt_folded.sv \
]

set RTL_SOURCES(axis_fixed_inv_sqrt_nodsp) [list \
  $_rtl_dir/ggx_latency_pkg.sv \
  $_rtl_dir/axis_fixed_sqrt.sv \
  $_rtl_dir/axis_fixed_div.sv \
  $_rtl_dir/axis_fixed_inv_sqrt_nodsp.sv \
]

set RTL_SOURCES(axis_fixed_norm3) [list \
  $_rtl_dir/ggx_latency_pkg.sv \
  $_rtl_dir/axis_fixed_norm3.sv \
  $_rtl_dir/axis_fixed_sqrt.sv \
  $_rtl_dir/axis_fixed_div.sv \
  $_rtl_dir/axis_fixed_inv_sqrt_nodsp.sv \
]

set RTL_SOURCES(axis_fixed_sqrt) [list \
  $_rtl_dir/ggx_latency_pkg.sv \
  $_rtl_dir/axis_fixed_sqrt.sv \
]

set RTL_SOURCES(axis_ggx_control) [list \
  $_rtl_dir/ggx_latency_pkg.sv \
  $_rtl_dir/axis_ggx_control.sv \
  $_rtl_dir/axis_skid_buffer.sv \
  $_rtl_dir/axis_fifo_2deep.sv \
  $_rtl_dir/axis_ggx_event_basis.sv \
  $_rtl_dir/axis_pre_ggx_sampler.v \
  $_rtl_dir/axis_top_lvl_sampler.sv \
  $_rtl_dir/axis_ggx_projected_area.sv \
  $_rtl_dir/axis_ggx_reproject_normalize.sv \
  $_rtl_dir/axis_oct32_encode.sv \
  $_rtl_dir/axis_fixed_norm3.sv \
  $_rtl_dir/axis_fixed_sqrt.sv \
  $_rtl_dir/axis_fixed_div.sv \
  $_rtl_dir/axis_fixed_inv_sqrt_nodsp.sv \
  $_rtl_dir/axis_fixed_inv_sqrt_folded.sv \
  $_rtl_dir/axis_sobol2d_stateless.sv \
  $_rtl_dir/axis_nested_uniform_scramble.sv \
  $_rtl_dir/axis_hash_combine_2d.sv \
  $_rtl_dir/axis_trig_lut.sv \
]

set RTL_SOURCES(axis_ggx_event_basis) [list \
  $_rtl_dir/ggx_latency_pkg.sv \
  $_rtl_dir/axis_ggx_event_basis.sv \
  $_rtl_dir/axis_fixed_norm3.sv \
  $_rtl_dir/axis_fixed_sqrt.sv \
  $_rtl_dir/axis_fixed_div.sv \
  $_rtl_dir/axis_fixed_inv_sqrt_nodsp.sv \
  $_rtl_dir/axis_fixed_inv_sqrt_folded.sv \
]

set RTL_SOURCES(axis_ggx_projected_area) [list \
  $_rtl_dir/ggx_latency_pkg.sv \
  $_rtl_dir/axis_ggx_projected_area.sv \
  $_rtl_dir/axis_fixed_sqrt.sv \
  $_rtl_dir/axis_trig_lut.sv \
]

set RTL_SOURCES(axis_ggx_reproject_normalize) [list \
  $_rtl_dir/ggx_latency_pkg.sv \
  $_rtl_dir/axis_ggx_reproject_normalize.sv \
  $_rtl_dir/axis_oct32_encode.sv \
  $_rtl_dir/axis_fifo_2deep.sv \
  $_rtl_dir/axis_fixed_sqrt.sv \
  $_rtl_dir/axis_fixed_div.sv \
  $_rtl_dir/axis_fixed_inv_sqrt_nodsp.sv \
  $_rtl_dir/axis_fixed_inv_sqrt_folded.sv \
  $_rtl_dir/axis_fixed_norm3.sv \
]

set RTL_SOURCES(axis_hash_combine_2d) [list \
  $_rtl_dir/ggx_latency_pkg.sv \
  $_rtl_dir/axis_hash_combine_2d.sv \
]

set RTL_SOURCES(axis_nested_uniform_scramble) [list \
  $_rtl_dir/ggx_latency_pkg.sv \
  $_rtl_dir/axis_nested_uniform_scramble.sv \
]

set RTL_SOURCES(axis_oct32_encode) [list \
  $_rtl_dir/ggx_latency_pkg.sv \
  $_rtl_dir/axis_oct32_encode.sv \
  $_rtl_dir/axis_fixed_div.sv \
]

set RTL_SOURCES(axis_pre_ggx_sampler) [list \
  $_rtl_dir/ggx_latency_pkg.sv \
  $_rtl_dir/axis_pre_ggx_sampler.v \
  $_rtl_dir/axis_top_lvl_sampler.sv \
  $_rtl_dir/axis_sobol2d_stateless.sv \
  $_rtl_dir/axis_hash_combine_2d.sv \
  $_rtl_dir/axis_nested_uniform_scramble.sv \
]

set RTL_SOURCES(axis_sobol2d_stateless) [list \
  $_rtl_dir/ggx_latency_pkg.sv \
  $_rtl_dir/axis_sobol2d_stateless.sv \
]

set RTL_SOURCES(axis_top_lvl_sampler) [list \
  $_rtl_dir/ggx_latency_pkg.sv \
  $_rtl_dir/axis_top_lvl_sampler.sv \
]

set RTL_SOURCES(axis_trig_lut) [list \
  $_rtl_dir/ggx_latency_pkg.sv \
  $_rtl_dir/axis_trig_lut.sv \
]
