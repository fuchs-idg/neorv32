#*****************************************************************************************
# Vivado (TM) v2023.1 (64-bit)
#
# project.tcl: Tcl script for re-creating project 'project'
#
# Generated by Vivado on Thu Jan 18 11:52:38 +0100 2024
# IP Build 3864474 on Sun May  7 20:36:21 MDT 2023
#
# This file contains the Vivado Tcl commands for re-creating the project to the state*
# when this script was generated. In order to re-create the project, please source this
# file in the Vivado Tcl Shell.
#
# * Note that the runs in the created project will be configured the same way as the
#   original project, however they will not be launched automatically. To regenerate the
#   run results please launch the synthesis/implementation runs as needed.
#
#*****************************************************************************************
# NOTE: In order to use this script for source control purposes, please make sure that the
#       following files are added to the source control system:-
#
# 1. This project restoration tcl script (project.tcl) that was generated.
#
# 2. The following source(s) files that were local or imported into the original project.
#    (Please see the '$orig_proj_dir' and '$origin_dir' variable setting below at the start of the script)
#
#    "C:/neorv32-fuchs/saspit/vivado/top.vhd"
#    "C:/neorv32-fuchs/saspit/vivado/Nexys-Video-Master.xdc"
#
# 3. The following remote source files that were added to the original project:-
#
#    "C:/neorv32-fuchs/rtl/core/neorv32_package.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_application_image.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_boot_rom.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_bootloader_image.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_cfs.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_fifo.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_cpu_decompressor.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_cpu_control.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_cpu_regfile.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_cpu_cp_shifter.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_cpu_cp_muldiv.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_cpu_cp_bitmanip.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_cpu_cp_fpu.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_cpu_cp_cfu.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_cpu_cp_cond.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_cpu_alu.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_cpu_lsu.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_cpu_pmp.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_cpu.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_crc.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_dcache.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_debug_dm.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_debug_dtm_bscane2.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_dma.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_dmem.entity.vhd"
#    "C:/neorv32-fuchs/rtl/core/mem/neorv32_dmem.default.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_gpio.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_gptmr.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_icache.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_imem.entity.vhd"
#    "C:/neorv32-fuchs/rtl/core/mem/neorv32_imem.default.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_intercon.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_mtime.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_neoled.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_onewire.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_pwm.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_sdi.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_slink.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_spi.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_sysinfo.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_xip.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_wishbone.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_wdt.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_uart.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_twi.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_trng.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_xirq.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_top.vhd"
#    "C:/neorv32-fuchs/rtl/core/neorv32_debug_dtm.vhd"
#
#*****************************************************************************************

# Check file required for this script exists
proc checkRequiredFiles { origin_dir} {
  set status true
  set files [list \
 "[file normalize "$origin_dir/top.vhd"]"\
 "[file normalize "$origin_dir/Nexys-Video-Master.xdc"]"\
  ]
  foreach ifile $files {
    if { ![file isfile $ifile] } {
      puts " Could not find local file $ifile "
      set status false
    }
  }

  set files [list \
 "[file normalize "$origin_dir/../../rtl/core/neorv32_package.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_application_image.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_boot_rom.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_bootloader_image.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_cfs.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_fifo.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_cpu_decompressor.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_cpu_control.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_cpu_regfile.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_cpu_cp_shifter.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_cpu_cp_muldiv.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_cpu_cp_bitmanip.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_cpu_cp_fpu.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_cpu_cp_cfu.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_cpu_cp_cond.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_cpu_alu.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_cpu_lsu.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_cpu_pmp.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_cpu.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_crc.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_dcache.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_debug_dm.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_debug_dtm_bscane2.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_dma.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_dmem.entity.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/mem/neorv32_dmem.default.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_gpio.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_gptmr.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_icache.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_imem.entity.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/mem/neorv32_imem.default.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_intercon.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_mtime.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_neoled.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_onewire.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_pwm.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_sdi.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_slink.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_spi.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_sysinfo.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_xip.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_wishbone.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_wdt.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_uart.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_twi.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_trng.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_xirq.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_top.vhd"]"\
 "[file normalize "$origin_dir/../../rtl/core/neorv32_debug_dtm.vhd"]"\
  ]
  foreach ifile $files {
    if { ![file isfile $ifile] } {
      puts " Could not find remote file $ifile "
      set status false
    }
  }

  return $status
}
# Set the reference directory for source file relative paths (by default the value is script directory path)
set origin_dir "."

# Use origin directory path location variable, if specified in the tcl shell
if { [info exists ::origin_dir_loc] } {
  set origin_dir $::origin_dir_loc
}

# Set the project name
set _xil_proj_name_ "project"

# Use project name variable, if specified in the tcl shell
if { [info exists ::user_project_name] } {
  set _xil_proj_name_ $::user_project_name
}

variable script_file
set script_file "project.tcl"

# Help information for this script
proc print_help {} {
  variable script_file
  puts "\nDescription:"
  puts "Recreate a Vivado project from this script. The created project will be"
  puts "functionally equivalent to the original project for which this script was"
  puts "generated. The script contains commands for creating a project, filesets,"
  puts "runs, adding/importing sources and setting properties on various objects.\n"
  puts "Syntax:"
  puts "$script_file"
  puts "$script_file -tclargs \[--origin_dir <path>\]"
  puts "$script_file -tclargs \[--project_name <name>\]"
  puts "$script_file -tclargs \[--help\]\n"
  puts "Usage:"
  puts "Name                   Description"
  puts "-------------------------------------------------------------------------"
  puts "\[--origin_dir <path>\]  Determine source file paths wrt this path. Default"
  puts "                       origin_dir path value is \".\", otherwise, the value"
  puts "                       that was set with the \"-paths_relative_to\" switch"
  puts "                       when this script was generated.\n"
  puts "\[--project_name <name>\] Create project with the specified name. Default"
  puts "                       name is the name of the project from where this"
  puts "                       script was generated.\n"
  puts "\[--help\]               Print help information for this script"
  puts "-------------------------------------------------------------------------\n"
  exit 0
}

if { $::argc > 0 } {
  for {set i 0} {$i < $::argc} {incr i} {
    set option [string trim [lindex $::argv $i]]
    switch -regexp -- $option {
      "--origin_dir"   { incr i; set origin_dir [lindex $::argv $i] }
      "--project_name" { incr i; set _xil_proj_name_ [lindex $::argv $i] }
      "--help"         { print_help }
      default {
        if { [regexp {^-} $option] } {
          puts "ERROR: Unknown option '$option' specified, please type '$script_file -tclargs --help' for usage info.\n"
          return 1
        }
      }
    }
  }
}

# Set the directory path for the original project from where this script was exported
set orig_proj_dir "[file normalize "$origin_dir/"]"

# Check for paths and files needed for project creation
set validate_required 0
if { $validate_required } {
  if { [checkRequiredFiles $origin_dir] } {
    puts "Tcl file $script_file is valid. All files required for project creation is accesable. "
  } else {
    puts "Tcl file $script_file is not valid. Not all files required for project creation is accesable. "
    return
  }
}

# Create project
create_project ${_xil_proj_name_} ./${_xil_proj_name_} -part xc7a200tsbg484-1

# Set the directory path for the new project
set proj_dir [get_property directory [current_project]]

# Reconstruct message rules
# None

# Set project properties
set obj [current_project]
set_property -name "board_part" -value "digilentinc.com:nexys_video:part0:1.2" -objects $obj
set_property -name "default_lib" -value "xil_defaultlib" -objects $obj
set_property -name "enable_resource_estimation" -value "0" -objects $obj
set_property -name "enable_vhdl_2008" -value "1" -objects $obj
set_property -name "ip_cache_permissions" -value "read write" -objects $obj
set_property -name "ip_output_repo" -value "$proj_dir/${_xil_proj_name_}.cache/ip" -objects $obj
set_property -name "mem.enable_memory_map_generation" -value "1" -objects $obj
set_property -name "platform.board_id" -value "nexys_video" -objects $obj
set_property -name "revised_directory_structure" -value "1" -objects $obj
set_property -name "sim.central_dir" -value "$proj_dir/${_xil_proj_name_}.ip_user_files" -objects $obj
set_property -name "sim.ip.auto_export_scripts" -value "1" -objects $obj
set_property -name "simulator_language" -value "Mixed" -objects $obj
set_property -name "sim_compile_state" -value "1" -objects $obj
set_property -name "target_language" -value "VHDL" -objects $obj

# Create 'sources_1' fileset (if not found)
if {[string equal [get_filesets -quiet sources_1] ""]} {
  create_fileset -srcset sources_1
}

# Set 'sources_1' fileset object
set obj [get_filesets sources_1]
set files [list \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_package.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_application_image.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_boot_rom.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_bootloader_image.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_cfs.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_fifo.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_cpu_decompressor.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_cpu_control.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_cpu_regfile.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_cpu_cp_shifter.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_cpu_cp_muldiv.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_cpu_cp_bitmanip.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_cpu_cp_fpu.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_cpu_cp_cfu.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_cpu_cp_cond.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_cpu_alu.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_cpu_lsu.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_cpu_pmp.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_cpu.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_crc.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_dcache.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_debug_dm.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_debug_dtm_bscane2.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_dma.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_dmem.entity.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/mem/neorv32_dmem.default.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_gpio.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_gptmr.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_icache.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_imem.entity.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/mem/neorv32_imem.default.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_intercon.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_mtime.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_neoled.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_onewire.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_pwm.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_sdi.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_slink.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_spi.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_sysinfo.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_xip.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_wishbone.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_wdt.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_uart.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_twi.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_trng.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_xirq.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_top.vhd"] \
 [file normalize "${origin_dir}/../../rtl/core/neorv32_debug_dtm.vhd"] \
]
add_files -norecurse -fileset $obj $files

# Add local files from the original project (-no_copy_sources specified)
set files [list \
 [file normalize "${origin_dir}/top.vhd" ]\
]
set added_files [add_files -fileset sources_1 $files]

# Set 'sources_1' fileset file properties for remote files
set file "$origin_dir/../../rtl/core/neorv32_package.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_application_image.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_boot_rom.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_bootloader_image.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_cfs.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_fifo.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_cpu_decompressor.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_cpu_control.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_cpu_regfile.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_cpu_cp_shifter.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_cpu_cp_muldiv.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_cpu_cp_bitmanip.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_cpu_cp_fpu.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_cpu_cp_cfu.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_cpu_cp_cond.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_cpu_alu.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_cpu_lsu.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_cpu_pmp.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_cpu.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_crc.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_dcache.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_debug_dm.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_debug_dtm_bscane2.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_dma.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_dmem.entity.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/mem/neorv32_dmem.default.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_gpio.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_gptmr.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_icache.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_imem.entity.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/mem/neorv32_imem.default.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_intercon.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_mtime.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_neoled.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_onewire.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_pwm.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_sdi.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_slink.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_spi.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_sysinfo.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_xip.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_wishbone.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_wdt.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_uart.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_twi.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_trng.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_xirq.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_top.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj

set file "$origin_dir/../../rtl/core/neorv32_debug_dtm.vhd"
set file [file normalize $file]
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj


# Set 'sources_1' fileset file properties for local files
set file "vivado/top.vhd"
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "VHDL" -objects $file_obj
set_property -name "library" -value "neorv32" -objects $file_obj


# Set 'sources_1' fileset properties
set obj [get_filesets sources_1]
set_property -name "dataflow_viewer_settings" -value "min_width=16" -objects $obj
set_property -name "top" -value "top" -objects $obj

# Create 'constrs_1' fileset (if not found)
if {[string equal [get_filesets -quiet constrs_1] ""]} {
  create_fileset -constrset constrs_1
}

# Set 'constrs_1' fileset object
set obj [get_filesets constrs_1]

# Add/Import constrs file and set constrs file properties
set file "[file normalize "$origin_dir/Nexys-Video-Master.xdc"]"
set file_added [add_files -norecurse -fileset $obj [list $file]]
set file "vivado/Nexys-Video-Master.xdc"
set file_obj [get_files -of_objects [get_filesets constrs_1] [list "*$file"]]
set_property -name "file_type" -value "XDC" -objects $file_obj

# Set 'constrs_1' fileset properties
set obj [get_filesets constrs_1]

# Create 'sim_1' fileset (if not found)
if {[string equal [get_filesets -quiet sim_1] ""]} {
  create_fileset -simset sim_1
}

# Set 'sim_1' fileset object
set obj [get_filesets sim_1]
# Empty (no sources present)

# Set 'sim_1' fileset properties
set obj [get_filesets sim_1]
set_property -name "top" -value "top" -objects $obj
set_property -name "top_lib" -value "neorv32" -objects $obj

puts "INFO: Project created:${_xil_proj_name_}"
# Create 'drc_1' gadget (if not found)
if {[string equal [get_dashboard_gadgets  [ list "drc_1" ] ] ""]} {
create_dashboard_gadget -name {drc_1} -type drc
}
set obj [get_dashboard_gadgets [ list "drc_1" ] ]
set_property -name "reports" -value "impl_1#impl_1_route_report_drc_0" -objects $obj

# Create 'methodology_1' gadget (if not found)
if {[string equal [get_dashboard_gadgets  [ list "methodology_1" ] ] ""]} {
create_dashboard_gadget -name {methodology_1} -type methodology
}
set obj [get_dashboard_gadgets [ list "methodology_1" ] ]
set_property -name "reports" -value "impl_1#impl_1_route_report_methodology_0" -objects $obj

# Create 'power_1' gadget (if not found)
if {[string equal [get_dashboard_gadgets  [ list "power_1" ] ] ""]} {
create_dashboard_gadget -name {power_1} -type power
}
set obj [get_dashboard_gadgets [ list "power_1" ] ]
set_property -name "reports" -value "impl_1#impl_1_route_report_power_0" -objects $obj

# Create 'timing_1' gadget (if not found)
if {[string equal [get_dashboard_gadgets  [ list "timing_1" ] ] ""]} {
create_dashboard_gadget -name {timing_1} -type timing
}
set obj [get_dashboard_gadgets [ list "timing_1" ] ]
set_property -name "reports" -value "impl_1#impl_1_route_report_timing_summary_0" -objects $obj

# Create 'utilization_1' gadget (if not found)
if {[string equal [get_dashboard_gadgets  [ list "utilization_1" ] ] ""]} {
create_dashboard_gadget -name {utilization_1} -type utilization
}
set obj [get_dashboard_gadgets [ list "utilization_1" ] ]
set_property -name "reports" -value "synth_1#synth_1_synth_report_utilization_0" -objects $obj
set_property -name "run.step" -value "synth_design" -objects $obj
set_property -name "run.type" -value "synthesis" -objects $obj

# Create 'utilization_2' gadget (if not found)
if {[string equal [get_dashboard_gadgets  [ list "utilization_2" ] ] ""]} {
create_dashboard_gadget -name {utilization_2} -type utilization
}
set obj [get_dashboard_gadgets [ list "utilization_2" ] ]
set_property -name "reports" -value "impl_1#impl_1_place_report_utilization_0" -objects $obj

move_dashboard_gadget -name {utilization_1} -row 0 -col 0
move_dashboard_gadget -name {power_1} -row 1 -col 0
move_dashboard_gadget -name {drc_1} -row 2 -col 0
move_dashboard_gadget -name {timing_1} -row 0 -col 1
move_dashboard_gadget -name {utilization_2} -row 1 -col 1
move_dashboard_gadget -name {methodology_1} -row 2 -col 1
