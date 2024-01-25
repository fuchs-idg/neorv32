puts "This script programs an mcs file containing the configuration bitstream and a neorv32 binary into the Nexys Video onboard Flash"

set proj_dir [get_property DIRECTORY [current_project]]

if { [current_hw_server] != {} } {
	puts "Server is already open."
	puts "Please completely close the server before using this script!"
	puts "(In hardware window, right-click on localhost and select Close Server)"
} else {
	connect_hw_server -allow_non_jtag
	open_hw_target

	set_property PROGRAM.FILE $proj_dir/project.runs/impl_1/top.bit [get_hw_devices xc7a200t_0]
	current_hw_device [get_hw_devices xc7a200t_0]
	refresh_hw_device -update_hw_probes false [lindex [get_hw_devices xc7a200t_0] 0]

	create_hw_cfgmem -hw_device [get_hw_devices xc7a200t_0] -mem_dev [lindex [get_cfgmem_parts {s25fl256sxxxxxx0-spi-x1_x2_x4}] 0]
	set_property PROGRAM.ADDRESS_RANGE  {use_file} [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7a200t_0] 0]]
	set_property PROGRAM.FILES [list "$proj_dir/BitAndBinary.mcs" ] [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7a200t_0] 0]]
	set_property PROGRAM.PRM_FILE {} [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7a200t_0] 0]]
	set_property PROGRAM.UNUSED_PIN_TERMINATION {pull-down} [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7a200t_0] 0]]
	set_property PROGRAM.BLANK_CHECK  0 [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7a200t_0] 0]]
	set_property PROGRAM.ERASE  1 [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7a200t_0] 0]]
	set_property PROGRAM.CFG_PROGRAM  1 [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7a200t_0] 0]]
	set_property PROGRAM.VERIFY  1 [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7a200t_0] 0]]
	set_property PROGRAM.CHECKSUM  0 [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7a200t_0] 0]]

	startgroup 
	create_hw_bitstream -hw_device [lindex [get_hw_devices xc7a200t_0] 0] [get_property PROGRAM.HW_CFGMEM_BITFILE [ lindex [get_hw_devices xc7a200t_0] 0]]; program_hw_devices [lindex [get_hw_devices xc7a200t_0] 0]; refresh_hw_device [lindex [get_hw_devices xc7a200t_0] 0];

	program_hw_cfgmem -hw_cfgmem [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7a200t_0] 0]]
	endgroup
}