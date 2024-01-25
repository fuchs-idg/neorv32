puts "This script produces an mcs file containing configuration bitstream and neorv32 binary."
puts "Name of the binary must be neorv32_exe.bin and it must reside in the directory above the project directory."

set proj_dir [get_property DIRECTORY [current_project]]
set bit_path $proj_dir/project.runs/impl_1/top.bit
set dat_path $proj_dir/../neorv32_exe.bin

write_cfgmem -force -format mcs -size 16 -interface SPIx1 -loadbit "up 0x00000000 $bit_path" -loaddata "up 0x00950000 $dat_path" "$proj_dir/BitAndBinary.mcs"