# SASPIT Project Directory

## saspit/sw/

Contains test and example programs in /programs/ as well as changed standard neorv32/sw/ components in /bootloader/ and /common/.

## saspit/vivado/

Contains everything needed to locally create a Vivado project:

- top.vhd is the top module
- Nexys-Video-Master.xdc is the main (and only) constraint file
- project.tcl is a TCl script to recreate the Vivado project for the neorv32 on the Nexys Video board on your local machine
- add_*.tcl are scripts to add IPs or HDL files to the base project, these scripts must be manually added to the bottom of the project.tcl

Also, there are utilities like the write_mcs.tcl and program_mcs.tcl scripts to write and then load bitfiles and binaries to the Nexys flash.

Put the "neorv32_exe.bin" into the saspit/vivado/ directory and make sure you have a valid bitstream "top.bit" in saspit/vivado/project/project.runs/impl_1.
Then, with the project opened in Vivado (and the Nexys Video board connected via the PROG port):

```TCl
(Tcl console) source ./write_mcs.tcl
(Tcl console) source ./program_mcs_to_flash.tcl
```

## saspit/openocd-files

Contains openocd config files to start a debug server for the NeoRV32 on the Nexys Video (and two other boards. 

When using openocd from within the WSL, the Nexys Video board (or more precisely, the FTDI FT2232H on the board connected to the PROG port) has to be attached to the WSL first.

The bash scripts attach-to-wsl.sh and detach-to-wsl.sh automate this process:

To start openocd:

```BASH
$ bash attach-to-wsl.sh
$ openocd -f nexysvideo_neorv32.cfg
```
To make the nexys video board available to Windows again, press ctrl-c to halt openocd then detach the device like this:

```BASH
$ bash detach-from-wsl.sh
```
## saspit/thirdparty

Contains thirdparty software and hardware.
