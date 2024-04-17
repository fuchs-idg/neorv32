# Modify this variable to fit your NEORV32 setup (neorv32 home folder)
NEORV32_HOME ?= ../../../..
USER_FLAGS ?= -O0

FATFS_PATH ?= ../../../thirdparty/fatfs/source
SDSPI_PATH ?= ../../../thirdparty/sdspi/sw/

include $(NEORV32_HOME)/sw/common/common.mk

sdcard_deps:
	cp $(FATFS_PATH)/diskio.h diskio.h
	cp $(FATFS_PATH)/ff.c ff.c
	cp $(FATFS_PATH)/ff.h ff.h
	cp $(FATFS_PATH)/ffconf.h ffconf.h
	cp $(SDSPI_PATH)/diskio.c diskio.c
	cp $(SDSPI_PATH)/diskiodrvr.h diskiodrvr.h
	cp $(SDSPI_PATH)/sdspidrv.c sdspidrv.c
	cp $(SDSPI_PATH)/sdspidrv.h sdspidrv.h

copy: neorv32_exe.bin
	cp neorv32_exe.bin /mnt/c/Users/phifuc/Desktop/neorv32_binary/neorv32_exe.bin
	cp neorv32_exe.bin $(NEORV32_HOME)/saspit/vivado/neorv32_exe.bin