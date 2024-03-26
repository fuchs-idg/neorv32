#!/bin/bash

RESULT=$(powershell.exe -Command "usbipd list" | grep 0403:6010 | cut -d ' ' -f1)

if [ ! -z "$RESULT" ]
then
	powershell.exe -Command "usbipd detach --busid $RESULT"
	echo "Detached device with busid $RESULT"
else
	echo "Device could not be detached from WSL :("
fi

