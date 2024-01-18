#!/bin/bash

RESULT=$(powershell.exe -Command "usbipd wsl list" | grep 0403:6010 | cut -d ' ' -f1)

if [ ! -z "$RESULT" ]
then
	powershell.exe -Command "usbipd wsl attach --busid $RESULT"
	echo "Attached device with busid $RESULT"
else
	echo "Device could not be attached to WSL :("
fi

