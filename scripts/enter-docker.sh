#!/usr/bin/env bash

docker run -it \
	--mount type=bind,source=$PWD,destination=/core \
	--mount type=bind,source="$PWD/../STM32CubeG4",dst=/STM32CubeG4 \
	--mount type=bind,source="$PWD/../FreeRTOS-Kernel",dst=/FreeRTOS-Kernel \
	--mount type=bind,source="$PWD/../RTT",dst=/RTT \
	core