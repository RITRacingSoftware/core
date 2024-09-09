#!/usr/bin/env bash

set -x

ELF=build/stm32/core-f33.elf

openocd -f ./openocd.cfg -c "program ${ELF} verify reset" -c "halt"
