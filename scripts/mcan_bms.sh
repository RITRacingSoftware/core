#!/usr/bin/bash

python3 scripts/program_can.py txoff_retry
#python3 scripts/program_can.py boot 1
python3 scripts/program_can.py program 7 ../core-bms/build/stm32/core-bms-f34.ihex
python3 scripts/program_can.py txon
