#!/usr/bin/bash

python3 scripts/program_can.py txoff_retry
python3 scripts/program_can.py c70off
#python3 scripts/program_can.py boot 1
python3 scripts/program_can.py program 4 ../core-vc/build/stm32/core-vc-f34.ihex
python3 scripts/program_can.py c70on
python3 scripts/program_can.py txon
