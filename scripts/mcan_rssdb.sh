#!/usr/bin/bash

python3 scripts/program_can.py txoff
python3 scripts/program_can.py c70off
python3 scripts/program_can.py program 1 ../core-ssdb/build_rear/stm32/RSSDB-f33.ihex
python3 scripts/program_can.py c70on
python3 scripts/program_can.py txon
