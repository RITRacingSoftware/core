#!/usr/bin/env bash

set -x


openocd -f ./openocd.cfg -c "program $1 verify reset" -c "exit"
