#!/usr/bin/bash

ELF=build/stm32/core-f33.elf

mkfifo .uploader_telnet_fifo
echo -e "program ${ELF} verify reset\nexit" > /tmp/uploader_telnet_fifo &
tail -f /tmp/uploader_telnet_fifo | telnet localhost 4444
if [ $? != 0 ]; then
    openocd -f ./openocd.cfg -c "program ${ELF} verify reset" -c "exit"
fi
rm .uploader_telnet_fifo
