#!/bin/sh

set -e

riscv64-unknown-elf-objcopy -v -O binary \
    target/riscv64imac-unknown-none-elf/release/examples/foo \
    firmware.bin

# dfu-util -a 0 -s 0x08000000:leave -D firmware.bin

kflash -p /dev/tty.usbserial-00320000000 -b 1500000 -B maixduino \
    ./firmware.bin
