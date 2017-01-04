#!/bin/bash

esptool.py --port COM4 --baud 115200 write_flash --flash_freq 40m --flash_mode qio --flash_size 32m 0x0000 ./bin/eagle.flash.bin 0x20000 ./bin/eagle.irom0text.bin
