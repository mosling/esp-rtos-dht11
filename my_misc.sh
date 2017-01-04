#!/bin/bash


export SDK_PATH=$SDK_PATH
export BIN_PATH=./bin

echo "SDK-PATH: $SDK_PATH"
echo "BIN-PATH: $BIN_PATH"
boot=none
echo "boot mode: $boot"
app=0
echo "generate bin: user1.bin"
spi_speed=40
echo "spi speed: $spi_speed MHz"
spi_mode=QIO
echo "spi mode: $spi_mode"
spi_size_map=4
echo "spi size: 4096KB"
echo "spi ota map:  512KB + 512KB"
echo ""

echo "start..."
echo ""

make BOOT=$boot APP=$app SPI_SPEED=$spi_speed SPI_MODE=$spi_mode SPI_SIZE_MAP=$spi_size_map
