#!/bin/bash
arm-none-eabi-gcc -mcpu=arm1176jzf-s -fpic -ffreestanding -c boot.S -o boot.o
arm-none-eabi-gcc -mcpu=arm1176jzf-s -fpic -ffreestanding -std=gnu99 -c kernel.c -o kernel.o -O2 -Wall -Wextra
arm-none-eabi-gcc -T linker.ld -o eos.elf -ffreestanding -O2 -nostdlib boot.o kernel.o -lgcc
arm-none-eabi-objcopy eos.elf -O binary kernel.img