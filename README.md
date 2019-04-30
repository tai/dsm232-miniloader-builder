# DSM-232 miniloader builder

This is an attempt to hack D-Link DSM-232 that runs on Cavium/Celstial CNW6611L/CNS1800L SOC platform.
Currently, it can generate SPI Flash image that contains miniloader and user-controllable bits signed with
GPL-published RSA key, ready to run on DSM-232.

In factory-shipped configuration, miniloader in SPI Flash can only run U-Boot on NAND Flash, that is signed
with closed D-Link RSA key. That means you don't have control on both U-Boot and NAND Flash layout. By using
this builder, you will gain complete control on chaining bootloader and NAND Flash layout.

# Usage
```
make -C app/blink
make spi
```

Currently, simple app that blinks LED on gpio2 will be included in SPI Flash image,
and miniloader will boot into it. See second-param.yaml, image.lds, and blink.lds for
on-SPI-flash and on-memory layout.

# Roadmap
Since latter 64KB of on-board 128KB SPI Flash is free for use, my plan is to put a
small monitor on that region to read/write NAND Flash. This way, in case of NAND flash
breakage (these things happen), I can always recover by just flashing SPI flash by
directly wiring to the chip.
