# DSM-232 miniloader builder

![Photo of DSM-232 SPI Flash being flashed](doc/images/dsm232-hack.jpg?raw=true)

This is an attempt to hack D-Link DSM-232 that runs on Cavium/Celstial CNW6611L/CNS1800L SOC platform.
Currently, it can generate SPI Flash image that contains miniloader and user-controllable bits signed with
GPL-published RSA key, ready to run on DSM-232.

In factory-shipped configuration, miniloader in SPI Flash can only run U-Boot on NAND Flash, that is signed
with closed D-Link RSA key. While bundled U-Boot did allow you to load non-signed kernel image, you really did not
have control on both U-Boot and NAND Flash layout. By using this builder, you will gain complete control on chaining
bootloader and NAND Flash configuration.

# Usage
```
make -C app/blink
make spi
```
Once spi.bin is generated, program it to on-board SPI flash chip.

# Roadmap
Since latter 64KB of on-board 128KB SPI Flash is free for use, my plan is to put a
small monitor on that region to read/write NAND Flash. This way, in case of NAND flash
breakage (these things happen), I can always recover by just flashing SPI flash by
directly wiring to the chip.

# NOTE
Currently, simple app that blinks LED on gpio2 will be included in SPI Flash image,
and miniloader will boot into it. See second-param.yaml, image.lds, and blink.lds for
on-SPI-flash and on-memory layout.

# NOTE 2
If you want to go by yourself, key takeaway is that miniloader and RSA key for DSM-260 v1.04 were essentially needed to
control early boot process of DSM-232. Due to secureboot limitation, miniloader bundled with DSM-232 GPL archive won't work.
This repository is a result of research across GPL-released archives for DSM-232 v1.02, DSM-260 v1.04, and DSM-260 v2.00.

# Photos
![Frontside of DSM-232 PCB](doc/images/dsm232-front.jpg?raw=true)
![Backside of DSM-232 PCB](doc/images/dsm232-back.jpg?raw=true)
