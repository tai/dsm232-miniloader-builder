/*
 * Linker script for binary placed in latter 64KB of SPI Flash
 */

MEMORY {
  /*
   * NOTE:
   * - Length limit comes from the case when placed between 0xb700-0xc000
   * - Origin is just random - PIC build should run from any address
   */
  ROM : ORIGIN = 0x4000000, LENGTH = 0x10000
}

OUTPUT_FORMAT(elf32-littlearm)
OUTPUT_ARCH(arm)
ENTRY(main)

SECTIONS {
  . = 0;
  . = ALIGN(4);
  .text : { *(.text.main) *(.text) } > ROM

  . = ALIGN(4);
  .rodata : { *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.rodata*))) } > ROM

  . = ALIGN(4);
  .data : { *(.data) } > ROM

  . = ALIGN(4);
  .bss : { *(.bss) } > ROM
}
