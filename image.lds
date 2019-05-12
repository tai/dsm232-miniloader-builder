MEMORY {
  ROM : ORIGIN = 0x00, LENGTH = 0x20000
}

SECTIONS {
  . = 0x00;

  /*
   * NOTE:
   * FILL pattern in factory image is 0x00 or 0xFF, but I have
   * changed it to 0x55 and 0x9X so decoder in my logic analyzer
   * does not get lost due to missing edges.
   */

  .data : {
    coreloader.bin.o(.data)
    FILL(0x55);

    . = 0xb500;
    second-param.bin.o(.data)
    FILL(0x00);

    . = 0xb600;
    second-param.sig.o(.data)
    FILL(0x00);

   /*
    * boot2 image
    * = boot2_start with config_flags=0, dual_boot=0
    */
    . = 0xb800;
    app.bin.o(.data)
    FILL(0x00);

    . = 0x1b800;
    app.sig.o(.data)
    FILL(0x9A);

/*
    . = 0xb700;
    FILL(0xFF);

    . = 0xc000;
    nvram.bin.o(.data)
    FILL(0x00);

    . = 0xe000;
    FILL(0xFF);

    . = 0x10000;
    FILL(0xFF);
*/
    . = 0x20000;
  } > ROM
  
  /DISCARD/ : { *(*) }
}
