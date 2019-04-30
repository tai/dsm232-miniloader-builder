This package is for insecure customer. 
---------------------------------------------------
Preparation
---------------------------------------------------
1. A PC with linux environment
2. install openssl package from open source society

---------------------------------------------------
Instruction
---------------------------------------------------
1. Select proper miniloader to same directoy with mk_nonsec_image
   Naming rule
   		miniloader.bin_bs[A]_entry[B]_ddr[C]_[D]_singleflash[E]_ncycle[F]_page[G]_dualboot[H]
   		
   		A - max allowed size for first boot loader
   		B - Flash entry for first boot loader
   		C - DDR frequency (0x1100 - 972M  0x1200 - 1026M  0x1300 - 1080M ... 0x1700 - 1296M 0x9c00 - 783M)
   		D - 512M (16bit) or 256M (16bit)
   		E - 0: miniloader at spi flash and the others at nand flash 1: whole image on one flash, SPI or NAND
   		F - address cycles of nand flash, 2 or 3
   		G - page size of nand flash 512/2048/4096
   		H - 0: single boot loader 1: dual boot loader  		
   		
2. copy boot loader to same folder with mk_nonsec_image (if dual boot loader enabled, copy two boot loaders to same folder)

3. use mk_nonsec_image to create flash image
   usage:  ./mk_nonsec_image miniloader_file_name parameter_file_name Safe_boot_file_name Second_boot_file_namer key_filename
   
	 For single boot loader, a flash.img will be created.
	 For dual boot loader, three images will be created, signed_miniloader.bin signed_first_bootloader.bin signed_second_bootloader.bin
