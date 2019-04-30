/*
 * (C) Copyright 2005
 * STMicroelectronics.
 * Configuration settings for the "Nomadik Hardware Kit" NHK-8815,
 * the evaluation board for the Nomadik 8815 System on Chip.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_ARM1176
#define CONFIG_ARCH_CNC1800L
#define CONFIG_256M_DDR

#define CONFIG_SKIP_LOWLEVEL_INIT /* we have already been loaded to RAM */
#define CONFIG_SYS_ARM_CACHE_WRITETHROUGH
/* commands */

#define CONFIG_CMD_CACHE

#define CONFIG_NET_MULTI
#define CONFIG_CMD_NET
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_NFS
/* There is no NOR flash, so undefine these commands */
#undef CONFIG_CMD_FLASH
#undef CONFIG_CMD_IMLS
#define CONFIG_SYS_NO_FLASH
/* There is NAND storage */
#define CONFIG_NAND_CNC1800L
#define CONFIG_CMD_JFFS2

/* user interface */
#define CONFIG_CNC1800L_CRYPTO

#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT		"CNC1800L> "
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#define CONFIG_CMDLINE_EDITING
#define CONFIG_SYS_CBSIZE		256	/* Console I/O Buffer Size */
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE \
					+ sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_BARGSIZE	CONFIG_SYS_CBSIZE /* Boot Arg Buffer Size */
#define CONFIG_SYS_MAXARGS	16
#define CONFIG_SYS_LOAD_ADDR	0x800000	/* default load address */
#define CONFIG_SYS_LOADS_BAUD_CHANGE

/* boot config */
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_CMDLINE_TAG
#define CONFIG_BOOTDELAY	1
#define CONFIG_AUTO_COMPLETE
#define CONFIG_ZERO_BOOTDELAY_CHECK

/* memory-related information */
#define CONFIG_NR_DRAM_BANKS	1
#define PHYS_SDRAM_1		0x00000000	/* DDR-SDRAM Bank #1 */
#ifdef DDR_SIZE_1G
#define PHYS_SDRAM_1_SIZE       0x40000000      /* 1G MB */
#elif defined DDR_SIZE_512M
#define PHYS_SDRAM_1_SIZE	0x20000000	/* 512 MB */
#else
#define PHYS_SDRAM_1_SIZE	0x10000000	/* 256 MB */
#endif
//#define PHYS_SDRAM_2		0x08000000	/* SDR-SDRAM BANK #2*/
//#define PHYS_SDRAM_2_SIZE	0x04000000	/* 64 MB */
#define CONFIG_SYS_SDRAM_BASE	PHYS_SDRAM_1
#define CONFIG_SYS_SDRAM_SIZE	PHYS_SDRAM_1_SIZE
#define CONFIG_SYS_TEXT_BASE        0x2000000
#define CONFIG_SYS_PHY_UBOOT_BASE   CONFIG_SYS_TEXT_BASE
#define CONFIG_SYS_INIT_SP_ADDR (CONFIG_SYS_TEXT_BASE - CONFIG_SYS_MALLOC_LEN - 512)

#define CONFIG_SYS_MEM_TOP_HIDE (0x2000000) /* 32MB */
#define CONFIG_STACKSIZE	(512 * 1024)	/* regular stack */

#define CONFIG_USE_IRQ
#ifdef CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ	(4 * 1024)	/* IRQ stack */
#define CONFIG_STACKSIZE_FIQ	(4 * 1024)	/* FIQ stack */
#endif

#define CONFIG_SYS_MEMTEST_START	0x00000000
#define CONFIG_SYS_MEMTEST_END		0x0FFFFFFF
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + 12 * 1024 * 1024)

/* timing informazion */
#define CONFIG_SYS_CPU_CLK              378000000
#define CONFIG_SYS_PCLK_FREQ            (CONFIG_SYS_CPU_CLK / 8)
#define CONFIG_SYS_HZ		            1000
#define CONFIG_SYS_TIMERBASE	        0x801E2000
#define CONFIG_SYS_TIMER_INTERVAL       10000
#define CONFIG_SYS_TIMER_RELOAD         0xffffffff //(CONFIG_SYS_TIMER_INTERVAL >> 4)  /* Divide by 16 */


/* usb */
#define CONFIG_CMD_FAT
#define CONFIG_CMD_USB

#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_CNC18XX
#define CONFIG_SYS_USB_EHCI_REGS_BASE	     0x80200000
#define CONFIG_SYS_USB_EHCI_MAX_ROOT_PORTS   1

#define CONFIG_DOS_PARTITION

#define CONFIG_USB_STORAGE

#define CONFIG_AUTO_UPDATE

#define CONFIG_SYS_NS16550
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_CLK          CONFIG_SYS_PCLK_FREQ
#define CONFIG_CONS_INDEX	        1
#define CONFIG_SYS_NS16550_REG_SIZE     (-4)
#define CONFIG_BAUDRATE		        115200
#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }
#define CONFIG_SYS_NS16550_COM1         0x801F1000
#define CONFIG_SYS_NS16550_COM2         0x801F2000

/* SD/MMC/SDIO */
#ifdef CONFIG_CNC1800L_MSHCI
#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#ifndef CONFIG_DOS_PARTITION
#define CONFIG_DOS_PARTITION
#endif
#else
 #undef CONFIG_MMC
 #undef CONFIG_CMD_MMC
 #undef CONFIG_GENERIC_MMC
#endif /* CONFIG_CNC1800L_MSHCI */

/*POWER CTL */ /* define 1 enable, 0 disable, undefine don't change */
// #define CNC_VIDEO_DAC_ENABLE 1
// #define CNC_AUDIO_DAC_ENABLE 1
// #define CNC_USB_ENABLE 1
// #define CNC_VIDEO_CLOCK_ENABLE 1
// #define CNC_AUDIO_CLOCK_ENABLE 1
// #define CNC_CRYPTO_CLOCK_ENABLE 1
// #define CNC_XPORT_CLOCK_ENABLE 1
// #define CNC_GFX_CLOCK_ENABLE 1
// #define CNC_VIB_CLOCK_ENABLE 1
// #define CNC_TVE_CLOCK_ENABLE 1


/* i2c */
#define CONFIG_CNC1800L_I2C
#define CONFIG_CMD_I2C
#define CONFIG_HARD_I2C
#define CONFIG_SYS_I2C_SPEED	100000
#define CONFIG_SYS_I2C_SLAVE    0x7f
#define CONFIG_SYS_I2C_BASE     0x80170000
#define CONFIG_SYS_I2C_CLK      CONFIG_SYS_PCLK_FREQ

#define CONFIG_CMD_GPIO
#define CONFIG_CNC1800L_GPIO
#define CNC1800L_GPIO_BASE      0x80260000
#define REG_GPIO_SWPORTA_DDR 0x24
#define REG_GPIO_SWPORTB_DDR 0x38
#define REG_GPIO_EXT_PORTA   0x60
#define REG_GPIO_EXT_PORTB   0x64
#define REG_GPIO_SWPORTA_DR  0x20
#define REG_GPIO_SWPORTB_DR  0x34

/* eeprom */
#define CONFIG_CMD_EEPROM
#if defined(CONFIG_CMD_EEPROM)
#define CONFIG_SYS_I2C_EEPROM_ADDR            0x50
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN        2
#define CONFIG_CNC1800L_ETHADDR_EEPROM_OFFSET    0x114
#endif

// Enable USB Ethernet adpators
//#define CONFIG_USB_HOST_ETHER
//#define CONFIG_USB_ETHER_SMSC95XX
//#define CONFIG_USB_ETHER_ASIX

// #define CONFIG_IP_DEFRAG	[> Allows faster download, TFTP and NFS <]
// #define CONFIG_TFTP_BLOCKSIZE	4096
// #define CONFIG_NFS_READ_SIZE	4096

#define CONFIG_CMD_NAND
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_BASE		0xA4000000
#define CNC1800L_NAND_CFG_BASE         0x80100000
#define CONFIG_CMD_NAND_YAFFS

#define CONFIG_CMD_UNZIP

/* environment */
#define CONFIG_CMD_SAVEENV
#define CONFIG_ENV_OVERWRITE

#if defined(CONFIG_ENV_IS_IN_NAND)

#define CONFIG_ENV_SIZE          0x00020000 /* Total Size of Environment Sector 128KB(for 2k pagesize of csm1800) */
#define CONFIG_ENV_OFFSET        0x000E0000 /* environment starts here/aligned with 128KB                         */
#define CONFIG_ENV_SECT_SIZE     0x00020000 /* 128 KB sectors                                                      */

#else

#define CONFIG_ENV_IS_IN_SPI_FLASH
#define CONFIG_ENV_SIZE          0x00010000 /* Total Size of Environment Sector 64KB */
#define CONFIG_ENV_OFFSET        0x000E0000 /* environment starts here               */
#define CONFIG_ENV_SECT_SIZE     0x00010000 /* 64 KB sectors                         */

#endif


//#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_CNC1800L
#define CONFIG_CMD_SF

//#define CONFIG_CMD_SPI
//#define CONFIG_CNC1800L_SPI
//#define CONFIG_SYS_SPI_BASE         0x10150000

//#define CONFIG_CNC1800L_XPORT
//#define CONFIG_XPORT_BASE           (0xb1400000)
//#define CONFIG_PINMUX_BASE          (0xb2110000)

//#define CONFIG_CNC1800L_TUNER
//#define CONFIG_CNC1800L_TEST

#define CONFIG_CMD_CSLOGO
#define CONFIG_CNC1800L_DF
#define GFXBASEADDRESS		0xf000000

#define CONFIG_CMD_FPC

#define CNC1800L_PINMUX_BASE 0xB2110000

/* reserve 10M for u-boot self, log buf, TLB, bd_t, irq stack, abort stack and so on */
#ifdef CONFIG_SYS_MEM_TOP_HIDE
    /*
     * Ethernet DMA buffer can only use memory less than 256MiB, 
     * so use 256MiB instead of 512MiB as base address on 512MiB RAM board
     * the buffer address to load files into memory for USB upgrade is also defined here
     */
    #if (PHYS_SDRAM_1_SIZE >= 0x10000000)
        #define CONFIG_MEM_DEFINE_BASE  (0x10000000 - CONFIG_SYS_MEM_TOP_HIDE - CONFIG_SYS_MALLOC_LEN - 0xa00000)
        #define CONFIG_USB_UPGRADE_DATA_ADDR 	(0x10000000)
    #else
        #define CONFIG_MEM_DEFINE_BASE  (PHYS_SDRAM_1_SIZE - CONFIG_SYS_MEM_TOP_HIDE - CONFIG_SYS_MALLOC_LEN - 0xa00000)
        #define CONFIG_USB_UPGRADE_DATA_ADDR 	(0x1000000)        
    #endif
#else
    #if (PHYS_SDRAM_1_SIZE >= 0x10000000)
        #define CONFIG_MEM_DEFINE_BASE  (0x10000000 - CONFIG_SYS_MALLOC_LEN - 0xa00000)
        #define CONFIG_USB_UPGRADE_DATA_ADDR 	(0x10000000)
    #else
        #define CONFIG_MEM_DEFINE_BASE  (PHYS_SDRAM_1_SIZE - CONFIG_SYS_MALLOC_LEN - 0xa00000)
        #define CONFIG_USB_UPGRADE_DATA_ADDR 	(0x1000000)  
    #endif
#endif

#define CONFIG_UBOOT_EXTERNAL_PROC_LEN (0x100000) //for PIC processes code such as hdmi or OTA that will be combined after u-boot.bin

#define CONFIG_CN100_DATA_LEN 0x800000
#define CONFIG_CN100_DATA_BASE (CONFIG_MEM_DEFINE_BASE - CONFIG_CN100_DATA_LEN)
#define CONFIG_SPI_DMA_BUF_LEN 0x800000
#define CONFIG_SPI_DMA_BUF_BASE (CONFIG_CN100_DATA_BASE - CONFIG_SPI_DMA_BUF_LEN)

#define CONFIG_XPORT_MEM_LEN 0x800000
#define CONFIG_XPORT_MEM_BASE (CONFIG_SPI_DMA_BUF_BASE - CONFIG_XPORT_MEM_LEN)
#define CRYPTO_MEM_SIZE 0x300000
#define CRYPTO_MEM_REGION (CONFIG_XPORT_MEM_BASE - CRYPTO_MEM_SIZE)
#define CONFIG_ALIGNED_PIC_PROC_MEM_SIZE 0X100000
#define CONFIG_ALIGNED_PIC_PROC_MEM_BASE (((CRYPTO_MEM_REGION - CONFIG_ALIGNED_PIC_PROC_MEM_SIZE-sizeof(int))+(sizeof(int)-1))&(0xFFFFFFFC))

/* this is needed to make hello_world.c and other stuff happy */
#define CONFIG_SYS_MAX_FLASH_SECT	512
#define CONFIG_SYS_MAX_FLASH_BANKS	1

#define CONFIG_CMD_CALL_EXTERNAL_PROC
#define CONFIG_CMD_UPGRADE

#endif /* __CONFIG_H */
