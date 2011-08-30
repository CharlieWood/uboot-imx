/*
 * Copyright (C) 2007, Guennadi Liakhovetski <lg@denx.de>
 *
 * (C) Copyright 2009-2010 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the MX51-3Stack Freescale board.
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

#include <asm/arch/mx51.h>

 /* High Level Configuration Options */
#define CONFIG_ARMV7		1	/* This is armv7 Cortex-A8 CPU core */
#define CONFIG_SYS_APCS_GNU

#define CONFIG_MXC		1
#define CONFIG_MX51_BBG		1	/* in a mx51 */
#define CONFIG_FLASH_HEADER	1
#define CONFIG_FLASH_HEADER_OFFSET 0x400
#define CONFIG_FLASH_HEADER_BARKER 0xB1

#define CONFIG_ARCH_CPU_INIT
#define CONFIG_ARCH_MMU

#define CONFIG_SKIP_RELOCATE_UBOOT

#define CONFIG_MX51_HCLK_FREQ	24000000	/* RedBoot says 26MHz */

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#define CONFIG_SYS_64BIT_VSPRINTF

#define BOARD_LATE_INIT
/*
 * Disabled for now due to build problems under Debian and a significant
 * increase in the final file size: 144260 vs. 109536 Bytes.
 */

#define CONFIG_CMDLINE_TAG		1	/* enable passing of ATAGs */
#define CONFIG_REVISION_TAG		1
#define CONFIG_SETUP_MEMORY_TAGS	1
#define CONFIG_INITRD_TAG		1

/*
 * Size of malloc() pool
 */
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + 2 * 1024 * 1024)
/* size in bytes reserved for initial data */
#define CONFIG_SYS_GBL_DATA_SIZE	128

/*
 * Hardware drivers
 */
#define CONFIG_MXC_UART 1
#define CONFIG_UART_BASE_ADDR   UART1_BASE_ADDR


//#define CONFIG_CMD_PING
//#define CONFIG_CMD_DHCP
//#define CONFIG_BOOTP_SUBNETMASK
//#define CONFIG_BOOTP_GATEWAY
//#define CONFIG_BOOTP_DNS
//#define CONFIG_CMD_MII
//#define CONFIG_CMD_NET
//#define CONFIG_NET_RETRY_COUNT	100
#define CONFIG_CMD_I2C

/*
 * Android support Configs
 */
#include <asm/mxc_key_defs.h>

#define CONFIG_USB_DEVICE
#define CONFIG_FASTBOOT		1
#define CONFIG_IMX_UDC		1
#define CONFIG_FASTBOOT_STORAGE_EMMC
#define CONFIG_FASTBOOT_VENDOR_ID	0xbb4
#define CONFIG_FASTBOOT_PRODUCT_ID	0xc01
#define CONFIG_FASTBOOT_BCD_DEVICE	0x311
#define CONFIG_FASTBOOT_MANUFACTURER_STR  "Freescale"
#define CONFIG_FASTBOOT_PRODUCT_NAME_STR "i.mx51"
#define CONFIG_FASTBOOT_CONFIGURATION_STR  "Android fastboot"
#define CONFIG_FASTBOOT_INTERFACE_STR    "Android fastboot"
#define CONFIG_FASTBOOT_SERIAL_NUM	 "12345"
#define CONFIG_FASTBOOT_MMC_NO		 2
#define CONFIG_FASTBOOT_TRANSFER_BUF	 0xA0000000
#define CONFIG_FASTBOOT_TRANSFER_BUF_SIZE 0x8000000 /* 128M byte */
#define CONFIG_MMC_8BIT_PORTS 4

#define CONFIG_ANDROID_RECOVERY
#define CONFIG_POWER_KEY	KEY_MENU
#define CONFIG_HOME_KEY		KEY_HOME
#define CONFIG_VOLUMEUP_KEY     KEY_VOLUMEUP

#define CONFIG_ANDROID_RECOVERY_MMCBOOT
#define CONFIG_ANDROID_RECOVERY_MMCUPDATE
#define CONFIG_ANDROID_RECOVERY_MMCDEVNO	0
#define CONFIG_ANDROID_BOOTMODE_FACTORY2
#define CONFIG_ANDROID_SOFTKEY_PAD

#ifdef CONFIG_ANDROID_RECOVERY_MMCBOOT
#define CONFIG_MMCBOOT_KEY	KEY_VOLUMEUP
#endif
#ifdef CONFIG_ANDROID_RECOVERY_MMCUPDATE
#define CONFIG_MMCUPDATE_KEY	KEY_VOLUMEDOWN
#endif

#define CONFIG_MTD_DEVICE
#define CONFIG_MTD_PARTITIONS


#define CONFIG_MXC_KPD
#define CONFIG_MXC_KEYMAPPING \
	{	\
		KEY_HOME, KEY_MENU, KEY_BACK, KEY_VOLUMEUP, KEY_VOLUMEDOWN,		\
		KEY_0, \
	}
#define CONFIG_MXC_KPD_COLMAX 3
#define CONFIG_MXC_KPD_ROWMAX 2
#define CONFIG_ANDROID_RECOVERY_BOOTARGS_MMC \
	"setenv bootargs ${bootargs} init=/init " \
	"androidboot.console=ttymxc0 di1_primary"
#define CONFIG_ANDROID_RECOVERY_BOOTCMD_MMC  \
	"run bootargs_base bootargs_android_recovery bootargs_display;"	\
	"mmc read 2 ${loadaddr} 0x2800 0x1800;"		\
	"mmc read 2 ${rd_loadaddr} 0x4000 0x4000;"		\
	"bootm ${loadaddr} ${rd_loadaddr}"
#define CONFIG_ANDROID_RECOVERY_CMD_FILE "/recovery/command"

#define CONFIG_ANDROID_SYSTEM_PARTITION_MMC 2
#define CONFIG_ANDROID_RECOVERY_PARTITION_MMC 4
#define CONFIG_ANDROID_CACHE_PARTITION_MMC 6

#ifdef CONFIG_ANDROID_RECOVERY_MMCBOOT
#define CONFIG_ANDROID_MMCBOOT_UIMAGE_FILE "/update/bin/uimage"
#define CONFIG_ANDROID_MMCBOOT_RAMDISK_FILE "/update/bin/uramdisk"
#endif
#ifdef CONFIG_ANDROID_RECOVERY_MMCUPDATE
#define CONFIG_ANDROID_MMCUP_UBOOT_IMAGE_PATH "/update/boot"
#define CONFIG_ANDROID_MMCUP_UBOOT_IMAGE_PREFIX "imx51_ivy"
#endif

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{9600, 19200, 38400, 57600, 115200}

/***********************************************************
 * Command definition
 ***********************************************************/

#include <config_cmd_default.h>

#undef CONFIG_CMD_PING
#undef CONFIG_CMD_DHCP
/* Enable below configure when supporting nand */
/* #define CONFIG_CMD_NAND */
#define CONFIG_CMD_MMC
#define CONFIG_CMD_ENV

#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY	0
#define CONFIG_ZERO_BOOTDELAY_CHECK

//#define CONFIG_PRIME	"FEC0"

#define CONFIG_LOADADDR		0x90800000	/* loadaddr env var */
#define CONFIG_RD_LOADADDR	(CONFIG_LOADADDR + 0x400000)

#define	CONFIG_EXTRA_ENV_SETTINGS					\
		"uboot_addr=0xa0000000\0"				\
		"uboot=u-boot.bin\0"			\
		"kernel=uImage\0"				\
		"rd_loadaddr=0x90C00000\0"	\
		"bootargs_base=setenv bootargs console=ttymxc0,115200 gpu_memory=64M "\
			"no_console_suspend "				\
			"crashkernel=1M@2560M panic=10\0"		\
		"prg_uboot=tftpboot ${loadaddr} ${uboot}; "		\
			"protect off ${uboot_addr} 0xa003ffff; "	\
			"erase ${uboot_addr} 0xa003ffff; "		\
			"cp.b ${loadaddr} ${uboot_addr} ${filesize}; "	\
			"setenv filesize; saveenv\0"			\
		"bootcmd=run bootcmd_emmc \0"				\
		"bootcmd_SD=run bootargs_base bootargs_android bootargs_display;"	\
		     "mmc read 0 ${loadaddr} 0x800 0x1800;"		\
		     "mmc read 0 ${rd_loadaddr} 0x2000 0x800;"		\
		     "bootm ${loadaddr} ${rd_loadaddr}\0"		\
		"bootcmd_emmc=run bootargs_base bootargs_android "	\
		     "bootargs_display_32bpp;"				\
		     "mmc read 2 ${loadaddr} 0x800 0x1800;"		\
		     "mmc read 2 ${rd_loadaddr} 0x2000 0x800;"		\
		     "bootm ${loadaddr} ${rd_loadaddr}\0"		\
		"bootcmd_android_factory2=run bootargs_base bootargs_android bootargs_factory2 "	\
		     "bootargs_display_32bpp;"				\
		     "mmc read 2 ${loadaddr} 0x800 0x1800;"		\
		     "mmc read 2 ${rd_loadaddr} 0x2000 0x800;"		\
		     "bootm ${loadaddr} ${rd_loadaddr}\0"		\
		"bootargs_factory2=setenv bootargs ${bootargs} androidboot.mode=factory2 \0"	\
		"bootargs_android=setenv bootargs ${bootargs} "		\
		     "androidboot.console=ttymxc0 init=/init\0"		\
		"bootcmd_android_recovery=run bootargs_base"		\
		     " bootargs_android_recovery bootargs_display;"	\
		     "mmc read 2 ${loadaddr} 0x2800 0x1800;"		\
		     "mmc read 2 ${rd_loadaddr} 0x4000 0x4000;"		\
		     "bootm ${loadaddr} ${rd_loadaddr}\0"		\
		"bootargs_android_recovery=setenv bootargs ${bootargs} "\
		     "androidboot.console=ttymxc0 init=/init\0"		\
		"bootargs_display=setenv bootargs ${bootargs} "		\
		     "\0"						\
		"bootargs_display_32bpp=setenv bootargs ${bootargs} "	\
		     "video=mxcdi0fb:bpp=32\0"				\
		"bootcmd_mmcboot=run bootargs_base "			\
		     "bootargs_mmcboot bootargs_display;"		\
		     "fatload mmc 0:1 ${loadaddr} /update/bin/uimage;"	\
		     "fatload mmc 0:1 ${rd_loadaddr} /update/bin/uramdisk.img;"	\
		     "bootm ${loadaddr} ${rd_loadaddr}\0"		\
		"bootargs_mmcboot=setenv bootargs ${bootargs} " 	\
		     "init=/init\0"					\




/*Support LAN9217*/
/*
#define CONFIG_SMC911X	1
#define CONFIG_SMC911X_16_BIT 1
#define CONFIG_SMC911X_BASE mx51_io_base_addr
*/

/*
 * The MX51 3stack board seems to have a hardware "peculiarity" confirmed under
 * U-Boot, RedBoot and Linux: the ethernet Rx signal is reaching the CS8900A
 * controller inverted. The controller is capable of detecting and correcting
 * this, but it needs 4 network packets for that. Which means, at startup, you
 * will not receive answers to the first 4 packest, unless there have been some
 * broadcasts on the network, or your board is on a hub. Reducing the ARP
 * timeout from default 5 seconds to 200ms we speed up the initial TFTP
 * transfer, should the user wish one, significantly.
 */
#define CONFIG_ARP_TIMEOUT	200UL

/*
 * Miscellaneous configurable options
 */
#define CONFIG_SYS_LONGHELP		/* undef to save memory */
#define CONFIG_SYS_PROMPT		"IVY U-Boot > "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE		256	/* Console I/O Buffer Size */
/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS	16	/* max number of command args */
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE /* Boot Argument Buffer Size */

#define CONFIG_SYS_MEMTEST_START	0	/* memtest works on */
#define CONFIG_SYS_MEMTEST_END		0x10000

#undef	CONFIG_SYS_CLKS_IN_HZ		/* everything, incl board info, in Hz */

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

#define CONFIG_SYS_HZ				1000

#define CONFIG_CMDLINE_EDITING	1

/*
 * Eth Configs
 */
//#define CONFIG_HAS_ETH1
#define CONFIG_NET_MULTI 1
//#define CONFIG_MXC_FEC
//#define CONFIG_MII
//#define CONFIG_DISCOVER_PHY

//#define CONFIG_FEC0_IOBASE	FEC_BASE_ADDR
//#define CONFIG_FEC0_PINMUX	-1
//#define CONFIG_FEC0_PHY_ADDR	0x1F
//#define CONFIG_FEC0_MIIBASE 	-1

/*
 * SPI Configs
 * */
#define CONFIG_FSL_SF		1
#define CONFIG_CMD_SPI
//#define CONFIG_CMD_SF
//#define CONFIG_SPI_FLASH_IMX_ATMEL	1
//#define CONFIG_SPI_FLASH_CS	1
#define CONFIG_IMX_ECSPI
#define CONFIG_IMX_SPI_PMIC
#define CONFIG_IMX_SPI_PMIC_CS 0
#define IMX_CSPI_VER_2_3        1

#define MAX_SPI_BYTES		(64 * 4)

/*
 * MMC Configs
 * */
#ifdef CONFIG_CMD_MMC
	#define CONFIG_MMC				1
	#define CONFIG_GENERIC_MMC
	#define CONFIG_IMX_MMC
	#define CONFIG_SYS_FSL_ESDHC_NUM	3
	#define CONFIG_SYS_FSL_ESDHC_ADDR       0
	#define CONFIG_SYS_MMC_ENV_DEV	2
	#define CONFIG_DOS_PARTITION	1
	#define CONFIG_CMD_FAT		1
	#define CONFIG_CMD_EXT2		1
	#define CONFIG_DYNAMIC_MMC_DEVNO
#endif

/*
 * I2C Configs
 */
#define CONFIG_HARD_I2C         1
#define CONFIG_I2C_MXC          1
#define CONFIG_SYS_I2C_PORT             I2C1_BASE_ADDR
#define CONFIG_SYS_I2C_SPEED            400000
#define CONFIG_SYS_I2C_SLAVE            0xfe

/*-----------------------------------------------------------------------
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE	(128 * 1024)	/* regular stack */

/*-----------------------------------------------------------------------
 * Physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS	1
#define PHYS_SDRAM_1		CSD0_BASE_ADDR
#define PHYS_SDRAM_1_SIZE	(512 * 1024 * 1024)
#define iomem_valid_addr(addr, size) \
	(addr >= PHYS_SDRAM_1 && addr <= (PHYS_SDRAM_1 + PHYS_SDRAM_1_SIZE))

/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */
#define CONFIG_SYS_NO_FLASH

/*-----------------------------------------------------------------------
 * NAND FLASH driver setup
 */
#define NAND_MAX_CHIPS         8
#define CONFIG_SYS_MAX_NAND_DEVICE    1
#define CONFIG_SYS_NAND_BASE          0x40000000

/* Monitor at beginning of flash */
/* #define CONFIG_FSL_ENV_IN_SF */
#define CONFIG_FSL_ENV_IN_MMC
/* #define CONFIG_FSL_ENV_IN_NAND */

#define CONFIG_ENV_SECT_SIZE    (128 * 1024)
#define CONFIG_ENV_SIZE         CONFIG_ENV_SECT_SIZE

#if defined(CONFIG_FSL_ENV_IN_NAND)
	#define CONFIG_ENV_IS_IN_NAND 1
	#define CONFIG_ENV_OFFSET	0x100000
#elif defined(CONFIG_FSL_ENV_IN_MMC)
	#define CONFIG_ENV_IS_IN_MMC	1
	#define CONFIG_ENV_OFFSET	(768 * 1024)
#elif defined(CONFIG_FSL_ENV_IN_SF)
	#define CONFIG_ENV_IS_IN_SPI_FLASH	1
	#define CONFIG_ENV_SPI_CS		1
	#define CONFIG_ENV_OFFSET       (768 * 1024)
#else
	#define CONFIG_ENV_IS_NOWHERE	1
#endif
/*
 * JFFS2 partitions
 */
#undef CONFIG_JFFS2_CMDLINE
#define CONFIG_JFFS2_DEV	"nand0"

#define CONFIG_LCD
#undef CONFIG_RAW_LOGO
#undef CONFIG_LCD_LOGO
#undef CONFIG_SPLASH_SCREEN
#if defined(CONFIG_LCD)
	/*
	 * Framebuffer and LCD
	 */
	#define CONFIG_VIDEO_MX5
	#define CONFIG_MXC_HSC
	#define CONFIG_IPU_CLKRATE	133000000
	#define CONFIG_SYS_CONSOLE_ENV_OVERWRITE
	#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
	#define CONFIG_SYS_CONSOLE_IS_IN_ENV
	#define LCD_BPP		LCD_COLOR16
	#define CONFIG_CMD_BMP
	#define CONFIG_BMP_8BPP
	#define CONFIG_BMP_16BPP
	#define CONFIG_FB_BASE	(TEXT_BASE + 0x300000)
	#define CONFIG_SPLASH_SCREEN_ALIGN
	#define CONFIG_SYS_WHITE_ON_BLACK
	#define LOGO_OFFSET_X 0
	#define LOGO_OFFSET_Y 0

	#define CONFIG_IMX_PWM
	#define IMX_PWM1_BASE    PWM1_BASE_ADDR
	#define IMX_PWM2_BASE    PWM2_BASE_ADDR
#endif

#define CONFIG_SAVE_KPANIC
#define CONFIG_SAVE_KPANIC_OFFSET	(31 * 1024 * 1024) /* offset in mmc */
#define CONFIG_SAVE_KPANIC_SIZE		(1000 * 1024)      /* size in mmc */
#define CONFIG_KPANIC_ADDR		0xa0000000	/* kpanic data addr */

#endif				/* __CONFIG_H */
