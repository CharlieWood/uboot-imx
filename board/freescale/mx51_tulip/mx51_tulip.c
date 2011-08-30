/*
 * Copyright (C) 2007, Guennadi Liakhovetski <lg@denx.de>
 *
 * (C) Copyright 2009-2010 Freescale Semiconductor, Inc.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/mx51.h>
#include <asm/arch/mx51_pins.h>
#include <asm/arch/iomux.h>
#include <asm/errno.h>
#include <i2c.h>
#if defined(CONFIG_VIDEO_MX5)
#include <asm/imx_pwm.h>
#include <linux/list.h>
#include <ipu.h>
#include <lcd.h>
#include <linux/fb.h>
#include <linux/mxcfb.h>
#endif
#include "board-imx51.h"
#ifdef CONFIG_IMX_ECSPI
#include <imx_spi.h>
#include <asm/arch/imx_spi_pmic.h>
#endif

#include <asm/errno.h>

#ifdef CONFIG_CMD_MMC
#include <mmc.h>
#include <fsl_esdhc.h>
#endif

#ifdef CONFIG_ARCH_MMU
#include <asm/mmu.h>
#include <asm/arch/mmu.h>
#endif

#ifdef CONFIG_GET_FEC_MAC_ADDR_FROM_IIM
#include <asm/imx_iim.h>
#endif

#ifdef CONFIG_ANDROID_RECOVERY
#include "../common/recovery.h"
#include <part.h>
#include <ext2fs.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <ubi_uboot.h>
#include <jffs2/load_kernel.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

static u32 system_rev;
static enum boot_device boot_dev;
u32	mx51_io_base_addr;

#ifdef CONFIG_VIDEO_MX5
extern unsigned char fsl_bmp_600x400[];
extern int fsl_bmp_600x400_size;

#if defined(CONFIG_BMP_8BPP)
short colormap[256];
#elif defined(CONFIG_BMP_16BPP)
short colormap[65536];
#else
short colormap[16777216];
#endif

struct pwm_device pwm0 = {
	.pwm_id = 0,
	.pwmo_invert = 0,
};

extern int ipuv3_fb_init(struct fb_videomode *mode, int di,
			int interface_pix_fmt,
			ipu_di_clk_parent_t di_clk_parent,
			int di_clk_val);

static struct fb_videomode byd_wxga = {
	/* 1024x600 @ 60 Hz , pixel clk @ 44.333MHz / 22557 ps */
	"HSD070PFW1", 60, 1024, 600, 22557, 84, 88, 8, 6, 4, 2,
	0,	/* FB_SYNC_CLK_LAT_FALL, */
	FB_VMODE_NONINTERLACED,
	0,
};

vidinfo_t panel_info;
#endif

static inline void setup_boot_device(void)
{
	uint *fis_addr = (uint *)IRAM_BASE_ADDR;

	switch (*fis_addr) {
	case NAND_FLASH_BOOT:
		boot_dev = NAND_BOOT;
		break;
	case SPI_NOR_FLASH_BOOT:
		boot_dev = SPI_NOR_BOOT;
		break;
	case MMC_FLASH_BOOT:
		boot_dev = MMC_BOOT;
		break;
	default:
		{
			uint soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
			uint bt_mem_ctl = soc_sbmr & 0x00000003;
			uint bt_mem_type = (soc_sbmr & 0x00000180) >> 7;

			switch (bt_mem_ctl) {
			case 0x3:
				if (bt_mem_type == 0)
					boot_dev = MMC_BOOT;
				else if (bt_mem_type == 3)
					boot_dev = SPI_NOR_BOOT;
				else
					boot_dev = UNKNOWN_BOOT;
				break;
			case 0x1:
				boot_dev = NAND_BOOT;
				break;
			default:
				boot_dev = UNKNOWN_BOOT;
			}
		}
		break;
	}
}

enum boot_device get_boot_device(void)
{
	return boot_dev;
}

u32 get_board_rev(void)
{
	return system_rev;
}

static inline void setup_soc_rev(void)
{
	int reg;
#ifdef CONFIG_ARCH_MMU
	reg = __REG(0x20000000 + ROM_SI_REV); /* Virtual address */
#else
	reg = __REG(ROM_SI_REV);
#endif

	switch (reg) {
	case 0x02:
		system_rev = 0x51000 | CHIP_REV_1_1;
		break;
	case 0x10:
		system_rev = 0x51000 | CHIP_REV_2_0;
		break;
	case 0x20:
		system_rev = 0x51000 | CHIP_REV_3_0;
		break;
	default:
		system_rev = 0x51000 | CHIP_REV_1_0;
	}
}

static inline void set_board_rev(void)
{
	if ((__REG(GPIO1_BASE_ADDR + 0x0) & (0x1 << 22)) == 0)
		system_rev |= BOARD_REV_2_0 << BOARD_VER_OFFSET;

}

inline int is_soc_rev(int rev)
{
	return (system_rev & 0xFF) - rev;
}

static int is_board_rev(int rev)
{
	return (((system_rev & 0x0F00) >> 8) == rev) ? 1 : 0;
}

#ifdef CONFIG_ARCH_MMU
void board_mmu_init(void)
{
	unsigned long ttb_base = PHYS_SDRAM_1 + 0x4000;
	unsigned long i;

	/*
	* Set the TTB register
	*/
	asm volatile ("mcr p15, 0, %0, c2, c0, 0" : : "r"(ttb_base) /*:*/);

	/*
	* Set the Domain Access Control Register
	*/
	i = ARM_ACCESS_DACR_DEFAULT;
	asm volatile ("mcr p15, 0, %0, c3, c0, 0" : : "r"(i) /*:*/);

	/*
	* First clear all TT entries - ie Set them to Faulting
	*/
	memset((void *)ttb_base, 0, ARM_FIRST_LEVEL_PAGE_TABLE_SIZE);
	/* Actual   Virtual  Size   Attributes          Function */
	/* Base     Base     MB     cached? buffered?  access permissions */
	/* xxx00000 xxx00000 */
	X_ARM_MMU_SECTION(0x000, 0x200, 0x1,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* ROM */
	X_ARM_MMU_SECTION(0x1FF, 0x1FF, 0x001,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* IRAM */
	X_ARM_MMU_SECTION(0x300, 0x300, 0x100,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* GPU */
	X_ARM_MMU_SECTION(0x400, 0x400, 0x200,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* IPUv3D */
	X_ARM_MMU_SECTION(0x600, 0x600, 0x300,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* periperals */
	X_ARM_MMU_SECTION(0x900, 0x000, 0x1FF,
			ARM_CACHEABLE, ARM_BUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* SDRAM */
	X_ARM_MMU_SECTION(0x900, 0x900, 0x200,
			ARM_CACHEABLE, ARM_BUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* SDRAM */
	X_ARM_MMU_SECTION(0x900, 0xE00, 0x200,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* SDRAM 0:128M*/
	X_ARM_MMU_SECTION(0xB80, 0xB80, 0x10,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* CS1 EIM control*/
	X_ARM_MMU_SECTION(0xCC0, 0xCC0, 0x040,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* CS4/5/NAND Flash buffer */

	/* Workaround for arm errata #709718 */
	/* Setup PRRR so device is always mapped to non-shared */
	asm volatile ("mrc p15, 0, %0, c10, c2, 0" : "=r"(i) : /*:*/);
	i &= (~(3 << 0x10));
	asm volatile ("mcr p15, 0, %0, c10, c2, 0" : : "r"(i) /*:*/);

	/* Enable MMU */
	MMU_ON();
}
#endif

int dram_init(void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = PHYS_SDRAM_1_SIZE;
	return 0;
}

static void setup_uart(void)
{
	unsigned int pad = PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE |
			 PAD_CTL_PUE_PULL | PAD_CTL_DRV_HIGH;
	mxc_request_iomux(MX51_PIN_UART1_RXD, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_UART1_RXD, pad | PAD_CTL_SRE_FAST);
	mxc_request_iomux(MX51_PIN_UART1_TXD, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_UART1_TXD, pad | PAD_CTL_SRE_FAST);
	mxc_request_iomux(MX51_PIN_UART1_RTS, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_UART1_RTS, pad);
	mxc_request_iomux(MX51_PIN_UART1_CTS, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_UART1_CTS, pad);
}

static void setup_expio(void)
{
	u32 reg;
#if 0
	/* CS5 setup */
	mxc_request_iomux(MX51_PIN_EIM_CS5, IOMUX_CONFIG_ALT0);
	writel(0x00410089, WEIM_BASE_ADDR + 0x78 + CSGCR1);
	writel(0x00000002, WEIM_BASE_ADDR + 0x78 + CSGCR2);
	/* RWSC=50, RADVA=2, RADVN=6, OEA=0, OEN=0, RCSA=0, RCSN=0 */
	writel(0x32260000, WEIM_BASE_ADDR + 0x78 + CSRCR1);
	/* APR = 0 */
	writel(0x00000000, WEIM_BASE_ADDR + 0x78 + CSRCR2);
	/* WAL=0, WBED=1, WWSC=50, WADVA=2, WADVN=6, WEA=0, WEN=0,
	 * WCSA=0, WCSN=0
	 */
	writel(0x72080F00, WEIM_BASE_ADDR + 0x78 + CSWCR1);
	if ((readw(CS5_BASE_ADDR + PBC_ID_AAAA) == 0xAAAA) &&
	    (readw(CS5_BASE_ADDR + PBC_ID_5555) == 0x5555)) {
		if (is_soc_rev(CHIP_REV_2_0) < 0) {
			reg = readl(CCM_BASE_ADDR + CLKCTL_CBCDR);
			reg = (reg & (~0x70000)) | 0x30000;
			writel(reg, CCM_BASE_ADDR + CLKCTL_CBCDR);
			/* make sure divider effective */
			while (readl(CCM_BASE_ADDR + CLKCTL_CDHIPR) != 0)
				;
			writel(0x0, CCM_BASE_ADDR + CLKCTL_CCDR);
		}
		mx51_io_base_addr = CS5_BASE_ADDR;
	} else {
		/* CS1 */
		writel(0x00410089, WEIM_BASE_ADDR + 0x18 + CSGCR1);
		writel(0x00000002, WEIM_BASE_ADDR + 0x18 + CSGCR2);
		/*  RWSC=50, RADVA=2, RADVN=6, OEA=0, OEN=0, RCSA=0, RCSN=0 */
		writel(0x32260000, WEIM_BASE_ADDR + 0x18 + CSRCR1);
		/* APR=0 */
		writel(0x00000000, WEIM_BASE_ADDR + 0x18 + CSRCR2);
		/* WAL=0, WBED=1, WWSC=50, WADVA=2, WADVN=6, WEA=0,
		 * WEN=0, WCSA=0, WCSN=0
		 */
		writel(0x72080F00, WEIM_BASE_ADDR + 0x18 + CSWCR1);
		mx51_io_base_addr = CS1_BASE_ADDR;
	}
#endif

	/* Reset interrupt status reg */
	writew(0x1F, mx51_io_base_addr + PBC_INT_REST);
	writew(0x00, mx51_io_base_addr + PBC_INT_REST);
	writew(0xFFFF, mx51_io_base_addr + PBC_INT_MASK);

	/* Reset the XUART and Ethernet controllers */
	reg = readw(mx51_io_base_addr + PBC_SW_RESET);
	reg |= 0x9;
	writew(reg, mx51_io_base_addr + PBC_SW_RESET);
	reg &= ~0x9;
	writew(reg, mx51_io_base_addr + PBC_SW_RESET);
}

static void board_sys_on_off_ctl(int on)
{
	unsigned int val;

	/* SYS_ON_OFF_CTL(GPIO1_23) pin control */
	val = readl(GPIO1_BASE_ADDR + 0x4);	/* GPIO1 GDIR */
	val |= 0x00000008;  /* configure GPIO1_3 lines as output */
	writel(val, GPIO1_BASE_ADDR + 0x4);
	val = readl(GPIO1_BASE_ADDR + 0x0);	/* GPIO1 DR */
	if (on)
		val |= 0x00000008;  /* configure GPIO1_3 lines to high */
	else
		val &= ~0x00000008;  /* configure GPIO1_3 lines to low */
	writel(val, GPIO1_BASE_ADDR + 0x0);
	mxc_request_iomux(MX51_PIN_GPIO1_3, IOMUX_CONFIG_ALT0);

	if (!on)
		while(1);	/* wait to power off */
}

#ifdef CONFIG_IMX_ECSPI
s32 spi_get_cfg(struct imx_spi_dev_t *dev)
{
	switch (dev->slave.cs) {
	case 0:
		/* pmic */
		dev->base = CSPI1_BASE_ADDR;
		dev->freq = 2500000;
		dev->ss_pol = IMX_SPI_ACTIVE_HIGH;
		dev->ss = 0;
		dev->fifo_sz = 64 * 4;
		dev->us_delay = 0;
		break;
	case 1:
		/* spi_nor */
		dev->base = CSPI1_BASE_ADDR;
		dev->freq = 2500000;
		dev->ss_pol = IMX_SPI_ACTIVE_LOW;
		dev->ss = 1;
		dev->fifo_sz = 64 * 4;
		dev->us_delay = 0;
		break;
	default:
		printf("Invalid Bus ID! \n");
		break;
	}

	return 0;
}

void spi_io_init(struct imx_spi_dev_t *dev)
{
	switch (dev->base) {
	case CSPI1_BASE_ADDR:
		/* 000: Select mux mode: ALT0 mux port: MOSI of instance: ecspi1 */
		mxc_request_iomux(MX51_PIN_CSPI1_MOSI, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_CSPI1_MOSI, 0x105);

		/* 000: Select mux mode: ALT0 mux port: MISO of instance: ecspi1. */
		mxc_request_iomux(MX51_PIN_CSPI1_MISO, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_CSPI1_MISO, 0x105);

		if (dev->ss == 0) {
			/* de-select SS1 of instance: ecspi1. */
			mxc_request_iomux(MX51_PIN_CSPI1_SS1, IOMUX_CONFIG_ALT3);
			mxc_iomux_set_pad(MX51_PIN_CSPI1_SS1, 0x85);
			/* 000: Select mux mode: ALT0 mux port: SS0 of instance: ecspi1. */
			mxc_request_iomux(MX51_PIN_CSPI1_SS0, IOMUX_CONFIG_ALT0);
			mxc_iomux_set_pad(MX51_PIN_CSPI1_SS0, 0x185);
		} else if (dev->ss == 1) {
			/* de-select SS0 of instance: ecspi1. */
			mxc_request_iomux(MX51_PIN_CSPI1_SS0, IOMUX_CONFIG_ALT3);
			mxc_iomux_set_pad(MX51_PIN_CSPI1_SS0, 0x85);
			/* 000: Select mux mode: ALT0 mux port: SS1 of instance: ecspi1. */
			mxc_request_iomux(MX51_PIN_CSPI1_SS1, IOMUX_CONFIG_ALT0);
			mxc_iomux_set_pad(MX51_PIN_CSPI1_SS1, 0x105);
		}

		/* 000: Select mux mode: ALT0 mux port: RDY of instance: ecspi1. */
		mxc_request_iomux(MX51_PIN_CSPI1_RDY, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_CSPI1_RDY, 0x180);

		/* 000: Select mux mode: ALT0 mux port: SCLK of instance: ecspi1. */
		mxc_request_iomux(MX51_PIN_CSPI1_SCLK, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_CSPI1_SCLK, 0x105);
		break;
	case CSPI2_BASE_ADDR:
	default:
		break;
	}
}
#endif

#ifdef CONFIG_I2C_MXC
static void setup_i2c(unsigned int module_base)
{
	unsigned int reg;

	switch (module_base) {
	case I2C1_BASE_ADDR:
		reg = IOMUXC_BASE_ADDR + 0x5c; /* i2c1 SDA */
		writel(0x14, reg);
		reg = IOMUXC_BASE_ADDR + 0x3f0;
		writel(0x10d, reg);
		reg = IOMUXC_BASE_ADDR + 0x9B4;
		writel(0x0, reg);

		reg = IOMUXC_BASE_ADDR + 0x68; /* i2c2 SCL */
		writel(0x14, reg);
		reg = IOMUXC_BASE_ADDR + 0x3fc;
		writel(0x10d, reg);
		reg = IOMUXC_BASE_ADDR + 0x9B0;
		writel(0x0, reg);
		break;
	case I2C2_BASE_ADDR:
		/* dummy here*/
		break;
	default:
		printf("Invalid I2C base: 0x%x\n", module_base);
		break;
	}
}

static void setup_core_voltage_i2c(void)
{
	unsigned int reg;
	unsigned char buf[1] = { 0 };

	puts("PMIC Mode: linear\n");

	writel(0x0, CCM_BASE_ADDR + CLKCTL_CACRR);

	/*Configure LDO4*/
	i2c_read(0x34, 0x12, 1, buf, 1);
	buf[0] = buf[0] | 0x40;
	if (i2c_write(0x34, 0x12, 1, buf, 1)) {
		puts("write to PMIC 0x12 failed!\n");
		return;
	}
	i2c_read(0x34, 0x12, 1, buf, 1);
	printf("PMIC 0x12: 0x%x \n", buf[0]);

	i2c_read(0x34, 0x10, 1, buf, 1);
	buf[0] = buf[0] | 0x40;
	if (i2c_write(0x34, 0x10, 1, buf, 1)) {
		puts("write to PMIC 0x10 failed!\n");
		return;
	}
	i2c_read(0x34, 0x10, 1, buf, 1);
	printf("PMIC 0x10: 0x%x \n", buf[0]);

	udelay(500);
}
#endif

#ifdef CONFIG_IMX_ECSPI
static void mc13892_adc_init(struct spi_slave *slave)
{
	unsigned int val;

	/* CONFIGURE Interrup Mask REG 0 */
	val = pmic_reg(slave, 1, 0, 0);
	val |= 0x03;		/* bit 1 ADCBISDONEM, bit 0 ADCDONEM */
	pmic_reg(slave, 1, val, 1);

	/* initialize ADC registers */
	pmic_reg(slave, 43, 0x008000, 1);
	pmic_reg(slave, 44, 0, 1);
	pmic_reg(slave, 45, 0, 1);
	pmic_reg(slave, 46, 0x0001c0, 1);
	pmic_reg(slave, 47, 0, 1);

	/* clear Interrup Status REG 0 ADCBISDONEI */
	pmic_reg(slave, 0, 0x02, 1);

	/* do ADC calibration */
	/* ADC0:
	   bit 23 - ADCBIS0=1
	   bit 17 - ADINC2=1
	   bit 16 - ADINC1=1
	   bit 15 - CHRGRAWDIV=1 */
	pmic_reg(slave, 43, 0x838000, 1);

	/* ADC1:
	   bit 23 - ADCBIS1=1
	   bit 1 -  RAND=1
	   bit 18-11 - ATO[7:0]=1
           bit 0 -  ADEN=1 */
	pmic_reg(slave, 44, 0x800007, 1);

	/* CONFIGURE ADC REG 3 */
	pmic_reg(slave, 46, 0x800000, 1);
}

static int mc13892_adc_convert(struct spi_slave *slave, int channel, unsigned int *result)
{
	unsigned int adc_0_reg, adc_1_reg, result_reg, val;
	int i;

	/* clear Interrup Status REG 0 ADCBISDONEI */
	pmic_reg(slave, 0, 0x02, 1);

	/* CONFIGURE ADC REG 0 */
	adc_0_reg = pmic_reg(slave, 43, 0, 0);
	/* ADC0:
	   bit 23 - ADCBIS0=1
	   bit 17 - ADINC2=1
	   bit 16 - ADINC1=1
	   bit 15 - CHRGRAWDIV=1 */
	adc_0_reg = (adc_0_reg & ~0x838000) | 0x838000;
	if (channel == 1)		/* battery current channel */
		adc_0_reg |= 0x04;	/* bit 2 - BATICON=1 */
	if (channel == 4)		/* charger current channel */
		adc_0_reg |= 0x02;	/* bit 1 - CHRGICON=1 */
	pmic_reg(slave, 43, adc_0_reg, 1);

	/* CONFIGURE ADC REG 1 */
	adc_1_reg = pmic_reg(slave, 44, 0, 0);
	/* ADC1:
	   bit 23 - ADCBIS1=1
	   bit 21 - ADTRIGIGN=1
	   bit 20 - ASC=1
	   bit 1 -  RAND=1
	   bit 18-11 - ATO[7:0]=1
           bit 0 -  ADEN=1 */
	adc_1_reg = 0xb00003;	/* 0xb00803 */
	/* bit 7:5 ADA1[2:0] */
	adc_1_reg |= (channel & 0x07) << 5;
	pmic_reg(slave, 44, adc_1_reg, 1);

	/* CONFIGURE ADC REG 3 */
	pmic_reg(slave, 46, 0x800000, 1);

	/* wait for adcbisdone interrupt */
	i = 1000;
	while (i--) {
		/* read Interrup Status REG 0 */
		val = pmic_reg(slave, 0, 0, 0);
		if (val & 0x02)		/* bit 1 ADCBISDONEM */
			break;
		udelay(1);
	}
	/* clear Interrup Status REG 0 ADCBISDONEI */
	pmic_reg(slave, 0, 0x02, 1);

	if (i==0) {
		printf("mc13892 adc timeout on channel %d\n", channel);
		return -1;
	}

	/* ADC1: bit 10:8 ADA2[2:0]=4, bit 7:5 ADA1[2:0]=0 */
	adc_1_reg = pmic_reg(slave, 44, 0, 0);
	adc_1_reg = (adc_1_reg & ~0x7e0) | (4<<8) | (0<<5);
	pmic_reg(slave, 44, adc_1_reg, 1);

	for (i = 0; i <= 3; i++) {
		/* read ADC result from ADC4 */
		result_reg = pmic_reg(slave, 47, 0, 0);
		result[i] = ((result_reg & 0x00000FFC) >> 2);
		result[i+4] = ((result_reg & 0x00FFC000) >> 14);
	}
	return 0;
}

/* get battery voltage, return value unit mV */
static int mc13892_get_bat_voltage(struct spi_slave *slave)
{
	unsigned int result[8];

	mc13892_adc_convert(slave, 7, result);
	return result[0] * 9384 / 1000;
}

/* get battery current, return value unit mA */
static int mc13892_get_bp_current(struct spi_slave *slave)
{
	unsigned int result[8];
	int current_raw, current_mA;

	mc13892_adc_convert(slave, 1, result);

	current_raw = result[0];
	if (current_raw & 0x200)
		current_mA = -5870 * (0x200 - (current_raw & 0x1FF)) / 1000;
	else
		current_mA = 5870 * (current_raw & 0x1FF) / 1000;

	return current_mA;
}

/* get baseband voltage, return value unit mV */
static int mc13892_get_bp_voltage(struct spi_slave *slave)
{
	unsigned int result[8];

	mc13892_adc_convert(slave, 2, result);
	return result[0] * 4692 / 1000;
}

static void check_battery(struct spi_slave *slave)
{
	int bat_vol_mV, bp_curr_mA, bp_vol_mV;

	bat_vol_mV = mc13892_get_bat_voltage(slave);
	bp_curr_mA = mc13892_get_bp_current(slave);
	bp_vol_mV = mc13892_get_bp_voltage(slave);
	printf("PMIC Battery Voltage: %d mV\n", bat_vol_mV);
	printf("PMIC AP Voltage: %d mV\n", bp_vol_mV);
	printf("PMIC AP Current: %d mA\n", bp_curr_mA);

	if (bat_vol_mV < 7000) {
		/* battery voltage lower than 7.0V */
		puts("Battery lower than 7.0V, have to shutdown!!!\n");
		//board_sys_on_off_ctl(0);
	}
}

static void setup_core_voltage_spi(void)
{
	struct spi_slave *slave;
	unsigned int val;
	unsigned int reg;

	puts("PMIC Mode: SPI\n");

#define REV_ATLAS_LITE_1_0         0x8
#define REV_ATLAS_LITE_1_1         0x9
#define REV_ATLAS_LITE_2_0         0x10
#define REV_ATLAS_LITE_2_1         0x11

	slave = spi_pmic_probe();

	/* Write needed to Power Gate 2 register */
	val = pmic_reg(slave, 34, 0, 0);
	val &= ~0x10000;
	pmic_reg(slave, 34, val, 1);

	/* For BBG2.5 and BBG3.0, set SPI register 48 = 0x21807B
	 * and  disable the PLIM.
	 * MC13892 has an inside charge timer which expires in 120 minutes.
	 * If ICHRG and CHGAUTOB are not set properly, this timer expiration
	 * will get system power recycled.
	 * Since BBG has no Li-Ion battery on board, sets
	 * ICHRG in externally powered mode and sets CHGAUTOB bit to avoid
	 * automatic charging, so that system will not get reset by this
	 * timer expiration.
	 * Set the charge regulator output voltage as 4.2V by default
	 * according to MC13892 spec
	 */
	if (is_board_rev(BOARD_REV_2_0))
		pmic_reg(slave, 48, 0x0023807B, 1);

	/* power up the system first */
	pmic_reg(slave, 34, 0x00200000, 1);

	if (is_soc_rev(CHIP_REV_2_0) <= 0) {
		/* Set core voltage to 1.1V */
		val = pmic_reg(slave, 24, 0, 0);
		val = (val & (~0x1f)) | 0x14;
		pmic_reg(slave, 24, val, 1);

		/* Setup VCC (SW2) to 1.25 */
		val = pmic_reg(slave, 25, 0, 0);
		val = (val & (~0x1f)) | 0x1a;
		pmic_reg(slave, 25, val, 1);

		/* Setup 1V2_DIG1 (SW3) to 1.25 */
		val = pmic_reg(slave, 26, 0, 0);
		val = (val & (~0x1f)) | 0x1a;
		pmic_reg(slave, 26, val, 1);
		udelay(50);
		/* Raise the core frequency to 800MHz */
		writel(0x0, CCM_BASE_ADDR + CLKCTL_CACRR);
	} else {
		/* TO 3.0 */
		/* Setup VCC (SW2) to 1.225 */
		val = pmic_reg(slave, 25, 0, 0);
		val = (val & (~0x1f)) | 0x19;
		pmic_reg(slave, 25, val, 1);

		/* Setup 1V2_DIG1 (SW3) to 1.2 */
		val = pmic_reg(slave, 26, 0, 0);
		val = (val & (~0x1f)) | 0x18;
		pmic_reg(slave, 26, val, 1);
	}

	if (((pmic_reg(slave, 7, 0, 0) & 0x1f) < REV_ATLAS_LITE_2_0) ||
		(((pmic_reg(slave, 7, 0, 0) >> 9) & 0x3) == 0)) {
		/* Set switchers in PWM mode for Atlas 2.0 and lower */
		/* Setup the switcher mode for SW1 & SW2*/
		val = pmic_reg(slave, 28, 0, 0);
		val = (val & (~0x3c0f)) | 0x1405;
		pmic_reg(slave, 28, val, 1);

		/* Setup the switcher mode for SW3 & SW4 */
		val = pmic_reg(slave, 29, 0, 0);
		val = (val & (~0xf0f)) | 0x505;
		pmic_reg(slave, 29, val, 1);
	} else {
		/* Set switchers in Auto in NORMAL mode & STANDBY mode for Atlas 2.0a */
		/* Setup the switcher mode for SW1 & SW2*/
		val = pmic_reg(slave, 28, 0, 0);
		val = (val & (~0x3c0f)) | 0x2008;
		pmic_reg(slave, 28, val, 1);

		/* Setup the switcher mode for SW3 & SW4 */
		val = pmic_reg(slave, 29, 0, 0);
		val = (val & (~0xf0f)) | 0x808;
		pmic_reg(slave, 29, val, 1);
	}

	/* Set VDIG to 1.65V, VGEN3 to 1.8V, VCAM to 2.5V */
	val = pmic_reg(slave, 30, 0, 0);
	val &= ~0x34030;
	val |= 0x10020;
	pmic_reg(slave, 30, val, 1);

	/* Set VVIDEO to 2.775V, VAUDIO to 2.775V, VSD to 3.15V */
	val = pmic_reg(slave, 31, 0, 0);
	val &= ~0x1fc;
	val |= 0x1e4;
	pmic_reg(slave, 31, val, 1);

	/* Configure VGEN3 and VCAM regulators to use external PNP */
	val = 0x208;
	pmic_reg(slave, 33, val, 1);
	udelay(200);

	/* Enable VGEN3, VCAM, VAUDIO, VVIDEO, VSD regulators */
	val = 0x49249;
	pmic_reg(slave, 33, val, 1);

	/* LEDR/G/B on */
	pmic_reg(slave, 53, 0x306306, 1);
	pmic_reg(slave, 54, 0x000306, 1);

	/* ADC init */
	mc13892_adc_init(slave);

	udelay(500);

	check_battery(slave);

	spi_pmic_free(slave);
}
#endif

#ifdef CONFIG_NET_MULTI

int board_eth_init(bd_t *bis)
{
	int rc = -ENODEV;

	return rc;
}
#endif

#ifdef CONFIG_CMD_MMC

struct fsl_esdhc_cfg esdhc_cfg[3] = {
	{MMC_SDHC1_BASE_ADDR, 1, 1},
	{MMC_SDHC2_BASE_ADDR, 1, 1},
	{MMC_SDHC3_BASE_ADDR, 1, 1},
};

#ifdef CONFIG_DYNAMIC_MMC_DEVNO
int get_mmc_env_devno()
{
	uint soc_sbmr = readl(SRC_BASE_ADDR + 0x4);	/* SBMR - SRC Boot Mode */
	/* Bit 20:19 - BT_SRC[1:0] */
	return (soc_sbmr & 0x00180000) >> 19;
}
#endif

int esdhc_gpio_init(bd_t *bis)
{
	s32 status = 0;
	u32 index = 0;

	for (index = 0; index < CONFIG_SYS_FSL_ESDHC_NUM;
		++index) {
		switch (index) {
		case 0:
			mxc_request_iomux(MX51_PIN_SD1_CMD,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
			mxc_request_iomux(MX51_PIN_SD1_CLK,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);

			mxc_request_iomux(MX51_PIN_SD1_DATA0,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
			mxc_request_iomux(MX51_PIN_SD1_DATA1,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
			mxc_request_iomux(MX51_PIN_SD1_DATA2,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
			mxc_request_iomux(MX51_PIN_SD1_DATA3,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
			mxc_iomux_set_pad(MX51_PIN_SD1_CMD,
					PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
					PAD_CTL_HYS_ENABLE | PAD_CTL_47K_PU |
					PAD_CTL_PUE_PULL |
					PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_SD1_CLK,
					PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
					PAD_CTL_HYS_NONE | PAD_CTL_47K_PU |
					PAD_CTL_PUE_PULL |
					PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_SD1_DATA0,
					PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
					PAD_CTL_HYS_ENABLE | PAD_CTL_47K_PU |
					PAD_CTL_PUE_PULL |
					PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_SD1_DATA1,
					PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
					PAD_CTL_HYS_ENABLE | PAD_CTL_47K_PU |
					PAD_CTL_PUE_PULL |
					PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_SD1_DATA2,
					PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
					PAD_CTL_HYS_ENABLE | PAD_CTL_47K_PU |
					PAD_CTL_PUE_PULL |
					PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_SD1_DATA3,
					PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
					PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PD |
					PAD_CTL_PUE_PULL |
					PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
			break;
		case 1:
			mxc_request_iomux(MX51_PIN_SD2_CMD,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
			mxc_request_iomux(MX51_PIN_SD2_CLK,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);

			mxc_request_iomux(MX51_PIN_SD2_DATA0,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
			mxc_request_iomux(MX51_PIN_SD2_DATA1,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
			mxc_request_iomux(MX51_PIN_SD2_DATA2,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
			mxc_request_iomux(MX51_PIN_SD2_DATA3,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
			mxc_iomux_set_pad(MX51_PIN_SD2_CMD,
					PAD_CTL_DRV_MAX | PAD_CTL_22K_PU |
					PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_SD2_CLK,
					PAD_CTL_DRV_MAX | PAD_CTL_22K_PU |
					PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_SD2_DATA0,
					PAD_CTL_DRV_MAX | PAD_CTL_22K_PU |
					PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_SD2_DATA1,
					PAD_CTL_DRV_MAX | PAD_CTL_22K_PU |
					PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_SD2_DATA2,
					PAD_CTL_DRV_MAX | PAD_CTL_22K_PU |
					PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_SD2_DATA3,
					PAD_CTL_DRV_MAX | PAD_CTL_22K_PU |
					PAD_CTL_SRE_FAST);
			break;
		case 2:
			mxc_request_iomux(MX51_PIN_NANDF_RDY_INT,
					IOMUX_CONFIG_ALT5 | IOMUX_CONFIG_SION);  // SD3_CMD
			mxc_request_iomux(MX51_PIN_NANDF_CS7,
					IOMUX_CONFIG_ALT5 | IOMUX_CONFIG_SION);    // SD3_CLK
			mxc_request_iomux(MX51_PIN_NANDF_WE_B,
					IOMUX_CONFIG_ALT2 | IOMUX_CONFIG_SION);
                        mxc_iomux_set_input(MUX_IN_ESDHC3_IPP_DAT0_IN_SELECT_INPUT,0);
			mxc_request_iomux(MX51_PIN_NANDF_RE_B,
					IOMUX_CONFIG_ALT2 | IOMUX_CONFIG_SION);
                        mxc_iomux_set_input(MUX_IN_ESDHC3_IPP_DAT1_IN_SELECT_INPUT,0);
			mxc_request_iomux(MX51_PIN_NANDF_WP_B,
					IOMUX_CONFIG_ALT2 | IOMUX_CONFIG_SION);
                        mxc_iomux_set_input(MUX_IN_ESDHC3_IPP_DAT2_IN_SELECT_INPUT,0);
			mxc_request_iomux(MX51_PIN_NANDF_RB0,
					IOMUX_CONFIG_ALT2 | IOMUX_CONFIG_SION);
                        mxc_iomux_set_input(MUX_IN_ESDHC3_IPP_DAT3_IN_SELECT_INPUT,0);
			mxc_request_iomux(MX51_PIN_NANDF_D12,
					IOMUX_CONFIG_ALT5 | IOMUX_CONFIG_SION);
			mxc_request_iomux(MX51_PIN_NANDF_D13,
					IOMUX_CONFIG_ALT5 | IOMUX_CONFIG_SION);
			mxc_request_iomux(MX51_PIN_NANDF_D14,
					IOMUX_CONFIG_ALT5 | IOMUX_CONFIG_SION);
			mxc_request_iomux(MX51_PIN_NANDF_D15,
					IOMUX_CONFIG_ALT5 | IOMUX_CONFIG_SION);

			mxc_iomux_set_pad(MX51_PIN_NANDF_RDY_INT,           // SD3_CMD
					PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH | PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU
					| PAD_CTL_PUE_PULL | PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_NANDF_CS7,             // SD3_CLK
					PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH | PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU
					| PAD_CTL_PUE_PULL | PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_NANDF_WE_B,
                                        PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH |
                                        PAD_CTL_HYS_NONE | PAD_CTL_47K_PU |
                                        PAD_CTL_PKE_ENABLE );
			mxc_iomux_set_pad(MX51_PIN_NANDF_RE_B,
                                        PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH |
                                        PAD_CTL_HYS_NONE | PAD_CTL_47K_PU |
                                        PAD_CTL_PKE_ENABLE );
			mxc_iomux_set_pad(MX51_PIN_NANDF_WP_B,
                                        PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH |
                                        PAD_CTL_HYS_NONE | PAD_CTL_100K_PU |
                                        PAD_CTL_PKE_ENABLE );
			mxc_iomux_set_pad(MX51_PIN_NANDF_RB0,
                                        PAD_CTL_DRV_LOW | PAD_CTL_DRV_VOT_HIGH |
                                        PAD_CTL_HYS_NONE | PAD_CTL_PUE_KEEPER |
					PAD_CTL_100K_PU |
                                        PAD_CTL_PKE_ENABLE );
			mxc_iomux_set_pad(MX51_PIN_NANDF_D12,
                                        PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH |
                                        PAD_CTL_100K_PU |
                                        PAD_CTL_PUE_KEEPER |
                                        PAD_CTL_PKE_ENABLE );
			mxc_iomux_set_pad(MX51_PIN_NANDF_D13,
                                        PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH |
                                        PAD_CTL_100K_PU |
                                        PAD_CTL_PUE_KEEPER |
                                        PAD_CTL_PKE_ENABLE );
			mxc_iomux_set_pad(MX51_PIN_NANDF_D14,
                                        PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH |
                                        PAD_CTL_100K_PU |
                                        PAD_CTL_PUE_KEEPER |
                                        PAD_CTL_PKE_ENABLE );
			mxc_iomux_set_pad(MX51_PIN_NANDF_D15,
                                        PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH |
                                        PAD_CTL_100K_PU |
                                        PAD_CTL_PUE_KEEPER |
                                        PAD_CTL_PKE_ENABLE );
			break;
		default:
			printf("Warning: you configured more ESDHC controller"
				"(%d) as supported by the board(2)\n",
				CONFIG_SYS_FSL_ESDHC_NUM);
			return status;
			break;
		}
		status |= fsl_esdhc_initialize(bis, &esdhc_cfg[index]);
	}

	return status;
}

int board_mmc_init(bd_t *bis)
{
	struct mmc *mmc;

	if (!esdhc_gpio_init(bis)) {
		/* mmc 0 - mini SD socket */
		mmc = find_mmc_device(0);
		mmc_init(mmc);
		/* mmc 2 - eMMC chip */
		mmc = find_mmc_device(2);
		mmc_init(mmc);
		return 0;
	}
	else
		return -1;
}

#endif

#ifdef CONFIG_LCD
void lcd_enable(void)
{
	int ret;
	unsigned int reg;

	/* LCD input power enable */
	mxc_request_iomux(MX51_PIN_DI1_PIN12, IOMUX_CONFIG_ALT4);
	reg = readl(GPIO3_BASE_ADDR + 0x4);
	reg |= 0x00000002;
	writel(reg, GPIO3_BASE_ADDR + 0x4);
	reg = readl(GPIO3_BASE_ADDR + 0x0);
	reg |= 0x00000002;
	writel(reg, GPIO3_BASE_ADDR + 0x0);

	/* PULL LCD POWER ON to HIGH to enable LCD power */
	mxc_request_iomux(MX51_PIN_DI1_PIN11, IOMUX_CONFIG_ALT4);
	reg = readl(GPIO3_BASE_ADDR + 0x4);
	reg |= 0x00000001;
	writel(reg, GPIO3_BASE_ADDR + 0x4);
	reg = readl(GPIO3_BASE_ADDR + 0x0);
	reg |= 0x00000001;
	writel(reg, GPIO3_BASE_ADDR + 0x0);

	/* Set LVDS chip to normal mode from reset mode */
	mxc_request_iomux(MX51_PIN_DISPB2_SER_RS, IOMUX_CONFIG_ALT4);
	reg = readl(GPIO3_BASE_ADDR + 0x4);
	reg |= 0x00000100;
	writel(reg, GPIO3_BASE_ADDR + 0x4);
	reg = readl(GPIO3_BASE_ADDR + 0x0);
	reg |= 0x00000100;
	writel(reg, GPIO3_BASE_ADDR + 0x0);

	/* Set LVDS RSVD to low */
	mxc_request_iomux(MX51_PIN_CSI2_D12, IOMUX_CONFIG_ALT3);
	reg = readl(GPIO4_BASE_ADDR + 0x4);
	reg |= 0x00000200;
	writel(reg, GPIO4_BASE_ADDR + 0x4);
	reg = readl(GPIO4_BASE_ADDR + 0x0);
	reg &= 0xFFFFFDFF;
	writel(reg, GPIO4_BASE_ADDR + 0x0);

	/* LCD backlight control, 20KHz PWM wave, 16% duty */
	imx_pwm_config(pwm0, 1250, 7500);
	imx_pwm_enable(pwm0);
	mxc_request_iomux(MX51_PIN_GPIO1_2, IOMUX_CONFIG_ALT1);

	ret = ipuv3_fb_init(&byd_wxga, 0, IPU_PIX_FMT_RGB24,
			DI_PCLK_PLL3, 0);
	if (ret)
		puts("LCD cannot be configured\n");
}
#endif

#ifdef CONFIG_VIDEO_MX5
static void panel_info_init(void)
{
	panel_info.vl_bpix = LCD_BPP;
	panel_info.vl_col = byd_wxga.xres;
	panel_info.vl_row = byd_wxga.yres;
	panel_info.cmap = colormap;
}
#endif

#ifdef CONFIG_SPLASH_SCREEN
void setup_splash_image(void)
{
	char *s;
	ulong addr;

	s = getenv("splashimage");

	if (s != NULL) {
		addr = simple_strtoul (s, NULL, 16);

#if defined(CONFIG_ARCH_MMU)
		addr = ioremap_nocache(iomem_to_phys(addr), fsl_bmp_600x400_size);
#endif
		memcpy((char *)addr, (char *)fsl_bmp_600x400, fsl_bmp_600x400_size);
	}
}
#endif

#if defined(CONFIG_MXC_KPD)
int setup_mxc_kpd(void)
{
	mxc_request_iomux(MX51_PIN_KEY_COL0, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_COL1, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_COL2, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_COL3, IOMUX_CONFIG_ALT0);
	//mxc_request_iomux(MX51_PIN_KEY_COL4, IOMUX_CONFIG_ALT0);
	//mxc_request_iomux(MX51_PIN_KEY_COL5, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_ROW0, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_ROW1, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_ROW2, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_ROW3, IOMUX_CONFIG_ALT0);

	return 0;
}
#endif

int board_init(void)
{
#ifdef CONFIG_MFG
/* MFG firmware need reset usb to avoid host crash firstly */
#define USBCMD 0x140
	int val = readl(OTG_BASE_ADDR + USBCMD);
	val &= ~0x1; /*RS bit*/
	writel(val, OTG_BASE_ADDR + USBCMD);
#endif
	setup_boot_device();
	setup_soc_rev();
	set_board_rev();

	gd->bd->bi_arch_number = MACH_TYPE_MX51_TULIP;	/* board id for linux */
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	setup_uart();
	setup_expio();
#ifdef CONFIG_I2C_MXC
	setup_i2c(I2C1_BASE_ADDR);
#endif

#ifdef CONFIG_VIDEO_MX5
	panel_info_init();

	gd->fb_base = CONFIG_FB_BASE;
#ifdef CONFIG_ARCH_MMU
	gd->fb_base = ioremap_nocache(iomem_to_phys(gd->fb_base), 0);
#endif
#endif
	return 0;
}

#ifdef CONFIG_ANDROID_RECOVERY
struct reco_envs supported_reco_envs[BOOT_DEV_NUM] = {
	{
	 .cmd = NULL,
	 .args = NULL,
	 },
	{
	 .cmd = NULL,
	 .args = NULL,
	 },
	{
	 .cmd = CONFIG_ANDROID_RECOVERY_BOOTCMD_MMC,
	 .args = CONFIG_ANDROID_RECOVERY_BOOTARGS_MMC,
	 },
};

int check_recovery_cmd_file(void)
{
	disk_partition_t info;
	ulong part_length;
	int filelen;
	int mmcdevno = 0;

#ifdef CONFIG_DYNAMIC_MMC_DEVNO
	mmcdevno = get_mmc_env_devno();
#else
	mmcdevno = CONFIG_SYS_MMC_ENV_DEV;
#endif

	switch (get_boot_device()) {
	case MMC_BOOT:
		{
			block_dev_desc_t *dev_desc = NULL;
			/* recovery on mmc 2 - eMMC chip */
			struct mmc *mmc = find_mmc_device(mmcdevno);

			dev_desc = get_dev("mmc", mmcdevno);

			if (NULL == dev_desc) {
				printf("** Block device MMC %d not supported\n", mmcdevno);
				return 0;
			}

			mmc_init(mmc);

			if (get_partition_info(dev_desc,
					CONFIG_ANDROID_CACHE_PARTITION_MMC,
					&info)) {
				printf("** Bad partition %d **\n",
					CONFIG_ANDROID_CACHE_PARTITION_MMC);
				return 0;
			}

			part_length = ext2fs_set_blk_dev(dev_desc,
							CONFIG_ANDROID_CACHE_PARTITION_MMC);
			if (part_length == 0) {
				printf("** Bad partition - mmc %d:%d **\n", mmcdevno,
					CONFIG_ANDROID_CACHE_PARTITION_MMC);
				ext2fs_close();
				return 0;
			}

			if (!ext2fs_mount(part_length)) {
				printf("** Bad ext2 partition or disk - mmc %d:%d **\n", mmcdevno,
					CONFIG_ANDROID_CACHE_PARTITION_MMC);
				ext2fs_close();
				return 0;
			}

			filelen = ext2fs_open(CONFIG_ANDROID_RECOVERY_CMD_FILE);

			ext2fs_close();
		}
		break;
	case NAND_BOOT:
		return 0;
		break;
	case SPI_NOR_BOOT:
		return 0;
		break;
	case UNKNOWN_BOOT:
	default:
		return 0;
		break;
	}

	return (filelen > 0) ? 1 : 0;

}
#endif

#ifdef BOARD_LATE_INIT
int board_late_init(void)
{
#ifdef CONFIG_I2C_MXC
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
	if (!i2c_probe(0x34))
		setup_core_voltage_i2c();
	else
#endif
#ifdef CONFIG_IMX_ECSPI
		setup_core_voltage_spi();
#endif

	return 0;
}
#endif

static void check_poweron(void)
{
	unsigned int reg;

	/* set SYS_ON_OFF_CTL pin to high, lock power on */
	board_sys_on_off_ctl(1);

	/* vibrator on if normal boot */ 
	/* pin VIBRATOR_ON, NANDF_D9 ALT3 - GPIO3_31 */ 
	reg = readl(GPIO3_BASE_ADDR + 0x4);	/* GPIO3 GDIR */
	reg |= 0x80000000;  /* configure GPIO3_31 lines as output */
	writel(reg, GPIO3_BASE_ADDR + 0x4);
	reg = readl(GPIO3_BASE_ADDR + 0x0);	/* GPIO3 DR */
	reg |= 0x80000000;  /* configure GPIO3_31 lines to high */
	writel(reg, GPIO3_BASE_ADDR + 0x0);
	mxc_request_iomux(MX51_PIN_NANDF_D9, IOMUX_CONFIG_ALT3);
	/* vibrator off after 0.2s */ 
	udelay(200000);
	reg = readl(GPIO3_BASE_ADDR + 0x0);	/* GPIO3 DR */
	reg &= ~0x80000000;  /* configure GPIO3_31 lines to high */
	writel(reg, GPIO3_BASE_ADDR + 0x0);
}

int checkboard(void)
{
	printf("Board: MX51 TULIP ");

	if (is_soc_rev(CHIP_REV_3_0) == 0) {
		printf("3.0 [");
	} else if ((is_soc_rev(CHIP_REV_2_0) == 0)
	 && (system_rev & (BOARD_REV_2_0 << BOARD_VER_OFFSET))) {
		printf("2.5 [");
	} else if (is_soc_rev(CHIP_REV_2_0) == 0) {
		printf("2.0 [");
	} else if (is_soc_rev(CHIP_REV_1_1) == 0) {
		printf("1.1 [");
	} else {
		printf("1.0 [");
	}

	switch (__REG(SRC_BASE_ADDR + 0x8)) {
	case 0x0001:
		printf("POR");
		break;
	case 0x0009:
		printf("RST");
		break;
	case 0x0010:
	case 0x0011:
		printf("WDOG");
		break;
	default:
		printf("unknown");
	}
	printf("]\n");

	printf("Boot Device: ");
	switch (get_boot_device()) {
	case NAND_BOOT:
		printf("NAND\n");
		break;
	case SPI_NOR_BOOT:
		printf("SPI NOR\n");
		break;
	case MMC_BOOT:
		printf("MMC\n");
		break;
	case UNKNOWN_BOOT:
	default:
		printf("UNKNOWN\n");
		break;
	}

	check_poweron();
	return 0;
}

