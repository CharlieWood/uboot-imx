#include <common.h>
#include <command.h>
#include <image.h>

#define MMC_BLOCK_SIZE	    512

extern int get_mmc_env_devno(void);
extern int do_fat_fsload(cmd_tbl_t *, int, int, char *[]);
extern int do_mmc(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
extern int do_mmcops(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
extern int do_reset(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);

static int load_uboot_image(ulong addr)
{
	int devno;
	int partno;
	char *ubootdir;
	char nxri[128];
	char dev[7];
	char addr_str[16];
	char *argv[6] = { "fatload", "mmc", dev, addr_str, nxri, NULL };
	cmd_tbl_t *bcmd;

	devno = CONFIG_ANDROID_RECOVERY_MMCDEVNO; 
	partno = 1;	/* genernal in first partition */

	/* Load the rescue image */
	bcmd = find_cmd("fatload");
	if (!bcmd) {
		printf("Error - 'fatload' command not present.\n");
		return 1;
	}

	ubootdir = CONFIG_ANDROID_MMCUP_UBOOT_IMAGE_PATH;
	sprintf(nxri, "%s/%s_u-boot.img", ubootdir, CONFIG_ANDROID_MMCUP_UBOOT_IMAGE_PREFIX);
	sprintf(dev, "%d:%d", devno, partno);
	sprintf(addr_str, "%lx", addr);

	printf("Load uboot image: %s %s %s %s %s\n", argv[0], argv[1], argv[2], argv[3], argv[4], argv[5]);

	if (do_fat_fsload(bcmd, 0, 5, argv) != 0) {
		printf("fat_fsload fail, can not load uboot image.\n");
		return 1;
	}

	return 0;
}

static image_header_t *image_check_verify (ulong img_addr, int verify)
{
	image_header_t *hdr = (image_header_t *)img_addr;

	if (!image_check_magic(hdr)) {
		puts ("Bad Magic Number\n");
		show_boot_progress (-1);
		return NULL;
	}
	show_boot_progress (2);

	if (!image_check_hcrc (hdr)) {
		puts ("Bad Header Checksum\n");
		show_boot_progress (-2);
		return NULL;
	}

	show_boot_progress (3);
	image_print_contents (hdr);

	if (verify) {
		puts ("   Verifying Checksum ... ");
		if (!image_check_dcrc (hdr)) {
			printf ("Bad Data CRC\n");
			show_boot_progress (-3);
			return NULL;
		}
		puts ("OK\n");
	}
	show_boot_progress (4);

	if (!image_check_target_arch (hdr)) {
		printf ("Unsupported Architecture 0x%x\n", image_get_arch (hdr));
		show_boot_progress (-4);
		return NULL;
	}
	return hdr;
}

static void *get_uboot_rawimage (ulong load_addr, ulong *raw_data, ulong *raw_len)
{
	image_header_t	*hdr;
	ulong		img_addr;

	/* copy from dataflash if needed */
	img_addr = genimg_get_image (load_addr);

	/* check image type, for FIT images get FIT kernel node */
	*raw_data = *raw_len = 0;
	switch (genimg_get_format ((void *)img_addr)) {
	case IMAGE_FORMAT_LEGACY:
		printf ("## Getting u-boot.bin from Legacy Image at %08lx ...\n",
				img_addr);
		hdr = image_check_verify (img_addr, 1);
		if (!hdr)
			return NULL;
		show_boot_progress (5);

		/* get os_data and os_len */
		switch (image_get_type (hdr)) {
		case IH_TYPE_FIRMWARE:
			img_addr = image_get_data (hdr);
			*raw_data = img_addr;
			*raw_len = image_get_data_size (hdr);
			break;
		default:
			printf ("Wrong Image Type for u-boot.img\n");
			show_boot_progress (-5);
			return NULL;
		}

		break;
	default:
		printf ("Wrong Image Format for u-boot.img\n");
		show_boot_progress (-108);
		return NULL;
	}

	printf ("   u-boot.bin data at 0x%08lx, len = 0x%08lx (%ld)\n",
			*raw_data, *raw_len, *raw_len);

	return (void *)img_addr;
}


static int update_uboot_image(unsigned char *img_addr, ulong img_len)
{
	int devno;
	int offset;

	char source[32], dest[32], length[32];
	char slot_no[32];
	unsigned int temp;

	char *mmc_write[6] = {"mmc", "write", NULL, NULL, NULL, NULL};
	char *mmc_init[2] = {"mmcinit", NULL,};

	/* ignore first 1k uboot image so that we do not overwrite the MBR
	   (including partition table) within first 512B on the SD/mmc card */
	offset = 1024;
	if (img_len<=offset) {
		printf("u-boot image size %ld too short\n", img_len);
		return -1;
	}

	img_addr += offset;
	img_len -= offset;

#ifdef CONFIG_DYNAMIC_MMC_DEVNO
	devno = get_mmc_env_devno();
#else
	devno = CONFIG_SYS_MMC_ENV_DEV;
#endif

	mmc_init[1] = slot_no;
	mmc_write[2] = slot_no;
	mmc_write[3] = source;
	mmc_write[4] = dest;
	mmc_write[5] = length;

	sprintf(slot_no, "%d", devno);
	sprintf(source, "0x%x", img_addr);

	/* block offset */
	temp = offset / MMC_BLOCK_SIZE;
	sprintf(dest, "0x%x", temp);
	/* block count */
	temp = (img_len + MMC_BLOCK_SIZE - 1) / MMC_BLOCK_SIZE;
	sprintf(length, "0x%x", temp);

#if 0
	printf("Initializing mmc %s ", mmc_init[1]);
	if (do_mmc(NULL, 0, 2, mmc_init)) {
		printf("FAIL:Init of MMC card\n");
		return -1;
	}
	else
		printf("OKAY\n");
#endif

	printf("command: %s %s %s %s %s %s\n", mmc_write[0], mmc_write[1], mmc_write[2], mmc_write[3], mmc_write[4], mmc_write[5]);
	printf("Writing mmc %s offset 0x%x\n", slot_no, offset);

	if (do_mmcops(NULL, 0, 6, mmc_write)) {
		printf("Wrote mmc FAILED!\n");
	} else {
		printf("Wrote mmc DONE!\n");
	}
	return 0;
}

static int erase_uboot_env(void)
{
	int devno;
	int offset;
	ulong data, len;

	char source[32], dest[32], length[32];
	char slot_no[32];
	unsigned int temp;

	char *mmc_write[6] = {"mmc", "write", NULL, NULL, NULL, NULL};
	char *mmc_init[2] = {"mmcinit", NULL,};

	/* u-boot envirment offset in mmc */
	offset = CONFIG_ENV_OFFSET;
	data = CONFIG_LOADADDR;
	len =  CONFIG_ENV_SIZE;
	memset(data, 0, len);

#ifdef CONFIG_DYNAMIC_MMC_DEVNO
	devno = get_mmc_env_devno();
#else
	devno = CONFIG_SYS_MMC_ENV_DEV;
#endif

	mmc_init[1] = slot_no;
	mmc_write[2] = slot_no;
	mmc_write[3] = source;
	mmc_write[4] = dest;
	mmc_write[5] = length;

	sprintf(slot_no, "%d", devno);
	sprintf(source, "0x%x", data);

	/* block offset */
	temp = offset / MMC_BLOCK_SIZE;
	sprintf(dest, "0x%x", temp);
	/* block count */
	temp = (len + MMC_BLOCK_SIZE - 1) / MMC_BLOCK_SIZE;
	sprintf(length, "0x%x", temp);

	printf("Erasing u-boot envirment...\n");
	printf("  command: %s %s %s %s %s %s\n", mmc_write[0], mmc_write[1], mmc_write[2], mmc_write[3], mmc_write[4], mmc_write[5]);
	printf("Writing mmc %s offset 0x%x\n", slot_no, offset);

	if (do_mmcops(NULL, 0, 6, mmc_write)) {
		printf("Wrote mmc FAILED!\n");
	} else {
		printf("Wrote mmc DONE!\n");
	}
	return 0;
}

void do_mmc_update_uboot(void)
{
	ulong img_data, img_len;
	unsigned char *img_addr;

	if (load_uboot_image(CONFIG_LOADADDR))
		return;

	img_addr = get_uboot_rawimage(CONFIG_LOADADDR, &img_data, &img_len);

	if (img_addr) {
		update_uboot_image(img_addr, img_len);
		erase_uboot_env();
	}

	do_reset(NULL, 0, 0, NULL);

	return;
}

