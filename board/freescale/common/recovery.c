/*
 * Freescale Android Recovery mode checking routing
 *
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */
#include <common.h>
#include <malloc.h>
#include "recovery.h"
#include <lcd.h>
#ifdef CONFIG_MXC_KPD
#include <mxc_keyb.h>
#endif

extern int check_recovery_cmd_file(void);
extern enum boot_device get_boot_device(void);

#define mdelay(n)   udelay((n)*1000)
#ifdef CONFIG_MXC_KPD

#define PRESSED_HOME	0x01
#define PRESSED_POWER	0x02
#define PRESSED_VOLUMEUP 0x04

#define RECOVERY_KEY_MASK (PRESSED_HOME | PRESSED_POWER)

#ifdef CONFIG_ANDROID_RECOVERY_MMCBOOT
#define PRESSED_MMCBOOT		0x10
#define MMCBOOT_KEY_MASK (PRESSED_MMCBOOT)
#define PRESSED_MMCUPDATE	0x20
#define MMCUPDATE_KEY_MASK (PRESSED_MMCUPDATE)
#endif

#ifdef CONFIG_ANDROID_BOOTMODE_FACTORY2
#define FACTORY2_KEY_MASK (PRESSED_VOLUMEUP | PRESSED_HOME)
#endif 

inline int test_key(int value, struct kpp_key_info *ki)
{
	return (ki->val == value) && (ki->evt == KDepress);
}

int select_mode(void)
{
#ifdef CONFIG_LCD
	struct kpp_key_info *key_info;
	int  count=1, keys, i;

	lcd_setfgcolor (CONSOLE_COLOR_BLUE);
	lcd_puts("\n\n\n             please press the 'volume+' key to select mode and 'volume-' to certain your select\n");
	lcd_puts("\n			 --------------------------------------------------------------\n");
	lcd_puts("			|                                                              |\n");
	lcd_puts("			|  0 --->   normal                                             |\n");
	lcd_puts("			|                                                              |\n");
	lcd_puts("			|  1 --->   factory test                                       |\n");
	lcd_puts("			|                                                              |\n");
	lcd_puts("			|  2 --->   flash system                                       |\n");
	lcd_puts("			|                                                              |\n");
	lcd_puts("			|  3 --->   flash uboot                                        |\n");
	lcd_puts("			|                                                              |\n");
	lcd_puts("			|  4 --->   recovery                                           |\n");
	lcd_puts("			|                                                              |\n");
	lcd_puts("			 --------------------------------------------------------------\n");
	lcd_puts("\n\n                        SELECTED MODE : ");
	lcd_setfgcolor (CONSOLE_COLOR_RED);
	lcd_puts("0 --->   normal      ");

	while(1){
	
		keys = mxc_kpp_getc(&key_info);
		mdelay(10);
		for (i = 0; i < keys; i++) {
			if (test_key(CONFIG_MMCBOOT_KEY, &key_info[i])){
				if((++count)%6 == 0)
					count =1;	
				switch (count) {
				case 2:
						lcd_puts("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b1 --->   factory test");
						break;
				case 3:
						lcd_puts("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b2 --->   flash system");
						break;
				case 4:
						lcd_puts("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b3 --->   flash uboot ");
						break;
				case 5:
						lcd_puts("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b4 --->   recovery    ");
						break;
				default:
						lcd_puts("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b0 --->   normal      ");
				}	
			} else if (test_key(CONFIG_MMCUPDATE_KEY, &key_info[i])) {
				free(key_info);
				switch (count){
				case 1:
						return 0;
				case 2:
						return FACTORY2_KEY_MASK;
				case 3:
						return MMCBOOT_KEY_MASK;
				case 4:
						return MMCUPDATE_KEY_MASK;
				case 5:	
						return RECOVERY_KEY_MASK;
				default :
						return 0;
				}
			} 
		}
	} 
#endif
	return 0;
}

int check_key_pressing(void)
{
	struct kpp_key_info *key_info;
	struct kpp_key_info *key_inf;
	int state = 0, keys, i, selmode=0;

	mxc_kpp_init();

#ifdef CONFIG_ANDROID_SOFTKEY_PAD
	puts("Detecting 'VOLUME+' to enter select mode interface... \n");
#else
	puts("Detecting HOME+MENU key for recovery...\n");
#ifdef CONFIG_ANDROID_RECOVERY_MMCBOOT
	puts("Detecting BACK key for booting on SDCard...\n");
#endif 

#ifdef CONFIG_ANDROID_RECOVERY_MMCUPDATE
	puts("Detecting VOLUME DOWN key for u-boot update on SDCard...\n");
#endif
#ifdef CONFIG_ANDROID_BOOTMODE_FACTORY2
	puts("Detecting VOLUMEUP+HOME key for factory2...\n");
#endif
#endif

#ifdef CONFIG_ANDROID_SOFTKEY_PAD

	keys = mxc_kpp_getc(&key_inf);
	if(keys < 1)
		return 0;

	for (i = 0; i < keys; i++) {
		if (test_key(CONFIG_MMCBOOT_KEY, &key_inf[i])) {
			selmode =1;
		}
	}

	free(key_inf);

	if(1 == selmode)
		return select_mode();
	else
		return 0;

#else
	/* Check for home + power */
	keys = mxc_kpp_getc(&key_info);
	if (keys < 1)
		return 0;

	for (i = 0; i < keys; i++) {
		if (test_key(CONFIG_POWER_KEY, &key_info[i]))
			state |= PRESSED_POWER;
		else if (test_key(CONFIG_HOME_KEY, &key_info[i]))
			state |= PRESSED_HOME;
#ifdef CONFIG_ANDROID_RECOVERY_MMCBOOT
		else if (test_key(CONFIG_MMCBOOT_KEY, &key_info[i]))
			state |= PRESSED_MMCBOOT;
#endif
#ifdef CONFIG_ANDROID_RECOVERY_MMCUPDATE
		else if (test_key(CONFIG_MMCUPDATE_KEY, &key_info[i]))
			state |= PRESSED_MMCUPDATE;
#endif
#ifdef CONFIG_ANDROID_BOOTMODE_FACTORY2
		else if (test_key(CONFIG_VOLUMEUP_KEY, &key_info[i]))
			state |= PRESSED_VOLUMEUP;
#endif
	}

	free(key_info);

	if ((state & RECOVERY_KEY_MASK) == RECOVERY_KEY_MASK)
		return RECOVERY_KEY_MASK;
#ifdef CONFIG_ANDROID_BOOTMODE_FACTORY2
	else if ((state & FACTORY2_KEY_MASK) == FACTORY2_KEY_MASK)
		return FACTORY2_KEY_MASK;
#endif
#ifdef CONFIG_ANDROID_RECOVERY_MMCBOOT
	else if ((state & MMCBOOT_KEY_MASK) == MMCBOOT_KEY_MASK)
		return MMCBOOT_KEY_MASK;
#endif
#ifdef CONFIG_ANDROID_RECOVERY_MMCUPDATE
	else if ((state & MMCUPDATE_KEY_MASK) == MMCUPDATE_KEY_MASK)
		return MMCUPDATE_KEY_MASK;
#endif
	return 0;
#endif
}
#else
/* If not using mxc keypad, currently we will detect power key on board */
int check_key_pressing(void)
{
	return 0;
}
#endif

extern struct reco_envs supported_reco_envs[];

void setup_recovery_env(void)
{
	char *env, *boot_args, *boot_cmd;
	int bootdev = get_boot_device();

	boot_cmd = supported_reco_envs[bootdev].cmd;
	boot_args = supported_reco_envs[bootdev].args;

	if (boot_cmd == NULL) {
		printf("Unsupported bootup device for recovery\n");
		return;
	}

	printf("setup env for recovery..\n");

	env = getenv("bootargs_android_recovery");
	/* Set env to recovery mode */
	/* Only set recovery env when these env not exist, give user a
	 * chance to change their recovery env */
	if (!env)
		setenv("bootargs_android_recovery", boot_args);

	env = getenv("bootcmd_android_recovery");
	if (!env)
		setenv("bootcmd_android_recovery", boot_cmd);
	setenv("bootcmd", "run bootcmd_android_recovery");
}

#ifdef CONFIG_ANDROID_RECOVERY_MMCBOOT
void setup_mmcboot_env(void)
{
	char *env;

	printf("setup env for mmcboot..\n");

	env = getenv("bootargs_mmcboot");
	/* Set env to sd install mode */
	if (env)
		setenv("bootcmd", "run bootcmd_mmcboot");
	else
		printf("Unsupported boot envirments for mmc boot\n");
}
#endif

#ifdef CONFIG_ANDROID_BOOTMODE_FACTORY2
void setup_mmc_factory2_boot_env(void)
{
	char *env, boot;

	puts("setup env for factory2 boot..\n");

	env = getenv("bootcmd_android_factory2");
	/* Set env to sd install mode */
	if (env)
		setenv("bootcmd", "run bootcmd_android_factory2");
	else
		printf("Unsupported boot envirments for factory boot\n");
}
#endif

/* export to lib_arm/board.c */
void check_recovery_mode(void)
{
	int ret;
	ret = check_key_pressing();
	if (ret==RECOVERY_KEY_MASK)
		setup_recovery_env();
#ifdef CONFIG_ANDROID_RECOVERY_MMCBOOT
	else if (ret==MMCBOOT_KEY_MASK)
		setup_mmcboot_env();
#endif
#ifdef CONFIG_ANDROID_RECOVERY_MMCUPDATE
	else if (ret==MMCUPDATE_KEY_MASK)
		do_mmc_update_uboot();
#endif
#ifdef CONFIG_ANDROID_BOOTMODE_FACTORY2
	else if (ret==FACTORY2_KEY_MASK)
		setup_mmc_factory2_boot_env();
#endif
	else if (check_recovery_cmd_file()) {
		puts("Recovery command file founded!\n");
		setup_recovery_env();
	}
}
