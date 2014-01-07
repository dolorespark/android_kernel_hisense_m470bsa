/*
 * arch/arm/mach-tegra/board-enterprise-kbc.c
 * Keys configuration for Nvidia tegra3 enterprise platform.
 *
 * Copyright (C) 2011 NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <mach/io.h>
#include <linux/io.h>
#include <mach/iomap.h>
#include <mach/kbc.h>
#include <linux/gpio_keys.h>

#include "board.h"
#include "board-enterprise.h"
#include "devices.h"
#include "gpio-names.h"
#define ENTERPRISE_ROW_COUNT	3
#define ENTERPRISE_COL_COUNT	3

static const u32 kbd_keymap[] = {
	KEY(0, 0, KEY_POWER),

	KEY(1, 0, KEY_HOME),
	KEY(1, 1, KEY_BACK),
	KEY(1, 2, KEY_VOLUMEDOWN),

	KEY(2, 0, KEY_MENU),
	KEY(2, 1, KEY_SEARCH),
	KEY(2, 2, KEY_VOLUMEUP),
};

static const struct matrix_keymap_data keymap_data = {
	.keymap		= kbd_keymap,
	.keymap_size	= ARRAY_SIZE(kbd_keymap),
};

static struct tegra_kbc_wake_key enterprise_wake_cfg[] = {
	[0] = {
		.row = 0,
		.col = 0,
	},
	[1] = {
		.row = 1,
		.col = 0,
	},
	[2] = {
		.row = 1,
		.col = 1,
	},
	[3] = {
		.row = 2,
		.col = 0,
	},
};

static struct tegra_kbc_platform_data enterprise_kbc_platform_data = {
	.debounce_cnt = 20 * 32, /* 20 ms debaunce time */
	.repeat_cnt = 1,
	.scan_count = 30,
	.wakeup = true,
	.keymap_data = &keymap_data,
	.wake_cnt = 4,
	.wake_cfg = &enterprise_wake_cfg[0],
#ifdef CONFIG_ANDROID
	.disable_ev_rep = true,
#endif
};
#define GPIO_KEY(_id, _gpio, _iswake,_active_low)       \
	    {                   \
		        .code = _id,            \
		        .gpio = _gpio,       \
		        .active_low = _active_low,      \
		        .desc = #_gpio,         \
		        .type = EV_KEY,         \
		        .wakeup = _iswake,      \
		        .debounce_interval = 0,    \
		    }





static struct gpio_keys_button m470_keys[] = {
	[0] = GPIO_KEY(KEY_VOLUMEUP, TEGRA_GPIO_PQ1, 0, 1),
	[1] = GPIO_KEY(KEY_VOLUMEDOWN, TEGRA_GPIO_PQ2, 0, 1),
	[2] = GPIO_KEY(KEY_POWER, TEGRA_GPIO_PV0, 1, 1),
};
#define PMC_WAKE_STATUS		0x14
#define PMC_WAKE2_STATUS	0x168
static int m470_wakeup_key(void)
{
    u64 status =
	readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);
	printk(" pmc_wake2_status is %llu \n",status);
		    
    if(status & (1 << (24)))
       return KEY_POWER;
    else
       return KEY_RESERVED;
	//return ((status & (1 << 35)) ? KEY_POWER : KEY_RESERVED);
					
     //return KEY_POWER;
}
		
					
static struct gpio_keys_platform_data m470_keys_pdata = {
        	.buttons	= m470_keys,
     		.nbuttons	= ARRAY_SIZE(m470_keys),
		.wakeup_key = m470_wakeup_key,
  	};
			
static struct platform_device m470_keys_device = {
	        .name   = "gpio-keys",
        	.id     = 0,
      		.dev    = {
 				.platform_data  = &m470_keys_pdata,
			},
};
					
					
					
					

int __init enterprise_kbc_init(void)
{
 
	struct tegra_kbc_platform_data *data = &enterprise_kbc_platform_data;
	int i;
	tegra_kbc_device.dev.platform_data = &enterprise_kbc_platform_data;
	pr_info("Registering tegra-kbc\n");

	BUG_ON((KBC_MAX_ROW + KBC_MAX_COL) > KBC_MAX_GPIO);
	for (i = 0; i < ENTERPRISE_ROW_COUNT; i++) {
		data->pin_cfg[i].num = i;
		data->pin_cfg[i].is_row = true;
		data->pin_cfg[i].en = true;
	}
	for (i = 0; i < ENTERPRISE_COL_COUNT; i++) {
		data->pin_cfg[i + KBC_PIN_GPIO_16].num = i;
		data->pin_cfg[i + KBC_PIN_GPIO_16].en = true;
	}

	platform_device_register(&tegra_kbc_device);
	pr_info("Registering successful tegra-kbc\n");

	return 0;
}

int __init enterprise_keys_init(void)
{
	int i;

	pr_info("Registering gpio keys\n");
	
			
	/* Enable gpio mode for other pins */
	for (i = 0; i < m470_keys_pdata.nbuttons; i++)
	{
	  tegra_gpio_enable(m470_keys_pdata.buttons[i].gpio);
	}
		        
	platform_device_register(&m470_keys_device);
	pr_info("%s() -- > %d\n", __func__, __LINE__);
	/* Register on-key through pmu interrupt */
	//platform_device_register(&cardhu_int_keys_device);
	return 0;
}
