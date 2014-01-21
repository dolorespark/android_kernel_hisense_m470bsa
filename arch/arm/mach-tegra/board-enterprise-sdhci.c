/*
 * arch/arm/mach-tegra/board-enterprise-sdhci.c
 *
 * Copyright (C) 2011-2012 NVIDIA Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/wlan_plat.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>
#include <linux/wl12xx.h>
#include <linux/fs.h>
#include <linux/random.h>

#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>
#include <mach/io_dpd.h>

#include "gpio-names.h"
#include "board.h"
#include "board-enterprise.h"


#define ENTERPRISE_WLAN_PWR	TEGRA_GPIO_WLAN_RST_N
#define ENTERPRISE_WLAN_WOW	TEGRA_GPIO_WLAN_HOST_WAKE
#define ENTERPRISE_SD_CD TEGRA_GPIO_PI5
#define MAC_ADDR_LEN		6
#define MAC_CMDLINE_LEN     17
#define MAC_BCK_LEN         12
#define MAC_BCK_OFFSET      0x4A

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static int enterprise_wifi_status_register(void (*callback)(int , void *), void *);

static int enterprise_wifi_reset(int on);
static int enterprise_wifi_power(int on);
static int enterprise_wifi_set_carddetect(int val);
static int enterprise_wifi_get_mac_addr(unsigned char *buf);
static int enterprise_wifi_validate_mac(unsigned char *buf);
static int enterprise_wifi_read_mac_from_bck(void);
extern unsigned int his_hw_ver;
extern unsigned int his_board_version;
extern char his_wifi_addr[18];

static struct wifi_platform_data enterprise_wifi_control = {
	.set_power      = enterprise_wifi_power,
	.set_reset      = enterprise_wifi_reset,
	.set_carddetect = enterprise_wifi_set_carddetect,
	.get_mac_addr = enterprise_wifi_get_mac_addr,
};

static struct wl12xx_platform_data enterprise_wl12xx_wlan_data __initdata = {
	.irq = TEGRA_GPIO_TO_IRQ(ENTERPRISE_WLAN_WOW),
	.board_ref_clock = WL12XX_REFCLOCK_26,
	.board_tcxo_clock = 1,
	.set_power = enterprise_wifi_power,
	.set_carddetect = enterprise_wifi_set_carddetect,
};

static struct resource wifi_resource[] = {
	[0] = {
		.name	= "bcm4329_wlan_irq",
		.start	= TEGRA_GPIO_TO_IRQ(ENTERPRISE_WLAN_WOW),
		.end	= TEGRA_GPIO_TO_IRQ(ENTERPRISE_WLAN_WOW),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct platform_device enterprise_brcm_wifi_device = {
	.name           = "bcm4329_wlan",
	.id             = 1,
	.num_resources	= 1,
	.resource	= wifi_resource,
	.dev            = {
		.platform_data = &enterprise_wifi_control,
	},
};

static struct resource sdhci_resource0[] = {
	[0] = {
		.start  = INT_SDMMC1,
		.end    = INT_SDMMC1,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC1_BASE,
		.end	= TEGRA_SDMMC1_BASE + TEGRA_SDMMC1_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource2[] = {
	[0] = {
		.start  = INT_SDMMC3,
		.end    = INT_SDMMC3,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC3_BASE,
		.end	= TEGRA_SDMMC3_BASE + TEGRA_SDMMC3_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource3[] = {
	[0] = {
		.start  = INT_SDMMC4,
		.end    = INT_SDMMC4,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC4_BASE,
		.end	= TEGRA_SDMMC4_BASE + TEGRA_SDMMC4_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

#ifdef CONFIG_MMC_EMBEDDED_SDIO
static struct embedded_sdio_data embedded_sdio_data2 = {
	.cccr   = {
		.sdio_vsn       = 2,
		.multi_block    = 1,
		.low_speed      = 0,
		.wide_bus       = 0,
		.high_power     = 1,
		.high_speed     = 1,
	},
	.cis  = {
		.vendor         = 0x02d0,
		.device         = 0x4330,
	},
};
#endif

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data2 = {
	.mmc_data = {
		.register_status_notify	= enterprise_wifi_status_register,
#ifdef CONFIG_MMC_EMBEDDED_SDIO
		.embedded_sdio = &embedded_sdio_data2,
#endif
		/* FIXME need to revert the built_in change
		once we use get the signal strength fix of
		bcmdhd driver from broadcom for bcm4329 chipset*/
		.built_in = 0,
	},
#ifndef CONFIG_MMC_EMBEDDED_SDIO
	.pm_flags = MMC_PM_KEEP_POWER,
#endif
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.tap_delay = 0x0F,
	.max_clk_limit = 45000000,
	.ddr_clk_limit = 41000000,
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data0 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.tap_delay = 0x0F,
	.ddr_clk_limit = 41000000,
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.is_8bit = 1,
	.tap_delay = 0x0F,
	.ddr_clk_limit = 41000000,
	.mmc_data = {
		.built_in = 1,
	}
};

static struct platform_device tegra_sdhci_device0 = {
	.name		= "sdhci-tegra",
	.id		= 0,
	.resource	= sdhci_resource0,
	.num_resources	= ARRAY_SIZE(sdhci_resource0),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data0,
	},
};

static struct platform_device tegra_sdhci_device2 = {
	.name		= "sdhci-tegra",
	.id		= 2,
	.resource	= sdhci_resource2,
	.num_resources	= ARRAY_SIZE(sdhci_resource2),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data2,
	},
};

static struct platform_device tegra_sdhci_device3 = {
	.name		= "sdhci-tegra",
	.id		= 3,
	.resource	= sdhci_resource3,
	.num_resources	= ARRAY_SIZE(sdhci_resource3),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data3,
	},
};

static int enterprise_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static int enterprise_wifi_set_carddetect(int val)
{
	pr_debug("%s: %d\n", __func__, val);
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

static int enterprise_wifi_power(int on)
{
	//struct tegra_io_dpd *sd_dpd;

	pr_debug("%s: %d\n", __func__, on);

	/*
	 * FIXME : we need to revisit IO DPD code
	 * on how should multiple pins under DPD get controlled
	 *
	 * enterprise GPIO WLAN enable is part of SDMMC1 pin group
	 */
	 if((his_board_version==0) || (his_board_version == 2) || (his_board_version == 3)){
            struct tegra_io_dpd *sd_dpd;
            sd_dpd = tegra_io_dpd_get(&tegra_sdhci_device2.dev);
            if (sd_dpd) {
                mutex_lock(&sd_dpd->delay_lock);
                tegra_io_dpd_disable(sd_dpd);
                mutex_unlock(&sd_dpd->delay_lock);
            }
	
	
            if (on) {
            	gpio_set_value(ENTERPRISE_WLAN_PWR, 1);
            	mdelay(200);
            } else {
            	gpio_set_value(ENTERPRISE_WLAN_PWR, 0);
            }

	
            if (sd_dpd) {
                mutex_lock(&sd_dpd->delay_lock);
                tegra_io_dpd_enable(sd_dpd);
                mutex_unlock(&sd_dpd->delay_lock);
            }
	
 	}
	 else{
		 if (on) {
				 gpio_set_value(ENTERPRISE_WLAN_PWR, 1);
				 mdelay(200);
			 } else {
				 gpio_set_value(ENTERPRISE_WLAN_PWR, 0);
			 }

	 }
	return 0;
}

static int enterprise_wifi_reset(int on)
{
	pr_debug("%s: do nothing\n", __func__);
	return 0;
}

/* DoPa (20140116) - if the wifi MAC address is missing from the
 * kernel's commandline, attempt to read it from the BCK partition;
 * if that fails, generate a random value.  In both cases, plug the
 * value into the field that normally contains the cmdline argument.
 */
static int enterprise_wifi_get_mac_addr(unsigned char *buf)
{
	/* see if the existing value is valid */
	if (enterprise_wifi_validate_mac(buf) != 0) {

		/* if not, try to get it from the BCK partition */
		if (enterprise_wifi_read_mac_from_bck() != 0 ||
			enterprise_wifi_validate_mac(buf) != 0) {

			/* if all else fails, fake it */
			uint rand_mac;
			srandom32((uint)jiffies);
			rand_mac = random32();
			buf[0] = 0x00;
			buf[1] = 0x34;
			buf[2] = 0x9a;
			buf[3] = (unsigned char)(rand_mac & 0x0F) | 0xF0;
			buf[4] = (unsigned char)(rand_mac >> 8);
			buf[5] = (unsigned char)(rand_mac >> 16);
			snprintf(his_wifi_addr, sizeof(his_wifi_addr), "%02X:%02X:%02X:%02X:%02X:%02X",
					 buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
		}
	}

	return 0;
}

/* DoPa (20140116) */
static int enterprise_wifi_validate_mac(unsigned char *buf)
{
	int i, j, s;
	char c, mac_valid;

	mac_valid = 0;
	for (i = 0, j = 0, s = 0; i < MAC_CMDLINE_LEN && j < MAC_ADDR_LEN ; i++) {
		c = his_wifi_addr[i];
		if (c == ':')
			continue;

		if (c >= '0' && c <= '9')
			c -= 0x30;
		else if (c >= 'A' && c <= 'F')
			c -= 0x37;
		else if (c >= 'a' && c <= 'f')
			c -= 0x57;
		else
			break;

		if (!s) {
			buf[j] = c << 4;
		} else {
			buf[j] += c;
			j++;
		}
		s = !s;
		mac_valid |= c;
	}

	if (i < MAC_CMDLINE_LEN || j < MAC_ADDR_LEN || !mac_valid ||
		((buf[0] == 0xFE) && (buf[1] == 0xFF) &&
		 (buf[2] == 0xFE) && (buf[3] == 0xFF) &&
		 (buf[4] == 0xFE) && (buf[5] == 0xFF)))
		return -1;

	return 0;
}

/* DoPa (20140116) */
static int enterprise_wifi_read_mac_from_bck(void)
{
	int i, j, err;
	struct file *f = NULL;
	loff_t pos;
	mm_segment_t fs;
	char readbuf[MAC_BCK_LEN];

	/* Yes, we know that reading from a "file" is verboten, but it was
	 * the manufacturer that made the policy decision to put this raw
	 * data in an unformatted partition, not your humble programmer.
	 */
	f = filp_open("/dev/block/platform/sdhci-tegra.3/by-name/BCK", O_RDONLY, 0);
	if (IS_ERR(f))
		return -1;

	pos = MAC_BCK_OFFSET;
	fs = get_fs();
	set_fs(get_ds());
	err = vfs_read(f, readbuf, MAC_BCK_LEN, &pos);
	set_fs(fs);
	filp_close(f, NULL);

	if (err < 0)
		return -1;

	for (i = 0, j = 0; i < MAC_BCK_LEN; i++, j++) {
		if (i && !(i & 1))
			his_wifi_addr[j++] = ':';
		his_wifi_addr[j] = readbuf[i];
	}
	his_wifi_addr[j] = 0;

	return 0;
}

#ifdef CONFIG_TEGRA_PREPOWER_WIFI
static int __init enterprise_wifi_prepower(void)
{
	//if ((!machine_is_tegra_enterprise()) && (!machine_is_tai()))
	//	return 0;

	enterprise_wifi_power(1);

	return 0;
}

subsys_initcall_sync(enterprise_wifi_prepower);
#endif

static int __init enterprise_wifi_init(void)
{
	if (tegra_get_commchip_id() == COMMCHIP_TI_WL18XX)
		wl12xx_set_platform_data(&enterprise_wl12xx_wlan_data);
	else
		platform_device_register(&enterprise_brcm_wifi_device);

	return 0;
}

int __init enterprise_sdhci_init(void)
{
	platform_device_register(&tegra_sdhci_device3);

	tegra_sdhci_platform_data0.cd_gpio = ENTERPRISE_SD_CD;
	platform_device_register(&tegra_sdhci_device0);

	/* TI wifi module does not use emdedded sdio */
	if (tegra_get_commchip_id() == COMMCHIP_TI_WL18XX) {
#ifdef CONFIG_MMC_EMBEDDED_SDIO
		tegra_sdhci_platform_data2.mmc_data.embedded_sdio = NULL;
#endif
	}

	platform_device_register(&tegra_sdhci_device2);
	enterprise_wifi_init();
	return 0;
}
