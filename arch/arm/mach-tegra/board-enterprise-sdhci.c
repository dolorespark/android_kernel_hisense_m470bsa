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

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static int enterprise_wifi_status_register(void (*callback)(int , void *), void *);

static int enterprise_wifi_reset(int on);
static int enterprise_wifi_power(int on);
static int enterprise_wifi_set_carddetect(int val);
static int enterprise_wifi_get_mac_addr(unsigned char *buf);
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

static int enterprise_wifi_get_mac_addr(unsigned char *buf)
{
	//struct file *filp=NULL;
	//struct inode *inode;
	//unsigned long magic;
	//off_t fsize;
	//loff_t pos = 0;
	//ssize_t retValue = 0;
	char readbuf[18];
	char tmp[MAC_ADDR_LEN*2];
	//mm_segment_t old_fs;
	//int sz;
	int i, ii=0;
	char mac_valid;
	uint rand_mac;
	//char iovbuf[MAC_ADDR_LEN];
	//printk("liuqiang : custom mac addr %s \n",his_wifi_addr);


	memcpy(readbuf,his_wifi_addr,18);

	for(i = 0; i < (MAC_ADDR_LEN*2+5); i++){
		if(readbuf[i] == ':')
			continue;
		else if((readbuf[i] >= 48)&&(readbuf[i] <= 57))
			readbuf[i] -= 48;
		else if((readbuf[i] >= 65)&&(readbuf[i] <= 70))
			readbuf[i] -= 55;
		else
			goto macerr;
		tmp[ii] = readbuf[i];
		ii++;
	}

	mac_valid = 0;

	for(i = 0; i <MAC_ADDR_LEN; i++){
		buf[i] = ((tmp[i*2] << 4) | tmp[i*2+1]);
		mac_valid |= buf[i];
		//printk("buf[%d]=0x%02x\n", i, buf[i]);
	}

	if((!mac_valid)||((buf[0] == 0xFE)&&(buf[1] == 0xFF)&&(buf[2] == 0xFE)&&
					  (buf[3] == 0xFF)&&(buf[4] == 0xFE)&&(buf[5] == 0xFF)))
		goto macerr;
	else
		goto exit;

macerr:
	/* Generate random MAC address */
	printk("[DHD]: MAC is invalid, use random MAC address!\n");

	srandom32((uint)jiffies);
	rand_mac = random32();
	buf[0] = 0x00;
	buf[1] = 0x34;
	buf[2] = 0x9a;
	buf[3] = (unsigned char)(rand_mac & 0x0F) | 0xF0;
	buf[4] = (unsigned char)(rand_mac >> 8);
	buf[5] = (unsigned char)(rand_mac >> 16);

	//for(i = 0; i <MAC_ADDR_LEN; i++){
 	//	printk("buf[%d]=0x%02x\n", i, buf[i]);
 	//}


 exit:
 	/* close file before return */
	//filp_close(filp, current->files);
 	/* restore previous address limit */
	//set_fs(old_fs);
 
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
