/*
 * arch/arm/mach-tegra/board-enterprise.c
 *
 * Copyright (c) 2011-2012, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/spi/spi.h>
#include <linux/tegra_uart.h>
#include <linux/fsl_devices.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/memblock.h>
#include <linux/rfkill-gpio.h>
//#include <linux/mfd/tlv320aic3262-registers.h>
//#include <linux/mfd/tlv320aic3262-core.h>

#include <linux/nfc/pn544.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <sound/max98088.h>
#include <linux/nfc/bcm2079x.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/io_dpd.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <mach/i2s.h>
#include <mach/tegra_asoc_pdata.h>
#include <mach/thermal.h>
#include <mach/tegra-bb-power.h>
#include <mach/tegra_fiq_debugger.h>

#include <linux/mfd/tlv320aic3256-registers.h>
#include <linux/mfd/tlv320aic3xxx-core.h>
#include <sound/tlv320aic325x.h>
#include "board.h"
#include "clock.h"
#include "board-enterprise.h"
#include "baseband-xmm-power.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "pm.h"
extern unsigned int his_hw_ver;
extern unsigned int his_board_version;

#ifdef CONFIG_TOUCHSCREEN_FT5X06
#include <linux/i2c/ft5x06_ts.h>
#endif

#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
static struct throttle_table throttle_freqs_tj[] = {
	      /*    CPU,    CBUS,    SCLK,     EMC */
	      { 1000000,  NO_CAP,  NO_CAP,  NO_CAP },
	      {  760000,  NO_CAP,  NO_CAP,  NO_CAP },
	      {  760000,  NO_CAP,  NO_CAP,  NO_CAP },
	      {  620000,  NO_CAP,  NO_CAP,  NO_CAP },
	      {  620000,  NO_CAP,  NO_CAP,  NO_CAP },
	      {  620000,  437000,  NO_CAP,  NO_CAP },
	      {  620000,  352000,  NO_CAP,  NO_CAP },
	      {  475000,  352000,  NO_CAP,  NO_CAP },
	      {  475000,  352000,  NO_CAP,  NO_CAP },
	      {  475000,  352000,  250000,  375000 },
	      {  475000,  352000,  250000,  375000 },
	      {  475000,  247000,  204000,  375000 },
	      {  475000,  247000,  204000,  204000 },
	      {  475000,  247000,  204000,  204000 },
	{ CPU_THROT_LOW,  247000,  204000,  102000 },
};
#endif

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static struct throttle_table throttle_freqs_tskin[] = {
	      /*    CPU,    CBUS,    SCLK,     EMC */
	      { 1000000,  NO_CAP,  NO_CAP,  NO_CAP },
	      {  760000,  NO_CAP,  NO_CAP,  NO_CAP },
	      {  760000,  NO_CAP,  NO_CAP,  NO_CAP },
	      {  620000,  NO_CAP,  NO_CAP,  NO_CAP },
	      {  620000,  NO_CAP,  NO_CAP,  NO_CAP },
	      {  620000,  437000,  NO_CAP,  NO_CAP },
	      {  620000,  352000,  NO_CAP,  NO_CAP },
	      {  475000,  352000,  NO_CAP,  NO_CAP },
	      {  475000,  352000,  NO_CAP,  NO_CAP },
	      {  475000,  352000,  250000,  375000 },
	      {  475000,  352000,  250000,  375000 },
	      {  475000,  247000,  204000,  375000 },
	      {  475000,	247000,  204000,  375000 },
	      {  475000,	247000,  204000,  375000 },
	      {  475000,	247000,  204000,  375000 },

};
#endif

static struct balanced_throttle throttle_list[] = {
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	{
		.id = BALANCED_THROTTLE_ID_TJ,
		.throt_tab_size = ARRAY_SIZE(throttle_freqs_tj),
		.throt_tab = throttle_freqs_tj,
	},
#endif
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	{
		.id = BALANCED_THROTTLE_ID_SKIN,
		.throt_tab_size = ARRAY_SIZE(throttle_freqs_tskin),
		.throt_tab = throttle_freqs_tskin,
	},
#endif
};

/* All units are in millicelsius */
static struct tegra_thermal_data thermal_data = {
	.shutdown_device_id = THERMAL_DEVICE_ID_NCT_EXT,
	.temp_shutdown = 90000,
#if defined(CONFIG_TEGRA_EDP_LIMITS) || defined(CONFIG_TEGRA_THERMAL_THROTTLE)
	.throttle_edp_device_id = THERMAL_DEVICE_ID_NCT_EXT,
#endif
#ifdef CONFIG_TEGRA_EDP_LIMITS
	.edp_offset = TDIODE_OFFSET,  /* edp based on tdiode */
	.hysteresis_edp = 3000,
#endif
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	.temp_throttle = 85000,
	.tc1 = 0,
	.tc2 = 1,
	.passive_delay = 2000,
#endif
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	.skin_device_id = THERMAL_DEVICE_ID_SKIN,
	.temp_throttle_skin = 43000,
        .tc1_skin = 0,
        .tc2_skin = 1,
        .passive_delay_skin = 5000,
 
        .skin_temp_offset = 3218, /* -2685,  476, 6333,*/
        .skin_period = 1100, /*5500, */
        .skin_devs_size = 2,
        .skin_devs = {
   /*			   {
                        // jprimero: also need to add THERMAL_DEVICE_ID_BATT value to
                        // arch/arm/mach-tegra/include/mach/thermal.h
                     
                        "batt",
                        THERMAL_DEVICE_ID_BATT,
                        {
                                0, 1, 2, 1,
                                2, 1, 1, 0,
                                0, 0, 0, -1,
                                0, -2, 0, -2,
                                -1, -2, 1, -1
                        }
                },
*/
                {
                        "nct_ext",
                        THERMAL_DEVICE_ID_NCT_EXT,
                        {
				3, 0, 1, 1,
				1, 0, 0, 2,
				0, 1, -1, -1,
				0, 0, 0, -1,
				-1, 0, -1, -4

                        }
                },
                {
                        "nct_int",
                        THERMAL_DEVICE_ID_NCT_INT,
                        {
				9, 1, 8, 7,
				9, -1, 4, -8,
				5, 7, 9, -5,
				-2, -3, 6, 7,
				5, 6, -4, 21



                        }
                },
        },
	
#endif
};


static struct resource enterprise_bluedroid_pm_resources1[] = {
	[0] = {
		.name   = "shutdown_gpio",
		.start  = TEGRA_GPIO_BT_REG_ON,
		.end    = TEGRA_GPIO_BT_REG_ON,
		.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name   = "reset_gpio",
		.start  = TEGRA_GPIO_PD4,
		.end    = TEGRA_GPIO_PD4,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device enterprise_bluedroid_pm_device1 = {
	.name = "bluedroid_pm",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(enterprise_bluedroid_pm_resources1),
	.resource       = enterprise_bluedroid_pm_resources1,
};

static struct resource enterprise_bluedroid_pm_resources2[] = {
	[0] = {
		.name   = "shutdown_gpio",
		.start  = TEGRA_GPIO_BT_REG_ON,
		.end    = TEGRA_GPIO_BT_REG_ON,
		.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name   = "reset_gpio",
		.start  = TEGRA_GPIO_PW2,
		.end    = TEGRA_GPIO_PW2,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device enterprise_bluedroid_pm_device2 = {
	.name = "bluedroid_pm",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(enterprise_bluedroid_pm_resources2),
	.resource       = enterprise_bluedroid_pm_resources2,
};

static void __init enterprise_bluedroid_pm(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);

    if(his_board_version==1){
        	printk("sunzizhi add hw_ver is %d\n",his_board_version);
        	platform_device_register(&enterprise_bluedroid_pm_device1);
	}
    else
	{
            printk("sunzizhi add hw_ver is %d\n",his_board_version);
            platform_device_register(&enterprise_bluedroid_pm_device2);
	}
	return;
}



static struct resource enterprise_bluesleep_resources[] = {
	[0] = {
		.name = "gpio_host_wake",
			.start  = TEGRA_GPIO_BT_HOST_WAKE,
			.end    = TEGRA_GPIO_BT_HOST_WAKE,
			.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "gpio_ext_wake",
			.start  = TEGRA_GPIO_HOST_BT_WAKE,
			.end    = TEGRA_GPIO_HOST_BT_WAKE,
			.flags  = IORESOURCE_IO,
	},
	[2] = {
		.name = "host_wake",
			.start  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_BT_HOST_WAKE),
			.end    = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_BT_HOST_WAKE),
			.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
//			.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device enterprise_bluesleep_device = {
	.name           = "bluesleep",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(enterprise_bluesleep_resources),
	.resource       = enterprise_bluesleep_resources,
};
extern void bluesleep_setup_uart_port(struct platform_device *uart_dev);

static void __init enterprise_setup_bluesleep(void)
{
	platform_device_register(&enterprise_bluesleep_device);
	bluesleep_setup_uart_port(&tegra_uartc_device);
	tegra_gpio_enable(TEGRA_GPIO_BT_HOST_WAKE);
	tegra_gpio_enable(TEGRA_GPIO_HOST_BT_WAKE);
	return;
}

static void __init enterprise_gps_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_GPS_PWN);
	tegra_gpio_enable(TEGRA_GPIO_GPS_RST_N);
}

static __initdata struct tegra_clk_init_table enterprise_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x","pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	40800000,	false},
	{ "blink",	"clk_32k",	32768,		true},
	{ "i2s0",	"pll_a_out0",	0,		false},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"clk_m",	13000000,	false},
	{ "dam0",	"clk_m",	13000000,	false},
	{ "dam1",	"clk_m",	13000000,	false},
	{ "dam2",	"clk_m",	13000000,	false},
	{ "audio0",	"i2s0_sync",	0,		false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "audio2",	"i2s2_sync",	0,		false},
	{ "audio3",	"i2s3_sync",	0,		false},
	{ "audio4",	"i2s4_sync",	0,		false},
	{ "vi",		"pll_p",	24000000,		false},
	{ "vi_sensor",	"pll_p",	0,		false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ NULL,		NULL,		0,		0},
};

static __initdata struct tegra_clk_init_table enterprise_clk_i2s2_table[] = {
	/* name		parent		rate		enabled */
	{ "i2s2",	"pll_a_out0",	0,		false},
	{ NULL,		NULL,		0,		0},
};

static __initdata struct tegra_clk_init_table enterprise_clk_i2s4_table[] = {
	/* name		parent		rate		enabled */
	{ "i2s4",	"pll_a_out0",	0,		false},
	{ NULL,		NULL,		0,		0},
};
#if 1
static struct aic3256_gpio_setup aic3256_gpio[] = {
	/* GPIO 1*/
	{
		.used		= 1,
		.in		= 0,
		.value		= AIC3256_GPIO_MFP5_FUNC_OUTPUT ,
	},
	/* GPIO 2*/
	{
		.used		= 0,
		.in		= 0,
	},
	/* GPIO 1 */
	{
		.used		= 0,
	},
	{// GPI2
		.used		= 0,
		.in		= 1,
	},
	{// GPO1
		.used		= 0,
	},
};
#endif


static struct bcm2079x_platform_data nfc_pdata = {
	.irq_gpio = TEGRA_GPIO_NFC_IRQ,
	.en_gpio = TEGRA_GPIO_NFC_PWN,
	.wake_gpio = TEGRA_GPIO_NFC_WAKE,
};

static struct i2c_board_info __initdata enterprise_bcm2079x_info = {
	I2C_BOARD_INFO("bcm2079x-i2c", 0x77),
	.platform_data = &nfc_pdata,
	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_NFC_IRQ),
};

static struct tegra_i2c_platform_data enterprise_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= {400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data enterprise_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	//.is_clkon_always = true,//opposite ops refer to i2c-tegra es
	.scl_gpio		= {TEGRA_GPIO_PT5, 0},
	.sda_gpio		= {TEGRA_GPIO_PT6, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data enterprise_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
    .bus_clk_rate	= {100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PBB1, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB2, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data enterprise_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 10000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PV4, 0},
	.sda_gpio		= {TEGRA_GPIO_PV5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data enterprise_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 390000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};

/* Equalizer filter coefs generated from the MAXIM MAX98088
 * evkit software tool */
static struct max98088_eq_cfg max98088_eq_cfg[] = {
	{
		.name = "FLAT",
		.rate = 44100,
		.band1 = {0x2000, 0xC002, 0x4000, 0x00E9, 0x0000},
		.band2 = {0x2000, 0xC00F, 0x4000, 0x02BC, 0x0000},
		.band3 = {0x2000, 0xC0A7, 0x4000, 0x0916, 0x0000},
		.band4 = {0x2000, 0xC5C2, 0x4000, 0x1A87, 0x0000},
		.band5 = {0x2000, 0xF6B0, 0x4000, 0x3F51, 0x0000},
	},
	{
		.name = "LOWPASS1K",
		.rate = 44100,
		.band1 = {0x205D, 0xC001, 0x3FEF, 0x002E, 0x02E0},
		.band2 = {0x5B9A, 0xC093, 0x3AB2, 0x088B, 0x1981},
		.band3 = {0x0D22, 0xC170, 0x26EA, 0x0D79, 0x32CF},
		.band4 = {0x0894, 0xC612, 0x01B3, 0x1B34, 0x3FFA},
		.band5 = {0x0815, 0x3FFF, 0xCF78, 0x0000, 0x29B7},
	},
	{ /* BASS=-12dB, TREBLE=+9dB, Fc=5KHz */
		.name = "HIBOOST",
		.rate = 44100,
		.band1 = {0x0815, 0xC001, 0x3AA4, 0x0003, 0x19A2},
		.band2 = {0x0815, 0xC103, 0x092F, 0x0B55, 0x3F56},
		.band3 = {0x0E0A, 0xC306, 0x1E5C, 0x136E, 0x3856},
		.band4 = {0x2459, 0xF665, 0x0CAA, 0x3F46, 0x3EBB},
		.band5 = {0x5BBB, 0x3FFF, 0xCEB0, 0x0000, 0x28CA},
	},
	{ /* BASS=12dB, TREBLE=+12dB */
		.name = "LOUD12DB",
		.rate = 44100,
		.band1 = {0x7FC1, 0xC001, 0x3EE8, 0x0020, 0x0BC7},
		.band2 = {0x51E9, 0xC016, 0x3C7C, 0x033F, 0x14E9},
		.band3 = {0x1745, 0xC12C, 0x1680, 0x0C2F, 0x3BE9},
		.band4 = {0x4536, 0xD7E2, 0x0ED4, 0x31DD, 0x3E42},
		.band5 = {0x7FEF, 0x3FFF, 0x0BAB, 0x0000, 0x3EED},
	},
	{
		.name = "FLAT",
		.rate = 16000,
		.band1 = {0x2000, 0xC004, 0x4000, 0x0141, 0x0000},
		.band2 = {0x2000, 0xC033, 0x4000, 0x0505, 0x0000},
		.band3 = {0x2000, 0xC268, 0x4000, 0x115F, 0x0000},
		.band4 = {0x2000, 0xDA62, 0x4000, 0x33C6, 0x0000},
		.band5 = {0x2000, 0x4000, 0x4000, 0x0000, 0x0000},
	},
	{
		.name = "LOWPASS1K",
		.rate = 16000,
		.band1 = {0x2000, 0xC004, 0x4000, 0x0141, 0x0000},
		.band2 = {0x5BE8, 0xC3E0, 0x3307, 0x15ED, 0x26A0},
		.band3 = {0x0F71, 0xD15A, 0x08B3, 0x2BD0, 0x3F67},
		.band4 = {0x0815, 0x3FFF, 0xCF78, 0x0000, 0x29B7},
		.band5 = {0x0815, 0x3FFF, 0xCF78, 0x0000, 0x29B7},
	},
	{ /* BASS=-12dB, TREBLE=+9dB, Fc=2KHz */
		.name = "HIBOOST",
		.rate = 16000,
		.band1 = {0x0815, 0xC001, 0x3BD2, 0x0009, 0x16BF},
		.band2 = {0x080E, 0xC17E, 0xF653, 0x0DBD, 0x3F43},
		.band3 = {0x0F80, 0xDF45, 0xEE33, 0x36FE, 0x3D79},
		.band4 = {0x590B, 0x3FF0, 0xE882, 0x02BD, 0x3B87},
		.band5 = {0x4C87, 0xF3D0, 0x063F, 0x3ED4, 0x3FB1},
	},
	{ /* BASS=12dB, TREBLE=+12dB */
		.name = "LOUD12DB",
		.rate = 16000,
		.band1 = {0x7FC1, 0xC001, 0x3D07, 0x0058, 0x1344},
		.band2 = {0x2DA6, 0xC013, 0x3CF1, 0x02FF, 0x138B},
		.band3 = {0x18F1, 0xC08E, 0x244D, 0x0863, 0x34B5},
		.band4 = {0x2BE0, 0xF385, 0x04FD, 0x3EC5, 0x3FCE},
		.band5 = {0x7FEF, 0x4000, 0x0BAB, 0x0000, 0x3EED},
	},
};


static struct max98088_pdata enterprise_max98088_pdata = {
	/* equalizer configuration */
	.eq_cfg = max98088_eq_cfg,
	.eq_cfgcnt = ARRAY_SIZE(max98088_eq_cfg),

	/* debounce time */
	.debounce_time_ms = 200,

	/* microphone configuration */
	.digmic_left_mode = 1,
	.digmic_right_mode = 1,

	/* receiver output configuration */
	.receiver_mode = 0,	/* 0 = amplifier, 1 = line output */
};

static struct aic3xxx_pdata aic3256_codec_pdata = {
	/* debounce time */
	.gpio_irq	= 1,
	.gpio_reset	= TEGRA_GPIO_CODEC_RST,
	.gpio		= aic3256_gpio,
	.naudint_irq	= TEGRA_GPIO_CDC_IRQ_N,
	.jackint_irq  = TEGRA_GPIO_M470_HP_DET,
	.keyint_irq = TEGRA_GPIO_M470_KEY_DET,
	.irq_base	= AIC3256_CODEC_IRQ_BASE,
	.debounce_time_ms = 512,
};


static struct i2c_board_info __initdata max98088_board_info = {
	I2C_BOARD_INFO("max98088", 0x10),
	.platform_data = &enterprise_max98088_pdata,
	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_HP_DET),
};

static struct i2c_board_info __initdata enterprise_codec_aic325x_info = {
	I2C_BOARD_INFO("tlv320aic325x", 0x18),
	.platform_data = &aic3256_codec_pdata,
	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_CDC_IRQ_N),
};
static void enterprise_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &enterprise_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &enterprise_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &enterprise_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &enterprise_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &enterprise_i2c5_platform_data;

	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);

	i2c_register_board_info(0, &max98088_board_info, 1);
	enterprise_codec_aic325x_info.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_CDC_IRQ_N);
	i2c_register_board_info(0, &enterprise_codec_aic325x_info, 1);
	i2c_register_board_info(0, &enterprise_bcm2079x_info, 1);
}

static struct platform_device *enterprise_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
	&tegra_uarte_device,
};

static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};
static struct tegra_uart_platform_data enterprise_uart_pdata;
static struct tegra_uart_platform_data enterprise_loopback_uart_pdata;

static void __init uart_debug_init(void)
{
	unsigned long rate;
	struct clk *c;

	/* UARTD is the debug port. */
	pr_info("Selecting UARTD as the debug console\n");
	enterprise_uart_devices[3] = &debug_uartd_device;
	debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->mapbase;
	debug_uart_clk = clk_get_sys("serial8250.0", "uartd");

	/* Clock enable for the debug channel */
	if (!IS_ERR_OR_NULL(debug_uart_clk)) {
		rate = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->uartclk;
		pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
		c = tegra_get_clock_by_name("pll_p");
		if (IS_ERR_OR_NULL(c))
			pr_err("Not getting the parent clock pll_p\n");
		else
			clk_set_parent(debug_uart_clk, c);

		clk_enable(debug_uart_clk);
		clk_set_rate(debug_uart_clk, rate);
	} else {
		pr_err("Not getting the clock %s for debug console\n",
				debug_uart_clk->name);
	}
}

static void __init enterprise_uart_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	enterprise_uart_pdata.parent_clk_list = uart_parent_clk;
	enterprise_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	enterprise_loopback_uart_pdata.parent_clk_list = uart_parent_clk;
	enterprise_loopback_uart_pdata.parent_clk_count =
						ARRAY_SIZE(uart_parent_clk);
	enterprise_loopback_uart_pdata.is_loopback = true;
	tegra_uarta_device.dev.platform_data = &enterprise_uart_pdata;
	tegra_uartb_device.dev.platform_data = &enterprise_uart_pdata;
	tegra_uartc_device.dev.platform_data = &enterprise_uart_pdata;
	tegra_uartd_device.dev.platform_data = &enterprise_uart_pdata;
	/* UARTE is used for loopback test purpose */
	tegra_uarte_device.dev.platform_data = &enterprise_loopback_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(enterprise_uart_devices,
				ARRAY_SIZE(enterprise_uart_devices));
}


/* add vibrator for enterprise */
static struct platform_device vibrator_device = {
	.name = "tegra-vibrator",
	.id = -1,
};

static noinline void __init enterprise_vibrator_init(void)
{
	platform_device_register(&vibrator_device);
}



static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

static struct tegra_asoc_platform_data enterprise_audio_aic325x_pdata = {
	.gpio_spkr_en = -1,
	.gpio_hp_det = TEGRA_GPIO_CDC_IRQ_N,
	.gpio_hp_mute = -1,
	.gpio_int_mic_en = -1,
	.gpio_ext_mic_en = -1,
	.debounce_time_hp = 200,
	.i2s_param[HIFI_CODEC]  = {
		.audio_port_id	= 1,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_I2S,
		.sample_size	= 16,
	},
};
static struct platform_device enterprise_audio_aic325x_device = {
	.name	= "tegra-snd-aic325x",
	.id	= 0,
	.dev	= {
		.platform_data  = &enterprise_audio_aic325x_pdata,
	},
};

static struct platform_device *enterprise_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_INPUT_KEYRESET)
	&tegra_keyreset_device,
#endif
#if defined(CONFIG_TEGRA_IOVMM_SMMU) || defined(CONFIG_TEGRA_IOMMU_SMMU)
	&tegra_smmu_device,
#endif
	&tegra_wdt0_device,
	&tegra_wdt1_device,
	&tegra_wdt2_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
	&tegra_camera,
	&tegra_spi_device4,
	&tegra_hda_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra_se_device,
#endif
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device0,
	&tegra_i2s_device1,
	&tegra_i2s_device3,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&baseband_dit_device,
	&tegra_pcm_device,
	&enterprise_audio_aic325x_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
};

#define MXT_CONFIG_CRC 0x62F903
/*
 * Config converted from memory-mapped cfg-file with
 * following version information:
 *
 *
 *
 *      FAMILY_ID=128
 *      VARIANT=1
 *      VERSION=32
 *      BUILD=170
 *      VENDOR_ID=255
 *      PRODUCT_ID=TBD
 *      CHECKSUM=0xC189B6
 *
 *
 */
#ifdef CONFIG_TOUCHSCREEN_FT5X06
	static struct ft5x06_platform_data ft_platform_data = {
		.x_max = 800,
		.y_max = 1280,
	};
	
	
	static const struct i2c_board_info ft5x06_i2c_touch_info[] = {
		{
			I2C_BOARD_INFO("ft5x06", 0x38),
			.irq		= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PZ3),
			.platform_data = &ft_platform_data,
		},
	};
	
	static int __init enterprise_ft5x06_touch_init(void)
	{
		tegra_gpio_enable(TEGRA_GPIO_PZ3);
		tegra_gpio_enable(TEGRA_GPIO_TP_VDD_EN);
		tegra_gpio_enable(TEGRA_GPIO_TS_WAKE);
	
		//gpio_request(TEGRA_GPIO_TS_WAKE, "tp_wake");
		gpio_direction_output(TEGRA_GPIO_TS_WAKE, 1);
		msleep(20);
		
		//gpio_request(TEGRA_GPIO_TP_VDD_EN, "tp_vdd_en");
		gpio_direction_output(TEGRA_GPIO_TP_VDD_EN, 1);
		
		i2c_register_board_info(1, ft5x06_i2c_touch_info, 1);
	
		return 0;
	}
	
#endif


static void enterprise_usb_hsic_postsupend(void)
{
	pr_debug("%s\n", __func__);
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L2);
#endif
}

static void enterprise_usb_hsic_preresume(void)
{
	pr_debug("%s\n", __func__);
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L2TOL0);
#endif
}

static void enterprise_usb_hsic_post_resume(void)
{
	pr_debug("%s\n", __func__);
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L0);
#endif
}

static void enterprise_usb_hsic_phy_power(void)
{
	pr_debug("%s\n", __func__);
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L0);
#endif
}

static void enterprise_usb_hsic_post_phy_off(void)
{
	pr_debug("%s\n", __func__);
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L2);
#endif
}

static struct tegra_usb_phy_platform_ops hsic_xmm_plat_ops = {
	.post_suspend = enterprise_usb_hsic_postsupend,
	.pre_resume = enterprise_usb_hsic_preresume,
	.port_power = enterprise_usb_hsic_phy_power,
	.post_resume = enterprise_usb_hsic_post_resume,
	.post_phy_off = enterprise_usb_hsic_post_phy_off,
};

static struct tegra_usb_platform_data tegra_ehci2_hsic_xmm_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_HSIC,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = false,
	},
	.ops = &hsic_xmm_plat_ops,
};

static struct tegra_usb_platform_data tegra_udc_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.builtin_host_disabled = false,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = ENT_TPS80031_IRQ_BASE +
				TPS80031_INT_VBUS_DET,
		.vbus_gpio = -1,
		.charging_supported = false,
		.remote_wakeup_supported = false,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.builtin_host_disabled = false,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.vbus_reg = "usb_vbus",
		.hot_plug = true,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};

struct platform_device *tegra_usb_hsic_host_register(void)
{
	struct platform_device *pdev;
	int val;

	pdev = platform_device_alloc(tegra_ehci2_device.name,
		tegra_ehci2_device.id);
	if (!pdev)
		return NULL;

	val = platform_device_add_resources(pdev, tegra_ehci2_device.resource,
		tegra_ehci2_device.num_resources);
	if (val)
		goto error;

	pdev->dev.dma_mask =  tegra_ehci2_device.dev.dma_mask;
	pdev->dev.coherent_dma_mask = tegra_ehci2_device.dev.coherent_dma_mask;

	val = platform_device_add_data(pdev, &tegra_ehci2_hsic_xmm_pdata,
			sizeof(struct tegra_usb_platform_data));
	if (val)
		goto error;

	val = platform_device_add(pdev);
	if (val)
		goto error;

	return pdev;

error:
	pr_err("%s: failed to add the host contoller device\n", __func__);
	platform_device_put(pdev);
	return NULL;
}

void tegra_usb_hsic_host_unregister(struct platform_device **platdev)
{
	struct platform_device *pdev = *platdev;

	if (pdev && &pdev->dev) {
		platform_device_unregister(pdev);
		*platdev = NULL;
	} else
		pr_err("%s: no platform device\n", __func__);
}

static void enterprise_usb_init(void)
{
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;

	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);
}





#ifdef CONFIG_BATTERY_BQ27x00
//BQ27541 GasGauge
static struct i2c_board_info   __initdata gen2_i2c_bq27541[] = {
	{
		I2C_BOARD_INFO("bq27541", 0x55),
	},
};
#endif

/*
** Enable pwm clock, heqi add
*/
static void __init tegra_enterprise_pwm_clk_init(void)
{
	struct clk *c;
	int ret = 0;

	c = tegra_get_clock_by_name(enterprise_clk_init_table[3].name);

	ret = clk_enable(c);
	if (ret) 
		pr_warning("Unable to enable clock %s: %d\n", enterprise_clk_init_table[3].name, ret);
}

static void __init tegra_enterprise_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	if (board_info.fab == BOARD_FAB_A04)
		tegra_clk_init_from_table(enterprise_clk_i2s4_table);
	else
		tegra_clk_init_from_table(enterprise_clk_i2s2_table);

	tegra_thermal_init(&thermal_data,
				throttle_list,
				ARRAY_SIZE(throttle_list));
	tegra_clk_init_from_table(enterprise_clk_init_table);
	//heqi add
	tegra_enterprise_pwm_clk_init();
	
	tegra_soc_device_init("tegra_enterprise");
	enterprise_pinmux_init();
	enterprise_i2c_init();
	enterprise_uart_init();
	enterprise_usb_init();
//#ifdef CONFIG_BT_BLUESLEEP
	//if (board_info.board_id == BOARD_E1239)
	//	enterprise_bt_rfkill_pdata[0].reset_gpio = TEGRA_GPIO_PF4;
//#endif
	platform_add_devices(enterprise_devices, ARRAY_SIZE(enterprise_devices));
	tegra_ram_console_debug_init();
#ifdef CONFIG_BATTERY_BQ27x00
        //ADD Battery GauGauge
	i2c_register_board_info(1, gen2_i2c_bq27541,
			ARRAY_SIZE(gen2_i2c_bq27541));
#endif
	enterprise_regulator_init();
	tegra_io_dpd_init();
	enterprise_sdhci_init();
#ifdef CONFIG_TEGRA_EDP_LIMITS
	enterprise_edp_init();
#endif
	//enterprise_kbc_init();
	enterprise_keys_init();
	
	//enterprise_nfc_init();
	//enterprise_touch_init();
#ifdef CONFIG_TOUCHSCREEN_FT5X06
		enterprise_ft5x06_touch_init();
#endif
	//enterprise_audio_init();
	//enterprise_baseband_init();
	enterprise_panel_init();
	//enterprise_bt_rfkill();
	//enterprise_setup_bluesleep();
	enterprise_bluedroid_pm();
	enterprise_gps_init();
	enterprise_setup_bluesleep();

	enterprise_emc_init();
	enterprise_sensors_init();
	enterprise_suspend_init();
	//enterprise_bpc_mgmt_init();
	tegra_release_bootloader_fb();
	tegra_serial_debug_init(TEGRA_UARTD_BASE, INT_WDT_CPU, NULL, -1, -1);
	enterprise_vibrator_init();
}

static void __init tegra_enterprise_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
#if (defined(CONFIG_BOARD_M470) || defined(CONFIG_BOARD_M470BSD) || defined(CONFIG_BOARD_M470BSS))
	tegra_reserve(0, SZ_8M, SZ_8M);
#else
	tegra_reserve(0, SZ_4M, SZ_8M);
#endif
#else
	tegra_reserve(SZ_128M, SZ_4M, SZ_8M);
#endif
	tegra_ram_console_debug_reserve(SZ_1M);
}

static const char *enterprise_dt_board_compat[] = {
	"nvidia,enterprise",
	NULL
};

static const char *tai_dt_board_compat[] = {
	"nvidia,tai",
	NULL
};

#if defined(CONFIG_BOARD_M470)
MACHINE_START(M470, "m470")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_enterprise_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_enterprise_init,
	.dt_compat	= enterprise_dt_board_compat,
MACHINE_END
#elif defined(CONFIG_BOARD_M470BSD)
MACHINE_START(M470BSD, "m470bsd")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_enterprise_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_enterprise_init,
	.dt_compat	= enterprise_dt_board_compat,
MACHINE_END
#elif defined(CONFIG_BOARD_M470BSS)
MACHINE_START(M470BSS, "m470bss")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_enterprise_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_enterprise_init,
	.dt_compat	= enterprise_dt_board_compat,
MACHINE_END
#else
MACHINE_START(TEGRA_ENTERPRISE, "tegra_enterprise")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_enterprise_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_enterprise_init,
	.dt_compat	= enterprise_dt_board_compat,
MACHINE_END
#endif

MACHINE_START(TAI, "tai")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_enterprise_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_enterprise_init,
	.dt_compat	= tai_dt_board_compat,
MACHINE_END
