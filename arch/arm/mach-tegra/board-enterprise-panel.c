/*
 * arch/arm/mach-tegra/board-enterprise-panel.c
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/tegra_pwm_bl.h>
#include <linux/pwm_backlight.h>
#include <asm/atomic.h>
#include <linux/nvhost.h>
#include <linux/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <mach/hardware.h>
#include <mach/clk.h>
#include <linux/clk.h>

#include "board.h"
#include "board-enterprise.h"
#include "devices.h"
#include "gpio-names.h"
#include "tegra3_host1x_devices.h"

//for m470
#define enterprise_en_lcd_1v8	TEGRA_GPIO_EN_LCD_1V8	//TEGRA_GPIO_PH5
#define enterprise_en_lcd_3v3	TEGRA_GPIO_EN_LCD_3V3	//TEGRA_GPIO_PC6
#define enterprise_lvds_shtdn_n	TEGRA_GPIO_LVDS_SHTDN_N	//TEGRA_GPIO_PN6
#define enterprise_en_vdd_pnl	TEGRA_GPIO_EN_VDD_PNL	//TEGRA_GPIO_PW1
#define enterprise_lcd_bl_en		TEGRA_GPIO_LCD_BL_EN	//TEGRA_GPIO_PH2
#define enterprise_lcd_bl_pwm	TEGRA_GPIO_BL_PWM
#define enterprise_en_vdd_bl		TEGRA_GPIO_EN_VDD_BL	//TEGRA_GPIO_PH3
#define enterprise_hdmi_hpd		TEGRA_GPIO_HDMI_HPD		//TEGRA_GPIO_PN7

/* default brightness, heqi */
#define DEFAULT_BRIGHTNESS		66

#ifdef CONFIG_TEGRA_DC
static struct regulator *enterprise_hdmi_reg;
static struct regulator *enterprise_hdmi_pll;
static struct regulator *enterprise_hdmi_vddio;
#endif

static atomic_t sd_brightness = ATOMIC_INIT(255);
struct clk *disp1_emc_min_clk;
static struct tegra_dc_platform_data enterprise_disp1_pdata;

static tegra_dc_bl_output enterprise_bl_output_measured_a02 = {
	1, 5, 9, 10, 11, 12, 12, 13,
	13, 14, 14, 15, 15, 16, 16, 17,
	17, 18, 18, 19, 19, 20, 21, 21,
	22, 22, 23, 24, 24, 25, 26, 26,
	27, 27, 28, 29, 29, 31, 31, 32,
	32, 33, 34, 35, 36, 36, 37, 38,
	39, 39, 40, 41, 41, 42, 43, 43,
	44, 45, 45, 46, 47, 47, 48, 49,
	49, 50, 51, 51, 52, 53, 53, 54,
	55, 56, 56, 57, 58, 59, 60, 61,
	61, 62, 63, 64, 65, 65, 66, 67,
	67, 68, 69, 69, 70, 71, 71, 72,
	73, 73, 74, 74, 75, 76, 76, 77,
	77, 78, 79, 79, 80, 81, 82, 83,
	83, 84, 85, 85, 86, 86, 88, 89,
	90, 91, 91, 92, 93, 93, 94, 95,
	95, 96, 97, 97, 98, 99, 99, 100,
	101, 101, 102, 103, 103, 104, 105, 105,
	107, 107, 108, 109, 110, 111, 111, 112,
	113, 113, 114, 115, 115, 116, 117, 117,
	118, 119, 119, 120, 121, 122, 123, 124,
	124, 125, 126, 126, 127, 128, 129, 129,
	130, 131, 131, 132, 133, 133, 134, 135,
	135, 136, 137, 137, 138, 139, 139, 140,
	142, 142, 143, 144, 145, 146, 147, 147,
	148, 149, 149, 150, 151, 152, 153, 153,
	153, 154, 155, 156, 157, 158, 158, 159,
	160, 161, 162, 163, 163, 164, 165, 165,
	166, 166, 167, 168, 169, 169, 170, 170,
	171, 172, 173, 173, 174, 175, 175, 176,
	176, 178, 178, 179, 180, 181, 182, 182,
	183, 184, 185, 186, 186, 187, 188, 188
};

static tegra_dc_bl_output enterprise_bl_output_measured_a03 = {
	0, 1, 2, 3, 4, 5, 6, 7,
	8, 9, 10, 12, 13, 14, 15, 16,
	17, 19, 20, 21, 22, 22, 23, 24,
	25, 26, 27, 28, 29, 29, 30, 32,
	33, 34, 35, 36, 38, 39, 40, 42,
	43, 44, 46, 47, 49, 50, 51, 52,
	53, 54, 55, 56, 57, 58, 59, 60,
	61, 63, 64, 66, 67, 69, 70, 71,
	72, 73, 74, 75, 76, 77, 78, 79,
	80, 81, 82, 83, 84, 84, 85, 86,
	87, 88, 89, 90, 91, 92, 93, 94,
	95, 96, 97, 98, 99, 100, 101, 102,
	103, 104, 105, 106, 107, 108, 109, 110,
	110, 111, 112, 113, 113, 114, 115, 116,
	116, 117, 118, 118, 119, 120, 121, 122,
	123, 124, 125, 126, 127, 128, 129, 130,
	130, 131, 132, 133, 134, 135, 136, 137,
	138, 139, 140, 141, 142, 143, 144, 145,
	146, 147, 148, 149, 150, 151, 152, 153,
	154, 155, 156, 157, 158, 159, 160, 160,
	161, 162, 163, 163, 164, 165, 165, 166,
	167, 168, 168, 169, 170, 171, 172, 173,
	174, 175, 176, 176, 177, 178, 179, 180,
	181, 182, 183, 184, 185, 186, 187, 188,
	189, 190, 191, 191, 192, 193, 194, 194,
	195, 196, 197, 197, 198, 199, 199, 200,
	202, 203, 205, 206, 208, 209, 211, 212,
	213, 215, 216, 218, 219, 220, 221, 222,
	223, 224, 225, 226, 227, 228, 229, 230,
	231, 232, 233, 234, 235, 236, 237, 238,
	239, 240, 241, 243, 244, 245, 247, 248,
	250, 251, 251, 252, 253, 254, 254, 255,
};

static p_tegra_dc_bl_output bl_output;

static void enterprise_backlight_init(void)
{
	gpio_request(enterprise_lcd_bl_pwm, "bl_pwm");
	
	tegra_gpio_disable(enterprise_lcd_bl_pwm);
}

static int brightness_old = 1;
static int enterprise_backlight_notify(struct device *unused, int brightness)
{
         static int count = 0;
	int cur_sd_brightness = atomic_read(&sd_brightness);

	/* SD brightness is a percentage, 8-bit value. */
	brightness = (brightness * cur_sd_brightness) / 255;
	
	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else
		brightness = (bl_output[brightness]*4)/5;//reduce bl 20% brightness

         if (brightness != 0) 
         {
		if (!count)
		{
			printk("%s: brightness = %d\n", __func__, brightness);
			count = 1;
		}
         }
         else
         {
		count = 0;
		gpio_set_value(enterprise_lcd_bl_en, 0);
         }
         
	return brightness;
}

static void enterprise_backlight_notify_after(struct device *unused, int brightness)
{
	if ((brightness_old == 0) && (brightness != 0))
	{
		msleep(15);
		gpio_set_value(enterprise_lcd_bl_en, 1);
	} 

	brightness_old = brightness;
}

static int enterprise_disp1_check_fb(struct device *dev, struct fb_info *info);

#if IS_EXTERNAL_PWM
static struct platform_pwm_backlight_data enterprise_disp1_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= DEFAULT_BRIGHTNESS,
	.pwm_period_ns	= 50000, //20KHz
	.notify		= enterprise_backlight_notify,
	.notify_after	= enterprise_backlight_notify_after,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb	= enterprise_disp1_check_fb,
};
#else
/*
 * In case which_pwm is TEGRA_PWM_PM0,
 * gpio_conf_to_sfio should be TEGRA_GPIO_PW0: set LCD_CS1_N pin to SFIO
 * In case which_pwm is TEGRA_PWM_PM1,
 * gpio_conf_to_sfio should be TEGRA_GPIO_PW1: set LCD_M1 pin to SFIO
 */
static struct platform_tegra_pwm_backlight_data enterprise_disp1_backlight_data = {
	.which_dc		= 0,
	.which_pwm		= TEGRA_PWM_PM1,
	.gpio_conf_to_sfio	= TEGRA_GPIO_PW1,
	.max_brightness		= 255,
	.dft_brightness		= 224,
	.notify		= enterprise_backlight_notify,
	.period			= 0xFF,
	.clk_div		= 0x3FF,
	.clk_select		= 0,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb	= enterprise_disp1_check_fb,
};
#endif


static struct platform_device enterprise_disp1_backlight_device = {
#if IS_EXTERNAL_PWM
	.name	= "pwm-backlight",
#else
	.name	= "tegra-pwm-bl",
#endif
	.id	= -1,
	.dev	= {
		.platform_data = &enterprise_disp1_backlight_data,
	},
};

#ifdef CONFIG_TEGRA_DC
static int enterprise_hdmi_vddio_enable(void)
{
	int ret;
	if (!enterprise_hdmi_vddio) {
		enterprise_hdmi_vddio = regulator_get(NULL, "hdmi_5v0");
		if (IS_ERR_OR_NULL(enterprise_hdmi_vddio)) {
			ret = PTR_ERR(enterprise_hdmi_vddio);
			pr_err("hdmi: couldn't get regulator hdmi_5v0\n");
			enterprise_hdmi_vddio = NULL;
			return ret;
		}
	}
	ret = regulator_enable(enterprise_hdmi_vddio);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator hdmi_5v0\n");
		regulator_put(enterprise_hdmi_vddio);
		enterprise_hdmi_vddio = NULL;
		return ret;
	}
	return ret;
}

static int enterprise_hdmi_vddio_disable(void)
{
	if (enterprise_hdmi_vddio) {
		regulator_disable(enterprise_hdmi_vddio);
		regulator_put(enterprise_hdmi_vddio);
		enterprise_hdmi_vddio = NULL;
	}
	return 0;
}

static int enterprise_hdmi_enable(void)
{
	int ret;
	if (!enterprise_hdmi_reg) {
		enterprise_hdmi_reg = regulator_get(NULL, "avdd_hdmi");
		if (IS_ERR_OR_NULL(enterprise_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			enterprise_hdmi_reg = NULL;
			return PTR_ERR(enterprise_hdmi_reg);
		}
	}
	ret = regulator_enable(enterprise_hdmi_reg);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi\n");
		return ret;
	}
	if (!enterprise_hdmi_pll) {
		enterprise_hdmi_pll = regulator_get(NULL, "avdd_hdmi_pll");
		if (IS_ERR_OR_NULL(enterprise_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			enterprise_hdmi_pll = NULL;
			regulator_put(enterprise_hdmi_reg);
			enterprise_hdmi_reg = NULL;
			return PTR_ERR(enterprise_hdmi_pll);
		}
	}
	ret = regulator_enable(enterprise_hdmi_pll);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi_pll\n");
		return ret;
	}
	return 0;
}

static int enterprise_hdmi_disable(void)
{

	regulator_disable(enterprise_hdmi_reg);
	regulator_put(enterprise_hdmi_reg);
	enterprise_hdmi_reg = NULL;

	regulator_disable(enterprise_hdmi_pll);
	regulator_put(enterprise_hdmi_pll);
	enterprise_hdmi_pll = NULL;

	return 0;
}

static struct resource enterprise_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0,	/* Filled in by enterprise_panel_init() */
		.end	= 0,	/* Filled in by enterprise_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "dsi_regs",
		.start	= TEGRA_DSI_BASE,
		.end	= TEGRA_DSI_BASE + TEGRA_DSI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource enterprise_disp2_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
		.start	= 0,
		.end	= 0,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_dc_sd_settings enterprise_sd_settings = {
	.enable = 1, /* Normal mode operation */
	.use_auto_pwm = false,
	.hw_update_delay = 0,
	.bin_width = -1,
	.aggressiveness = 1,
	.phase_in_adjustments = true,
	.use_vid_luma = false,
	/* Default video coefficients */
	.coeff = {5, 9, 2},
	.fc = {0, 0},
	/* Immediate backlight changes */
	.blp = {1024, 255},
	/* Gammas: R: 2.2 G: 2.2 B: 2.2 */
	/* Default BL TF */
	.bltf = {
			{
				{57, 65, 74, 83},
				{93, 103, 114, 126},
				{138, 151, 165, 179},
				{194, 209, 225, 242},
			},
			{
				{58, 66, 75, 84},
				{94, 105, 116, 127},
				{140, 153, 166, 181},
				{196, 211, 227, 244},
			},
			{
				{60, 68, 77, 87},
				{97, 107, 119, 130},
				{143, 156, 170, 184},
				{199, 215, 231, 248},
			},
			{
				{64, 73, 82, 91},
				{102, 113, 124, 137},
				{149, 163, 177, 192},
				{207, 223, 240, 255},
			},
		},
	/* Default LUT */
	.lut = {
			{
				{250, 250, 250},
				{194, 194, 194},
				{149, 149, 149},
				{113, 113, 113},
				{82, 82, 82},
				{56, 56, 56},
				{34, 34, 34},
				{15, 15, 15},
				{0, 0, 0},
			},
			{
				{246, 246, 246},
				{191, 191, 191},
				{147, 147, 147},
				{111, 111, 111},
				{80, 80, 80},
				{55, 55, 55},
				{33, 33, 33},
				{14, 14, 14},
				{0, 0, 0},
			},
			{
				{239, 239, 239},
				{185, 185, 185},
				{142, 142, 142},
				{107, 107, 107},
				{77, 77, 77},
				{52, 52, 52},
				{30, 30, 30},
				{12, 12, 12},
				{0, 0, 0},
			},
			{
				{224, 224, 224},
				{173, 173, 173},
				{133, 133, 133},
				{99, 99, 99},
				{70, 70, 70},
				{46, 46, 46},
				{25, 25, 25},
				{7, 7, 7},
				{0, 0, 0},
			},
		},
	.sd_brightness = &sd_brightness,
	.bl_device = &enterprise_disp1_backlight_device,
};

static struct tegra_fb_data enterprise_hdmi_fb_data = {
	.win		= 0,
	.xres		= 800,
	.yres		= 1280,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_out enterprise_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,
	.parent_clk	= "pll_d2_out0",

	.dcc_bus	= 3,
	.hotplug_gpio	= enterprise_hdmi_hpd,

	.max_pixclock	= KHZ2PICOS(148500),

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.enable		= enterprise_hdmi_enable,
	.disable	= enterprise_hdmi_disable,
	.postsuspend	= enterprise_hdmi_vddio_disable,
	.hotplug_init	= enterprise_hdmi_vddio_enable,
};

static struct tegra_dc_platform_data enterprise_disp2_pdata = {
	.flags		= 0,
	.default_out	= &enterprise_disp2_out,
	.fb		= &enterprise_hdmi_fb_data,
	.emc_clk_rate	= 300000000,
};

static int enterprise_panel_enable(void)
{

	gpio_direction_output(enterprise_en_lcd_3v3, 1);
	msleep(20);
	gpio_direction_output(enterprise_en_lcd_1v8, 1);
	gpio_direction_output(enterprise_lvds_shtdn_n, 1);

	if(enterprise_disp1_pdata.min_emc_clk_rate)  {
		clk_enable(disp1_emc_min_clk);
		clk_set_rate(disp1_emc_min_clk,enterprise_disp1_pdata.min_emc_clk_rate);
	}

	return 0;
}

static int enterprise_panel_disable(void)
{
	int ret = 0;
	
	printk("%s\n", __func__);

	gpio_direction_output(enterprise_lvds_shtdn_n, 0);
	gpio_direction_output(enterprise_en_lcd_1v8, 0);
	msleep(20);
	gpio_direction_output(enterprise_en_lcd_3v3, 0);

	if(enterprise_disp1_pdata.min_emc_clk_rate)  {
		clk_set_rate(disp1_emc_min_clk, 0);
		clk_disable(disp1_emc_min_clk);
	}
	return ret;
}

/* heqi add */
static void enterprise_panel_bl_shutdown(void)
{
	printk("%s: system is shutting down, firstly shut down backlight\n", __func__);
	
	gpio_direction_output(enterprise_lcd_bl_en, 0);
	mdelay(5);
	tegra_gpio_enable(enterprise_lcd_bl_pwm);
	gpio_direction_output(enterprise_lcd_bl_pwm, 0);
	mdelay(15);
	gpio_direction_output(enterprise_en_vdd_bl, 0);

	mdelay(300);
}
#endif

#ifdef CONFIG_TEGRA_DC
static struct tegra_dc_mode enterprise_panel_modes[] = {
	{
		.pclk = 66770000,
		.h_ref_to_sync = 1,
		.v_ref_to_sync = 1,
		.h_sync_width = 30,
		.v_sync_width = 2,
		.h_back_porch = 30,
		.v_back_porch = 2,
		.h_active = 800,
		.v_active = 1280,
		.h_front_porch = 4,
		.v_front_porch = 4,
	},
};

static struct tegra_fb_data enterprise_fb_data = {
	.win		= 0,
	.xres		= 800,
	.yres		= 1280,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_out enterprise_disp1_out = {
	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	.sd_settings	= &enterprise_sd_settings,
	.parent_clk	= "pll_p",

	.parent_clk_backup = "pll_d2_out0",

	.type		= TEGRA_DC_OUT_RGB,
	.depth		= 24,
	.dither		= TEGRA_DC_ORDERED_DITHER,

	.modes		= enterprise_panel_modes,
	.n_modes	= ARRAY_SIZE(enterprise_panel_modes),

	.enable		= enterprise_panel_enable,
	.disable	= enterprise_panel_disable,
	.blshutdown = enterprise_panel_bl_shutdown,

	.height	= 151,
	.width	= 94,
};
static struct tegra_dc_platform_data enterprise_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &enterprise_disp1_out,
	.emc_clk_rate	= 204000000,
	.min_emc_clk_rate = 204000000,
	.fb		= &enterprise_fb_data,
};

static struct nvhost_device enterprise_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= enterprise_disp1_resources,
	.num_resources	= ARRAY_SIZE(enterprise_disp1_resources),
	.dev = {
		.platform_data = &enterprise_disp1_pdata,
	},
};

static int enterprise_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &enterprise_disp1_device.dev;
}

static struct nvhost_device enterprise_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= enterprise_disp2_resources,
	.num_resources	= ARRAY_SIZE(enterprise_disp2_resources),
	.dev = {
		.platform_data = &enterprise_disp2_pdata,
	},
};
#endif

#if defined(CONFIG_TEGRA_NVMAP)
static struct nvmap_platform_carveout enterprise_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0,	/* Filled in by enterprise_panel_init() */
		.size		= 0,	/* Filled in by enterprise_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data enterprise_nvmap_data = {
	.carveouts	= enterprise_carveouts,
	.nr_carveouts	= ARRAY_SIZE(enterprise_carveouts),
};

static struct platform_device enterprise_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &enterprise_nvmap_data,
	},
};
#endif

static struct platform_device *enterprise_gfx_devices[] __initdata = {
#if defined(CONFIG_TEGRA_NVMAP)
	&enterprise_nvmap_device,
#endif
#if IS_EXTERNAL_PWM
	&tegra_pwfm0_device,
#else
	&tegra_pwfm0_device,
#endif
};

static struct platform_device *enterprise_bl_devices[]  = {
	&enterprise_disp1_backlight_device,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend enterprise_panel_early_suspender;

static void enterprise_panel_early_suspend(struct early_suspend *h)
{
	/* power down LCD, add use a black screen for HDMI */
	if (num_registered_fb > 0)
		fb_blank(registered_fb[0], FB_BLANK_POWERDOWN);
	if (num_registered_fb > 1)
		fb_blank(registered_fb[1], FB_BLANK_NORMAL);

#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_store_default_gov();
	cpufreq_change_gov(cpufreq_conservative_gov);
#endif
}

static void enterprise_panel_late_resume(struct early_suspend *h)
{
	unsigned i;

        for (i = 0; i < num_registered_fb; i++)
		fb_blank(registered_fb[i], FB_BLANK_UNBLANK);

#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_restore_default_gov();
#endif
	
}
#endif

int __init enterprise_panel_init(void)
{
	int err;
	struct resource __maybe_unused *res;
	struct board_info board_info;

	tegra_get_board_info(&board_info);

	BUILD_BUG_ON(ARRAY_SIZE(enterprise_bl_output_measured_a03) != 256);
	BUILD_BUG_ON(ARRAY_SIZE(enterprise_bl_output_measured_a02) != 256);

	bl_output = enterprise_bl_output_measured_a03;

#if defined(CONFIG_TEGRA_NVMAP)
	enterprise_carveouts[1].base = tegra_carveout_start;
	enterprise_carveouts[1].size = tegra_carveout_size;
#endif

	gpio_direction_input(enterprise_hdmi_hpd);

	enterprise_backlight_init();

#ifdef CONFIG_HAS_EARLYSUSPEND
	enterprise_panel_early_suspender.suspend = enterprise_panel_early_suspend;
	enterprise_panel_early_suspender.resume = enterprise_panel_late_resume;
	enterprise_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&enterprise_panel_early_suspender);
#endif

#ifdef CONFIG_TEGRA_GRHOST
	err = tegra3_register_host1x_devices();
	if (err)
		return err;
#endif

	err = platform_add_devices(enterprise_gfx_devices,
				ARRAY_SIZE(enterprise_gfx_devices));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&enterprise_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;
#endif

	/* Copy the bootloader fb to the fb. */
	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,
		min(tegra_fb_size, tegra_bootloader_fb_size));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	if (!err)
		err = nvhost_device_register(&enterprise_disp1_device);

	disp1_emc_min_clk = clk_get(&enterprise_disp1_device.dev, "emc_min");
	if (IS_ERR(disp1_emc_min_clk)) {
		dev_dbg(&enterprise_disp1_device.dev, "no peripheral clock\n");
		clk_put(disp1_emc_min_clk);
	}

	res = nvhost_get_resource_byname(&enterprise_disp2_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;
	if (!err)
		err = nvhost_device_register(&enterprise_disp2_device);
#endif

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_NVAVP)
	if (!err)
		err = nvhost_device_register(&nvavp_device);
#endif

	if (!err)
		err = platform_add_devices(enterprise_bl_devices,
				ARRAY_SIZE(enterprise_bl_devices));
	return err;
}

