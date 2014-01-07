/*
 * tegra_aic325x.c - Tegra machine ASoC driver for boards using TI 3262 codec.
 *
 * Author: Vinod G. <vinodg@nvidia.com>
 * Copyright (C) 2011 - NVIDIA, Inc.
 *
 * Based on code copyright/by:
 *
 * (c) 2010, 2011 Nvidia Graphics Pvt. Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <asm/mach-types.h>

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif

#include <mach/tegra_asoc_pdata.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/delay.h>

#include <linux/mfd/tlv320aic3256-registers.h>
#include <linux/mfd/tlv320aic3xxx-core.h>
#include "../codecs/tlv320aic325x.h"

#include "tegra_pcm.h"
#include "tegra_asoc_utils.h"

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
#include "tegra20_das.h"
#else
#include "tegra30_ahub.h"
#include "tegra30_i2s.h"
#include "tegra30_dam.h"
#endif
#include "../../../arch/arm/mach-tegra/board-enterprise.h"
#include "../../../arch/arm/mach-tegra/gpio-names.h"

#define DRV_NAME "tegra-snd-aic325x"

#define GPIO_SPKR_EN    BIT(0)
#define GPIO_HP_MUTE    BIT(1)
#define GPIO_INT_MIC_EN BIT(2)
#define GPIO_EXT_MIC_EN BIT(3)

#define DAI_LINK_HIFI		0
#define DAI_LINK_SPDIF		1
#define DAI_LINK_BTSCO		2
#define DAI_LINK_VOICE_CALL	3
#define DAI_LINK_BT_VOICE_CALL	4
#define NUM_DAI_LINKS	5

static int adc_eq_status = 2;
int eq_status = 2;
static struct snd_soc_codec * eq_codec;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
const char *tegra_i2s_dai_name[TEGRA30_NR_I2S_IFC] = {
	"tegra30-i2s.0",
	"tegra30-i2s.1",
	"tegra30-i2s.2",
	"tegra30-i2s.3",
	"tegra30-i2s.4",
};
#endif

static int spk_ctrl = 0;
struct tegra_aic325x {
	struct tegra_asoc_utils_data util_data;
	struct tegra_asoc_platform_data *pdata;
	struct regulator *audio_reg;
	int gpio_requested;
	bool init_done;
	int is_call_mode;
	int is_device_bt;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	struct codec_config codec_info[NUM_I2S_DEVICES];
	struct snd_soc_card *pcard;
#endif
};


static int tegra_aic325x_adc_eq_ctrl_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int tegra_aic325x_adc_eq_ctrl_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = adc_eq_status;

	return 0;
}

static int tegra_aic325x_adc_eq_ctrl_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int new_adc_eq_status = ucontrol->value.integer.value[0];
	int adc_status = snd_soc_read(eq_codec, AIC3256_ADC_FLAG);
	
	//printk("\n\nnew adc eq = %d, old adc eq = %d\n", new_adc_eq_status, adc_eq_status);
	if(new_adc_eq_status == adc_eq_status)
		return 1;

	if(new_adc_eq_status == 1)
	{
		//printk("yang set adc eq, adc power = %d\n", (adc_status & 0x04)||(adc_status & 0x40));
		if((adc_status & 0x04)||(adc_status & 0x40))
		{
			//printk("yang adc working and switch to hp mic eq\n");
			snd_soc_write(eq_codec, AIC3256_ADC_ADAPTIVE_CRAM_REG,			0x04);
			/****************************1*****************************/
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_36, 		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_36+1,	0xD2);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_36+2,	0x0C);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_40, 		0x81);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_40+1,	0x2D);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_40+2,	0xF4);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_44, 		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_44+1,	0xD2);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_44+2,	0x0C);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_48, 		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_48+1,	0xD0);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_48+2,	0xA8);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_52, 		0x82);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_52+1,	0x59);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_52+2,	0x1E);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_44, 		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_44+1,	0xD2);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_44+2,	0x0C);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_48, 		0x81);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_48+1,	0x2D);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_48+2,	0xF4);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_52, 		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_52+1,	0xD2);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_52+2,	0x0C);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_56, 		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_56+1,	0xD0);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_56+2,	0xA8);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_60, 		0x82);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_60+1,	0x59);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_60+2,	0x1E);
		/****************************2*****************************/
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_56,		0x7F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_56+1,	0xA8);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_56+2,	0x54);
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_60,		0x9B);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_60+1,	0x61);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_60+2,	0x8B);
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_64,		0x7D);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_64+1,	0xFF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_64+2,	0x52);
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_68,		0x64);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_68+1,	0x9E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_68+2,	0x75);
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_72,		0x82);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_72+1,	0x58);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_72+2,	0x59);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_64, 		0x7F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_64+1,	0xA8);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_64+2,	0x54);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_68, 		0x9B);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_68+1,	0x61);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_68+2,	0x8B);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_72, 		0x7D);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_72+1,	0xFF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_72+2,	0x52);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_76, 		0x64);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_76+1,	0x9E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_76+2,	0x75);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_80, 		0x82);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_80+1,	0x58);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_80+2,	0x59);
		
			/*************************3********************************/
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_76, 		0x7F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_76+1,	0xA8);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_76+2,	0x54);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_80, 		0x9B);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_80+1,	0x61);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_80+2,	0x8B);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_84, 		0x7D);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_84+1,	0xFF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_84+2,	0x52);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_88, 		0x64);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_88+1,	0x9E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_88+2,	0x75);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_92, 		0x82);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_92+1,	0x58);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_92+2,	0x59);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_84, 		0x7F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_84+1,	0xA8);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_84+2,	0x54);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_88, 		0x9B);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_88+1,	0x61);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_88+2,	0x8B);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_92, 		0x7D);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_92+1,	0xFF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_92+2,	0x52);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_96, 		0x64);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_96+1,	0x9E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_96+2,	0x75);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_100, 	0x82);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_100+1,	0x58);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_100+2,	0x59);
		
		
			/*************************4********************************/
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_96, 		0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_96+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_96+2,	0xE5);
	
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_100, 	0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_100+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_100+2,	0xE5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_104, 	0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_104+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_104+2,	0xE5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_108, 	0x13);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_108+1,	0xAE);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_108+2,	0xDB);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_112, 	0xE7);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_112+1,	0xE6);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_112+2,	0xB2);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_104, 	0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_104+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_104+2,	0xE5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_108, 	0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_108+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_108+2,	0xE5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_112, 	0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_112+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_112+2,	0xE5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_116, 	0x13);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_116+1,	0xAE);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_116+2,	0xDB);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_120, 	0xE7);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_120+1,	0xE6);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_120+2,	0xB2);
			/****************************5*****************************/
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_116, 	0x7F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_116+1,	0xA8);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_116+2,	0x54);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_120, 	0xDF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_120+1,	0x2C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_120+2,	0xB6);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_124, 	0x7D);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_124+1,	0xFF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_124+2,	0x52);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_8, 		0x20);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_8+1,		0xD3);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_8+2,		0x4A);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_12, 		0x82);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_12+1,	0x58);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_12+2,	0x59);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_124, 	0x7F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_124+1,	0xA8);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_124+2,	0x54);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_8, 		0xDF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_8+1,	0x2C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_8+2,	0xB6);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_12, 	0x7D);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_12+1,	0xFF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_12+2,	0x52);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_16, 	0x20);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_16+1,	0xD3);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_16+2,	0x4A);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_20, 	0x82);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_20+1,	0x58);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_20+2,	0x59);

			snd_soc_write(eq_codec, AIC3256_ADC_ADAPTIVE_CRAM_REG, 			0x05);
			mdelay(1);

			if(snd_soc_read(eq_codec,  AIC3256_ADC_ADAPTIVE_CRAM_REG)&0x01)
			{
				//printk("adc switch not finish %d\n", snd_soc_read(eq_codec,  AIC3256_ADC_ADAPTIVE_CRAM_REG));
				mdelay(1);
			}//else
			 //  printk("adc switch ok %d\n", snd_soc_read(eq_codec,  AIC3256_ADC_ADAPTIVE_CRAM_REG));
		}

			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_36, 		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_36+1,	0xD2);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_36+2,	0x0C);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_40, 		0x81);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_40+1,	0x2D);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_40+2,	0xF4);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_44, 		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_44+1,	0xD2);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_44+2,	0x0C);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_48, 		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_48+1,	0xD0);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_48+2,	0xA8);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_52, 		0x82);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_52+1,	0x59);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_52+2,	0x1E);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_44, 		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_44+1,	0xD2);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_44+2,	0x0C);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_48, 		0x81);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_48+1,	0x2D);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_48+2,	0xF4);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_52, 		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_52+1,	0xD2);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_52+2,	0x0C);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_56, 		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_56+1,	0xD0);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_56+2,	0xA8);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_60, 		0x82);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_60+1,	0x59);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_60+2,	0x1E);
		/****************************2*****************************/
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_56,		0x7F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_56+1,	0xA8);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_56+2,	0x54);
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_60,		0x9B);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_60+1,	0x61);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_60+2,	0x8B);
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_64,		0x7D);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_64+1,	0xFF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_64+2,	0x52);
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_68,		0x64);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_68+1,	0x9E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_68+2,	0x75);
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_72,		0x82);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_72+1,	0x58);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_72+2,	0x59);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_64, 		0x7F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_64+1,	0xA8);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_64+2,	0x54);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_68, 		0x9B);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_68+1,	0x61);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_68+2,	0x8B);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_72, 		0x7D);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_72+1,	0xFF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_72+2,	0x52);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_76, 		0x64);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_76+1,	0x9E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_76+2,	0x75);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_80, 		0x82);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_80+1,	0x58);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_80+2,	0x59);
		
			/*************************3********************************/
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_76, 		0x7F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_76+1,	0xA8);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_76+2,	0x54);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_80, 		0x9B);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_80+1,	0x61);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_80+2,	0x8B);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_84, 		0x7D);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_84+1,	0xFF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_84+2,	0x52);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_88, 		0x64);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_88+1,	0x9E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_88+2,	0x75);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_92, 		0x82);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_92+1,	0x58);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_92+2,	0x59);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_84, 		0x7F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_84+1,	0xA8);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_84+2,	0x54);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_88, 		0x9B);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_88+1,	0x61);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_88+2,	0x8B);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_92, 		0x7D);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_92+1,	0xFF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_92+2,	0x52);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_96, 		0x64);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_96+1,	0x9E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_96+2,	0x75);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_100, 	0x82);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_100+1,	0x58);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_100+2,	0x59);
		
		
			/*************************4********************************/
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_96, 		0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_96+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_96+2,	0xE5);
	
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_100, 	0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_100+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_100+2,	0xE5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_104, 	0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_104+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_104+2,	0xE5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_108, 	0x13);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_108+1,	0xAE);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_108+2,	0xDB);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_112, 	0xE7);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_112+1,	0xE6);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_112+2,	0xB2);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_104, 	0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_104+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_104+2,	0xE5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_108, 	0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_108+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_108+2,	0xE5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_112, 	0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_112+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_112+2,	0xE5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_116, 	0x13);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_116+1,	0xAE);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_116+2,	0xDB);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_120, 	0xE7);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_120+1,	0xE6);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_120+2,	0xB2);
			/****************************5*****************************/
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_116, 	0x7F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_116+1,	0xA8);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_116+2,	0x54);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_120, 	0xDF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_120+1,	0x2C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_120+2,	0xB6);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_124, 	0x7D);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_124+1,	0xFF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_124+2,	0x52);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_8, 		0x20);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_8+1,		0xD3);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_8+2,		0x4A);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_12, 		0x82);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_12+1,	0x58);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_12+2,	0x59);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_124, 	0x7F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_124+1,	0xA8);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_124+2,	0x54);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_8, 		0xDF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_8+1,	0x2C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_8+2,	0xB6);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_12, 	0x7D);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_12+1,	0xFF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_12+2,	0x52);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_16, 	0x20);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_16+1,	0xD3);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_16+2,	0x4A);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_20, 	0x82);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_20+1,	0x58);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_20+2,	0x59);

		

	}
	else
	{
		//printk("yang remove adc eq\n");
		
		//printk("yang set adc eq, adc power = %d\n", (adc_status & 0x04)||(adc_status & 0x40));
		if((adc_status & 0x04)||(adc_status & 0x40))
		{
			//printk("yang adc working and switch to built-in mic eq\n");
			snd_soc_write(eq_codec, AIC3256_ADC_ADAPTIVE_CRAM_REG,			0x04);

			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_36,		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_36+1,	0xD2);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_36+2,	0x0C);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_40,		0x81);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_40+1,	0x2D);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_40+2,	0xF4);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_44,		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_44+1,	0xD2);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_44+2,	0x0C);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_48,		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_48+1,	0xD0);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_48+2,	0xA8);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_52,		0x82);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_52+1,	0x59);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_52+2,	0x1E);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_44,		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_44+1,	0xD2);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_44+2,	0x0C);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_48,		0x81);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_48+1,	0x2D);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_48+2,	0xF4);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_52,		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_52+1,	0xD2);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_52+2,	0x0C);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_56,		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_56+1,	0xD0);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_56+2,	0xA8);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_60,		0x82);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_60+1,	0x59);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_60+2,	0x1E);
		/****************************2*****************************/
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_56,		0x6C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_56+1,	0x0E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_56+2,	0x35);
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_60,		0xCF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_60+1,	0xDF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_60+2,	0x8F);
	
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_64,		0x22);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_64+1,	0x17);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_64+2,	0x8E);
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_68,		0x3F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_68+1,	0x8F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_68+2,	0x09);
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_72,		0xD2);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_72+1,	0xFD);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_72+2,	0x09);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_64,		0x6C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_64+1,	0x0E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_64+2,	0x35);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_68,		0xCF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_68+1,	0xDF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_68+2,	0x8F);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_72,		0x22);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_72+1,	0x17);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_72+2,	0x8E);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_76,		0x3F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_76+1,	0x8F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_76+2,	0x09);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_80,		0xD2);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_80+1,	0xFD);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_80+2,	0x09);
		
			/*************************3********************************/
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_76,		0x6B);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_76+1,	0x8A);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_76+2,	0xCA);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_80,		0xCB);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_80+1,	0x7F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_80+2,	0x4E);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_84,		0x25);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_84+1,	0x26);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_84+2,	0x98);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_88,		0x44);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_88+1,	0xAA);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_88+2,	0x81);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_92,		0xCE);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_92+1,	0xFA);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_92+2,	0xFF);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_84,		0x6B);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_84+1,	0x8A);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_84+2,	0xCA);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_88,		0xCB);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_88+1,	0x7F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_88+2,	0x4E);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_92,		0x25);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_92+1,	0x26);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_92+2,	0x98);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_96,		0x44);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_96+1,	0xAA);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_96+2,	0x81);
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_100, 	0xCE);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_100+1,	0xFA);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_100+2,	0xFF);
		
		
			/*************************4********************************/
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_96,		0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_96+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_96+2,	0xE5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_100, 	0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_100+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_100+2,	0xE5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_104, 	0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_104+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_104+2,	0xE5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_108, 	0x13);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_108+1,	0xAE);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_108+2,	0xDB);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_112, 	0xE7);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_112+1,	0xE6);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_112+2,	0xB2);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_104, 	0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_104+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_104+2,	0xE5);

			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_108, 	0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_108+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_108+2,	0xE5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_112, 	0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_112+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_112+2,	0xE5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_116, 	0x13);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_116+1,	0xAE);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_116+2,	0xDB);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_120, 	0xE7);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_120+1,	0xE6);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_120+2,	0xB2);
			/****************************5*****************************/
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_116, 	0x73);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_116+1,	0x1F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_116+2,	0x8B);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_120, 	0xB8);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_120+1,	0xC3);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_120+2,	0x5B);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_124, 	0x64);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_124+1,	0xF5);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_124+2,	0xDA);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_8,		0x47);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_8+1, 	0x3C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_8+2, 	0xA5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_12,		0xA7);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_12+1,	0xEA);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_12+2,	0x99);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_124, 	0x73);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_124+1,	0x1F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_124+2,	0x8B);
	
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_8,		0xB8);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_8+1,	0xC3);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_8+2,	0x5B);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_12, 	0x64);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_12+1,	0xF5);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_12+2,	0xDA);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_16, 	0x47);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_16+1,	0x3C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_16+2,	0xA5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_20, 	0xA7);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_20+1,	0xEA);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_20+2,	0x99);
			

			snd_soc_write(eq_codec, AIC3256_ADC_ADAPTIVE_CRAM_REG, 			0x05);
			mdelay(1);

			if(snd_soc_read(eq_codec,  AIC3256_ADC_ADAPTIVE_CRAM_REG)&0x01)
			{
				//printk("adc switch not finish %d\n", snd_soc_read(eq_codec,  AIC3256_ADC_ADAPTIVE_CRAM_REG));
				mdelay(1);
			}//else
			 //	printk("adc sswitch ok %d\n", snd_soc_read(eq_codec,  AIC3256_ADC_ADAPTIVE_CRAM_REG));

		}
		    snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_36,		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_36+1,	0xD2);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_36+2,	0x0C);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_40,		0x81);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_40+1,	0x2D);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_40+2,	0xF4);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_44,		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_44+1,	0xD2);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_44+2,	0x0C);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_48,		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_48+1,	0xD0);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_48+2,	0xA8);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_52,		0x82);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_52+1,	0x59);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_52+2,	0x1E);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_44,		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_44+1,	0xD2);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_44+2,	0x0C);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_48,		0x81);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_48+1,	0x2D);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_48+2,	0xF4);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_52,		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_52+1,	0xD2);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_52+2,	0x0C);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_56,		0x7E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_56+1,	0xD0);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_56+2,	0xA8);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_60,		0x82);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_60+1,	0x59);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_60+2,	0x1E);
		/****************************2*****************************/
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_56,		0x6C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_56+1,	0x0E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_56+2,	0x35);
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_60,		0xCF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_60+1,	0xDF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_60+2,	0x8F);
	
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_64,		0x22);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_64+1,	0x17);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_64+2,	0x8E);
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_68,		0x3F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_68+1,	0x8F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_68+2,	0x09);
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_72,		0xD2);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_72+1,	0xFD);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_72+2,	0x09);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_64,		0x6C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_64+1,	0x0E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_64+2,	0x35);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_68,		0xCF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_68+1,	0xDF);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_68+2,	0x8F);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_72,		0x22);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_72+1,	0x17);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_72+2,	0x8E);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_76,		0x3F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_76+1,	0x8F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_76+2,	0x09);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_80,		0xD2);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_80+1,	0xFD);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_80+2,	0x09);
		
			/*************************3********************************/
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_76,		0x6B);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_76+1,	0x8A);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_76+2,	0xCA);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_80,		0xCB);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_80+1,	0x7F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_80+2,	0x4E);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_84,		0x25);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_84+1,	0x26);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_84+2,	0x98);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_88,		0x44);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_88+1,	0xAA);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_88+2,	0x81);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_92,		0xCE);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_92+1,	0xFA);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_92+2,	0xFF);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_84,		0x6B);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_84+1,	0x8A);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_84+2,	0xCA);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_88,		0xCB);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_88+1,	0x7F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_88+2,	0x4E);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_92,		0x25);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_92+1,	0x26);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_92+2,	0x98);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_96,		0x44);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_96+1,	0xAA);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_96+2,	0x81);
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_100, 	0xCE);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_100+1,	0xFA);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_100+2,	0xFF);
		
		
			/*************************4********************************/
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_96,		0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_96+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_96+2,	0xE5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_100, 	0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_100+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_100+2,	0xE5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_104, 	0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_104+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_104+2,	0xE5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_108, 	0x13);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_108+1,	0xAE);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_108+2,	0xDB);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_112, 	0xE7);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_112+1,	0xE6);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_112+2,	0xB2);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_104, 	0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_104+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_104+2,	0xE5);

			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_108, 	0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_108+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_108+2,	0xE5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_112, 	0x1C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_112+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_112+2,	0xE5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_116, 	0x13);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_116+1,	0xAE);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_116+2,	0xDB);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_120, 	0xE7);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_120+1,	0xE6);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_120+2,	0xB2);
			/****************************5*****************************/
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_116, 	0x73);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_116+1,	0x1F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_116+2,	0x8B);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_120, 	0xB8);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_120+1,	0xC3);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_120+2,	0x5B);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_124, 	0x64);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_124+1,	0xF5);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK8_REG_124+2,	0xDA);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_8,		0x47);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_8+1, 	0x3C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_8+2, 	0xA5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_12,		0xA7);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_12+1,	0xEA);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_12+2,	0x99);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_124, 	0x73);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_124+1,	0x1F);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK9_REG_124+2,	0x8B);
	
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_8,		0xB8);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_8+1,	0xC3);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_8+2,	0x5B);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_12, 	0x64);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_12+1,	0xF5);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_12+2,	0xDA);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_16, 	0x47);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_16+1,	0x3C);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_16+2,	0xA5);
		
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_20, 	0xA7);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_20+1,	0xEA);
			snd_soc_write(eq_codec, AIC3252_ADC_ADAPTIVE_BANK10_REG_20+2,	0x99);


		
	}

	adc_eq_status = new_adc_eq_status;
	
	return 1;
}

struct snd_kcontrol_new tegra_aic325x_adc_eq_ctrl_control = {
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "ADC Eq Ctrl Switch",
	.private_value = 0xffff,
	.info = tegra_aic325x_adc_eq_ctrl_info,
	.get = tegra_aic325x_adc_eq_ctrl_get,
	.put = tegra_aic325x_adc_eq_ctrl_put
};




static int tegra_aic325x_eq_ctrl_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int tegra_aic325x_eq_ctrl_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = eq_status;

	return 0;
}

static int tegra_aic325x_eq_ctrl_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int new_eq_status = ucontrol->value.integer.value[0];
	//printk("new = %d, eq = %d\n", new_eq_status, eq_status);
	if(new_eq_status == eq_status)
		return 1;

	if(new_eq_status == 1)
	{
		//printk("yang set eq %d\n", snd_soc_read(eq_codec, AIC3256_DAC_FLAG_1)&0x80);
		if(snd_soc_read(eq_codec, AIC3256_DAC_FLAG_1)&0x80)
		{
			//printk("yang dac working and switch to spk eq\n");
			snd_soc_write(eq_codec, AIC3256_DAC_ADAPTIVE_CRAM_REG,			0x04);
			/****************************1*****************************/
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_12, 	0x6F);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_12+1,	0xFC);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_12+2,	0xC4);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_16, 	0x90);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_16+1,	0x03);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_16+2,	0x3C);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_20, 	0x6F);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_20+1,	0xFC);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_20+2,	0xC4);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_24, 	0x7D);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_24+1,	0xA1);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_24+2,	0x5F);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_28, 	0x84);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_28+1,	0xA7);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_28+2,	0x34);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_20, 	0x6F);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_20+1,	0xFC);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_20+2,	0xC4);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_24, 	0x90);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_24+1,	0x03);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_24+2,	0x3C);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_28, 	0x6F);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_28+1,	0xFC);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_28+2,	0xC4);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_32, 	0x7D);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_32+1,	0xA1);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_32+2,	0x5F);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_36, 	0x84);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_36+1,	0xA7);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_36+2,	0x34);
		/****************************2*****************************/
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_32, 	0x7F);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_32+1,	0x28);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_32+2,	0xE9);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_36, 	0x84);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_36+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_36+2,	0x39);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_40, 	0x78);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_40+1,	0xAB);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_40+2,	0x8F);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_44, 	0x7B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_44+1,	0xD1);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_44+2,	0xC7);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_48, 	0x88);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_48+1,	0x2B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_48+2,	0x86);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_40, 	0x7F);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_40+1,	0x28);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_40+2,	0xE9);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_44, 	0x84);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_44+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_44+2,	0x39);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_48, 	0x78);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_48+1,	0xAB);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_48+2,	0x8F);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_52, 	0x7B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_52+1,	0xD1);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_52+2,	0xC7);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_56, 	0x88);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_56+1,	0x2B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_56+2,	0x86);
		
			/*************************3********************************/
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_52, 	0x7F);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_52+1,	0x28);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_52+2,	0xE9);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_56, 	0x84);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_56+1,	0x4C);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_56+2,	0xC9);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_60, 	0x78);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_60+1,	0xAB);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_60+2,	0x8F);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_64, 	0x7B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_64+1,	0xB3);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_64+2,	0x37);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_68, 	0x88);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_68+1,	0x2B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_68+2,	0x86);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_60, 	0x7F);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_60+1,	0x28);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_60+2,	0xE9);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_64, 	0x84);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_64+1,	0x4C);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_64+2,	0xC9);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_68, 	0x78);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_68+1,	0xAB);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_68+2,	0x8F);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_72, 	0x7B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_72+1,	0xB3);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_72+2,	0x37);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_76, 	0x88);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_76+1,	0x2B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_76+2,	0x86);
		
		
			/*************************4********************************/
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_72, 	0x7B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_72+1,	0xB4);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_72+2,	0x90);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_76, 	0xB7);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_76+1,	0x8E);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_76+2,	0x06);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_80, 	0x66);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_80+1,	0xE2);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_80+2,	0x77);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_84, 	0x48);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_84+1,	0x71);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_84+2,	0xFA);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_88, 	0x9D);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_88+1,	0x68);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_88+2,	0xF8);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_80, 	0x7B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_80+1,	0xB4);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_80+2,	0x90);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_84, 	0xB7);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_84+1,	0x8E);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_84+2,	0x06);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_88, 	0x66);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_88+1,	0xE2);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_88+2,	0x77);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_92, 	0x48);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_92+1,	0x71);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_92+2,	0xFA);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_96, 	0x9D);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_96+1,	0x68);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_96+2,	0xF8);
			/****************************5*****************************/
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_92, 	0x7B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_92+1,	0x2A);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_92+2,	0xDD);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_96, 	0xC5);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_96+1,	0xC4);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_96+2,	0x0F);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_100,	0x63);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_100+1,	0xBD);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_100+2,	0x30);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_104,	0x3A);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_104+1,	0x3B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_104+2,	0xF1);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_108,	0xA1);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_108+1,	0x17);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_108+2,	0xF2);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_100,	0x7B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_100+1,	0x2A);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_100+2,	0xDD);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_104,	0xC5);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_104+1,	0xC4);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_104+2,	0x0F);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_108,	0x63);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_108+1,	0xBD);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_108+2,	0x30);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_112,	0x3A);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_112+1,	0x3B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_112+2,	0xF1);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_116,	0xA1);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_116+1,	0x17);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_116+2,	0xF2);

			snd_soc_write(eq_codec, AIC3256_DAC_ADAPTIVE_CRAM_REG, 			0x05);
			mdelay(1);

			if(snd_soc_read(eq_codec,  AIC3256_DAC_ADAPTIVE_CRAM_REG)&0x01)
			{
				//printk("switch not finish %d\n", snd_soc_read(eq_codec,  AIC3256_DAC_ADAPTIVE_CRAM_REG));
				mdelay(1);
			}
		}

			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_12, 	0x6F);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_12+1,	0xFC);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_12+2,	0xC4);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_16, 	0x90);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_16+1,	0x03);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_16+2,	0x3C);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_20, 	0x6F);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_20+1,	0xFC);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_20+2,	0xC4);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_24, 	0x7D);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_24+1,	0xA1);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_24+2,	0x5F);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_28, 	0x84);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_28+1,	0xA7);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_28+2,	0x34);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_20, 	0x6F);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_20+1,	0xFC);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_20+2,	0xC4);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_24, 	0x90);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_24+1,	0x03);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_24+2,	0x3C);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_28, 	0x6F);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_28+1,	0xFC);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_28+2,	0xC4);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_32, 	0x7D);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_32+1,	0xA1);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_32+2,	0x5F);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_36, 	0x84);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_36+1,	0xA7);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_36+2,	0x34);
		/****************************2*****************************/
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_32, 	0x7F);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_32+1,	0x28);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_32+2,	0xE9);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_36, 	0x84);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_36+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_36+2,	0x39);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_40, 	0x78);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_40+1,	0xAB);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_40+2,	0x8F);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_44, 	0x7B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_44+1,	0xD1);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_44+2,	0xC7);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_48, 	0x88);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_48+1,	0x2B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_48+2,	0x86);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_40, 	0x7F);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_40+1,	0x28);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_40+2,	0xE9);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_44, 	0x84);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_44+1,	0x2E);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_44+2,	0x39);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_48, 	0x78);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_48+1,	0xAB);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_48+2,	0x8F);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_52, 	0x7B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_52+1,	0xD1);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_52+2,	0xC7);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_56, 	0x88);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_56+1,	0x2B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_56+2,	0x86);
		
			/*************************3********************************/
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_52, 	0x7F);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_52+1,	0x28);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_52+2,	0xE9);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_56, 	0x84);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_56+1,	0x4C);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_56+2,	0xC9);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_60, 	0x78);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_60+1,	0xAB);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_60+2,	0x8F);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_64, 	0x7B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_64+1,	0xB3);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_64+2,	0x37);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_68, 	0x88);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_68+1,	0x2B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_68+2,	0x86);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_60, 	0x7F);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_60+1,	0x28);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_60+2,	0xE9);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_64, 	0x84);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_64+1,	0x4C);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_64+2,	0xC9);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_68, 	0x78);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_68+1,	0xAB);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_68+2,	0x8F);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_72, 	0x7B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_72+1,	0xB3);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_72+2,	0x37);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_76, 	0x88);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_76+1,	0x2B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_76+2,	0x86);
		
		
			/*************************4********************************/
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_72, 	0x7B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_72+1,	0xB4);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_72+2,	0x90);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_76, 	0xB7);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_76+1,	0x8E);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_76+2,	0x06);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_80, 	0x66);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_80+1,	0xE2);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_80+2,	0x77);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_84, 	0x48);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_84+1,	0x71);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_84+2,	0xFA);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_88, 	0x9D);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_88+1,	0x68);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_88+2,	0xF8);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_80, 	0x7B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_80+1,	0xB4);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_80+2,	0x90);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_84, 	0xB7);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_84+1,	0x8E);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_84+2,	0x06);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_88, 	0x66);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_88+1,	0xE2);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_88+2,	0x77);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_92, 	0x48);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_92+1,	0x71);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_92+2,	0xFA);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_96, 	0x9D);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_96+1,	0x68);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_96+2,	0xF8);
			/****************************5*****************************/
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_92, 	0x7B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_92+1,	0x2A);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_92+2,	0xDD);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_96, 	0xC5);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_96+1,	0xC4);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_96+2,	0x0F);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_100,	0x63);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_100+1,	0xBD);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_100+2,	0x30);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_104,	0x3A);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_104+1,	0x3B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_104+2,	0xF1);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_108,	0xA1);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_108+1,	0x17);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_108+2,	0xF2);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_100,	0x7B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_100+1,	0x2A);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_100+2,	0xDD);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_104,	0xC5);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_104+1,	0xC4);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_104+2,	0x0F);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_108,	0x63);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_108+1,	0xBD);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_108+2,	0x30);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_112,	0x3A);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_112+1,	0x3B);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_112+2,	0xF1);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_116,	0xA1);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_116+1,	0x17);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_116+2,	0xF2);
		

	}
	else
	{
		//printk("yang remove eq\n");
		
		if(snd_soc_read(eq_codec, AIC3256_DAC_FLAG_1)&0x80)
		{
			//printk("yang dac working and switch to hp eq\n");
			snd_soc_write(eq_codec, AIC3256_DAC_ADAPTIVE_CRAM_REG,			0x04);

			
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_12, 	0x7f);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_12+1, 	0xff);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_12+2, 	0xff);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_16, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_16+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_16+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_20, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_20+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_20+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_24, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_24+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_24+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_28, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_28+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_28+2, 	0x0);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_20, 	0x7f);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_20+1, 	0xff);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_20+2, 	0xff);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_24, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_24+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_24+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_28, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_28+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_28+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_32, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_32+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_32+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_36, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_36+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_36+2, 	0x0);
		/****************************2*****************************/
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_32, 	0x7f);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_32+1, 	0xff);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_32+2, 	0xff);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_36, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_36+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_36+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_40, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_40+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_40+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_44, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_44+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_44+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_48, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_48+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_48+2, 	0x0);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_40, 	0x7f);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_40+1, 	0xff);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_40+2, 	0xff);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_44, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_44+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_44+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_48, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_48+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_48+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_52, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_52+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_52+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_56, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_56+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_56+2, 	0x0);

			/*************************3********************************/
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_52, 	0x7f);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_52+1, 	0xff);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_52+2, 	0xff);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_56, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_56+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_56+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_60, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_60+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_60+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_64, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_64+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_64+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_68, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_68+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_68+2, 	0x0);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_60, 	0x7f);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_60+1, 	0xff);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_60+2, 	0xff);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_64, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_64+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_64+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_68, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_68+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_68+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_72, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_72+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_72+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_76, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_76+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_76+2, 	0x0);


			/*************************4********************************/
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_72, 	0x7f);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_72+1, 	0xff);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_72+2, 	0xff);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_76, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_76+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_76+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_80, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_80+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_80+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_84, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_84+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_84+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_88, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_88+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_88+2, 	0x0);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_80, 	0x7f);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_80+1, 	0xff);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_80+2, 	0xff);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_84, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_84+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_84+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_88, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_88+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_88+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_92, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_92+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_92+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_96, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_96+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_96+2, 	0x0);

			/*************************5********************************/
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_92, 	0x7f);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_92+1, 	0xff);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_92+2, 	0xff);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_96, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_96+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_96+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_100, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_100+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_100+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_104, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_104+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_104+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_108, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_108+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_108+2, 	0x0);
		
			//-------
			
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_100, 	0x7f);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_100+1, 	0xff);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_100+2, 	0xff);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_104, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_104+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_104+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_108, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_108+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_108+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_112, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_112+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_112+2, 	0x0);
		
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_116, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_116+1, 	0x0);
			snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_116+2, 	0x0);

			snd_soc_write(eq_codec, AIC3256_DAC_ADAPTIVE_CRAM_REG, 			0x05);
			mdelay(1);

			if(snd_soc_read(eq_codec,  AIC3256_DAC_ADAPTIVE_CRAM_REG)&0x01)
			{
				//printk("switch not finish %d\n", snd_soc_read(eq_codec,  AIC3256_DAC_ADAPTIVE_CRAM_REG));
				mdelay(1);
			}//else
				//printk("switch ok %d\n", snd_soc_read(eq_codec,  AIC3256_DAC_ADAPTIVE_CRAM_REG));

		}
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_12, 	0x7f);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_12+1, 	0xff);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_12+2, 	0xff);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_16, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_16+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_16+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_20, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_20+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_20+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_24, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_24+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_24+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_28, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_28+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_28+2, 	0x0);
	
		//-------
		
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_20, 	0x7f);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_20+1, 	0xff);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_20+2, 	0xff);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_24, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_24+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_24+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_28, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_28+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_28+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_32, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_32+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_32+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_36, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_36+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_36+2, 	0x0);
	/****************************2*****************************/
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_32, 	0x7f);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_32+1, 	0xff);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_32+2, 	0xff);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_36, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_36+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_36+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_40, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_40+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_40+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_44, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_44+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_44+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_48, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_48+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_48+2, 	0x0);
	
		//-------
		
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_40, 	0x7f);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_40+1, 	0xff);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_40+2, 	0xff);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_44, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_44+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_44+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_48, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_48+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_48+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_52, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_52+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_52+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_56, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_56+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_56+2, 	0x0);

		/*************************3********************************/
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_52, 	0x7f);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_52+1, 	0xff);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_52+2, 	0xff);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_56, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_56+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_56+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_60, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_60+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_60+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_64, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_64+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_64+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_68, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_68+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_68+2, 	0x0);
	
		//-------
		
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_60, 	0x7f);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_60+1, 	0xff);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_60+2, 	0xff);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_64, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_64+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_64+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_68, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_68+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_68+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_72, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_72+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_72+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_76, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_76+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_76+2, 	0x0);


		/*************************4********************************/
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_72, 	0x7f);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_72+1, 	0xff);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_72+2, 	0xff);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_76, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_76+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_76+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_80, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_80+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_80+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_84, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_84+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_84+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_88, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_88+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_88+2, 	0x0);
	
		//-------
		
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_80, 	0x7f);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_80+1, 	0xff);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_80+2, 	0xff);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_84, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_84+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_84+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_88, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_88+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_88+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_92, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_92+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_92+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_96, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_96+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_96+2, 	0x0);

		/*************************5********************************/
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_92, 	0x7f);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_92+1, 	0xff);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_92+2, 	0xff);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_96, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_96+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_96+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_100, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_100+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_100+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_104, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_104+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_104+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_108, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_108+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK44_REG_108+2, 	0x0);
	
		//-------
		
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_100, 	0x7f);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_100+1, 	0xff);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_100+2, 	0xff);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_104, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_104+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_104+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_108, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_108+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_108+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_112, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_112+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_112+2, 	0x0);
	
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_116, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_116+1, 	0x0);
		snd_soc_write(eq_codec, AIC3252_DAC_ADAPTIVE_BANK45_REG_116+2, 	0x0);

		
	}

	eq_status = new_eq_status;
	
	return 1;
}

struct snd_kcontrol_new tegra_aic325x_eq_ctrl_control = {
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Eq Ctrl Switch",
	.private_value = 0xffff,
	.info = tegra_aic325x_eq_ctrl_info,
	.get = tegra_aic325x_eq_ctrl_get,
	.put = tegra_aic325x_eq_ctrl_put
};

static int tegra_aic325x_spk_ctrl_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int tegra_aic325x_spk_ctrl_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = gpio_get_value(TEGRA_GPIO_EN_CODEC_PA);

	return 0;
}

static int tegra_aic325x_spk_ctrl_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int is_spk_ctrl_new = ucontrol->value.integer.value[0];

	//printk("********************%s set SPK_CTRL %d\n", __func__, is_spk_ctrl_new);
	if(spk_ctrl > 1)	
	{
		if(is_spk_ctrl_new > 0)
		{
			//printk("open spk\n");
			gpio_direction_output(TEGRA_GPIO_EN_CODEC_PA,  1);
			udelay(5);
			gpio_direction_output(TEGRA_GPIO_EN_CODEC_PA,  0);
			udelay(5);
			gpio_direction_output(TEGRA_GPIO_EN_CODEC_PA,  1);
			
		}
		else
		{
			//printk("close spk\n");
			gpio_direction_output(TEGRA_GPIO_EN_CODEC_PA,  0);
		}
	}
	else if(spk_ctrl==1)
	{
		printk("close spk\n");
		gpio_direction_output(TEGRA_GPIO_EN_CODEC_PA,  0);
		spk_ctrl +=1;
	}
	else
		spk_ctrl += 1;
	return 1;
}

struct snd_kcontrol_new tegra_aic325x_spk_ctrl_control = {
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Spk Ctrl Switch",
	.private_value = 0xffff,
	.info = tegra_aic325x_spk_ctrl_info,
	.get = tegra_aic325x_spk_ctrl_get,
	.put = tegra_aic325x_spk_ctrl_put
};

static int tegra_aic325x_call_mode_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int tegra_aic325x_call_mode_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_aic325x *machine = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = machine->is_call_mode;

	return 0;
}

static int tegra_aic325x_call_mode_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_aic325x *machine = snd_kcontrol_chip(kcontrol);
	int is_call_mode_new = ucontrol->value.integer.value[0];
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	int codec_dap_id, codec_dap_sel, bb_dap_id, bb_dap_sel;
#else /*assumes tegra3*/
	int codec_index;
	unsigned int i;
#endif

	if (machine->is_call_mode == is_call_mode_new)
		return 0;

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	bb_dap_id = TEGRA20_DAS_DAP_ID_3;
	bb_dap_sel = TEGRA20_DAS_DAP_SEL_DAP3;

	if (machine->is_device_bt) {
		codec_dap_id = TEGRA20_DAS_DAP_ID_4;
		codec_dap_sel = TEGRA20_DAS_DAP_SEL_DAP4;
	} else {
		codec_dap_id = TEGRA20_DAS_DAP_ID_2;
		codec_dap_sel = TEGRA20_DAS_DAP_SEL_DAP2;
	}
#else /*assumes tegra3*/
	if (machine->is_device_bt)
		codec_index = BT_SCO;
	else
		codec_index = HIFI_CODEC;
#endif

	if (is_call_mode_new) {
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		tegra20_das_set_tristate(codec_dap_id, 1);
		tegra20_das_set_tristate(bb_dap_id, 1);
		tegra20_das_connect_dap_to_dap(codec_dap_id,
			bb_dap_sel, 0, 0, 0);
		tegra20_das_connect_dap_to_dap(bb_dap_id,
			codec_dap_sel, 1, 0, 0);
		tegra20_das_set_tristate(codec_dap_id, 0);
		tegra20_das_set_tristate(bb_dap_id, 0);
#else /*assumes tegra3*/
		if (machine->codec_info[codec_index].rate == 0 ||
			machine->codec_info[codec_index].channels == 0)
				return -EINVAL;

		for (i = 0; i < machine->pcard->num_links; i++)
			machine->pcard->dai_link[i].ignore_suspend = 1;

		tegra30_make_voice_call_connections(
			&machine->codec_info[codec_index],
			&machine->codec_info[BASEBAND]);
#endif
	} else {
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		tegra20_das_set_tristate(codec_dap_id, 1);
		tegra20_das_set_tristate(bb_dap_id, 1);
		tegra20_das_connect_dap_to_dap(bb_dap_id,
			bb_dap_sel, 0, 0, 0);
		tegra20_das_connect_dap_to_dap(codec_dap_id,
			codec_dap_sel, 0, 0, 0);
		tegra20_das_set_tristate(codec_dap_id, 0);
		tegra20_das_set_tristate(bb_dap_id, 0);
#else /*assumes tegra3*/
		tegra30_break_voice_call_connections(
			&machine->codec_info[codec_index],
			&machine->codec_info[BASEBAND]);

		for (i = 0; i < machine->pcard->num_links; i++)
			machine->pcard->dai_link[i].ignore_suspend = 0;
#endif
	}

	machine->is_call_mode = is_call_mode_new;
	g_is_call_mode = machine->is_call_mode;

	return 1;
}

struct snd_kcontrol_new tegra_aic325x_call_mode_control = {
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Call Mode Switch",
	.private_value = 0xffff,
	.info = tegra_aic325x_call_mode_info,
	.get = tegra_aic325x_call_mode_get,
	.put = tegra_aic325x_call_mode_put
};

static int tegra_aic325x_get_mclk(int srate)
{
	int mclk = 0;
	switch (srate) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	default:
		mclk = -EINVAL;
		break;
	}

	return mclk;
}

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
static int tegra_aic325x_set_dam_cif(int dam_ifc, int srate,
			int channels, int bit_size, int src_on, int src_srate,
			int src_channels, int src_bit_size)
{
	tegra30_dam_set_gain(dam_ifc, TEGRA30_DAM_CHIN1, 0x1000);
	tegra30_dam_set_samplerate(dam_ifc, TEGRA30_DAM_CHOUT,
				srate);
	tegra30_dam_set_samplerate(dam_ifc, TEGRA30_DAM_CHIN1,
				srate);
	tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHIN1,
		channels, bit_size, channels,
				bit_size);
	tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHOUT,
		channels, bit_size, channels,
				bit_size);

	if (src_on) {
		tegra30_dam_set_gain(dam_ifc, TEGRA30_DAM_CHIN0_SRC, 0x1000);
		tegra30_dam_set_samplerate(dam_ifc, TEGRA30_DAM_CHIN0_SRC,
			src_srate);
		tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHIN0_SRC,
			src_channels, src_bit_size, 1, 16);
	}

	return 0;
}
#endif

static int tegra_aic325x_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk, sample_size, daifmt;
	int err, rate;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);
#endif

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		sample_size = 16;
		break;
	default:
		return -EINVAL;
	}

	srate = params_rate(params);

	mclk = tegra_aic325x_get_mclk(srate);
	if (mclk < 0) {
		dev_err(card->dev, "tegra_aic325x_get_mclk < 0\n");
		return mclk;
	}
	daifmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	rate = clk_get_rate(machine->util_data.clk_cdev1);
	err = snd_soc_dai_set_fmt(codec_dai, daifmt);
	if (err < 0) {
		dev_err(card->dev, "codec_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_fmt(cpu_dai, daifmt);
	if (err < 0) {
		dev_err(card->dev, "cpu_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_pll(codec_dai, 0, AIC3256_CLK_REG_1,
					rate, params_rate(params));
	if (err < 0) {
		dev_err(card->dev, "Failed to set pll\n");
		return err;
	}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	err = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC1,
					TEGRA20_DAS_DAP_ID_1);
	if (err < 0) {
		dev_err(card->dev, "failed to set dap-dac path\n");
		return err;
	}

	err = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_1,
					TEGRA20_DAS_DAP_SEL_DAC1);
	if (err < 0) {
		dev_err(card->dev, "failed to set dac-dap path\n");
		return err;
	}

#else /*assumes tegra3*/
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK  && i2s->is_dam_used)
		tegra_aic325x_set_dam_cif(i2s->dam_ifc, srate,
			params_channels(params), sample_size, 0, 0, 0, 0);
#endif
	return 0;
}
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
static int tegra_aic325x_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(rtd->card);
	struct codec_config *codec_info;
	struct codec_config *bb_info;
	int codec_index;

	if (!i2s->is_dam_used)
		return 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/*dam configuration*/
		if (!i2s->dam_ch_refcount)
			i2s->dam_ifc = tegra30_dam_allocate_controller();

		tegra30_dam_allocate_channel(i2s->dam_ifc, TEGRA30_DAM_CHIN1);
		i2s->dam_ch_refcount++;
		tegra30_dam_enable_clock(i2s->dam_ifc);
		tegra30_dam_set_gain(i2s->dam_ifc, TEGRA30_DAM_CHIN1, 0x1000);

		tegra30_ahub_set_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX1 +
				(i2s->dam_ifc*2), i2s->txcif);

		/*
		*make the dam tx to i2s rx connection if this is the only client
		*using i2s for playback
		*/
		if (i2s->playback_ref_count == 1)
			tegra30_ahub_set_rx_cif_source(
				TEGRA30_AHUB_RXCIF_I2S0_RX0 + i2s->id,
				TEGRA30_AHUB_TXCIF_DAM0_TX0 + i2s->dam_ifc);

		/* enable the dam*/
		tegra30_dam_enable(i2s->dam_ifc, TEGRA30_DAM_ENABLE,
				TEGRA30_DAM_CHIN1);
	} else {

		i2s->is_call_mode_rec = machine->is_call_mode;

		if (!i2s->is_call_mode_rec)
			return 0;

		if (machine->is_device_bt)
			codec_index = BT_SCO;
		else
			codec_index = HIFI_CODEC;

		codec_info = &machine->codec_info[codec_index];
		bb_info = &machine->codec_info[BASEBAND];

		/* allocate a dam for voice call recording */

		i2s->call_record_dam_ifc = tegra30_dam_allocate_controller();
		tegra30_dam_allocate_channel(i2s->call_record_dam_ifc,
			TEGRA30_DAM_CHIN0_SRC);
		tegra30_dam_allocate_channel(i2s->call_record_dam_ifc,
			TEGRA30_DAM_CHIN1);
		tegra30_dam_enable_clock(i2s->call_record_dam_ifc);

		/* configure the dam */
		tegra_aic325x_set_dam_cif(i2s->call_record_dam_ifc,
			codec_info->rate, codec_info->channels,
			codec_info->bitsize, 1, bb_info->rate,
			bb_info->channels, bb_info->bitsize);

		/* setup the connections for voice call record */

		tegra30_ahub_unset_rx_cif_source(i2s->rxcif);
		tegra30_ahub_set_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX0 +
			(i2s->call_record_dam_ifc*2),
			TEGRA30_AHUB_TXCIF_I2S0_TX0 + bb_info->i2s_id);
		tegra30_ahub_set_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX1 +
			(i2s->call_record_dam_ifc*2),
			TEGRA30_AHUB_TXCIF_I2S0_TX0 + codec_info->i2s_id);
		tegra30_ahub_set_rx_cif_source(i2s->rxcif,
			TEGRA30_AHUB_TXCIF_DAM0_TX0 + i2s->call_record_dam_ifc);

		/* enable the dam*/

		tegra30_dam_enable(i2s->call_record_dam_ifc, TEGRA30_DAM_ENABLE,
				TEGRA30_DAM_CHIN1);
		tegra30_dam_enable(i2s->call_record_dam_ifc, TEGRA30_DAM_ENABLE,
				TEGRA30_DAM_CHIN0_SRC);
	}

	return 0;
}

static void tegra_aic325x_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);

	if (!i2s->is_dam_used)
		return;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* disable the dam*/
		tegra30_dam_enable(i2s->dam_ifc, TEGRA30_DAM_DISABLE,
				TEGRA30_DAM_CHIN1);

		/* disconnect the ahub connections*/
		tegra30_ahub_unset_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX1 +
					(i2s->dam_ifc*2));

		/* disable the dam and free the controller */
		tegra30_dam_disable_clock(i2s->dam_ifc);
		tegra30_dam_free_channel(i2s->dam_ifc, TEGRA30_DAM_CHIN1);
		i2s->dam_ch_refcount--;
		if (!i2s->dam_ch_refcount)
			tegra30_dam_free_controller(i2s->dam_ifc);
	 } else {

		if (!i2s->is_call_mode_rec)
			return;

		i2s->is_call_mode_rec = 0;

		/* disable the dam*/
		tegra30_dam_enable(i2s->call_record_dam_ifc,
			TEGRA30_DAM_DISABLE, TEGRA30_DAM_CHIN1);
		tegra30_dam_enable(i2s->call_record_dam_ifc,
			TEGRA30_DAM_DISABLE, TEGRA30_DAM_CHIN0_SRC);

		/* disconnect the ahub connections*/
		tegra30_ahub_unset_rx_cif_source(i2s->rxcif);
		tegra30_ahub_unset_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX0 +
			(i2s->call_record_dam_ifc*2));
		tegra30_ahub_unset_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX1 +
			(i2s->call_record_dam_ifc*2));

		/* free the dam channels and dam controller */
		tegra30_dam_disable_clock(i2s->call_record_dam_ifc);
		tegra30_dam_free_channel(i2s->call_record_dam_ifc,
			TEGRA30_DAM_CHIN1);
		tegra30_dam_free_channel(i2s->call_record_dam_ifc,
			TEGRA30_DAM_CHIN0_SRC);
		tegra30_dam_free_controller(i2s->call_record_dam_ifc);
	 }

	return;
}
#endif


static int tegra_aic325x_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(rtd->card);

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 0);

	return 0;
}
static struct snd_soc_ops tegra_aic325x_hifi_ops = {
	.hw_params = tegra_aic325x_hw_params,
	.hw_free = tegra_aic325x_hw_free,
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	.startup = tegra_aic325x_startup,
	.shutdown = tegra_aic325x_shutdown,
#endif
};
static struct snd_soc_jack tegra_aic325x_hp_jack;

#ifdef CONFIG_SWITCH

static struct switch_dev aic325x_wired_switch_dev = {
	.name = "h2w",
};

/* These values are copied from WiredAccessoryObserver */
enum headset_state {
	BIT_NO_HEADSET = 0,
	BIT_HEADSET = (1 << 0),
	BIT_HEADSET_NO_MIC = (1 << 1),
};

static int aic325x_headset_switch_notify(struct notifier_block *self,
	unsigned long action, void *dev)
{
	int state = 0;

	switch (action) {
	case SND_JACK_HEADPHONE:
		state |= BIT_HEADSET_NO_MIC;
		break;
	case SND_JACK_HEADSET:
		state |= BIT_HEADSET;
		break;
	default:
		state |= BIT_NO_HEADSET;
	}

	switch_set_state(&aic325x_wired_switch_dev, state);

	return NOTIFY_OK;
}

static struct notifier_block aic325x_headset_switch_nb = {
	.notifier_call = aic325x_headset_switch_notify,
};

#else

static struct snd_soc_jack_pin tegra_aic325x_hp_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};
#endif
static const struct snd_soc_dapm_widget tegra_aic325x_dapm_widgets[] = {

	SND_SOC_DAPM_HP("Headphone jack", NULL),
	SND_SOC_DAPM_LINE("Line out", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_LINE("Linein", NULL),
	SND_SOC_DAPM_MIC("Ext Mic", NULL),
	SND_SOC_DAPM_SPK("SPK out", NULL),


};

static const struct snd_soc_dapm_route aic325x_audio_map[] = {

	/* Headphone connected to HPL, HPR */
	{"Headphone jack", NULL, "HPL"},
	{"Headphone jack", NULL, "HPR"},


	/* Line Out connected to LOL, LOR */
	{"Line out", NULL, "LOL"},
	{"Line out", NULL, "LOR"},

	{"SPK out", NULL, "LOL"},
	{"SPK out", NULL, "LOR"},

//	{"IN1_L", NULL, "Linein"},
//	{"IN1_R", NULL, "Linein"},
	{"IN1_L", NULL, "Ext Mic"},
	{"IN1_R", NULL, "Ext Mic"},

	{"IN2_L", NULL, "Int Mic"}, /*nikesh*/
	{"IN2_R", NULL, "Int Mic"},
};

static const struct snd_kcontrol_new tegra_aic325x_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
	SOC_DAPM_PIN_SWITCH("Earpiece"),
	SOC_DAPM_PIN_SWITCH("Headphone jack"),
	SOC_DAPM_PIN_SWITCH("Mic Jack"),

	SOC_DAPM_PIN_SWITCH("Ext Mic"),
	SOC_DAPM_PIN_SWITCH("Linein"),
	SOC_DAPM_PIN_SWITCH("Line out"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("SPK out"),

};

static int tegra_aic325x_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = codec->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(rtd->cpu_dai);
#endif
	int ret;

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	if (machine->codec_info[BASEBAND].i2s_id != -1)
		i2s->is_dam_used = true;
#endif

	if (machine->init_done)
		return 0;

	machine->init_done = true;

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	machine->pcard = card;
#endif

	if (machine_is_whistler()) {
		machine->audio_reg = regulator_get(NULL, "avddio_audio");
		if (IS_ERR(machine->audio_reg)) {
			dev_err(card->dev, "cannot get avddio_audio reg\n");
			ret = PTR_ERR(machine->audio_reg);
			return ret;
		}

		ret = regulator_enable(machine->audio_reg);
		if (ret) {
			dev_err(card->dev, "cannot enable avddio_audio reg\n");
			regulator_put(machine->audio_reg);
			machine->audio_reg = NULL;
			return ret;
		}
	}

	if (gpio_is_valid(pdata->gpio_spkr_en)) {
		ret = gpio_request(pdata->gpio_spkr_en, "spkr_en");
		if (ret) {
			dev_err(card->dev, "cannot get spkr_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_SPKR_EN;

		gpio_direction_output(pdata->gpio_spkr_en, 0);
	}

	if (gpio_is_valid(pdata->gpio_hp_mute)) {
		ret = gpio_request(pdata->gpio_hp_mute, "hp_mute");
		if (ret) {
			dev_err(card->dev, "cannot get hp_mute gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_HP_MUTE;

		gpio_direction_output(pdata->gpio_hp_mute, 0);
	}

	if (gpio_is_valid(pdata->gpio_int_mic_en)) {
		ret = gpio_request(pdata->gpio_int_mic_en, "int_mic_en");
		if (ret) {
			dev_err(card->dev, "cannot get int_mic_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_INT_MIC_EN;

		/* Disable int mic; enable signal is active-high */
		gpio_direction_output(pdata->gpio_int_mic_en, 0);
	}

	if (gpio_is_valid(pdata->gpio_ext_mic_en)) {
		ret = gpio_request(pdata->gpio_ext_mic_en, "ext_mic_en");
		if (ret) {
			dev_err(card->dev, "cannot get ext_mic_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_EXT_MIC_EN;

		/* Enable ext mic; enable signal is active-low */
		gpio_direction_output(pdata->gpio_ext_mic_en, 0);
	}
	
	ret = snd_soc_add_codec_controls(codec, tegra_aic325x_controls,
				   ARRAY_SIZE(tegra_aic325x_controls));
	if (ret < 0)
		return ret;

	snd_soc_dapm_new_controls(dapm, tegra_aic325x_dapm_widgets,
					ARRAY_SIZE(tegra_aic325x_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, aic325x_audio_map,
					ARRAY_SIZE(aic325x_audio_map));


	ret = snd_soc_jack_new(codec, "Headphone jack", SND_JACK_HEADSET,
			&tegra_aic325x_hp_jack);
	if (ret < 0)
		return ret;

#ifdef CONFIG_SWITCH

	snd_soc_jack_notifier_register(&tegra_aic325x_hp_jack,
		&aic325x_headset_switch_nb);
#else /*gpio based headset detection*/

	snd_soc_jack_add_pins(&tegra_aic325x_hp_jack,
		ARRAY_SIZE(tegra_aic325x_hp_jack_pins),
		tegra_aic325x_hp_jack_pins);

#endif
	aic3256_hs_jack_detect(codec, &tegra_aic325x_hp_jack,
		SND_JACK_HEADSET);
	eq_codec = codec;

	ret = snd_ctl_add(codec->card->snd_card,
				snd_ctl_new1(&tegra_aic325x_adc_eq_ctrl_control,
					machine));
	if (ret < 0)
		return ret;

	ret = snd_ctl_add(codec->card->snd_card,
				snd_ctl_new1(&tegra_aic325x_eq_ctrl_control,
					machine));
	if (ret < 0)
		return ret;

	ret = snd_ctl_add(codec->card->snd_card,
				snd_ctl_new1(&tegra_aic325x_spk_ctrl_control,
					machine));
	if (ret < 0)
	{
		printk("%s(%d): register spk ctrl fail ret = %d\n", __func__, __LINE__, ret);
		return ret;
	}
	/* Add call mode switch control */
	ret = snd_ctl_add(codec->card->snd_card,
			snd_ctl_new1(&tegra_aic325x_call_mode_control,
				machine));
	if (ret < 0)
		return ret;

	snd_soc_dapm_nc_pin(dapm, "IN2_L");
	snd_soc_dapm_nc_pin(dapm, "IN2_R");
	snd_soc_dapm_sync(dapm);

	ret = tegra_asoc_utils_register_ctls(&machine->util_data);
	if (ret < 0) {
		dev_err(card->dev, "Register ctls failed\n");
		return ret;
	}

	snd_soc_update_bits(codec, AIC3256_OUT_PWR_CTRL, 0x0c, 0x0c);
	
	return 0;
}

static struct snd_soc_dai_link tegra_aic325x_dai[] = {
	[DAI_LINK_HIFI] = {
		.name = "TLV320AIC325x",
		.stream_name = "TLV320AIC325x",
		/*.codec_name = "tlv320aic325x.4-0018",*/
		.codec_name = "tlv320aic325x-codec",
		.platform_name = "tegra-pcm-audio",
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		.cpu_dai_name = "tegra20-i2s.0",
#else
		.cpu_dai_name = "tegra30-i2s.1",
#endif
		.codec_dai_name = "tlv320aic325x-MM_EXT",
		.init = tegra_aic325x_init,
		.ops = &tegra_aic325x_hifi_ops,
		},

};

static struct snd_soc_card snd_soc_tegra_aic325x = {
	.name = "tegra-aic325x",
	.dai_link = tegra_aic325x_dai,
	.num_links = ARRAY_SIZE(tegra_aic325x_dai),
//	.controls =  tegra_aic325x_controls,
//	.num_controls = ARRAY_SIZE(tegra_aic325x_controls),
//	.dapm_widgets = tegra_aic325x_dapm_widgets,
//	.num_dapm_widgets = ARRAY_SIZE(tegra_aic325x_dapm_widgets),
//	.dapm_routes = aic325x_audio_map,
//	.num_dapm_routes = ARRAY_SIZE(aic325x_audio_map),
};

static __devinit int tegra_aic325x_driver_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_tegra_aic325x;
	struct tegra_aic325x *machine;
	struct tegra_asoc_platform_data *pdata;
	int ret;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	int i;
#endif
	//printk(KERN_ALERT "Entered %s\n", __func__);

	pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		return -EINVAL;
	}

	machine = kzalloc(sizeof(struct tegra_aic325x), GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate tegra_aic325x struct\n");
		return -ENOMEM;
	}

	machine->pdata = pdata;

	ret = tegra_asoc_utils_init(&machine->util_data, &pdev->dev, card);

	if (ret) {
		dev_err(card->dev, "tegra_asoc_utils_init failed\n");
		goto err_free_machine;
	}
	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

#ifdef CONFIG_SWITCH
	/* Add h2w switch class support */

	ret = switch_dev_register(&aic325x_wired_switch_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "not able to register switch device %d\n",
			ret);
		goto err_fini_utils;
	}

#endif

#ifndef CONFIG_ARCH_TEGRA_2x_SOC

	for (i = 0; i < NUM_I2S_DEVICES ; i++)
		machine->codec_info[i].i2s_id =
					pdata->i2s_param[i].audio_port_id;

	tegra_aic325x_dai[DAI_LINK_HIFI].cpu_dai_name =
	tegra_i2s_dai_name[machine->codec_info[HIFI_CODEC].i2s_id];

#endif

	if (machine_is_tegra_enterprise()) {
#if 0
		tegra_aic325x_dai[DAI_LINK_HIFI].codec_name =
					"tlv320aic325x.0-0018";
#endif
	}

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_switch_unregister;
	}
	if (!card->instantiated) {
		dev_err(&pdev->dev, "No TI AIC3256 codec\n");
		goto err_unregister_card;
	}
#if 1
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	ret = tegra_asoc_utils_set_parent(&machine->util_data,
				pdata->i2s_param[HIFI_CODEC].is_i2s_master);
	if (ret) {
		dev_err(&pdev->dev, "tegra_asoc_utils_set_parent failed (%d)\n",
			ret);
		goto err_unregister_card;
	}
#endif
#endif
	return 0;

err_unregister_card:
	snd_soc_unregister_card(card);
err_switch_unregister:
#ifdef CONFIG_SWITCH
	switch_dev_unregister(&aic325x_wired_switch_dev);
#endif
err_fini_utils:
	tegra_asoc_utils_fini(&machine->util_data);
err_free_machine:
	kfree(machine);
	return ret;
}

static int __devexit tegra_aic325x_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	snd_soc_unregister_card(card);

#ifdef CONFIG_SWITCH
	switch_dev_unregister(&aic325x_wired_switch_dev);
#endif

	tegra_asoc_utils_fini(&machine->util_data);

	if (machine->gpio_requested & GPIO_EXT_MIC_EN)
		gpio_free(pdata->gpio_ext_mic_en);
	if (machine->gpio_requested & GPIO_INT_MIC_EN)
		gpio_free(pdata->gpio_int_mic_en);
	if (machine->gpio_requested & GPIO_HP_MUTE)
		gpio_free(pdata->gpio_hp_mute);
	if (machine->gpio_requested & GPIO_SPKR_EN)
		gpio_free(pdata->gpio_spkr_en);

	kfree(machine);

	return 0;
}

static struct platform_driver tegra_aic325x_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
	},
	.probe = tegra_aic325x_driver_probe,
	.remove = __devexit_p(tegra_aic325x_driver_remove),
};

static int __init tegra_aic325x_modinit(void)
{
	return platform_driver_register(&tegra_aic325x_driver);
}

module_init(tegra_aic325x_modinit);

static void __exit tegra_aic325x_modexit(void)
{
	platform_driver_unregister(&tegra_aic325x_driver);
}
module_exit(tegra_aic325x_modexit);

/* Module information */
MODULE_AUTHOR("Vinod G. <vinodg@nvidia.com>");
MODULE_DESCRIPTION("Tegra+AIC3262 machine ASoC driver");
MODULE_DESCRIPTION("Tegra ALSA SoC");
MODULE_LICENSE("GPL");

