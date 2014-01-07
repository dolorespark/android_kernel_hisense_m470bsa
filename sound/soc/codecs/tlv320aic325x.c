/*
 * linux/sound/soc/codecs/tlv320aic325x.c
 *
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *
 * Based on sound/soc/codecs/wm8753.c by Liam Girdwood
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 * Rev 0.1   ASoC driver support			31-04-2009
 * The AIC32 ASoC driver is ported for the codec AIC325x.
 *
 *
 * Rev 1.0   Mini DSP support				11-05-2009
 * Added mini DSP programming support
 *
 * Rev 1.1   Mixer controls				18-01-2011
 * Added all the possible mixer controls.
 *
 * Rev 1.2   Additional Codec driver support		2-02-2011
 * Support for AIC3253, AIC3206, AIC3256
 *
 * Rev 2.0   Ported the Codec driver to 2.6.39 kernel	30-03-2012
 *
 * Rev 2.1   PLL DAPM support added to the codec driver	03-04-2012
 *
 * Rev 2.2   Added event handlers for DAPM widgets	16-05-2012
 *	     Updated ENUM declerations
 *
 */

/*
 * Includes
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/mutex.h>

#include <linux/slab.h>
#include <linux/firmware.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/jack.h>
#include <linux/input.h>

#include "tlv320aic325x.h"
#include <linux/mfd/tlv320aic3256-registers.h>
#include <linux/mfd/tlv320aic3xxx-core.h>
#include "../../../arch/arm/mach-tegra/board-enterprise.h"
#include "../../../arch/arm/mach-tegra/gpio-names.h"

static struct snd_soc_codec *aic3256_lb_codec;
static int jack_status = 0;
bool pressdown = false;
bool pressup = false;
int rescheduled = 0;
extern bool HeadsetIN;
extern int his_hpdet;
/*
 * enable debug prints in the driver
 */
#define DBG
#ifdef DBG
	#define dprintk(x...)   printk(x)
#else
	#define dprintk(x...)
#endif

/* User defined Macros kcontrol builders */
#define SOC_SINGLE_AIC325x(xname)                                       \
	{                                                               \
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,     \
		.info = __new_control_info, .get = __new_control_get, \
		.put = __new_control_put,                       \
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,      \
	}


/*
* Function Prototype
*/
static int aic325x_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *);
static int aic325x_mute(struct snd_soc_dai *dai, int mute);
static int aic325x_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt);
static int aic325x_set_bias_level(struct snd_soc_codec *codec,
					enum snd_soc_bias_level level);
static int aic325x_dai_set_pll(struct snd_soc_dai *dai, int pll_id, int source,
				unsigned int Fin, unsigned int Fout);

static unsigned int aic325x_codec_read(struct snd_soc_codec *codec,
			unsigned int reg);

static int aic325x_codec_write(struct snd_soc_codec *codec, unsigned int reg,
			unsigned int value);

static int __new_control_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo);
static int __new_control_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
static int __new_control_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
static int aic325x_hp_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event);
//static int aic325x_change_page(struct snd_soc_codec *codec, u8 new_page);
static int aic3256_get_runstate(struct snd_soc_codec *codec);
static int aic3256_dsp_pwrdwn_status(struct snd_soc_codec *codec);
static int aic3256_dsp_pwrup(struct snd_soc_codec *codec, int state);
static int aic3256_restart_dsps_sync(struct snd_soc_codec *codec, int rs);

static inline unsigned int dsp_non_sync_mode(unsigned int state)
			{ return (!((state & 0x03) && (state & 0x30))); }

static void aic3256_firmware_load(const struct firmware *fw, void *context);
/*
* Global Variable
*/
static u8 aic325x_reg_ctl;

/* whenever aplay/arecord is run, aic325x_hw_params() function gets called.
 * This function reprograms the clock dividers etc. this flag can be used to
 * disable this when the clock dividers are programmed by pps config file
 */

//static int soc_static_freq_config = 1;

static const char * const mute[] = { "Unmute", "Mute" };

/* DAC Volume Soft Step Control */
static const char * const dacsoftstep_control[] = { "1 step/sample",
						"1 step/2 sample",
						"disabled" };
SOC_ENUM_SINGLE_DECL(dac_vol_soft_setp_enum, AIC3256_DAC_CHN_REG, 0,
						dacsoftstep_control);

/* Volume Mode Selection Control */
static const char * const volume_extra[] = { "L&R Ind Vol", "LVol=RVol",
								"RVol=LVol" };
/* DAC Volume Mode Selection */
SOC_ENUM_SINGLE_DECL(dac_vol_extra_enum, AIC3256_DAC_MUTE_CTRL_REG, 0,
						volume_extra);

/* Beep Master Volume Control */
SOC_ENUM_SINGLE_DECL(beep_master_vol_enum, AIC3256_BEEP_CTRL_REG2, 6,
						volume_extra);

/* Headset Detection Enable/Disable Control */
static const char * const headset_detection[] = { "Enabled", "Disabled" };
SOC_ENUM_SINGLE_DECL(hs_det_ctrl_enum, AIC3256_HEADSET_DETECT, 7,
						headset_detection);

/* MIC BIAS Voltage Control */
static const char * const micbias_voltage[] = { "1.04/1.25V", "1.425/1.7V",
						"2.075/2.5V", "POWER SUPPY" };
SOC_ENUM_SINGLE_DECL(micbias_voltage_enum, AIC3256_MICBIAS_CTRL, 4,
						micbias_voltage);

/* IN1L to Left MICPGA Positive Terminal Selection */
static const char * const micpga_selection[] = { "off", "10k", "20k", "40k" };
SOC_ENUM_SINGLE_DECL(IN1L_LMICPGA_P_sel_enum, AIC3256_LMICPGA_PIN_CFG, 6,
						micpga_selection);

/* IN2L to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(IN2L_LMICPGA_P_sel_enum, AIC3256_LMICPGA_PIN_CFG, 4,
						micpga_selection);

/* IN3L to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(IN3L_LMICPGA_P_sel_enum, AIC3256_LMICPGA_PIN_CFG, 4,
						micpga_selection);

/* IN1R to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(IN1R_LMICPGA_P_sel_enum, AIC3256_LMICPGA_PIN_CFG, 2,
						micpga_selection);

/* CM1L to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(CM1L_LMICPGA_P_sel_enum, AIC3256_LMICPGA_PIN_CFG, 0,
						micpga_selection);

/* IN2R to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(IN2R_LMICPGA_P_sel_enum, AIC3256_LMICPGA_NIN_CFG, 6,
						micpga_selection);

/* IN3R to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(IN3R_LMICPGA_P_sel_enum, AIC3256_LMICPGA_NIN_CFG, 4,
						micpga_selection);

/*CM2L to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(CM2L_LMICPGA_P_sel_enum, AIC3256_LMICPGA_NIN_CFG, 2,
						micpga_selection);

/* IN1R to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(in1r_rmicpga_enum, AIC3256_RMICPGA_PIN_CFG, 6,
						micpga_selection);

/* IN2R to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(in2r_rmicpga_enum, AIC3256_RMICPGA_PIN_CFG, 4,
						micpga_selection);

/* IN3R to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(in3r_rmicpga_enum, AIC3256_RMICPGA_PIN_CFG, 2,
						micpga_selection);

/* IN2L to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(in2l_rmicpga_enum, AIC3256_RMICPGA_PIN_CFG, 0,
						micpga_selection);

/* CM1R to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(cm1r_rmicpga_enum, AIC3256_RMICPGA_NIN_CFG, 6,
						micpga_selection);

/* IN1L to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(in1l_rmicpga_enum, AIC3256_RMICPGA_NIN_CFG, 4,
						micpga_selection);

/* IN3L to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(in3l_rmicpga_enum, AIC3256_RMICPGA_NIN_CFG, 2,
						micpga_selection);

/* CM2R to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(cm2r_rmicpga_enum, AIC3256_RMICPGA_NIN_CFG, 0,
						micpga_selection);

/* Power up/down */
static const char * const powerup[] = { "Power Down", "Power Up" };

/* Mic Bias Power up/down */
SOC_ENUM_SINGLE_DECL(micbias_pwr_ctrl_enum, AIC3256_MICBIAS_CTRL, 6, powerup);

/* Left DAC Power Control */
SOC_ENUM_SINGLE_DECL(ldac_power_enum, AIC3256_DAC_CHN_REG, 7, powerup);

/* Right DAC Power Control */
SOC_ENUM_SINGLE_DECL(rdac_power_enum, AIC3256_DAC_CHN_REG, 6, powerup);

/* Left ADC Power Control */
SOC_ENUM_SINGLE_DECL(ladc_pwr_ctrl_enum, AIC3256_ADC_CHN_REG, 7, powerup);
/* Right ADC Power Control */
SOC_ENUM_SINGLE_DECL(radc_pwr_ctrl_enum, AIC3256_ADC_CHN_REG, 6, powerup);

/* HeadPhone Driver Power Control */
SOC_ENUM_DOUBLE_DECL(hp_pwr_ctrl_enum, AIC3256_OUT_PWR_CTRL, 5, 4, powerup);

/*Line-Out Driver Power Control */
SOC_ENUM_DOUBLE_DECL(lineout_pwr_ctrl_enum, AIC3256_OUT_PWR_CTRL, 3, 2,
						powerup);

/* Mixer Amplifiers Power Control */
SOC_ENUM_DOUBLE_DECL(mixer_amp_pwr_ctrl_enum, AIC3256_OUT_PWR_CTRL, 1, 0,
						powerup);

/* Mic Bias Generation */
static const char * const vol_generation[] = { "AVDD", "LDOIN" };
SOC_ENUM_SINGLE_DECL(micbias_voltage_ctrl_enum, AIC3256_MICBIAS_CTRL, 3,
						vol_generation);

/* DAC Data Path Control */
static const char * const path_control[] = { "Disabled", "LDAC Data",
						"RDAC Data", "L&RDAC Data" };
/* Left DAC Data Path Control */
SOC_ENUM_SINGLE_DECL(ldac_data_path_ctrl_enum, AIC3256_DAC_CHN_REG, 4,
						path_control);

/* Right DAC Data Path Control */
SOC_ENUM_SINGLE_DECL(rdac_data_path_ctrl_enum, AIC3256_DAC_CHN_REG, 2,
						path_control);

/* Audio gain control (AGC) Enable/Disable Control */
static const char * const disable_enable[] = { "Disabled", "Enabled" };

/* Left Audio gain control (AGC) Enable/Disable Control */
SOC_ENUM_SINGLE_DECL(left_agc_enable_disable_enum, AIC3256_LEFT_AGC_REG1, 7,
						disable_enable);

/* Left/Right Audio gain control (AGC) Enable/Disable Control */
SOC_ENUM_SINGLE_DECL(right_agc_enable_disable_enum, AIC3256_RIGHT_AGC_REG1, 7,
						disable_enable);

/* Left MICPGA Gain Enabled/Disable */
SOC_ENUM_SINGLE_DECL(left_micpga_ctrl_enum, AIC3256_LMICPGA_VOL_CTRL, 7,
						disable_enable);

/* Right MICPGA Gain Enabled/Disable */
SOC_ENUM_SINGLE_DECL(right_micpga_ctrl_enum, AIC3256_RMICPGA_VOL_CTRL, 7,
						disable_enable);

/* DRC Enable/Disable Control */
SOC_ENUM_DOUBLE_DECL(drc_ctrl_enum, AIC3256_DRC_CTRL_REG1, 6, 5,
						disable_enable);

/* Beep generator Enable/Disable control */
SOC_ENUM_SINGLE_DECL(beep_gen_ctrl_enum, AIC3256_BEEP_CTRL_REG1, 7,
						disable_enable);

/* Headphone ground centered mode enable/disable control */
SOC_ENUM_SINGLE_DECL(hp_gnd_centred_mode_ctrl, AIC3256_HP_DRIVER_CONF_REG, 4,
						disable_enable);

/* Audio loopback enable/disable control */
SOC_ENUM_SINGLE_DECL(audio_loopback_enum, AIC3256_AIS_REG_3, 5,
						disable_enable);

/* DMIC intput Selection control */
static const char * const dmic_input_sel[] = { "GPIO", "SCLK", "DIN" };
SOC_ENUM_SINGLE_DECL(dmic_input_enum, AIC3256_ADC_CHN_REG, 4, dmic_input_sel);

/*charge pump Enable*/
static const char * const charge_pump_ctrl_enum[] = { "Power_Down",
							"Reserved",
							"Power_Up" };
SOC_ENUM_SINGLE_DECL(charge_pump_ctrl, AIC3256_POW_CFG, 0,
						charge_pump_ctrl_enum);

/* DAC volume DB scale */
static const DECLARE_TLV_DB_SCALE(dac_vol_tlv, -6350, 50, 0);
/* ADC volume DB scale */
static const DECLARE_TLV_DB_SCALE(adc_vol_tlv, -1200, 50, 0);
/* Output Gain in DB scale */
static const DECLARE_TLV_DB_SCALE(output_gain_tlv, -600, 100, 0);
/* MicPGA Gain in DB */
static const DECLARE_TLV_DB_SCALE(micpga_gain_tlv, 0, 50, 0);

/* Various Controls For AIC325x */
static const struct snd_kcontrol_new aic325x_snd_controls[] = {
	/* IN1L to HPL Volume Control */
	SOC_SINGLE("IN1L to HPL volume control", AIC3256_IN1L_HPL_CTRL,
						0, 0x72, 1),

	/* IN1R to HPR Volume Control */
	SOC_SINGLE("IN1R to HPR volume control", AIC3256_IN1R_HPR_CTRL,
						0, 0x72, 1),

	/* IN1L to HPL routing */
	SOC_SINGLE("IN1L to HPL Route", AIC3256_HPL_ROUTE_CTRL, 2, 1, 0),

	/* MAL output to HPL */
	SOC_SINGLE("MAL Output to HPL Route", AIC3256_HPL_ROUTE_CTRL, 1, 1, 0),

	/*MAR output to HPL */
	SOC_SINGLE("MAR Output to HPL Route", AIC3256_HPL_ROUTE_CTRL, 0, 1, 0),

	/* IN1R to HPR routing */
	SOC_SINGLE("IN1R to HPR Route", AIC3256_HPR_ROUTE_CTRL, 2, 1, 0),

	/* MAR to HPR routing */
	SOC_SINGLE("MAR Output to HPR Route", AIC3256_HPR_ROUTE_CTRL, 1, 1, 0),

	/* HPL Output to HRP routing */
	SOC_SINGLE("HPL Output to HPR Route", AIC3256_HPR_ROUTE_CTRL, 0, 1, 0),

	/* MAL Output to LOL routing*/
	SOC_SINGLE("MAL Output to LOL Route", AIC3256_LOL_ROUTE_CTRL, 1, 1, 0),

	/* LOR Output to LOL routing*/
	SOC_SINGLE("LOR Output to LOL Route", AIC3256_LOL_ROUTE_CTRL, 0, 1, 0),

	/* MAR Output to LOR routing*/
	SOC_SINGLE("MAR Outout to LOR Route", AIC3256_LOR_ROUTE_CTRL, 1, 1, 0),

	/* DRC Threshold value Control */
	SOC_SINGLE("DRC Threshold value",
					AIC3256_DRC_CTRL_REG1, 2, 0x07, 0),

	/* DRC Hysteresis value control */
	SOC_SINGLE("DRC Hysteresis value",
					AIC3256_DRC_CTRL_REG1, 0, 0x03, 0),

	/* DRC Hold time control */
	SOC_SINGLE("DRC hold time", AIC3256_DRC_CTRL_REG2, 3, 0x0F, 0),

	/* DRC Attack rate control */
	SOC_SINGLE("DRC attack rate", AIC3256_DRC_CTRL_REG3, 4, 0x0F, 0),

	/* DRC Decay rate control */
	SOC_SINGLE("DRC decay rate", AIC3256_DRC_CTRL_REG3, 0, 0x0F, 0),

	/* Beep Length MSB control */
	SOC_SINGLE("Beep Length MSB", AIC3256_BEEP_CTRL_REG3, 0, 255, 0),

	/* Beep Length MID control */
	SOC_SINGLE("Beep Length MID", AIC3256_BEEP_CTRL_REG4, 0, 255, 0),

	/* Beep Length LSB control */
	SOC_SINGLE("Beep Length LSB", AIC3256_BEEP_CTRL_REG5, 0, 255, 0),

	/* Beep Sin(x) MSB control */
	SOC_SINGLE("Beep Sin(x) MSB", AIC3256_BEEP_CTRL_REG6, 0, 255, 0),
	/* Beep Sin(x) LSB control */
	SOC_SINGLE("Beep Sin(x) LSB", AIC3256_BEEP_CTRL_REG7, 0, 255, 0),

	/* Beep Cos(x) MSB control */
	SOC_SINGLE("Beep Cos(x) MSB", AIC3256_BEEP_CTRL_REG8, 0, 255, 0),

	/* Beep Cos(x) LSB control */
	SOC_SINGLE("Beep Cos(x) LSB", AIC3256_BEEP_CTRL_REG9, 0, 255, 0),


	/* Left/Right DAC Digital Volume Control */
	SOC_DOUBLE_R_SX_TLV("DAC Digital Volume Control",
			AIC3256_LDAC_VOL, AIC3256_RDAC_VOL, 8, 0xffffff81, 0x30,
			dac_vol_tlv),

	/* Left/Right ADC Fine Gain Adjust */
	SOC_DOUBLE("L&R ADC Fine Gain Adjust", AIC3256_ADC_FGA, 4, 0, 0x04, 0),

	/* Left/Right ADC Volume Control */
	SOC_DOUBLE_R_SX_TLV("ADC Digital Volume Control",
		AIC3256_LADC_VOL, AIC3256_RADC_VOL, 7, 0xffffff68, 0x28 ,
						adc_vol_tlv),

	/*HP Driver Gain Control*/
	SOC_DOUBLE_R_SX_TLV("HP Driver Gain", AIC3256_HPL_GAIN,
					AIC3256_HPR_GAIN, 6, 0xfffffffa,
					0xe, output_gain_tlv),

	/*LO Driver Gain Control*/
	SOC_DOUBLE_R_SX_TLV("Line Driver Gain", AIC3256_LOL_GAIN,
					AIC3256_LOR_GAIN, 6,
					0xfffffffa, 0x1d , output_gain_tlv),


	/* Mixer Amplifier Volume Control */
	SOC_DOUBLE_R("Mixer_Amp_Vol_Ctrl",
			AIC3256_MAL_CTRL_REG, AIC3256_MAR_CTRL_REG,
			0, 0x28, 1),


	/*Left/Right MICPGA Volume Control */
	SOC_DOUBLE_R_TLV("L_R_MICPGA_Vol_Ctrl",
	AIC3256_LMICPGA_VOL_CTRL, AIC3256_RMICPGA_VOL_CTRL, 0, 0x5F,
			0, micpga_gain_tlv),

	/* Beep generator Volume Control */
	SOC_DOUBLE_R("Beep_gen_Vol_Ctrl",
			AIC3256_BEEP_CTRL_REG1, AIC3256_BEEP_CTRL_REG2,
			0, 0x3F, 1),

	/* Left/Right AGC Target level control */
	SOC_DOUBLE_R("AGC Target Level Control",
			AIC3256_LEFT_AGC_REG1, AIC3256_RIGHT_AGC_REG1,
			4, 0x07, 1),

	/* Left/Right AGC Hysteresis Control */
	SOC_DOUBLE_R("AGC Hysteresis Control",
			AIC3256_LEFT_AGC_REG1, AIC3256_RIGHT_AGC_REG1,
			0, 0x03, 0),

	/*Left/Right AGC Maximum PGA applicable */
	SOC_DOUBLE_R("AGC Maximum PGA Control",
			AIC3256_LEFT_AGC_REG3, AIC3256_RIGHT_AGC_REG3,
			0, 0x7F, 0),

	/* Left/Right AGC Noise Threshold */
	SOC_DOUBLE_R("AGC Noise Threshold",
			AIC3256_LEFT_AGC_REG2, AIC3256_RIGHT_AGC_REG2,
			1, 0x1F, 1),

	/* Left/Right AGC Attack Time control */
	SOC_DOUBLE_R("AGC Attack Time control",
			AIC3256_LEFT_AGC_REG4, AIC3256_RIGHT_AGC_REG4,
			3, 0x1F, 0),

	/* Left/Right AGC Decay Time control */
	SOC_DOUBLE_R("AGC Decay Time control",
			AIC3256_LEFT_AGC_REG5, AIC3256_RIGHT_AGC_REG5,
			3, 0x1F, 0),

	/* Left/Right AGC Noise Debounce control */
	SOC_DOUBLE_R("AGC Noice bounce control",
			AIC3256_LEFT_AGC_REG6, AIC3256_RIGHT_AGC_REG6,
			0, 0x1F, 0),

	/* Left/Right AGC Signal Debounce control */
	SOC_DOUBLE_R("AGC_Signal bounce ctrl",
		AIC3256_LEFT_AGC_REG7, AIC3256_RIGHT_AGC_REG7, 0, 0x0F, 0),

	/* DAC Signal Processing Block Control*/
	SOC_SINGLE("DAC PRB Selection(1 to 25)", AIC3256_DAC_PRB, 0, 0x19, 0),
	/* ADC Signal Processing Block Control */
	SOC_SINGLE("ADC PRB Selection(1 to 18)", AIC3256_ADC_PRB, 0, 0x12, 0),

	/*charge pump configuration for n/8 peak load current*/
	SOC_SINGLE("Charge_pump_peak_load_conf",
				AIC3256_CHRG_CTRL_REG, 4, 8, 0),

	/*charge pump clock divide control*/
	SOC_SINGLE("charge_pump_clk_divider_ctrl", AIC3256_CHRG_CTRL_REG,
			0, 16, 0),

	/*HPL, HPR master gain control in ground centerd mode */
	SOC_SINGLE("HP_gain_ctrl_gnd_centered_mode",
				AIC3256_HP_DRIVER_CONF_REG, 5, 3, 0),

	/*headphone amplifier compensation adjustment */
	SOC_SINGLE(" hp_amp_compensation_adjustment",
				AIC3256_HP_DRIVER_CONF_REG, 7, 1, 0),

	/*headphone driver power configuration*/
	SOC_SINGLE(" HP_drv_pwr_conf",
				AIC3256_HP_DRIVER_CONF_REG, 2, 4, 0),

	/*DC offset correction*/
	SOC_SINGLE("DC offset correction", AIC3256_HP_DRIVER_CONF_REG, 0, 4, 0),


	SOC_ENUM("Mic_Bias_Power_ctrl", micbias_pwr_ctrl_enum),

	SOC_ENUM("HP_gnd_centred_mode_ctrl", hp_gnd_centred_mode_ctrl),

};

/*
 *----------------------------------------------------------------------------
 * @struct  snd_soc_codec_dai_ops |
 *          It is SoC Codec DAI Operations structure
 *----------------------------------------------------------------------------
 */
struct snd_soc_dai_ops aic325x_dai_ops = {
	.hw_params = aic325x_hw_params,
	.digital_mute = aic325x_mute,
/*	.set_sysclk = aic325x_set_dai_sysclk, */
	.set_fmt = aic325x_set_dai_fmt,
	.set_pll = aic325x_dai_set_pll,
};

/*
 * It is SoC Codec DAI structure which has DAI capabilities viz., playback
 * and capture, DAI runtime information viz. state of DAI and pop wait state,
 * and DAI private data. The aic31xx rates ranges from 8k to 192k The PCM
 * bit format supported are 16, 20, 24 and 32 bits
 */


static struct snd_soc_dai_driver tlv320aic325x_dai_driver[] = {
	{
	.name = "tlv320aic325x-MM_EXT",
	.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = AIC325x_RATES,
			.formats = AIC325x_FORMATS,
		},
	.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = AIC325x_RATES,
			.formats = AIC325x_FORMATS,
		},
	.ops = &aic325x_dai_ops,
},

};

/* Left HPL Mixer */
static const struct snd_kcontrol_new hpl_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("L_DAC switch", AIC3256_HPL_ROUTE_CTRL, 3, 1, 0),
	SOC_DAPM_SINGLE("IN1_L switch", AIC3256_HPL_ROUTE_CTRL, 2, 1, 0),
	SOC_DAPM_SINGLE("MAL switch", AIC3256_HPL_ROUTE_CTRL, 1, 1, 0),
	SOC_DAPM_SINGLE("MAR switch", AIC3256_HPL_ROUTE_CTRL, 0, 1, 0),
};

static const char * const adc_mux_text[] = {
	"Analog", "Digital"
};

SOC_ENUM_SINGLE_DECL(adcl_enum, AIC3256_ADC_CHN_REG, 3, adc_mux_text);
SOC_ENUM_SINGLE_DECL(adcr_enum, AIC3256_ADC_CHN_REG, 2, adc_mux_text);

static const struct snd_kcontrol_new adcl_mux =
	SOC_DAPM_ENUM("Left ADC Route", adcl_enum);

static const struct snd_kcontrol_new adcr_mux =
	SOC_DAPM_ENUM("Right ADC Route", adcr_enum);

/* Right HPR Mixer */
static const struct snd_kcontrol_new hpr_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("L_DAC switch", AIC3256_HPR_ROUTE_CTRL, 4, 1, 0),
	SOC_DAPM_SINGLE("R_DAC switch", AIC3256_HPR_ROUTE_CTRL, 3, 1, 0),
	SOC_DAPM_SINGLE("IN1_R switch", AIC3256_HPR_ROUTE_CTRL, 2, 1, 0),
	SOC_DAPM_SINGLE("MAR switch", AIC3256_HPR_ROUTE_CTRL, 1, 1, 0),
};

/* Left Line out mixer */
static const struct snd_kcontrol_new lol_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("R_DAC switch", AIC3256_LOL_ROUTE_CTRL, 4, 1, 0),
	SOC_DAPM_SINGLE("L_DAC switch", AIC3256_LOL_ROUTE_CTRL, 3, 1, 0),
	SOC_DAPM_SINGLE("MAL switch", AIC3256_LOL_ROUTE_CTRL, 1, 1, 0),
	SOC_DAPM_SINGLE("LOR switch", AIC3256_LOL_ROUTE_CTRL, 0, 1, 0),
};
/* Right Line out Mixer */
static const struct snd_kcontrol_new lor_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("R_DAC switch", AIC3256_LOR_ROUTE_CTRL, 3, 1, 0),
	SOC_DAPM_SINGLE("MAR switch", AIC3256_LOR_ROUTE_CTRL, 1, 1, 0),
};

/* Left Input Mixer */
static const struct snd_kcontrol_new left_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("IN1_L switch", AIC3256_LMICPGA_PIN_CFG, 6, 3, 0),
	SOC_DAPM_SINGLE("IN2_L switch", AIC3256_LMICPGA_PIN_CFG, 4, 3, 0),
	SOC_DAPM_SINGLE("IN3_L switch", AIC3256_LMICPGA_PIN_CFG, 2, 3, 0),
	SOC_DAPM_SINGLE("IN1_R switch", AIC3256_LMICPGA_PIN_CFG, 0, 3, 0),

	SOC_DAPM_SINGLE("CM1L switch", AIC3256_LMICPGA_NIN_CFG, 6, 3, 0),
	SOC_DAPM_SINGLE("IN2_R switch", AIC3256_LMICPGA_NIN_CFG, 4, 3, 0),
	SOC_DAPM_SINGLE("IN3_R switch", AIC3256_LMICPGA_NIN_CFG, 2, 3, 0),
	SOC_DAPM_SINGLE("CM2L switch", AIC3256_LMICPGA_NIN_CFG, 0, 3, 0),
};

/* Right Input Mixer */
static const struct snd_kcontrol_new right_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("IN1_R switch", AIC3256_RMICPGA_PIN_CFG, 6, 3, 0),
	SOC_DAPM_SINGLE("IN2_R switch", AIC3256_RMICPGA_PIN_CFG, 4, 3, 0),
	SOC_DAPM_SINGLE("IN3_R switch", AIC3256_RMICPGA_PIN_CFG, 2, 3, 0),
	SOC_DAPM_SINGLE("IN2_L switch", AIC3256_RMICPGA_PIN_CFG, 0, 3, 0),
	SOC_DAPM_SINGLE("CM1R switch", AIC3256_RMICPGA_NIN_CFG, 6, 3, 0),
	SOC_DAPM_SINGLE("IN1_L switch", AIC3256_RMICPGA_NIN_CFG, 4, 3, 0),
	SOC_DAPM_SINGLE("IN3_L switch", AIC3256_RMICPGA_NIN_CFG, 2, 3, 0),
	SOC_DAPM_SINGLE("CM2R switch", AIC3256_RMICPGA_NIN_CFG, 0, 3, 0),
};

/**$
 * pll_power_on_event: provide delay after widget power up
 * @w: pointer variable to dapm_widget,
 * @kcontrolr: pointer variable to sound control,
 * @event:	integer to event,
 *
 * Return value: 0 for success
 */
static int pll_power_on_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	//struct snd_soc_codec *codec = w->codec;

	if (event == (SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD))
		mdelay(10);
	return 0;
}

/**
 *aic325x_dac_event: Headset popup reduction and powering up dsps together
 *			when they are in sync mode
 * @w: pointer variable to dapm_widget
 * @kcontrol: pointer to sound control
 * @event: event element information
 *
 * Returns 0 for success.
 */
static int aic325x_dac_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	int reg_mask = 0;
	int ret_wbits = 0;
	int run_state_mask;
	int sync_needed = 0, non_sync_state = 0;
	int other_dsp = 0, run_state = 0;
	struct aic325x_priv *aic325x = snd_soc_codec_get_drvdata(w->codec);

	if (w->shift == 7) {
		reg_mask = AIC3256_LDAC_POWER_STATUS_MASK ;
		run_state_mask = AIC3XXX_COPS_MDSP_D_L;
	}
	if (w->shift == 6) {
		reg_mask = AIC3256_RDAC_POWER_STATUS_MASK ;
		run_state_mask = AIC3XXX_COPS_MDSP_D_R ;
	}

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret_wbits = aic3xxx_wait_bits(w->codec->control_data,
						AIC3256_DAC_FLAG,
						reg_mask, reg_mask,
						AIC3256_TIME_DELAY,
						AIC3256_DELAY_COUNTER);
		sync_needed = aic3xxx_reg_read(w->codec->control_data,
						AIC3256_DAC_PRB);
			non_sync_state =
				dsp_non_sync_mode(aic325x->dsp_runstate);
			other_dsp =
				aic325x->dsp_runstate & AIC3XXX_COPS_MDSP_A;

		if (sync_needed && non_sync_state && other_dsp) {
			run_state =
				aic3256_get_runstate(
					aic325x->codec);
			aic3256_dsp_pwrdwn_status(aic325x->codec);
			aic3256_dsp_pwrup(aic325x->codec, run_state);
		}
		aic325x->dsp_runstate |= run_state_mask;
		if (!ret_wbits) {
			dev_err(w->codec->dev, "DAC_post_pmu timed out\n");
			return -1;
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		ret_wbits = aic3xxx_wait_bits(w->codec->control_data,
			AIC3256_DAC_FLAG, reg_mask, 0,
			AIC3256_TIME_DELAY, AIC3256_DELAY_COUNTER);
		aic325x->dsp_runstate =
			(aic325x->dsp_runstate & ~run_state_mask);
	if (!ret_wbits) {
		dev_err(w->codec->dev, "DAC_post_pmd timed out\n");
		return -1;
	}
		break;
	default:
		BUG();
		return -EINVAL;
	}
	return 0;
}

/**
 * aic325x_adc_event: To get DSP run state to perform synchronization
 * @w: pointer variable to dapm_widget
 * @kcontrol: pointer to sound control
 * @event: event element information
 *
 * Returns 0 for success.
 */
static int aic325x_adc_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{

	int run_state = 0;
	int non_sync_state = 0, sync_needed = 0;
	int other_dsp = 0;
	int run_state_mask = 0;
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(w->codec);
	int reg_mask = 0;
	int ret_wbits = 0;

	if (w->shift == 7) {
		reg_mask = AIC3256_LADC_POWER_MASK;
		run_state_mask = AIC3XXX_COPS_MDSP_A_L;
	}
	if (w->shift == 6) {
		reg_mask = AIC3256_RADC_POWER_MASK;
		run_state_mask = AIC3XXX_COPS_MDSP_A_R;
	}

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret_wbits = aic3xxx_wait_bits(w->codec->control_data,
						AIC3256_ADC_FLAG , reg_mask,
						reg_mask, AIC3256_TIME_DELAY,
						AIC3256_DELAY_COUNTER);

		sync_needed = aic3xxx_reg_read(w->codec->control_data,
						AIC3256_DAC_PRB);
		non_sync_state = dsp_non_sync_mode(aic3256->dsp_runstate);
		other_dsp = aic3256->dsp_runstate & AIC3XXX_COPS_MDSP_D;
		if (sync_needed && non_sync_state && other_dsp) {
			run_state = aic3256_get_runstate(
						aic3256->codec);
			aic3256_dsp_pwrdwn_status(aic3256->codec);
			aic3256_dsp_pwrup(aic3256->codec, run_state);
		}
		aic3256->dsp_runstate |= run_state_mask;
		if (!ret_wbits) {
			dev_err(w->codec->dev, "ADC POST_PMU timedout\n");
			return -1;
		}
		break;

	case SND_SOC_DAPM_POST_PMD:
		ret_wbits = aic3xxx_wait_bits(w->codec->control_data,
						AIC3256_ADC_FLAG, reg_mask, 0,
						AIC3256_TIME_DELAY,
						AIC3256_DELAY_COUNTER);
		aic3256->dsp_runstate = (aic3256->dsp_runstate &
					 ~run_state_mask);
		if (!ret_wbits) {
			dev_err(w->codec->dev, "ADC POST_PMD timedout\n");
			return -1;
		}
		break;

	default:
		BUG();
		return -EINVAL;
	}

	return 0;
}

/* AIC325x Widget Structure */
static const struct snd_soc_dapm_widget aic325x_dapm_widgets[] = {

	/* dapm widget (stream domain) for left DAC */
	SND_SOC_DAPM_DAC_E("Left DAC", "Left Playback", AIC3256_DAC_CHN_REG,
			7, 0, aic325x_dac_event, SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_POST_PMD),

	/* dapm widget (stream domain) for right DAC */
	SND_SOC_DAPM_DAC_E("Right DAC", "Right Playback", AIC3256_DAC_CHN_REG,
			6, 0, aic325x_dac_event, SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_POST_PMD),

	/* dapm widget (path domain) for left DAC_L Mixer */
	SND_SOC_DAPM_MIXER("HPL Output Mixer", SND_SOC_NOPM, 0, 0,
				&hpl_output_mixer_controls[0],
				ARRAY_SIZE(hpl_output_mixer_controls)),

	/* dapm widget (path domain) for right DAC_R mixer */
	SND_SOC_DAPM_MIXER("HPR Output Mixer", SND_SOC_NOPM, 0, 0,
	&hpr_output_mixer_controls[0], ARRAY_SIZE(hpr_output_mixer_controls)),

	/* dapm widget for Left Head phone Power */
	SND_SOC_DAPM_PGA_E("HPL PGA", AIC3256_OUT_PWR_CTRL, 5, 0, NULL, 0,
				aic325x_hp_event, 
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	/* dapm widget (path domain) for Left Line-out Output Mixer */
	SND_SOC_DAPM_MIXER("LOL Output Mixer", SND_SOC_NOPM, 0, 0,
	&lol_output_mixer_controls[0], ARRAY_SIZE(lol_output_mixer_controls)),

	/* dapm widget for Left Line-out Power */
	SND_SOC_DAPM_PGA_E("LOL PGA", AIC3256_OUT_PWR_CTRL, 3, 0, NULL, 0,
				NULL, SND_SOC_DAPM_POST_PMU),



	/* dapm widget for Right Head phone Power */
	SND_SOC_DAPM_PGA_E("HPR PGA", AIC3256_OUT_PWR_CTRL, 4, 0, NULL, 0,
				aic325x_hp_event,  
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	/* dapm widget for (path domain) Right Line-out Output Mixer */
	SND_SOC_DAPM_MIXER("LOR Output Mixer", SND_SOC_NOPM, 0, 0,
			&lor_output_mixer_controls[0],
			ARRAY_SIZE(lor_output_mixer_controls)),

	/* dapm widget for Right Line-out Power */
	SND_SOC_DAPM_PGA_E("LOR PGA", AIC3256_OUT_PWR_CTRL, 2, 0, NULL, 0,
				NULL, SND_SOC_DAPM_POST_PMU),

	/* dapm supply widget for Charge pump */
	SND_SOC_DAPM_SUPPLY("Charge Pump", AIC3256_POW_CFG, 1, 0, NULL,
						SND_SOC_DAPM_POST_PMU),
	/* Input DAPM widget for CM */
	SND_SOC_DAPM_INPUT("CM"),
	/* Input DAPM widget for CM1L */
	SND_SOC_DAPM_INPUT("CM1L"),
	/* Input DAPM widget for CM2L */
	SND_SOC_DAPM_INPUT("CM2L"),
	/* Input DAPM widget for CM1R */
	SND_SOC_DAPM_INPUT("CM1R"),
	/* Input DAPM widget for CM2R */
	SND_SOC_DAPM_INPUT("CM2R"),

	/* Stream widget for Left ADC */
	SND_SOC_DAPM_ADC_E("Left ADC", "Left Capture", AIC3256_ADC_CHN_REG,
			7, 0, aic325x_adc_event, SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_POST_PMD),


	/* Stream widget for Right ADC */
	SND_SOC_DAPM_ADC_E("Right ADC", "Right Capture", AIC3256_ADC_CHN_REG,
			6, 0, aic325x_adc_event, SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_POST_PMD),

	/* Left Inputs to Left MicPGA */
	SND_SOC_DAPM_PGA("Left MicPGA", AIC3256_LMICPGA_VOL_CTRL ,
			7, 1, NULL, 0),

	/* Right Inputs to Right MicPGA */
	SND_SOC_DAPM_PGA("Right MicPGA", AIC3256_RMICPGA_VOL_CTRL,
			7, 1, NULL, 0),

	/* Left MicPGA to Mixer PGA Left */
	SND_SOC_DAPM_PGA("MAL PGA", AIC3256_OUT_PWR_CTRL , 1, 0, NULL, 0),

	/* Right Inputs to Mixer PGA Right */
	SND_SOC_DAPM_PGA("MAR PGA", AIC3256_OUT_PWR_CTRL, 0, 0, NULL, 0),

	/* dapm widget for Left Input Mixer*/
	SND_SOC_DAPM_MIXER("Left Input Mixer", SND_SOC_NOPM, 0, 0,
			&left_input_mixer_controls[0],
			ARRAY_SIZE(left_input_mixer_controls)),

	/* dapm widget for Right Input Mixer*/
	SND_SOC_DAPM_MIXER("Right Input Mixer", SND_SOC_NOPM, 0, 0,
			&right_input_mixer_controls[0],
			ARRAY_SIZE(right_input_mixer_controls)),
	/* dapm widget (platform domain) name for HPLOUT */
	SND_SOC_DAPM_OUTPUT("HPL"),

	/* dapm widget (platform domain) name for HPROUT */
	SND_SOC_DAPM_OUTPUT("HPR"),

	/* dapm widget (platform domain) name for LOLOUT */
	SND_SOC_DAPM_OUTPUT("LOL"),

	/* dapm widget (platform domain) name for LOROUT */
	SND_SOC_DAPM_OUTPUT("LOR"),

	/* dapm widget (platform domain) name for LINE1L */
	SND_SOC_DAPM_INPUT("IN1_L"),

	/* dapm widget (platform domain) name for LINE1R */
	SND_SOC_DAPM_INPUT("IN1_R"),

	/* dapm widget (platform domain) name for LINE2L */
	SND_SOC_DAPM_INPUT("IN2_L"),

	/* dapm widget (platform domain) name for LINE2R */
	SND_SOC_DAPM_INPUT("IN2_R"),

	/* dapm widget (platform domain) name for LINE3L */
	SND_SOC_DAPM_INPUT("IN3_L"),

	/* dapm widget (platform domain) name for LINE3R */
	SND_SOC_DAPM_INPUT("IN3_R"),

	/* DAPM widget for MICBIAS power control */
	SND_SOC_DAPM_MICBIAS("Mic Bias", AIC3256_MICBIAS_CTRL, 6, 0),

	/* Left DMIC Input Widget */
	SND_SOC_DAPM_INPUT("Left DMIC"),
	/* Right DMIC Input Widget */
	SND_SOC_DAPM_INPUT("Right DMIC"),

	/* Left Channel ADC Route */
	SND_SOC_DAPM_MUX("Left ADC Route", SND_SOC_NOPM, 0, 0, &adcl_mux),
	/* Right Channel ADC Route */
	SND_SOC_DAPM_MUX("Right ADC Route", SND_SOC_NOPM, 0, 0, &adcr_mux),

	/* Supply widget for PLL */
	SND_SOC_DAPM_SUPPLY("PLLCLK", AIC3256_CLK_REG_2, 7, 0,
			pll_power_on_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	/* Supply widget for CODEC_CLK_IN */
	SND_SOC_DAPM_SUPPLY("CODEC_CLK_IN", SND_SOC_NOPM, 0, 0, NULL, 0),
	/* Supply widget for NDAC divider */
	SND_SOC_DAPM_SUPPLY("NDAC_DIV", AIC3256_NDAC_CLK_REG_6, 7, 0, NULL, 0),
	/* Supply widget for MDAC divider */
	SND_SOC_DAPM_SUPPLY("MDAC_DIV", AIC3256_MDAC_CLK_REG_7, 7, 0, NULL, 0),
	/* Supply widget for NADC divider */
	SND_SOC_DAPM_SUPPLY("NADC_DIV", AIC3256_NADC_CLK_REG_8, 7, 0, NULL, 0),
	/* Supply widget for MADC divider */
	SND_SOC_DAPM_SUPPLY("MADC_DIV", AIC3256_MADC_CLK_REG_9, 7, 0, NULL, 0),
	/* Supply widget for Bit Clock divider */
	SND_SOC_DAPM_SUPPLY("BCLK_N_DIV", AIC3256_CLK_REG_11, 7, 0, NULL, 0),
};

static const struct snd_soc_dapm_route aic325x_dapm_routes[] = {

	/* PLL routing */
	{"CODEC_CLK_IN", NULL, "PLLCLK"},
	{"NDAC_DIV", NULL, "CODEC_CLK_IN"},
	{"NADC_DIV", NULL, "CODEC_CLK_IN"},
	{"MDAC_DIV", NULL, "NDAC_DIV"},
	{"MADC_DIV", NULL, "NADC_DIV"},
	{"BCLK_N_DIV", NULL, "MADC_DIV"},
	{"BCLK_N_DIV", NULL, "MDAC_DIV"},

	/* Clock routing for ADC */
	{"Left ADC", NULL, "MADC_DIV"},
	{"Right ADC", NULL, "MADC_DIV"},

	/* Clock routing for DAC */
	{"Left DAC", NULL, "MDAC_DIV" },
	{"Right DAC", NULL, "MDAC_DIV"},

	/* Left Headphone Output */
	{"HPL Output Mixer", "L_DAC switch", "Left DAC"},
	{"HPL Output Mixer", "IN1_L switch", "IN1_L"},
	{"HPL Output Mixer", "MAL switch", "MAL PGA"},
	{"HPL Output Mixer", "MAR switch", "MAR PGA"},

	/* Right Headphone Output */
	{"HPR Output Mixer", "R_DAC switch", "Right DAC"},
	{"HPR Output Mixer", "IN1_R switch", "IN1_R"},
	{"HPR Output Mixer", "MAR switch", "MAR PGA"},
	{"HPR Output Mixer", "L_DAC switch", "Left DAC"},

	/* HP output mixer to HP PGA */
	{"HPL PGA", NULL, "HPL Output Mixer"},
	{"HPR PGA", NULL, "HPR Output Mixer"},

	/* HP PGA to HP output pins */
	{"HPL", NULL, "HPL PGA"},
	{"HPR", NULL, "HPR PGA"},

	/* Charge pump to HP PGA */
	{"HPL PGA", NULL, "Charge Pump"},
	{"HPR PGA", NULL, "Charge Pump"},

	/* Left Line-out Output */
	{"LOL Output Mixer", "L_DAC switch", "Left DAC"},
	{"LOL Output Mixer", "MAL switch", "MAL PGA"},
	{"LOL Output Mixer", "R_DAC switch", "Right DAC"},
	{"LOL Output Mixer", "LOR switch", "LOR PGA"},

	/* Right Line-out Output */
	{"LOR Output Mixer", "R_DAC switch", "Right DAC"},
	{"LOR Output Mixer", "MAR switch", "MAR PGA"},

	{"LOL PGA", NULL, "LOL Output Mixer"},
	{"LOR PGA", NULL, "LOR Output Mixer"},

	{"LOL", NULL, "LOL PGA"},
	{"LOR", NULL, "LOR PGA"},

	/* ADC portions */
	/* Left Positive PGA input */
	{"Left Input Mixer", "IN1_L switch", "IN1_L"},
	{"Left Input Mixer", "IN2_L switch", "IN2_L"},
	{"Left Input Mixer", "IN3_L switch", "IN3_L"},
	{"Left Input Mixer", "IN1_R switch", "IN1_R"},
	/* Left Negative PGA input */
	{"Left Input Mixer", "IN2_R switch", "IN2_R"},
	{"Left Input Mixer", "IN3_R switch", "IN3_R"},
	{"Left Input Mixer", "CM1L switch", "CM1L"},
	{"Left Input Mixer", "CM2L switch", "CM2L"},
	/* Right Positive PGA Input */
	{"Right Input Mixer", "IN1_R switch", "IN1_R"},
	{"Right Input Mixer", "IN2_R switch", "IN2_R"},
	{"Right Input Mixer", "IN3_R switch", "IN3_R"},
	{"Right Input Mixer", "IN2_L switch", "IN2_L"},
	/* Right Negative PGA Input */
	{"Right Input Mixer", "IN1_L switch", "IN1_L"},
	{"Right Input Mixer", "IN3_L switch", "IN3_L"},
	{"Right Input Mixer", "CM1R switch", "CM1R"},
	{"Right Input Mixer", "CM2R switch", "CM2R"},

	{"CM1L", NULL, "CM"},
	{"CM2L", NULL, "CM"},
	{"CM1R", NULL, "CM"},
	{"CM1R", NULL, "CM"},

	/* Left MicPGA */
	{"Left MicPGA", NULL, "Left Input Mixer"},

	/* Right MicPGA */
	{"Right MicPGA", NULL, "Right Input Mixer"},

	{"Left ADC", NULL, "Left MicPGA"},
	{"Right ADC", NULL, "Right MicPGA"},

	{"Left ADC Route", "Analog", "Left MicPGA"},
	{"Left ADC Route", "Digital", "Left DMIC"},

	/* Selection of Digital/Analog Mic */
	{"Right ADC Route", "Analog", "Right MicPGA"},
	{"Right ADC Route", "Digital", "Right DMIC"},

	{"Left ADC", NULL, "Left ADC Route"},
	{"Right ADC", NULL, "Right ADC Route"},

	{"MAL PGA", NULL, "Left MicPGA"},
	{"MAR PGA", NULL, "Right MicPGA"},
};




/* aic3256_firmware_load: This function is called by the
 *		request_firmware_nowait function as soon
 *		as the firmware has been loaded from the file.
 *		The firmware structure contains the data and$
 *		the size of the firmware loaded.
 * @fw: pointer to firmware file to be dowloaded
 * @context: pointer variable to codec
 *
 * Returns 0 for success.
 */
void aic3256_firmware_load(const struct firmware *fw, void *context)
{
	struct snd_soc_codec *codec = context;
	struct aic325x_priv *private_ds = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	aic3xxx_cfw_lock(private_ds->cfw_p, 1); /* take the lock */
	if (private_ds->cur_fw != NULL)
		release_firmware(private_ds->cur_fw);
	private_ds->cur_fw = NULL;

	if (fw != NULL)	{
		dev_dbg(codec->dev, "Firmware binary load\n");
		private_ds->cur_fw = (void *)fw;
		ret = aic3xxx_cfw_reload(private_ds->cfw_p, (void *)fw->data,
			fw->size);
		if (ret < 0) { /* reload failed */
			dev_err(codec->dev, "Firmware binary load failed\n");
			release_firmware(private_ds->cur_fw);
			private_ds->cur_fw = NULL;
			fw = NULL;
		}
	} else {
		dev_err(codec->dev, "Codec Firmware failed\n");
		ret = -1;
	}
	aic3xxx_cfw_lock(private_ds->cfw_p, 0); /* release the lock */
	if (ret >= 0) {
		/* init function for transition */
		aic3xxx_cfw_transition(private_ds->cfw_p, "INIT");
		aic3xxx_cfw_add_modes(codec, private_ds->cfw_p);
		aic3xxx_cfw_add_controls(codec, private_ds->cfw_p);
		aic3xxx_cfw_setmode_cfg(private_ds->cfw_p, 0, 0);
	}
}

/**
 * aic325x_hp_event: - To handle headphone related task before and after
 *			headphone powrup and power down
 * @w: pointer variable to dapm_widget
 * @kcontrol: mixer control
 * @event: event element information
 *
 * Returns 0 for success.
 */

#define AIC3256_DC_OFFSET_CORRECTION_MASK	(0b00000011)
#define AIC3256_DC_OFFSET_CORRECTION_ALL	(0b00000011)

#define AIC3256_GROUND_CENTER_HP_MASK	(0b00010000)
#define AIC3256_GROUND_CENTER_HP_EN	(0b00010000)
 
#define AIC3256_HP_GAIN_MASK		(0b00011111)
#define AIC3256_HP_GAIN_ZERO		(0b00000000)
#define AIC3256_HP_MUTE_MASK		(0b01000000)
  
#define AIC3256_HP_STAGE_MASK		(0b00001100)
#define AIC3256_HP_STAGE_100		(0)
#define AIC3256_HP_STAGE_75			(1)
#define AIC3256_HP_STAGE_50			(2)
#define AIC3256_HP_STAGE_25			(3)
#define AIC3256_HP_STAGE_SHIFT		(2)

#define AIC3256_HP_SELECT_MASK		(0b00000100)
#define AIC3256_HP_SELECT_HPL		(0b00000100)
#define AIC3256_HP_SELECT_HPR		(0b00000000)

#define AIC3256_HP_POWER_FLAG_MASK		(0b00000100)

#define AIC3256_TIME_DELAY					5
#define AIC3256_DELAY_COUNTER					100

#define AIC3256_HPL_POWER_MASK	0x20
#define AIC3256_HPR_POWER_MASK	0x10

#define AIC3256_HPL_POWER_STATUS_MASK	0x20
#define AIC3256_HPR_POWER_STATUS_MASK	0x02


static int aic325x_hp_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
#if 0
	u8 value;
	int counter;
	struct snd_soc_codec *codec = w->codec;
	if (event & SND_SOC_DAPM_POST_PMU) {
		counter = 200;
		do {
			value = snd_soc_read(codec, AIC3256_PWR_CTRL_REG);
			counter--;
		} while (counter && ((value & HP_DRIVER_BUSY_MASK) == 0));
	}
	if (event & SND_SOC_DAPM_POST_PMD)
		return 0;
return 0;
#else
	u8 value;
	int counter;
	struct snd_soc_codec *codec = w->codec;
	int hp_select;
	int ret_wbits = 0;
	int mute_reg = 0;
	int hpl_hpr = 0;
	int reg_mask = 0;
	
	if (w->shift == 5) {
		reg_mask = AIC3256_HPL_POWER_STATUS_MASK;
		hp_select = AIC3256_HP_SELECT_HPL;
		mute_reg = AIC3256_HPL_GAIN;
	}
	if (w->shift == 4) {
		reg_mask = AIC3256_HPR_POWER_STATUS_MASK;
		hp_select = AIC3256_HP_SELECT_HPR;
		mute_reg = AIC3256_HPR_GAIN;
	}
	
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
			printk("hp PRE PMU\n");
			snd_soc_update_bits(codec, AIC3256_POW_CFG,
				AIC3256_AVDD_CONNECTED_TO_DVDD_MASK,
				AIC3256_DISABLE_AVDD_TO_DVDD);
			snd_soc_update_bits(w->codec, AIC3256_HP_DRIVER_CONF_REG,
			AIC3256_DC_OFFSET_CORRECTION_MASK|AIC3256_GROUND_CENTER_HP_MASK,
			AIC3256_DC_OFFSET_CORRECTION_ALL|AIC3256_GROUND_CENTER_HP_EN);
			
			snd_soc_update_bits(w->codec, mute_reg,
			AIC3256_HP_GAIN_MASK, AIC3256_HP_GAIN_ZERO);
			
			snd_soc_update_bits(w->codec, AIC3256_HP_DRIVER_CONF_REG,
			AIC3256_HP_STAGE_MASK ,
			AIC3256_HP_STAGE_25 << AIC3256_HP_STAGE_SHIFT);
		break;

	case SND_SOC_DAPM_POST_PMU:
			printk("hp POST PMU\n");
			snd_soc_update_bits(w->codec, AIC3256_CM_CTRL_REG,
			AIC3256_HP_SELECT_MASK, hp_select);
			ret_wbits = aic3xxx_wait_bits(w->codec->control_data,
					      AIC3256_PWR_CTRL_REG, AIC3256_HP_POWER_FLAG_MASK,
					      AIC3256_HP_POWER_FLAG_MASK, AIC3256_TIME_DELAY,
					      AIC3256_DELAY_COUNTER);
			if (!ret_wbits) {
				dev_err(w->codec->dev, "HP POST_PMU timedout\n");
				return -1;
			}
			snd_soc_update_bits(codec, AIC3256_POW_CFG,
				AIC3256_AVDD_CONNECTED_TO_DVDD_MASK,
				AIC3256_ENABLE_AVDD_TO_DVDD);
			snd_soc_update_bits(w->codec, AIC3256_HP_DRIVER_CONF_REG,
			AIC3256_HP_STAGE_MASK ,
			AIC3256_HP_STAGE_100 << AIC3256_HP_STAGE_SHIFT);	
		
		break;

	case SND_SOC_DAPM_PRE_PMD:
			printk("hp PRE PMD\n");
			snd_soc_update_bits(codec, AIC3256_POW_CFG,
				AIC3256_AVDD_CONNECTED_TO_DVDD_MASK,
				AIC3256_DISABLE_AVDD_TO_DVDD);
			snd_soc_update_bits(w->codec, AIC3256_HP_DRIVER_CONF_REG,
			AIC3256_HP_STAGE_MASK ,
			AIC3256_HP_STAGE_25 << AIC3256_HP_STAGE_SHIFT);
			
			hpl_hpr = snd_soc_read(w->codec, AIC3256_OUT_PWR_CTRL);
			if ((hpl_hpr & 0x30) == 0x30) {
				snd_soc_update_bits(w->codec, AIC3256_OUT_PWR_CTRL,
							AIC3256_HPL_POWER_MASK, 0x0);
				mdelay(1);
				snd_soc_update_bits(w->codec, AIC3256_OUT_PWR_CTRL,
							AIC3256_HPR_POWER_MASK, 0x0);
			}
		break;

	case SND_SOC_DAPM_POST_PMD:
			printk("hp POST PMD\n");
			ret_wbits = aic3xxx_wait_bits(w->codec->control_data,
							  AIC3256_DAC_FLAG_1, reg_mask, 0,
							  AIC3256_TIME_DELAY,
							AIC3256_DELAY_COUNTER);
			if (!ret_wbits) {
				dev_err(w->codec->dev, "HP POST_PMD timedout\n");
				return -1;
			}
			snd_soc_update_bits(codec, AIC3256_POW_CFG,
				AIC3256_AVDD_CONNECTED_TO_DVDD_MASK,
				AIC3256_ENABLE_AVDD_TO_DVDD);
			snd_soc_update_bits(w->codec, mute_reg,	AIC3256_HP_MUTE_MASK, 0);		
		break;
	default:
			BUG();
		return -EINVAL;
	}	

	return 0;

#endif

}
/*
 *----------------------------------------------------------------------------
 * Function : __new_control_info
 * Purpose  : This function is to initialize data for new control required to
 *            program the AIC325x registers.
 *----------------------------------------------------------------------------
 */
static int __new_control_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 65535;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_get
 * Purpose  : This function is to read data of new control for
 *            program the AIC325x registers.
 *----------------------------------------------------------------------------
 */
static int __new_control_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u32 val;
	val = aic325x_codec_read(codec, aic325x_reg_ctl);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_put
 * Purpose  : new_control_put is called to pass data from user/application to
 *            the driver.
 *----------------------------------------------------------------------------
 */
static int __new_control_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct aic325x_priv *aic325x = snd_soc_codec_get_drvdata(codec);
	u32 data_from_user = ucontrol->value.integer.value[0];
	u8 data[2];

	aic325x_reg_ctl = data[0] = (u8) ((data_from_user & 0xFF00) >> 8);
	data[1] = (u8) ((data_from_user & 0x00FF));

	if (!data[0])
		aic325x->page_no = data[1];
	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		dev_err(codec->dev, "Error in i2c write\n");
		return -EIO;
	}
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic325x_hw_write
 * Purpose  : i2c write function
 *----------------------------------------------------------------------------
 */
int aic325x_hw_write(struct snd_soc_codec *codec, const char *buf,
						unsigned int count)
{
	//struct i2c_client *client = codec->control_data;
	u8 data[3];
	int ret;

	data[0] = *buf;
	data[1] = *(buf+1);
	data[2] = *(buf+2);

	ret = i2c_master_send(codec->control_data, data, count);

	if (ret < count) {
		printk(KERN_ERR "#%s: I2C write Error: bytes written = %d\n\n",
				__func__, ret);
		return -EIO;
	}
	return ret;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic325x_hw_read
 * Purpose  : i2c read function
 *----------------------------------------------------------------------------
 */
int aic325x_hw_read(struct snd_soc_codec *codec, unsigned int reg)
{
	struct i2c_client *client = codec->control_data;
	int data = 0;

	if (i2c_master_send(client, (char *)&reg, 1) < 0) {
		printk(KERN_ERR "%s: I2C write Error\n", __func__);
		return -EIO;
	}

	if (i2c_master_recv(client, (char *)&data, 1) < 0) {
		printk(KERN_ERR "%s: I2C read Error\n", __func__);
		return -EIO;
	}

	return data & 0x00FF;
}


/**
 * Methods for CFW Operations
 *
 * Due to incompatibilites between structures used by MFD and CFW
 * we need to transform the register format before linking to
 * CFW operations.
 */
static inline unsigned int aic3256_ops_cfw2reg(unsigned int reg)
{
	union cfw_register *c = (union cfw_register *) &reg;
	union aic3xxx_reg_union mreg;

	mreg.aic3xxx_register.offset = c->offset;
	mreg.aic3xxx_register.page = c->page;
	mreg.aic3xxx_register.reserved = 0;

	return mreg.aic3xxx_register_int;
}

static int aic3256_ops_reg_read(struct snd_soc_codec *codec, unsigned int reg)
{
	return aic3xxx_reg_read(codec->control_data, aic3256_ops_cfw2reg(reg));
}

static int aic3256_ops_reg_write(struct snd_soc_codec *codec, unsigned int reg,
					unsigned char val)
{
	return aic3xxx_reg_write(codec->control_data,
					aic3256_ops_cfw2reg(reg), val);
}

static int aic3256_ops_set_bits(struct snd_soc_codec *codec, unsigned int reg,
					unsigned char mask, unsigned char val)
{
	return aic3xxx_set_bits(codec->control_data,
				aic3256_ops_cfw2reg(reg), mask, val);

}

static int aic3256_ops_bulk_read(struct snd_soc_codec *codec, unsigned int reg,
					int count, u8 *buf)
{
	return aic3xxx_bulk_read(codec->control_data,
					aic3256_ops_cfw2reg(reg), count, buf);
}

static int aic3256_ops_bulk_write(struct snd_soc_codec *codec, unsigned int reg,
					int count, const u8 *buf)
{
	return aic3xxx_bulk_write(codec->control_data,
					aic3256_ops_cfw2reg(reg), count, buf);
}


/*
********************************************************************************
Function Name : aic3256_ops_dlock_lock
Argument      : pointer argument to the codec
Return value  : Integer
Purpose	      : To Read the run state of the DAC and ADC
by reading the codec and returning the run state

Run state Bit format

------------------------------------------------------
D31|..........| D7 | D6|  D5  |  D4  | D3 | D2 | D1  |   D0  |
R               R    R   LADC   RADC    R    R   LDAC   RDAC
------------------------------------------------------

********************************************************************************
*/
int aic3256_ops_lock(struct snd_soc_codec *codec)
{
	mutex_lock(&codec->mutex);
	/* Reading the run state of adc and dac */
	return aic3256_get_runstate(codec);
}

/*
*******************************************************************************
Function name	: aic3256_ops_dlock_unlock
Argument	: pointer argument to the codec
Return Value	: integer returning 0
Purpose		: To unlock the mutex acqiured for reading
run state of the codec
********************************************************************************
*/
int aic3256_ops_unlock(struct snd_soc_codec *codec)
{
	/*Releasing the lock of mutex */
	mutex_unlock(&codec->mutex);
	return 0;
}
/*
*******************************************************************************
Function Name	: aic3256_ops_dlock_stop
Argument	: pointer Argument to the codec
mask tells us the bit format of the
codec running state

Bit Format:
------------------------------------------------------
D31|..........| D7 | D6| D5 | D4 | D3 | D2 | D1 | D0 |
R               R    R   AL   AR    R    R   DL   DR
------------------------------------------------------
R  - Reserved
A  - minidsp_A
D  - minidsp_D
********************************************************************************
*/
int aic3256_ops_stop(struct snd_soc_codec *codec, int mask)
{
	int run_state = 0;

	run_state = aic3256_get_runstate(codec);

	if (mask & AIC3XXX_COPS_MDSP_A) /* power-down ADCs */
		aic3xxx_set_bits(codec->control_data,
					AIC3256_ADC_DATAPATH_SETUP, 0xC0, 0);

	if (mask & AIC3XXX_COPS_MDSP_D) /* power-down DACs */
		aic3xxx_set_bits(codec->control_data,
					AIC3256_DAC_DATAPATH_SETUP, 0xC0, 0);

	if ((mask & AIC3XXX_COPS_MDSP_A) &&
		!aic3xxx_wait_bits(codec->control_data, AIC3256_ADC_FLAG,
					AIC3256_ADC_POWER_MASK,
					0, AIC3256_TIME_DELAY,
					AIC3256_DELAY_COUNTER))
		goto err;

	if ((mask & AIC3XXX_COPS_MDSP_D) &&
		 !aic3xxx_wait_bits(codec->control_data, AIC3256_DAC_FLAG,
					AIC3256_DAC_POWER_MASK,	0,
					AIC3256_TIME_DELAY,
					AIC3256_DELAY_COUNTER))
			goto err;
	return run_state;

err:
	dev_err(codec->dev, "Unable to turn off ADCs or DACs at [%s:%d]",
				__FILE__, __LINE__);
	return -EINVAL;

}

/*
****************************************************************************
Function name	: aic3256_ops_dlock_restore
Argument	: pointer argument to the codec, run_state
Return Value	: integer returning 0
Purpose		: To unlock the mutex acqiured for reading
run state of the codec and to restore the states of the dsp
******************************************************************************
*/
static int aic3256_ops_restore(struct snd_soc_codec *codec, int run_state)
{
	int sync_state;

	/*	This is for read the sync mode register state */
	sync_state = aic3xxx_reg_read(codec->control_data, AIC3256_DAC_PRB);
	/* checking whether the sync mode has been set -
		- or not and checking the current state */
	if (((run_state & 0x30) && (run_state & 0x03)) && (sync_state & 0x80))
		aic3256_restart_dsps_sync(codec, run_state);
	else
		aic3256_dsp_pwrup(codec, run_state);

	return 0;
}
/**
 * aic3256_ops_adaptivebuffer_swap: To swap the coefficient buffers
 *                               of minidsp according to mask
 * @pv: pointer argument to the codec,
 * @mask: tells us which dsp has to be chosen for swapping
 *
 * Return Value    : returning 0 on success
 */
int aic3256_ops_adaptivebuffer_swap(struct snd_soc_codec *codec, int mask)
{
	const int sbuf[][2] = {
		{ AIC3XXX_ABUF_MDSP_A, AIC3256_ADC_ADAPTIVE_CRAM_REG },
		{ AIC3XXX_ABUF_MDSP_D1, AIC3256_DAC_ADAPTIVE_CRAM_REG},
		/* { AIC3XXX_ABUF_MDSP_D2, AIC3256_DAC_ADAPTIVE_BANK2_REG }, */
	};
	int i;

	for (i = 0; i < sizeof(sbuf)/sizeof(sbuf[0]); ++i) {
		if (!(mask & sbuf[i][0]))
			continue;
		aic3xxx_set_bits(codec->control_data, sbuf[i][1], 0x1, 0x1);
		if (!aic3xxx_wait_bits(codec->control_data,
			sbuf[i][1], 0x1, 0, 15, 1))
			goto err;
	}
	return 0;
err:
	dev_err(codec->dev, "miniDSP buffer swap failure at [%s:%d]",
				__FILE__, __LINE__);
	return -EINVAL;
}

/*****************************************************************************
Function name	: aic3256_get_runstate
Argument	: pointer argument to the codec
Return Value	: integer returning the runstate
Purpose		: To read the current state of the dac's and adc's
 ******************************************************************************/

static int aic3256_get_runstate(struct snd_soc_codec *codec)
{
	unsigned int dac, adc;
	/* Read the run state */
	dac = aic3xxx_reg_read(codec->control_data, AIC3256_DAC_FLAG);
	adc = aic3xxx_reg_read(codec->control_data, AIC3256_ADC_FLAG);

	return (((adc>>6)&1)<<5)  |
		(((adc>>2)&1)<<4) |
		(((dac>>7)&1)<<1) |
		(((dac>>3)&1)<<0);
}

/*****************************************************************************
Function name	: aic3256_dsp_pwrdwn_status
Argument	: pointer argument to the codec , cur_state of dac's and adc's
Return Value	: integer returning 0
Purpose		: To read the status of dsp's
 ******************************************************************************/

int aic3256_dsp_pwrdwn_status(
		struct snd_soc_codec *codec /* ptr to the priv data structure */
		)
{

	aic3xxx_set_bits(codec->control_data, AIC3256_ADC_DATAPATH_SETUP,
				0XC0, 0);
	aic3xxx_set_bits(codec->control_data, AIC3256_DAC_DATAPATH_SETUP,
				0XC0, 0);

	if (!aic3xxx_wait_bits(codec->control_data, AIC3256_ADC_FLAG,
			AIC3256_ADC_POWER_MASK, 0, AIC3256_TIME_DELAY,
			AIC3256_DELAY_COUNTER))
		goto err;

	if (!aic3xxx_wait_bits(codec->control_data, AIC3256_DAC_FLAG,
			AIC3256_DAC_POWER_MASK, 0, AIC3256_TIME_DELAY,
			AIC3256_DELAY_COUNTER))
		goto err;

	return 0;

err:
	dev_err(codec->dev, "DAC/ADC Power down timedout at [%s:%d]",
				__FILE__, __LINE__);
	return -EINVAL;

}

static int aic3256_dsp_pwrup(struct snd_soc_codec *codec, int state)
{
	int adc_reg_mask = 0;
	int adc_power_mask = 0;
	int dac_reg_mask = 0;
	int dac_power_mask = 0;
	int ret_wbits;

	if (state & AIC3XXX_COPS_MDSP_A_L) {
		adc_reg_mask	|= 0x80;
		adc_power_mask	|= AIC3256_LADC_POWER_MASK;
	}
	if (state & AIC3XXX_COPS_MDSP_A_R) {
		adc_reg_mask	|= 0x40;
		adc_power_mask	|= AIC3256_RADC_POWER_MASK;
	}

	if (state & AIC3XXX_COPS_MDSP_A)
		aic3xxx_set_bits(codec->control_data,
					AIC3256_ADC_DATAPATH_SETUP,
					0XC0, adc_reg_mask);

	if (state & AIC3XXX_COPS_MDSP_D_L) {
		dac_reg_mask	|= 0x80;
		dac_power_mask	|= AIC3256_LDAC_POWER_MASK;
	}

	if (state & AIC3XXX_COPS_MDSP_D_R) {
		dac_reg_mask	|= 0x40;
		dac_power_mask	|= AIC3256_RDAC_POWER_MASK;
	}

	if (state & AIC3XXX_COPS_MDSP_D)
		aic3xxx_set_bits(codec->control_data,
					AIC3256_DAC_DATAPATH_SETUP,
					0XC0, dac_reg_mask);

	if (state & AIC3XXX_COPS_MDSP_A) {
		ret_wbits = aic3xxx_wait_bits(codec->control_data,
				AIC3256_ADC_FLAG, AIC3256_ADC_POWER_MASK,
				adc_power_mask, AIC3256_TIME_DELAY,
			AIC3256_DELAY_COUNTER);
		if (!ret_wbits)
			dev_err(codec->dev, "ADC Power down timedout\n");
	}

	if (state & AIC3XXX_COPS_MDSP_D) {
		ret_wbits = aic3xxx_wait_bits(codec->control_data,
				AIC3256_DAC_FLAG, AIC3256_DAC_POWER_MASK,
				dac_power_mask, AIC3256_TIME_DELAY,
				AIC3256_DELAY_COUNTER);
		if (!ret_wbits)
			dev_err(codec->dev, "ADC Power down timedout\n");
	}

	return 0;
}

static int aic3256_restart_dsps_sync(struct snd_soc_codec *codec, int run_state)
{

	aic3256_dsp_pwrdwn_status(codec);
	aic3256_dsp_pwrup(codec, run_state);
	return 0;
}

static const struct aic3xxx_codec_ops aic3256_cfw_codec_ops = {
	.reg_read	=	aic3256_ops_reg_read,
	.reg_write	=	aic3256_ops_reg_write,
	.set_bits	=	aic3256_ops_set_bits,
	.bulk_read	=	aic3256_ops_bulk_read,
	.bulk_write	=	aic3256_ops_bulk_write,
	.lock		=	aic3256_ops_lock,
	.unlock		=	aic3256_ops_unlock,
	.stop		=	aic3256_ops_stop,
	.restore	=	aic3256_ops_restore,
	.bswap		=	aic3256_ops_adaptivebuffer_swap,
};

/**
 * aic325x_codec_read: provide read api to read aic3256 registe space
 * @codec: pointer variable to codec having codec information,
 * @reg: register address,
 *
 * Return: Return value will be value read.
 */
unsigned int aic325x_codec_read(struct snd_soc_codec *codec, unsigned int reg)
{
	u8 value;

	union aic3xxx_reg_union *aic_reg = (union aic3xxx_reg_union *)&reg;
	value = aic3xxx_reg_read(codec->control_data, reg);
	dev_dbg(codec->dev, "p %d, r 30 %x %x\n",
		aic_reg->aic3xxx_register.page,
		aic_reg->aic3xxx_register.offset, value);
	return value;
}

/**
 * aic325x_codec_write: provide write api to write at aic3256 registe space
 * @codec: Pointer variable to codec having codec information,
 * @reg: Register address,
 * @value: Value to be written to address space
 *
 * Return: Total no of byte written to address space.
 */
int aic325x_codec_write(struct snd_soc_codec *codec, unsigned int reg,
				unsigned int value)
{
	union aic3xxx_reg_union *aic_reg = (union aic3xxx_reg_union *)&reg;
	dev_dbg(codec->dev, "p %d, w 30 %x %x\n",
		aic_reg->aic3xxx_register.page,
		aic_reg->aic3xxx_register.offset, value);
	return aic3xxx_reg_write(codec->control_data, reg, value);
}

/**
 * aic325x_hw_params: This function is to set the hardware parameters
 *		for AIC3256.
 *		The functions set the sample rate and audio serial data word
 *		length.
 * @substream: pointer variable to sn_pcm_substream,
 * @params: pointer to snd_pcm_hw_params structure,
 * @dai: ponter to dai Holds runtime data for a DAI,
 *
 * Return: Return 0 on success.
 */
static int aic325x_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
					struct snd_soc_dai *dai)
{

	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct aic325x_priv *aic325x = snd_soc_codec_get_drvdata(codec);

	//int i, j, value;
	u8 data = 0;
	//unsigned int channels = params_channels(params);

	/* Setting the playback status */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		aic325x->playback_stream = 1;
	else if ((substream->stream !=
			SNDRV_PCM_STREAM_PLAYBACK) && (codec->active < 2))
		aic325x->playback_stream = 0;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		aic325x->record_stream = 1;
	else if ((substream->stream != SNDRV_PCM_STREAM_CAPTURE) && \
						(codec->active < 2))
		aic325x->record_stream = 0;

	//printk("%s record_stream %d  playback_stream %d\n",  __func__, aic325x->record_stream, aic325x->playback_stream);
	//dprintk(KERN_INFO "Function: %s, %d\n", __func__, aic325x->sysclk);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		data |= (0x00);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		data |= (0x08);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		data |= (0x10);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		data |= (0x18);
		break;
	}

	snd_soc_update_bits(codec, AIC3256_INTERFACE_SET_REG_1,
				AIC3256_INTERFACE_REG_MASK, data);

	return 0;
}

/**
 * aic325x_mute: This function is to mute or unmute the left and right DAC
 * @dai: ponter to dai Holds runtime data for a DAI,
 * @mute: integer value one if we using mute else unmute,
 *
 * Return: return 0 on success.
 */
static int aic325x_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct aic325x_priv *aic325x = snd_soc_codec_get_drvdata(codec);
	//dprintk(KERN_INFO "Function: %s\n", __func__);
	//printk("%s record_stream %d  playback_stream %d\n",  __func__, aic325x->record_stream, aic325x->playback_stream);
	if (mute) {
		if (aic325x->playback_stream &&  !dai->playback_active) {
	//dprintk(KERN_INFO "mute dac\n");
			if(gpio_get_value(TEGRA_GPIO_EN_CODEC_PA))
			{
				gpio_direction_output(TEGRA_GPIO_EN_CODEC_PA,  0);
				//printk("close spk pa\n");
			}
			snd_soc_update_bits(codec, AIC3256_DAC_MUTE_CTRL_REG,
						AIC3256_DAC_MUTE_MASK,
						AIC3256_DAC_MUTE_ON);
		}
		if (aic325x->record_stream  &&  !dai->capture_active) {
	//dprintk(KERN_INFO "mute adc\n");			
			snd_soc_update_bits(codec, AIC3256_ADC_FGA,
						AIC3256_ADC_MUTE_MASK,
						AIC3256_ADC_MUTE_ON);
		}
	} else {
		if (aic325x->playback_stream && (dai->playback_active > 0)) {
	//dprintk(KERN_INFO "unmute dac\n");				
			snd_soc_update_bits(codec, AIC3256_DAC_MUTE_CTRL_REG,
						AIC3256_DAC_MUTE_MASK,
						(~AIC3256_DAC_MUTE_ON));
		}
		if (aic325x->record_stream && (dai->capture_active> 0)) {
	//dprintk(KERN_INFO "unmute adc\n");					
			snd_soc_update_bits(codec, AIC3256_ADC_FGA,
						AIC3256_ADC_MUTE_MASK,
						(~AIC3256_ADC_MUTE_ON));
		}
	}
	//dprintk(KERN_INFO "Function: %s Exiting\n", __func__);

	return 0;
}

/**
 * aic325x_set_dai_fmt: This function is to set the DAI format
 * @codec_dai: ponter to dai Holds runtime data for a DAI,
 * @fmt: asi format info,
 *
 * return: return 0 on success.
 */
static int aic325x_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec;
	struct aic325x_priv *aic325x;

	u8 iface_reg;
	u8 iface_reg1;

	codec	= codec_dai->codec;
	aic325x	= snd_soc_codec_get_drvdata(codec);

	iface_reg = snd_soc_read(codec, AIC3256_INTERFACE_SET_REG_1);
	iface_reg = iface_reg & ~(3 << 6 | 3 << 2);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		aic325x->master = 1;
		iface_reg |= AIC3256_BIT_CLK_MASTER | AIC3256_WORD_CLK_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		aic325x->master = 0;
		iface_reg1 &= ~(AIC3256_BIT_CLK_MASTER |
					AIC3256_WORD_CLK_MASTER);
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		aic325x->master = 0;
		iface_reg1 |= AIC3256_BIT_CLK_MASTER;
		iface_reg1 &= ~(AIC3256_WORD_CLK_MASTER);
		break;
	default:
		printk(KERN_INFO "Invalid DAI master/slave interface\n");
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface_reg |= (AIC3256_DSP_MODE << AIC3256_CLK_REG_3_SHIFT);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		iface_reg |= (AIC3256_RIGHT_JUSTIFIED_MODE <<
				AIC3256_CLK_REG_3_SHIFT);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface_reg |= (AIC3256_LEFT_JUSTIFIED_MODE <<
					AIC3256_CLK_REG_3_SHIFT);
		break;
	default:
		printk(KERN_INFO "Invalid DAI interface format\n");
		return -EINVAL;
	}

	snd_soc_update_bits(codec, AIC3256_INTERFACE_SET_REG_1,
				AIC3256_INTERFACE_REG_MASK, iface_reg);
	return 0;
}

/**
 * aic325x_dai_set_pll: This function is to Set pll for aic3256 codec dai
 * @dai: ponter to dai Holds runtime data for a DAI, $
 * @pll_id: integer pll_id
 * @fin: frequency in,
 * @fout: Frequency out,
 *
 * Return: return 0 on success
*/
static int aic325x_dai_set_pll(struct snd_soc_dai *dai, int pll_id, int source,
				unsigned int Fin, unsigned int Fout)
{
	struct snd_soc_codec *codec = dai->codec;
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);
	//int ret;

	aic3xxx_cfw_set_pll(aic3256->cfw_p, dai->id);
	return 0;
}

/**
 *
 * aic325x_set_bias_level: This function is to get triggered
 *			 when dapm events occurs.
 * @codec: pointer variable to codec having informaton related to codec,
 * @level: Bias level-> ON, PREPARE, STANDBY, OFF.
 *
 * Return: Return 0 on success.
 */
static int aic325x_set_bias_level(struct snd_soc_codec *codec,
					enum snd_soc_bias_level level)
{
	//u8 value ;

	switch (level) {

	/* full On */
	case SND_SOC_BIAS_ON:
		/* all power is driven by DAPM system */
		break;

	/* partial On */
	case SND_SOC_BIAS_PREPARE:
		snd_soc_update_bits(codec, AIC3256_AIS_REG_2, 0, 0);
		break;

	/* Off, with power */
	case SND_SOC_BIAS_STANDBY:

		snd_soc_update_bits(codec, AIC3256_AIS_REG_3,
				AIC3256_BCLK_WCLK_BUFFER_POWER_CONTROL_MASK, 0);

		snd_soc_update_bits(codec, AIC3256_REF_PWR_UP_CONF_REG,
				AIC3256_REF_PWR_UP_MASK,
				AIC3256_FORCED_REF_PWR_UP);

		/*
		 * all power is driven by DAPM system,
		 * so output power is safe if bypass was set
		 */

		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			snd_soc_update_bits(codec, AIC3256_POW_CFG,
				AIC3256_AVDD_CONNECTED_TO_DVDD_MASK,
				AIC3256_DISABLE_AVDD_TO_DVDD);
			snd_soc_update_bits(codec, AIC3256_PWR_CTRL_REG,
					AIC3256_ANALOG_BLOCK_POWER_CONTROL_MASK,
					AIC3256_ENABLE_ANALOG_BLOCK);
			mdelay(40);
			//snd_soc_update_bits(codec, AIC3256_MICBIAS_CTRL, 0x40, 0x40);
			//snd_soc_update_bits(codec, AIC3256_HEADSET_DETECT, 0x80, 0x80);

		}
		break;

		/* Off, without power */
	case SND_SOC_BIAS_OFF:
		if (codec->dapm.bias_level == SND_SOC_BIAS_STANDBY) {
			//snd_soc_update_bits(codec, AIC3256_HEADSET_DETECT, 0x80, 0x0);
			//snd_soc_update_bits(codec, AIC3256_MICBIAS_CTRL, 0x40, 0x0);
			
			snd_soc_update_bits(codec, AIC3256_REF_PWR_UP_CONF_REG,
					AIC3256_REF_PWR_UP_MASK,
					AIC3256_AUTO_REF_PWR_UP);
			//snd_soc_update_bits(codec, AIC3256_PWR_CTRL_REG,
			//		AIC3256_ANALOG_BLOCK_POWER_CONTROL_MASK,
			//		AIC3256_DISABLE_ANALOG_BLOCK);
			snd_soc_update_bits(codec, AIC3256_POW_CFG,
				AIC3256_AVDD_CONNECTED_TO_DVDD_MASK,
				AIC3256_ENABLE_AVDD_TO_DVDD);
				
		}
	/* force all power off */
		break;
	}
	codec->dapm.bias_level = level;

	return 0;
}

/**
 *
 * aic325x_suspend; This function is to suspend the AIC3256 driver.
 * @codec: pointer variable to codec having informaton related to codec,
 *
 * Return: Return 0 on success.
 */
static int aic325x_suspend(struct snd_soc_codec *codec,pm_message_t states)
{
	//dprintk(KERN_INFO "Function: %s\n", __func__);
	aic325x_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}


/**
 * aic325x_resume: This function is to resume the AIC3256 driver
 *		 from off state to standby
 * @codec: pointer variable to codec having informaton related to codec,
 *
 * Return: Return 0 on success.
 */
static int aic325x_resume(struct snd_soc_codec *codec)
{
	aic325x_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}

enum headset_accessory_state {
	BIT_NO_ACCESSORY = 0,
	BIT_HEADSET = (1 << 0),
	BIT_HEADPHONE = (1 << 1),
};
#if 0
static void aic3256_hs_jack_report(struct snd_soc_codec *codec,
					struct snd_soc_jack *jack, int report)
{
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);
	int status, state = 0, switch_state = BIT_NO_ACCESSORY;

	mutex_lock(&aic3256->io_lock);

	/* Sync status */
	status = snd_soc_read(codec, AIC3256_HEADSET_DETECT);
	/* We will check only stereo MIC and headphone */
	switch (status & AIC3256_JACK_TYPE_MASK) {
	case AIC3256_JACK_WITH_MIC:
		state |= SND_JACK_HEADSET;
		break;
	case AIC3256_JACK_WITHOUT_MIC:
		state |= SND_JACK_HEADPHONE;
	}

	mutex_unlock(&aic3256->io_lock);

	snd_soc_jack_report(jack, state, report);

	if ((state & SND_JACK_HEADSET) == SND_JACK_HEADSET)
		switch_state |= BIT_HEADSET;
	else if (state & SND_JACK_HEADPHONE)
		switch_state |= BIT_HEADPHONE;

}
#endif

static void aic3256_hs_jack_report(struct snd_soc_codec *codec,
				   struct snd_soc_jack *jack, int report)
{
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);
	int status, state = 0;
	mutex_lock(&aic3256->io_lock);
	/* Sync status */
	status = snd_soc_read(codec, AIC3256_HEADSET_DETECT);
	printk(" ____________________  aic3256_hs_jack_report   status = 0x%x\n",status);
	/* We will check only stereo MIC and headphone */
	switch (status & AIC3256_JACK_TYPE_MASK) {
	case AIC3256_JACK_WITH_MIC:
		state |= SND_JACK_HEADSET;
		jack_status = 1;
		break;
	case AIC3256_JACK_WITHOUT_MIC:
		state |= SND_JACK_HEADPHONE;
		jack_status = 1;
		break;
	default:
		jack_status = 0;
		break;
		}	
	mutex_unlock(&aic3256->io_lock);
	printk("*******report state = %x, jack_status = %d\n", state, jack_status);
	snd_soc_jack_report(jack, state, report);

}
static void aic3256_hs_jack_insert_report(struct snd_soc_codec *codec,
				   struct snd_soc_jack *jack, int report)
{
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);
	int status = 0x71, state = 0;
	mutex_lock(&aic3256->io_lock);
	if((rescheduled == 0) && gpio_get_value(TEGRA_GPIO_M470_HP_DET)){	
		/*
		//status = snd_soc_read(codec, AIC3256_MICBIAS_CTRL);
		//printk(" %s *******************  AIC3256_MICBIAS_CTRL status = 0x%x \n",__func__,status);
		status = snd_soc_read(codec, AIC3256_HEADSET_DETECT);
		printk(" %s [2]*******************  AIC3256_HEADSET_DETECT status = 0x%x \n",__func__,status);
		//IPHONE5 headset 
		if((status != 0xb7) && (status != 0xf7)){
			//printk(" _________________  IPHONE\n");
			if(status == 0x97)
				status = 0xf7;
		}*/
		if(gpio_get_value(TEGRA_GPIO_M470_KEY_DET)){
			printk(" HIGH\n");
			status = 0xb7;
		}
		else{
			printk("LOW\n");
			status = 0xf7;
		}
		HeadsetIN = true;
	}
	if((rescheduled == 1) && gpio_get_value(TEGRA_GPIO_M470_HP_DET)){
		printk("____first detect:\n");
		rescheduled = 0;
		if(gpio_get_value(TEGRA_GPIO_M470_KEY_DET))
			printk(" ------HIGH\n");
		else
			printk("-------LOW\n");
		queue_delayed_work(aic3256->workqueue, &aic3256->delayed_work,
				   msecs_to_jiffies(5000));
	}
	else if(gpio_get_value(TEGRA_GPIO_M470_HP_DET) == 0){
		return ;
	}	
		
	/* We will check only stereo MIC and headphone */
	switch (status & AIC3256_JACK_TYPE_MASK) {
	case AIC3256_JACK_WITH_MIC:
		state |= SND_JACK_HEADSET;
		jack_status = 1;
		break;
	case AIC3256_JACK_WITHOUT_MIC:
		state |= SND_JACK_HEADPHONE;
		jack_status = 1;
		break;
	default:
		jack_status = 0;
		break;
		}	
	mutex_unlock(&aic3256->io_lock);
	printk(" %s *******************  status = 0x%x	jack_status = %d\n",__func__,status,jack_status);
	snd_soc_jack_report(jack, state, report);

}
static void aic3256_hs_jack_remove_report(struct snd_soc_codec *codec,
				   struct snd_soc_jack *jack, int report)
{
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);
	int status, state = 0;
	mutex_lock(&aic3256->io_lock);
	/* Sync status */
	status = 0;
	//printk(" %s _________________  state = %d   report = 0x%x	\n",__func__,state,report);
	/* We will check only stereo MIC and headphone */
	switch (status & AIC3256_JACK_TYPE_MASK) {
	case AIC3256_JACK_WITH_MIC:
		state |= SND_JACK_HEADSET;
		jack_status = 1;
		break;
	case AIC3256_JACK_WITHOUT_MIC:
		state |= SND_JACK_HEADPHONE;
		jack_status = 1;
		break;
	default:
		jack_status = 0;
		break;
		}	
	mutex_unlock(&aic3256->io_lock);
	snd_soc_jack_report(jack, state, report);

}

/**
 * aic3256_hs_jack_detect: Detect headphone jack during boot time
 * @codec: pointer variable to codec having information related to codec
 * @jack: Pointer variable to snd_soc_jack having information of codec
 *	     and pin number$
 * @report: Provides informaton of whether it is headphone or microphone
 *
*/
void aic3256_hs_jack_detect(struct snd_soc_codec *codec,
			struct snd_soc_jack *jack, int report)
{
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);
	struct aic3256_jack_data *hs_jack = &aic3256->hs_jack;
	printk("__________aic3256_hs_jack_detect:\n");
	hs_jack->jack = jack;
	hs_jack->report = report;
	if(his_hpdet){
		if(gpio_get_value(TEGRA_GPIO_M470_HP_DET)){
			printk("     begin detect:\n");
			rescheduled = 1;
			aic3256_hs_jack_insert_report(codec, hs_jack->jack, hs_jack->report);
		}
	}
	else	
		aic3256_hs_jack_report(codec, hs_jack->jack, hs_jack->report);
}
EXPORT_SYMBOL_GPL(aic3256_hs_jack_detect);
static void aic3256_accessory_work(struct work_struct *work)
{
	struct aic325x_priv *aic3256 = container_of(work,
						    struct aic325x_priv,
						    delayed_work.work);
	struct snd_soc_codec *codec = aic3256->codec;
	struct aic3256_jack_data *hs_jack = &aic3256->hs_jack;
	printk(" __________________   aic3256_accessory_work\n");
	if(his_hpdet){
		if(gpio_get_value(TEGRA_GPIO_M470_HP_DET))
			aic3256_hs_jack_insert_report(codec, hs_jack->jack, hs_jack->report);
		else
			aic3256_hs_jack_remove_report(codec, hs_jack->jack, hs_jack->report);		
	}
	else	
		aic3256_hs_jack_report(codec, hs_jack->jack, hs_jack->report);
}

static irqreturn_t aic3256_audio_handler(int irq, void *data)
{
	struct snd_soc_codec *codec = data;
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);
	struct aic3256_jack_data *hs_jack = &aic3256->hs_jack;
	if(his_hpdet){
		if(gpio_get_value(TEGRA_GPIO_M470_HP_DET)){
			aic3256_hs_jack_insert_report(codec, hs_jack->jack, hs_jack->report);
		}
		if(gpio_get_value(TEGRA_GPIO_M470_HP_DET) == 0){
			cancel_delayed_work_sync(&aic3256->delayed_work);
			aic3256_hs_jack_remove_report(codec, hs_jack->jack, hs_jack->report);	
		}
	}
	else
		queue_delayed_work(aic3256->workqueue, &aic3256->delayed_work,
				   msecs_to_jiffies(200));
	wake_lock_timeout(&aic3256->aic325x_irq_lock, HZ/2);
	return IRQ_HANDLED;
}
static void aic3256_reschedule_keydown_work(struct snd_soc_codec *codec,
					  unsigned long delay)
{
	unsigned long flags;
	spin_lock_irqsave(&codec->lock, flags);
	__cancel_delayed_work(&codec->keydown_work);
	schedule_delayed_work(&codec->keydown_work, delay);
	spin_unlock_irqrestore(&codec->lock, flags);
}
static void aic3256_reschedule_keyup_work(struct snd_soc_codec *codec,
					  unsigned long delay)
{
	unsigned long flags;
	spin_lock_irqsave(&codec->lock, flags);
	__cancel_delayed_work(&codec->keyup_work);
	schedule_delayed_work(&codec->keyup_work, delay);
	spin_unlock_irqrestore(&codec->lock, flags);
}

static void aic3256_reschedule_work(struct snd_soc_codec *codec,
					  unsigned long delay)
{
	unsigned long flags;
	spin_lock_irqsave(&codec->lock, flags);
	__cancel_delayed_work(&codec->dwork);
	schedule_delayed_work(&codec->dwork, delay);
	spin_unlock_irqrestore(&codec->lock, flags);
}
static void aic3256_keydown_handler(struct work_struct *work){
	struct snd_soc_codec *codec = container_of(work, struct snd_soc_codec, keydown_work.work);
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);
	unsigned int value;
	printk("aic3256_keydown_handler:\n");
	wake_lock_timeout(&aic3256->aic325x_wake_lock, HZ*1);
	if(his_hpdet){
		if(HeadsetIN == false)
			return;
		if(pressdown&&gpio_get_value(TEGRA_GPIO_M470_HP_DET)){
			if(gpio_get_value(TEGRA_GPIO_M470_KEY_DET))
			{
				schedule_delayed_work(&codec->keydown_work, msecs_to_jiffies(10));
			}
			else
			{
				pressdown = false;
				input_report_key(aic3256->idev, KEY_MEDIA, 1);
				input_sync(aic3256->idev);
				printk("**************************pressdown\n");
			}
		}
	}
}
static void aic3256_keyup_handler(struct work_struct *work){
	struct snd_soc_codec *codec = container_of(work, struct snd_soc_codec, keyup_work.work);
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);
	unsigned int value;
	wake_lock_timeout(&aic3256->aic325x_wake_lock, HZ/10);
	if(his_hpdet){
		if(HeadsetIN == false)
			return;
		if(pressup&&gpio_get_value(TEGRA_GPIO_M470_HP_DET)){
			pressup = false;
			input_report_key(aic3256->idev, KEY_MEDIA, 0);
			input_sync(aic3256->idev);
			printk("**************************pressup\n");
		}
	}
}
static void aic3256_work_handler(struct work_struct *work)
{
	struct snd_soc_codec *codec = container_of(work, struct snd_soc_codec, dwork.work);
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);
	unsigned int value;
	wake_lock_timeout(&aic3256->aic325x_wake_lock, HZ/10);
	value = snd_soc_read(codec, AIC3256_INT_FLAG2);
	if(value & 0b00100000)
	{
		schedule_delayed_work(&codec->dwork, msecs_to_jiffies(10));
	}
	else
	{
		input_report_key(aic3256->idev, KEY_MEDIA, 0);																										 
		input_sync(aic3256->idev);// Button press report to Android
		//printk("******button press up\n");
	}
}
static irqreturn_t aic3256_button_handler(int irq, void *data)
{
	struct snd_soc_codec *codec = data;
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);
	printk("_______________ aic3256_button_handler\n");
	if(his_hpdet){
		if(pressdown){
			aic3256_reschedule_keydown_work(data, msecs_to_jiffies(400));
			printk(" ___________________   aic3256_button_handler  pressdown = %d\n",pressdown);

		}
		if(pressup){
			aic3256_reschedule_keyup_work(data, msecs_to_jiffies(450));
			printk(" ___________________   aic3256_button_handler  pressup = %d\n",pressup);

		}
	}
	else{
		input_report_key(aic3256->idev, KEY_MEDIA, 1);
		input_sync(aic3256->idev);
		aic3256_reschedule_work(data, msecs_to_jiffies(10));
	}
	return IRQ_HANDLED;
}

//add by ych 20120118 for factory test[s]
static int snd_loopback_test_state = 0; //0--normal 1--ap headset loopback 2--ap mic&speaker 3--modem headset 4--modem mic&speaker

static ssize_t snd_loopback_test_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if(snd_loopback_test_state == 0)
	{
		return sprintf(buf,"%s\n","normal");
	}
	else if(snd_loopback_test_state == 1)
	{
		return sprintf(buf,"%s\n","ap_headset");
	}
	else if(snd_loopback_test_state == 2)
	{
		return sprintf(buf,"%s\n","ap_mic");
	}
	else if(snd_loopback_test_state == 3)
	{
		return sprintf(buf, "%s\n", "spk_play");
	}
	else if(snd_loopback_test_state == 4)
	{
		return sprintf(buf, "%s\n", "hp_play");
	}
	else if(snd_loopback_test_state == 5)
	{
		
		if(jack_status)
		{
			return sprintf(buf, "%s\n", "jack_in");
		}
		else 
		{
			return sprintf(buf, "%s\n", "jack_out");
		}
	}
	else if(snd_loopback_test_state == 6)
	{
		return sprintf(buf, "%s\n", "dump_regs");
	}
	return 0;
}

static int snd_loopback_test_set(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	//struct snd_soc_codec codec  = aic3262->codec;
	int counter;
	//printk("********* lp enter buf: %s*********\n",buf);

	if((strncmp(buf,"normal",6) == 0) && (snd_loopback_test_state != 0))
	{
		printk("***** normal test *****\n");
		if(snd_loopback_test_state == 1)
		{
			printk("***** close ap_headset test *****\n");
			snd_soc_write(aic3256_lb_codec, AIC3256_CLK_REG_2, 0x11);
			snd_soc_write(aic3256_lb_codec, AIC3256_NADC_CLK_REG_8, 0x02);
			snd_soc_write(aic3256_lb_codec, AIC3256_MADC_CLK_REG_9, 0x10);
			snd_soc_write(aic3256_lb_codec, AIC3256_DAC_MUTE_CTRL_REG, 0x00);
			snd_soc_write(aic3256_lb_codec, AIC3256_HEADSET_DETECT, 0x97);
			snd_soc_write(aic3256_lb_codec, AIC3256_BEEP_CTRL_REG4, 0x00);
			snd_soc_write(aic3256_lb_codec, AIC3256_ADC_CHN_REG, 0x00);
			//p1
			snd_soc_write(aic3256_lb_codec, AIC3256_POW_CFG, 0x00);
			snd_soc_write(aic3256_lb_codec, AIC3256_OUT_PWR_CTRL, 0x00);
			snd_soc_write(aic3256_lb_codec, AIC3256_HPL_ROUTE_CTRL, 0x00);
			snd_soc_write(aic3256_lb_codec, AIC3256_HPR_ROUTE_CTRL, 0x00);
			snd_soc_write(aic3256_lb_codec, AIC3256_HPL_GAIN, 0x40);
			snd_soc_write(aic3256_lb_codec, AIC3256_HPR_GAIN, 0x40);
			snd_soc_write(aic3256_lb_codec, AIC3256_LOL_GAIN, 0x40);
			snd_soc_write(aic3256_lb_codec, AIC3256_LOR_GAIN, 0x40);
			snd_soc_write(aic3256_lb_codec, AIC3256_MICBIAS_CTRL, 0x00);
			snd_soc_write(aic3256_lb_codec, AIC3256_LMICPGA_PIN_CFG, 0x00);
			snd_soc_write(aic3256_lb_codec, AIC3256_LMICPGA_NIN_CFG, 0x00);
			snd_soc_write(aic3256_lb_codec, AIC3256_RMICPGA_PIN_CFG, 0x00);
			snd_soc_write(aic3256_lb_codec, AIC3256_RMICPGA_NIN_CFG, 0x00);
			snd_soc_write(aic3256_lb_codec, AIC3256_LMICPGA_VOL_CTRL, 0x80);
			snd_soc_write(aic3256_lb_codec, AIC3256_RMICPGA_VOL_CTRL, 0x80);
		}
		else if(snd_loopback_test_state == 2)
		{
			printk("***** close ap_mic test *****\n");
			snd_soc_write(aic3256_lb_codec, AIC3256_CLK_REG_2, 0x11);
			snd_soc_write(aic3256_lb_codec, AIC3256_NADC_CLK_REG_8, 0x02);
			snd_soc_write(aic3256_lb_codec, AIC3256_MADC_CLK_REG_9, 0x10);
			snd_soc_write(aic3256_lb_codec, AIC3256_ADC_CHN_REG, 0x00);
			snd_soc_write(aic3256_lb_codec, AIC3256_OUT_PWR_CTRL, 0x00);
			snd_soc_write(aic3256_lb_codec, AIC3256_LOL_ROUTE_CTRL, 0x08);
			snd_soc_write(aic3256_lb_codec, AIC3256_LOR_ROUTE_CTRL, 0x08);
			snd_soc_write(aic3256_lb_codec, AIC3256_LOL_GAIN, 0x00);
			snd_soc_write(aic3256_lb_codec, AIC3256_LOR_GAIN, 0x00);
			snd_soc_write(aic3256_lb_codec, AIC3256_LMICPGA_PIN_CFG, 0x00);
			snd_soc_write(aic3256_lb_codec, AIC3256_LMICPGA_NIN_CFG, 0x00);
			snd_soc_write(aic3256_lb_codec, AIC3256_RMICPGA_PIN_CFG, 0x00);
			snd_soc_write(aic3256_lb_codec, AIC3256_RMICPGA_NIN_CFG, 0x00);
			snd_soc_write(aic3256_lb_codec, AIC3256_LMICPGA_VOL_CTRL, 0xc6);
			snd_soc_write(aic3256_lb_codec, AIC3256_RMICPGA_VOL_CTRL, 0xc6);
			gpio_direction_output(184,  0);
		}
		else if(snd_loopback_test_state == 3)
		{
			printk("***** close spk_play test *****\n");
			gpio_direction_output(184,  0);
			//p0
			snd_soc_write(aic3256_lb_codec, AIC3256_CLK_REG_2, 0x11);
			snd_soc_write(aic3256_lb_codec, AIC3256_NDAC_CLK_REG_6, 0x02);
			snd_soc_write(aic3256_lb_codec, AIC3256_MDAC_CLK_REG_7, 0x08);
			snd_soc_write(aic3256_lb_codec, AIC3256_DIN_CTL, 0x03);
			snd_soc_write(aic3256_lb_codec, AIC3256_DAC_CHN_REG, 0x14);
			snd_soc_write(aic3256_lb_codec, AIC3256_DAC_MUTE_CTRL_REG, 0x00);
			snd_soc_write(aic3256_lb_codec, AIC3256_BEEP_CTRL_REG4, 0x00);
			
			//p1
			snd_soc_write(aic3256_lb_codec, AIC3256_POW_CFG, 0x08);
			snd_soc_write(aic3256_lb_codec, AIC3256_OUT_PWR_CTRL, 0x0c);
			snd_soc_write(aic3256_lb_codec, AIC3256_LOL_ROUTE_CTRL, 0x08);
			snd_soc_write(aic3256_lb_codec, AIC3256_LOR_ROUTE_CTRL, 0x08);
			snd_soc_write(aic3256_lb_codec, AIC3256_HPL_GAIN, 0x01);
			snd_soc_write(aic3256_lb_codec, AIC3256_HPR_GAIN, 0x01);
			snd_soc_write(aic3256_lb_codec, AIC3256_LOL_GAIN, 0x06);
			snd_soc_write(aic3256_lb_codec, AIC3256_LOR_GAIN, 0x06);
			snd_soc_write(aic3256_lb_codec, AIC3256_MICBIAS_CTRL, 0x40);
			snd_soc_write(aic3256_lb_codec, AIC3256_LMICPGA_VOL_CTRL, 0xC6);
			snd_soc_write(aic3256_lb_codec, AIC3256_RMICPGA_VOL_CTRL, 0xC6);
		}
		else if(snd_loopback_test_state == 4)
		{
			printk("***** close hp_play test *****\n");

		}
		else if(snd_loopback_test_state == 5)
		{
			printk("***** close jack_detect *****\n");
		}
		else if(snd_loopback_test_state == 6)
		{
			printk("***** close spk_call_test *****\n");

			gpio_direction_output(99,  0);

		}
		else if(snd_loopback_test_state == 7)
		{
			printk("***** close hp_call_test *****\n");
		}
		else if(snd_loopback_test_state == 8)
		{
			printk("***** close ear_call_test *****\n");
		}
		snd_loopback_test_state = 0;
	}
	else if((strncmp(buf,"ap_headset",10) == 0) && (snd_loopback_test_state != 1))
	{
		printk("***** ap_headset test *****\n");
		snd_soc_write(aic3256_lb_codec, AIC3256_CLK_REG_2, 0x91);
		snd_soc_write(aic3256_lb_codec, AIC3256_NADC_CLK_REG_8, 0x82);
		snd_soc_write(aic3256_lb_codec, AIC3256_MADC_CLK_REG_9, 0x90);
		snd_soc_write(aic3256_lb_codec, AIC3256_DAC_MUTE_CTRL_REG, 0x0c);
		snd_soc_write(aic3256_lb_codec, AIC3256_HEADSET_DETECT, 0xF7);
		snd_soc_write(aic3256_lb_codec, AIC3256_BEEP_CTRL_REG4, 0x01);
		snd_soc_write(aic3256_lb_codec, AIC3256_ADC_CHN_REG, 0x80);
		//p1
		snd_soc_write(aic3256_lb_codec, AIC3256_POW_CFG, 0x08);
		snd_soc_write(aic3256_lb_codec, AIC3256_OUT_PWR_CTRL, 0x33);
		snd_soc_write(aic3256_lb_codec, AIC3256_HPL_ROUTE_CTRL, 0x02);
		snd_soc_write(aic3256_lb_codec, AIC3256_HPR_ROUTE_CTRL, 0x02);
		snd_soc_write(aic3256_lb_codec, AIC3256_HPL_GAIN, 0x00);
		snd_soc_write(aic3256_lb_codec, AIC3256_HPR_GAIN, 0x00);
		snd_soc_write(aic3256_lb_codec, AIC3256_LOL_GAIN, 0x06);
		snd_soc_write(aic3256_lb_codec, AIC3256_LOR_GAIN, 0x06);
		snd_soc_write(aic3256_lb_codec, AIC3256_MICBIAS_CTRL, 0x40);
		snd_soc_write(aic3256_lb_codec, AIC3256_LMICPGA_PIN_CFG, 0x01);
		snd_soc_write(aic3256_lb_codec, AIC3256_LMICPGA_NIN_CFG, 0x40);
		snd_soc_write(aic3256_lb_codec, AIC3256_RMICPGA_PIN_CFG, 0x40);
		snd_soc_write(aic3256_lb_codec, AIC3256_RMICPGA_NIN_CFG, 0x10);
		snd_soc_write(aic3256_lb_codec, AIC3256_LMICPGA_VOL_CTRL, 0x1E);
		snd_soc_write(aic3256_lb_codec, AIC3256_RMICPGA_VOL_CTRL, 0x22);
		snd_loopback_test_state = 1;		
	}
	else if((strncmp(buf,"ap_mic",6) == 0) && (snd_loopback_test_state != 2))
	{
		printk("***** ap_mic test *****\n");
		snd_soc_write(aic3256_lb_codec, AIC3256_CLK_REG_2, 0x91);
		snd_soc_write(aic3256_lb_codec, AIC3256_NADC_CLK_REG_8, 0x82);
		snd_soc_write(aic3256_lb_codec, AIC3256_MADC_CLK_REG_9, 0x90);
		snd_soc_write(aic3256_lb_codec, AIC3256_ADC_CHN_REG, 0x80);
		snd_soc_write(aic3256_lb_codec, AIC3256_OUT_PWR_CTRL, 0x0f);
		snd_soc_write(aic3256_lb_codec, AIC3256_LOL_ROUTE_CTRL, 0x02);
		snd_soc_write(aic3256_lb_codec, AIC3256_LOR_ROUTE_CTRL, 0x02);
		snd_soc_write(aic3256_lb_codec, AIC3256_LOL_GAIN, 0x00);
		snd_soc_write(aic3256_lb_codec, AIC3256_LOR_GAIN, 0x00);
		snd_soc_write(aic3256_lb_codec, AIC3256_MICBIAS_CTRL, 0x40);
		snd_soc_write(aic3256_lb_codec, AIC3256_LMICPGA_PIN_CFG, 0x10);
		snd_soc_write(aic3256_lb_codec, AIC3256_LMICPGA_NIN_CFG, 0x10);
		snd_soc_write(aic3256_lb_codec, AIC3256_RMICPGA_PIN_CFG, 0x10);
		snd_soc_write(aic3256_lb_codec, AIC3256_RMICPGA_NIN_CFG, 0x40);
		snd_soc_write(aic3256_lb_codec, AIC3256_LMICPGA_VOL_CTRL, 0x04);
		snd_soc_write(aic3256_lb_codec, AIC3256_RMICPGA_VOL_CTRL, 0x04);
		
		gpio_direction_output(184,  1);

		
		snd_loopback_test_state = 2;
	}
	else if((strncmp(buf,"spk_play", 8) == 0) && (snd_loopback_test_state != 3))
	{
		printk("***** spk_play test *****\n");
		//p0
		snd_soc_write(aic3256_lb_codec, AIC3256_CLK_REG_2, 0x91);
		snd_soc_write(aic3256_lb_codec, AIC3256_NDAC_CLK_REG_6, 0x82);
		snd_soc_write(aic3256_lb_codec, AIC3256_MDAC_CLK_REG_7, 0x88);
		snd_soc_write(aic3256_lb_codec, AIC3256_DIN_CTL, 0x02);
		snd_soc_write(aic3256_lb_codec, AIC3256_DAC_CHN_REG, 0xd4);
		snd_soc_write(aic3256_lb_codec, AIC3256_DAC_MUTE_CTRL_REG, 0x00);
		snd_soc_write(aic3256_lb_codec, AIC3256_BEEP_CTRL_REG4, 0x01);

		//p1
		snd_soc_write(aic3256_lb_codec, AIC3256_POW_CFG, 0x08);
		snd_soc_write(aic3256_lb_codec, AIC3256_OUT_PWR_CTRL, 0x0c);
		snd_soc_write(aic3256_lb_codec, AIC3256_LOL_ROUTE_CTRL, 0x08);
		snd_soc_write(aic3256_lb_codec, AIC3256_LOR_ROUTE_CTRL, 0x08);
		snd_soc_write(aic3256_lb_codec, AIC3256_HPL_GAIN, 0x01);
		snd_soc_write(aic3256_lb_codec, AIC3256_HPR_GAIN, 0x01);
		snd_soc_write(aic3256_lb_codec, AIC3256_LOL_GAIN, 0x06);
		snd_soc_write(aic3256_lb_codec, AIC3256_LOR_GAIN, 0x06);
		snd_soc_write(aic3256_lb_codec, AIC3256_MICBIAS_CTRL, 0x40);
		snd_soc_write(aic3256_lb_codec, AIC3256_LMICPGA_VOL_CTRL, 0xC6);
		snd_soc_write(aic3256_lb_codec, AIC3256_RMICPGA_VOL_CTRL, 0xC6);
		
		gpio_direction_output(184,  1);
		
		snd_loopback_test_state = 3;
	}
	else if((strncmp(buf,"hp_play", 7) == 0) && (snd_loopback_test_state != 4))
	{
		printk("***** hp_play test *****\n");
		snd_loopback_test_state = 4;
	}
	else if((strncmp(buf,"jack_detect", 11) == 0) && (snd_loopback_test_state != 5))
	{
		printk("****** jack_detect test *****\n");
		snd_loopback_test_state = 5;
	}
	else if(strncmp(buf,"dump_regs", 9) == 0) 
	{
		printk("****** dump_regs *****\n");

		printk(KERN_INFO "#Page0 REGS..\n");
		for (counter = MAKE_REG(0, 0); counter < MAKE_REG(0, 0) + 128; counter++) {
			printk(KERN_INFO "#%2d -> 0x%x\n", counter,
				snd_soc_read(aic3256_lb_codec, counter));
		}
	
		printk(KERN_INFO "#Page1 REGS..\n");
		for (counter = MAKE_REG(1, 0); counter < MAKE_REG(1, 0) + 128; counter++) {
			printk(KERN_INFO "#%2d -> 0x%x\n", (counter&0xff),
				snd_soc_read(aic3256_lb_codec, counter));
		}
#if 0
		printk(KERN_INFO "#Page3 REGS..\n");
		for (counter = MAKE_REG(3, 0); counter < MAKE_REG(3, 0) + 128; counter++) {
			printk(KERN_INFO "#%2d -> 0x%x\n", (counter&0xff),
				snd_soc_read(aic3256_lb_codec, counter));
		}
	
		printk(KERN_INFO "#Page4 REGS..\n");
		for (counter = MAKE_REG(4, 0); counter < MAKE_REG(4, 0) + 128; counter++) {
			printk(KERN_INFO "#%2d -> 0x%x\n",
				(counter&0xff), snd_soc_read(aic3256_lb_codec, counter));
		}
#endif
	}
	else if((strncmp(buf,"spk_call_test", 13) == 0) && (snd_loopback_test_state != 6))
	{
		printk("***** builtin-mic to spk test *****\n");
		snd_loopback_test_state = 6;
	}
	else if((strncmp(buf,"hp_call_test", 12) == 0) && (snd_loopback_test_state != 7))
	{
		printk("***** hp-mic to hp test *****\n");
		snd_loopback_test_state = 7;
	}
	else if((strncmp(buf,"ear_call_test", 13) == 0) && (snd_loopback_test_state != 8))
	{
		printk("***** mic to ear test *****\n");
		snd_loopback_test_state = 8;
	}

	return strlen(buf);
}

static DEVICE_ATTR(snd_lb_test, 0644, snd_loopback_test_show,snd_loopback_test_set);
//add by ych 20120118 for factory test [e]


/**
 * aic325x_probe: This is first driver function called by the SoC core driver.
 * @codec: pointer variable to codec having informaton related to codec,
 *
 * Return: Return 0 on success.
 */
static int aic325x_probe(struct snd_soc_codec *codec)
{
	int ret = 0;
	//struct i2c_adapter *adapter;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct aic325x_priv *aic325x;
	struct aic3256 *control;
	struct aic3256_jack_data *jack;
	struct aic3xxx *control2; 
	if (codec == NULL)
		dev_err(codec->dev, "codec pointer is NULL\n");

	codec->control_data = dev_get_drvdata(codec->dev->parent);

	control = codec->control_data;

	control2 = codec->control_data;

	aic325x = kzalloc(sizeof(struct aic325x_priv), GFP_KERNEL);
	if (aic325x == NULL)
		return -ENOMEM;
	
	mutex_init(&aic325x->io_lock);
	wake_lock_init(&aic325x->aic325x_wake_lock, WAKE_LOCK_SUSPEND,
			"aic325x_wake_lock");
	wake_lock_init(&aic325x->aic325x_irq_lock, WAKE_LOCK_SUSPEND,
			"aic325x_irq_lock");
	snd_soc_codec_set_drvdata(codec, aic325x);
	aic325x->pdata = dev_get_platdata(codec->dev->parent);
	aic325x->codec = codec;

	aic325x->cur_fw = NULL;

	aic325x->cfw_p = &(aic325x->cfw_ps);

	 aic3xxx_cfw_init(aic325x->cfw_p, &aic3256_cfw_codec_ops,
				aic325x->codec);
	aic325x->workqueue = create_singlethread_workqueue("aic3262-codec");
	if (!aic325x->workqueue) {
		ret = -ENOMEM;
		goto work_err;
	}
	INIT_DELAYED_WORK(&aic325x->delayed_work, aic3256_accessory_work);
	jack = &aic325x->hs_jack;
	aic325x->idev = input_allocate_device();
	if (aic325x->idev <= 0)
		printk("Allocate failed\n");

	input_set_capability(aic325x->idev, EV_KEY, KEY_MEDIA);
	ret = input_register_device(aic325x->idev);
	if (ret < 0) {
		dev_err(codec->dev, "register input dev fail\n");
		goto input_dev_err;
	}
	if (control2->irq) {
		ret = aic3xxx_request_irq(codec->control_data,
			AIC3256_IRQ_HEADSET_DETECT,
			aic3256_audio_handler, 0,
			"aic3256_irq_headset", codec);

		if (ret) {
			dev_err(codec->dev, "HEADSET detect irq request"
			"failed: %d\n", ret);
			goto irq_err;
		}

		ret = aic3xxx_request_irq(codec->control_data,
			AIC3256_IRQ_BUTTON_PRESS,
			aic3256_button_handler, 0, "aic3262_irq_button",
			codec);

		if (ret) {
			dev_err(codec->dev, "button press irq request"
			"failed: %d\n", ret);
			goto irq_err;
		}
	}
	spin_lock_init(&codec->lock);
	INIT_DELAYED_WORK(&codec->dwork, aic3256_work_handler);
	if(his_hpdet){
		INIT_DELAYED_WORK(&codec->keydown_work, aic3256_keydown_handler);
		INIT_DELAYED_WORK(&codec->keyup_work, aic3256_keyup_handler);
	}
	snd_soc_dapm_new_controls(dapm, aic325x_dapm_widgets,
				ARRAY_SIZE(aic325x_dapm_widgets));

	ret = snd_soc_dapm_add_routes(dapm, aic325x_dapm_routes,
				ARRAY_SIZE(aic325x_dapm_routes));
	if (!ret)
		dprintk("#Completed adding DAPM routes = %d\n",
			ARRAY_SIZE(aic325x_dapm_routes));


	ret = device_create_file(codec->dev, &dev_attr_snd_lb_test);
		if(ret)
			dev_info(codec->dev, "error creating sysfs files\n");
	aic3256_lb_codec = codec;// add by ych for factory_test 20120828



	/* firmware load */
	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				"tlv320aic3206_fw_v1.bin",
				codec->dev, GFP_KERNEL, codec,
				aic3256_firmware_load);

	return ret;
irq_err:
	input_unregister_device(aic325x->idev);
	input_free_device(aic325x->idev);
input_dev_err:
work_err:
	kfree(aic325x);
	return 0;

}

/*
 *----------------------------------------------------------------------------
 * Function : aic325x_remove
 * Purpose  : to remove aic325x soc device
 *
 *----------------------------------------------------------------------------
 */
static int aic325x_remove(struct snd_soc_codec *codec)
{
	/* power down chip */
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);
	//struct aic3xxx *control = codec->control_data;

	aic325x_set_bias_level(codec, SND_SOC_BIAS_OFF);

	if (aic3256->cur_fw != NULL)
		release_firmware(aic3256->cur_fw);

	kfree(aic3256);
	return 0;
}


static struct snd_soc_codec_driver soc_codec_driver_aic325x = {
	.probe = aic325x_probe,
	.remove = aic325x_remove,
	.suspend = aic325x_suspend,
	.resume = aic325x_resume,
	.read = aic325x_codec_read,
	.write = aic325x_codec_write,
	.set_bias_level = aic325x_set_bias_level,
	.controls = aic325x_snd_controls ,
	.num_controls = ARRAY_SIZE(aic325x_snd_controls),
	.reg_cache_size = 0,
	.reg_word_size = sizeof(u8),
	.reg_cache_default = NULL,
};

static int aic3256_probe(struct platform_device *pdev)
{
	int ret;
	ret = snd_soc_register_codec(
					&pdev->dev,
					&soc_codec_driver_aic325x,
					tlv320aic325x_dai_driver,
					ARRAY_SIZE(tlv320aic325x_dai_driver)
					);
	return ret;
}

static int aic3256_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver aic325x_codec_driver = {
	.driver = {
		.name = "tlv320aic325x-codec",
		.owner = THIS_MODULE,
	},
	.probe = aic3256_probe,
	.remove = __devexit_p(aic3256_remove),
};

/*
 *----------------------------------------------------------------------------
 * Function : tlv320aic3256_modinit
 * Purpose  : module init function. First function to run.
 *
 *----------------------------------------------------------------------------
 */
static int __init tlv320aic325x_modinit(void)
{
	return platform_driver_register(&aic325x_codec_driver);
}
module_init(tlv320aic325x_modinit);

/*
 *----------------------------------------------------------------------------
 * Function : tlv320aic3256_exit
 * Purpose  : module init function. First function to run.
 *
 *----------------------------------------------------------------------------
 */
static void __exit tlv320aic325x_exit(void)
{
	platform_driver_unregister(&aic325x_codec_driver);
}

module_exit(tlv320aic325x_exit);


MODULE_ALIAS("platform:tlv320aic325x-codec");
MODULE_DESCRIPTION("ASoC TLV320AIC325x codec driver");
MODULE_AUTHOR("Aravindan Muthukumar");
MODULE_AUTHOR("Suresh Pm");
MODULE_LICENSE("GPL");
