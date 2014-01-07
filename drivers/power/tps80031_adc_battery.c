/*
 * drivers/power/tps80031_adc_battery.c
 *
 * ADC battery driver for TI's tps80031
 *
 * Copyright (c) 2012, Hisense Corporation.
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
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/err.h>
#include <linux/regulator/machine.h>
#include <linux/mutex.h>

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>

#include <linux/mfd/core.h>
#include <linux/mfd/tps80031.h>
#include <linux/tps80031-charger.h>
#include <mach/thermal.h>

#define CHARGERUSB_CINLIMIT	0xee

#define CONTROLLER_STAT1	0xe3
#define TPS80031_BAT_TEMP_OV	BIT(0)
#define TPS80031_VBUS_DET	BIT(2)
#define TPS80031_VAC_DET	BIT(3)
#define TPS80031_FAULT_WDG	BIT(4)
#define TPS80031_CH_GATED	BIT(6)

#define CHARGERUSB_STATUS_INT1	0xe6
#define TPS80031_VBUS_OVP	BIT(0)
#define TPS80031_BAT_OVP	BIT(3)

#define LINEAR_CHARGE_STS	0xde
#define END_OF_CHARGE		BIT(5)
#define	TPS80031_VBATOV 	BIT(4)
#define TPS80031_VSYSOV		BIT(3)
#define	TPS80031_DPPM		BIT(2)
#define	TPS80031_CV_STS		BIT(1)
#define TPS80031_CC_STS 	BIT(0)
 
#define STS_HW_CONDITIONS	0x21
#define TOGGLE1			0x90
#define TOGGLE1_FGDITHS		BIT(7)
#define TOGGLE1_FGDITHR		BIT(6)
#define TOGGLE1_FGS		BIT(5)
#define	TOGGLE1_FGR		BIT(4)
#define TOGGLE1_GPADCR		BIT(1)
#define GPCH0_LSB		0x3b
#define GPCH0_MSB		0x3c
#define GPCH0_MSB_COLLISION_GP	BIT(4)
#define GPSELECT_ISB		0x35
#define GPADC_CTRL		0x2e
#define MISC1			0xe4
#define CTRL_P1			0x36
#define CTRL_P1_SP1		BIT(3)
#define CTRL_P1_EOCRT		BIT(2)
#define CTRL_P1_EOCP1		BIT(1)
#define CTRL_P1_BUSY		BIT(0)
#define TPS80031_STS_VYSMIN_HI	BIT(4)


//Fule Gauge Func for Current Cal
#define FG_REG_00		0xC0
#define FG_REG_00_CC_AUTOCLEAR	BIT(2)
#define FG_REG_00_CC_CAL_EN	BIT(1)
#define FG_REG_08		0xC8
#define FG_REG_09		0xC9
#define FG_REG_10		0xCA
#define FG_REG_11		0xCB

//To-Do@20121019,Get First Battery Voltage from bootloader(Refer to T96)
#define SAMPLE_TIMES 		3		
#define DRIVER_VERSION		"1.0.2"

#define WINDOW_T  (3*60)		//Don't allow capacity changes happen in 3 min when plug/unplug event happen
#if (defined(CONFIG_BOARD_M470)||defined(CONFIG_BOARD_M470BSD)||defined(CONFIG_BOARD_M470BSS))
#define CHARGER_THROSHOLD		2
#define CHARGER_THROSHOLD_WINDOW	1
#define CHARGER_THROSHOLD_MID		3
#else //For Phone
#define CHARGER_THROSHOLD		2
#define CHARGER_THROSHOLD_WINDOW	1
#define CHARGER_THROSHOLD_MID		3
#endif
static unsigned long chargerT=0; 
static unsigned long unchargerT=0;
static unsigned long long run_time = 0;
static int batt_continue_reverse_times = 0;

#define TEMP_BAD_VALUE		-10000
#if (defined(CONFIG_BOARD_M470)||defined(CONFIG_BOARD_M470BSD)||defined(CONFIG_BOARD_M470BSS))
#define BATTERY_REVERSE_LOW_POWER_BASE	120//120mV for Pad
#define BATTERY_CHRGING_TIME_94_95	900//15 Min
#define BATTERY_CHRGING_TIME_95_96	900//15 Min
#define BATTERY_CHRGING_TIME_96_97	1200//20 Min
#define BATTERY_CHRGING_TIME_97_98	1200//20 Min
/*
3.0~3.5V:1.28A
3.8~3.9V:1.2A
3.9~4.1V:1.0A ~ 500mA
accurate value:
1500 mA * (1 / 60) / 3500 = 0.00714 = 0.714 % ~~  (3 / 4)
1300 mA * (1 / 60) / 3500 = 0.00619 = 0.619 % ~~  (3 / 5)
1000 mA * (1 / 60) / 3500 = 0.00476 = 0.476 % ~~  (1 / 2)
safety value should be bigger for high-current user case,just consider 3/2 and 1/1
*/
#define BATTERY_CHRGING_CAP_PER_MIN_H	3/2//0~50%
#define BATTERY_CHRGING_CAP_PER_MIN_L	1//50~85% without any bracket,or it will be always zero
#define BATTERY_CHRGING_CAP_PER_MIN_H_O	1//0~50% for usb charging source
#define BATTERY_CHRGING_CAP_PER_MIN_L_O	2/3//50~85% for usb charging source
#else
#define BATTERY_REVERSE_LOW_POWER_BASE	100//100mV  for Phone
#define BATTERY_CHRGING_TIME_94_95	600//10 Min
#define BATTERY_CHRGING_TIME_95_96	600//10 Min
#define BATTERY_CHRGING_TIME_96_97	900//15 Min
#define BATTERY_CHRGING_TIME_97_98	1200//20 Min
#define BATTERY_CHRGING_CAP_PER_MIN_H	3/2//0~50%
#define BATTERY_CHRGING_CAP_PER_MIN_L	1  //50~85%
#endif
static struct timeval battery_current_time;

#define DBG_TPS8003X_FMT(fmt) "[BATT]" fmt
#ifdef CONFIG_BATTERY_ADC_TPS8003X_DEBUG
#define DBG_TPS8003X_INFO(fmt, ...) printk(KERN_DEBUG DBG_TPS8003X_FMT(fmt), ##__VA_ARGS__)
#else
#define DBG_TPS8003X_INFO(fmt, ...) (void)0
#endif

#ifdef	CONFIG_BOARD_US9230
#define	NO_BATT_NTC_VOL	840  //To-Do For MP NTC
#else
#define	NO_BATT_NTC_VOL	1250
#endif

extern enum charging_type tps8003x_charger_type(void);
extern bool tps8003x_vbus_status(void);
extern long adc_battery_charging_continue_time(void);

static int battery_ini_flag = 0;
extern unsigned int cap_of_battery;//Battery voltage from bootloader
static uint32_t  battery_full_time_check = 0;
static uint32_t  battery_95_start_time = 0;
static long   charging_one_half_time = 0;

static int adc_battery_temp = 200;
static bool battery_probe_done = false;
extern unsigned int tegra_power_reason;
#define State_Charger_PlugIn	0x00000800//AC usb plugin
#define State_Normal_USB_PlugIn	0x40000000//normal usb

#ifdef CONFIG_BATTERY_BQ27x00
extern bool battery_standalone_fg(void);
#endif


typedef struct 	{
	uint32_t vbatt;
	uint32_t cbatt_ac;
        uint32_t cbatt_usb;
        uint32_t cbatt_dis;
} BattMapInttoInt;


struct batt_voltage_array{
       int        initflag;
       int        update_index;
       uint32_t   vol[SAMPLE_TIMES];
};

static struct batt_voltage_array batt_vol_array;


struct fast_poll_vol{
       uint32_t   update_index;
       uint32_t   vol[SAMPLE_TIMES];
};

static struct fast_poll_vol fast_poll_array;



struct batt_time{
	 bool suspend_flag;
	 bool charging_in_s3;
	 uint32_t last_poll_time;
	 uint32_t poll_time;
	 uint32_t init_time;
	 uint32_t start_time;
};
/*we should allow get the true value when init*/
struct batt_time batt_sleep_time = {
	.suspend_flag = false,
	.charging_in_s3 = false,
	.last_poll_time = 0,
	.poll_time = 0,
	.init_time = 0,
	.start_time = 0,
};
#if (defined(CONFIG_BOARD_M470)||defined(CONFIG_BOARD_M470BSD)||defined(CONFIG_BOARD_M470BSS))
//same with M370
static int tps80031_temp_table[] = { 
	/* adc code for temperature in degree C */
	741, 737, 733, 730, 726, /* -5, -1 */
	721, 717, 713, 709, 704, 699, 695, 690, 685, 680, /* 00 - 09 */
	674, 669, 664, 658, 653, 647, 641, 635, 629, 623, /* 10 - 19 */
	617, 611, 605, 598, 592, 585, 579, 572, 565, 559, /* 20 - 29 */
	552, 545, 538, 531, 524, 517, 510, 503, 496, 489, /* 30 - 39 */
	482, 475, 468, 461, 454, 448, 441, 434, 427, 420, /* 40 - 49 */
	413, 407, 400, 393, 387, 380, 374, 367, 361, 354, /* 50 - 59 */
	348, 342, 336, 330, 324, 318, 312, 307, 301, 295, /* 60 - 69 */
};
#elif defined(CONFIG_BOARD_US9230)
static int tps80031_temp_table[] = {  //Version Pre-V0.00 for US9230 20121110(Just take 9202's table for example,should be update later)
	/* adc code for temperature in degree C */
	736, 731, 726, 721, 715, /* -5, -1 */
	710, 705, 699, 693, 688, 682, 676, 670, 664, 657, /* 00 - 09 */
	651, 645, 638, 632, 625, 619, 612, 605, 599, 592, /* 10 - 19 */
	585, 578, 572, 565, 558, 551, 545, 538, 531, 524, /* 20 - 29 */
	518, 511, 504, 498, 491, 485, 478, 472, 465, 459, /* 30 - 39 */
	453, 447, 441, 435, 429, 423, 417, 411, 406, 400, /* 40 - 49 */
	395, 389, 384, 379, 374, 369, 364, 359, 354, 350, /* 50 - 59 */
	345, 341, 336, 332, 328, 324, 320, 316, 312, 308 /* 60 - 69 */
};
#endif

#if (defined(CONFIG_BOARD_M470)||defined(CONFIG_BOARD_M470BSD)||defined(CONFIG_BOARD_M470BSS))
//To-Do Update for M470
static BattMapInttoInt batt_cap[] =
{
 { 4200, 96, 96, 100 },
 { 4190, 96, 96, 100 },
 { 4180, 95, 95, 100 },
 { 4170, 94, 95, 100 },
 { 4160, 93, 94, 100 },
 { 4150, 93, 94, 100 },
 { 4140, 92, 93, 100 },
 { 4130, 90, 93, 100 },
 { 4120, 89, 92, 100 },
 { 4110, 87, 92, 100 },
 { 4100, 85, 90, 100 },
 { 4090, 83, 89, 100 },
 { 4080, 81, 87, 100 },
 { 4070, 79, 86, 99 },
 { 4060, 77, 85, 99 },
 { 4050, 75, 84, 98 },
 { 4040, 73, 82, 95 },
 { 4030, 71, 81, 93 },
 { 4020, 68, 79, 92 },
 { 4010, 65, 77, 91 },
 { 4000, 63, 75, 90 },
 { 3990, 59, 73, 88 },
 { 3980, 56, 70, 87 },
 { 3970, 54, 69, 86 },
 { 3960, 51, 67, 85 },
 { 3950, 49, 65, 83 },
 { 3940, 47, 64, 82 },
 { 3930, 45, 61, 80 },
 { 3920, 43, 59, 78 },
 { 3910, 41, 57, 77 },
 { 3900, 39, 54, 75 },
 { 3890, 37, 51, 73 },
 { 3880, 35, 49, 71 },
 { 3870, 32, 47, 70 },
 { 3860, 30, 44, 69 },
 { 3850, 28, 40, 66 },
 { 3840, 27, 38, 65 },
 { 3830, 25, 35, 63 },
 { 3820, 23, 32, 61 },
 { 3810, 21, 30, 60 },
 { 3800, 20, 29, 58 },
 { 3790, 18, 25, 56 },
 { 3780, 17, 21, 53 },
 { 3770, 15, 18, 51 },
 { 3760, 14, 17, 49 },
 { 3750, 13, 15, 46 },
 { 3740, 11, 12, 42 },
 { 3730, 10, 12, 40 },
 { 3720, 10, 10, 37 },
 { 3710, 9, 10, 33 },
 { 3700, 9, 9, 28 },
 { 3690, 9, 9, 23 },
 { 3680, 6, 9, 21 },
 { 3670, 6, 8, 20 },
 { 3660, 6, 8, 18 },
 { 3650, 5, 7, 17 },
 { 3640, 5, 7, 17 },
 { 3630, 5, 6, 16 },
 { 3620, 5, 6, 16 },
 { 3610, 4, 6, 15 },
 { 3600, 4, 6, 14 },
 { 3590, 4, 6, 13 },
 { 3580, 4, 5, 12 },
 { 3570, 4, 5, 12 },
 { 3560, 4, 5, 11 },
 { 3550, 3, 5, 10 },
 { 3540, 3, 4, 10 },
 { 3530, 3, 4, 8 },
 { 3520, 3, 4, 8 },
 { 3510, 3, 4, 8 },
 { 3500, 2, 4, 7 },
 { 3490, 2, 4, 7 },
 { 3480, 2, 3, 7 },
 { 3470, 2, 3, 5 },
 { 3460, 2, 3, 5 },
 { 3450, 1, 2, 5 },
 { 3440, 1, 2, 5 },
 { 3430, 1, 2, 5 },
 { 3420, 1, 2, 4 },
 { 3410, 1, 2, 4 },
 { 3400, 0, 2, 4 },
 { 3390, 0, 1, 3 },
 { 3380, 0, 1, 3 },
 { 3370, 0, 1, 3 },
 { 3360, 0, 1, 2 },
 { 3350, 0, 1, 2 },
 { 3340, 0, 0, 2 },
 { 3330, 0, 0, 1 },
 { 3320, 0, 0, 1 },
 { 3310, 0, 0, 1 },
 { 3300, 0, 0, 0 },
};
#elif defined(CONFIG_BOARD_US9230)
//Version Pre-V0.04 for US9230 20121225(Dischraging Current up to 1200mA)
static BattMapInttoInt batt_cap[] =
{
 { 4200, 96, 96, 100 },
 { 4190, 96, 96, 100 },
 { 4180, 95, 95, 100 },
 { 4170, 94, 95, 100 },
 { 4160, 93, 94, 100 },
 { 4150, 93, 94, 100 },
 { 4140, 92, 93, 100 },
 { 4130, 90, 93, 100 },
 { 4120, 89, 92, 100 },
 { 4110, 87, 92, 100 },
 { 4100, 85, 90, 100 },
 { 4090, 83, 89, 100 },
 { 4080, 81, 87, 100 },
 { 4070, 79, 86, 99 },
 { 4060, 77, 85, 99 },
 { 4050, 75, 84, 98 },
 { 4040, 73, 82, 95 },
 { 4030, 71, 81, 93 },
 { 4020, 68, 79, 92 },
 { 4010, 65, 77, 91 },
 { 4000, 63, 75, 90 },
 { 3990, 61, 73, 89 },
 { 3980, 60, 70, 88 },
 { 3970, 57, 69, 85 },
 { 3960, 55, 67, 83 },
 { 3950, 53, 65, 82 },
 { 3940, 50, 64, 80 },
 { 3930, 47, 61, 78 },
 { 3920, 44, 59, 77 },
 { 3910, 40, 57, 75 },
 { 3900, 36, 54, 73 },
 { 3890, 32, 51, 72 },
 { 3880, 29, 49, 70 },
 { 3870, 25, 47, 69 },
 { 3860, 22, 44, 68 },
 { 3850, 20, 40, 66 },
 { 3840, 18, 38, 65 },
 { 3830, 16, 35, 63 },
 { 3820, 15, 32, 62 },
 { 3810, 13, 30, 60 },
 { 3800, 12, 29, 58 },
 { 3790, 10, 25, 57 },
 { 3780, 9, 21, 55 },
 { 3770, 7, 18, 52 },
 { 3760, 6, 17, 50 },
 { 3750, 5, 15, 46 },
 { 3740, 5, 12, 43 },
 { 3730, 5, 12, 42 },
 { 3720, 4, 10, 40 },
 { 3710, 4, 10, 38 },
 { 3700, 4, 9, 35 },
 { 3690, 4, 9, 32 },
 { 3680, 3, 9, 30 },
 { 3670, 3, 8, 29 },
 { 3660, 3, 8, 27 },
 { 3650, 3, 7, 25 },
 { 3640, 3, 7, 23 },
 { 3630, 3, 5, 21 },
 { 3620, 3, 5, 20 },
 { 3610, 2, 5, 18 },
 { 3600, 2, 4, 16 },
 { 3590, 2, 4, 15 },
 { 3580, 2, 4, 14 },
 { 3570, 2, 4, 14 },
 { 3560, 2, 2, 13 },
 { 3550, 2, 2, 13 },
 { 3540, 1, 2, 12 },
 { 3530, 1, 2, 11 },
 { 3520, 1, 1, 10 },
 { 3510, 1, 1, 9 },
 { 3500, 0, 1, 9 },
 { 3490, 0, 1, 7 },
 { 3480, 0, 0, 7 },
 { 3470, 0, 0, 5 },
 { 3460, 0, 0, 5 },
 { 3450, 0, 0, 3 },
 { 3440, 0, 0, 2 },
 { 3430, 0, 0, 2 },
 { 3420, 0, 0, 1 },
 { 3410, 0, 0, 1 },
 { 3400, 0, 0, 1 },
 { 3390, 0, 0, 0 },
 { 3380, 0, 0, 0 },
 { 3370, 0, 0, 0 },
 { 3360, 0, 0, 0 },
 { 3350, 0, 0, 0 },
 { 3340, 0, 0, 0 },
 { 3330, 0, 0, 0 },
 { 3320, 0, 0, 0 },
 { 3310, 0, 0, 0 },
 { 3300, 0, 0, 0 },
};
#endif

struct tps80031_battery_cache {
	uint8_t charging_status;
	uint8_t usb_online;
	uint8_t ac_online;
	uint8_t capacity_unplug;
	uint8_t capacity_plug;
	uint8_t capacity_sts;
	int32_t chg_current;
};



struct tps80031_device_info {
	struct device		*dev;
	struct i2c_client	*client;
	struct power_supply	bat;
	struct power_supply	ac;
#ifdef  CONFIG_TPS80032_USB_CHARGER
	struct power_supply	usb;
#endif
	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_battery;
	struct work_struct handler;
	bool need_updated_immediately;
	uint32_t poll_interval;	
	uint32_t poll_index;	
	uint32_t bat_vol;
	int32_t bat_temp;
	int32_t chg_current;
	uint8_t charging_status;
	uint8_t capacity_sts;
	struct 	tps80031_battery_cache cache;
	struct early_suspend early_suspend;
	
	uint8_t usb_online;
	uint8_t ac_online;
	struct mutex adc_lock;
	uint32_t vbat_max;
	uint32_t vbat_min;
	uint32_t slow_poll;
	uint32_t fast_poll;
	uint32_t fast_poll_vol;
	uint32_t shut_down_vol;
	uint32_t shut_down_lvl;
	uint32_t poll_max_times;
	uint32_t sleep_up_time;
	uint32_t adc_retry_times;
	uint32_t max_reverse_times;
	uint32_t init_adjust_vol;
	uint32_t init_vol_from_btloader;
	uint32_t battery_detect_by_ntc;
	uint32_t battery_present;
	int32_t  inter_resis;
};

static enum power_supply_property tps80031_bat_props[] = {
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_PRESENT,
#ifdef CONFIG_BATTERY_FUEL_GAUGE_DETECT
	POWER_SUPPLY_PROP_FUEL_GAUGE,
#endif
};
#ifdef   CONFIG_TPS80032_USB_CHARGER
static enum power_supply_property tps80031_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};
#endif

static enum power_supply_property tps80031_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int tps80031_reg_read(struct tps80031_device_info *di, int sid, int reg,
					uint8_t *val)
{
	int ret;

	ret = tps80031_read(di->dev->parent, sid, reg, val);
	if (ret < 0)
		dev_err(di->dev, "Failed read register 0x%02x\n",
					reg);
	return ret;
}

static int tps80031_reg_write(struct tps80031_device_info *di, int sid, int reg,
					uint8_t val)
{
	int ret;

	ret = tps80031_write(di->dev->parent, sid, reg, val);
	if (ret < 0)
		dev_err(di->dev, "Failed write register 0x%02x\n",
					reg);
	return ret;
}


static int tps80031_battery_voltage(struct tps80031_device_info *di)
{
	int voltage = 0;
	int i = 0;
	mutex_lock(&di->adc_lock);
	voltage = tps80031_gpadc_conversion(BATTERY_VOLTAGE);
	mutex_unlock(&di->adc_lock);
	while (voltage < 0)
	{
		dev_err(di->dev, "ADC Get Voltage Failed\n");
		i ++ ;
		voltage = tps80031_gpadc_conversion(BATTERY_VOLTAGE);
		if(i >= di->adc_retry_times)
		goto out;
	}

	voltage = ((voltage * 5000) / 4096);//DIV4/0xFFF/mV
out:
	return voltage;
}


//Get Battery Temperature for Charger
int battery_temp_by_adc(void)
{
	return adc_battery_temp;
}
EXPORT_SYMBOL_GPL(battery_temp_by_adc);

//Battery Charging current
static int tps80031_battery_current_now(struct tps80031_device_info *di)
{
	int charge;
	if(di->ac_online == 1 || di->usb_online  == 1) {
		mutex_lock(&di->adc_lock);
		charge = tps80031_gpadc_conversion(BATTERY_CHARGING_CURRENT);
		mutex_unlock(&di->adc_lock);
		if (charge < 0)
			return charge;
	}else
	charge = 0;

	return charge;
}


static int tps80031_battery_temp(struct tps80031_device_info *di)
{
	int adc_code, temp;
	int i = 0;
	mutex_lock(&di->adc_lock);
	adc_code = tps80031_gpadc_conversion(BATTERY_TEMPERATURE);
	mutex_unlock(&di->adc_lock);

	while (adc_code < 0)
	{
		dev_err(di->dev, "ADC Get Temperture Failed\n");
		i ++ ;
		adc_code = tps80031_gpadc_conversion(BATTERY_TEMPERATURE);
		if(i >= di->adc_retry_times)
		return  TEMP_BAD_VALUE;
	}

	adc_code = (adc_code * 1250) /4096;//mv

	if(di->battery_detect_by_ntc) {
		if(adc_code > NO_BATT_NTC_VOL)
			di->battery_present = 0;
		else
			di->battery_present = 1;
	}

	for (temp = 0; temp < ARRAY_SIZE(tps80031_temp_table); temp++) {
		if (adc_code >= tps80031_temp_table[temp])
			break;
	}
	/* first 5 values are for negative temperature */
	temp = (temp - 5) * 10; /* in tenths of degree Celsius */

	return  temp;
}

static int tps80031_battery_charging_status(struct tps80031_device_info *di)
{
	uint8_t status = di->charging_status;
	uint8_t chg_status;
	uint8_t chg_usb_status;
	uint8_t con_status;
	int ret;
#ifdef   CONFIG_TPS80032_USB_CHARGER
	if(di->ac_online == 1 || di->usb_online == 1) {
#else
	if(di->ac_online == 1) {
#endif
		//CC/CV status doesn't work well
		ret = tps80031_reg_read(di, SLAVE_ID2, CONTROLLER_STAT1, &con_status);
		if (ret < 0){
			dev_err(di->dev,"%s Get CONTROLLER_STAT1 Failed\n",__func__);
			goto  out;
		}
		
		if(con_status & TPS80031_VBUS_DET) {
			if( (con_status & TPS80031_BAT_TEMP_OV ) || (con_status & TPS80031_FAULT_WDG))
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			else
			{
				ret = tps80031_reg_read(di, SLAVE_ID2, CHARGERUSB_STATUS_INT1, &chg_usb_status);
				if (ret < 0){
					dev_err(di->dev,"%s Get CHARGERUSB_STATUS_INT1 Failed\n",__func__);
					goto  out;
				}

				ret = tps80031_reg_read(di, SLAVE_ID2, LINEAR_CHARGE_STS, &chg_status);
				if (ret < 0){
					dev_err(di->dev,"%s Get LINEAR_CHARGE_STS Failed\n",__func__);
					goto  out;
				}
				if(chg_status & END_OF_CHARGE)
				status = POWER_SUPPLY_STATUS_FULL;
				else if( ((chg_status & TPS80031_VBATOV)||(chg_status & TPS80031_VSYSOV)) ||
					 ((chg_usb_status & TPS80031_VBUS_OVP)|| (chg_usb_status & TPS80031_BAT_OVP)))
				status = POWER_SUPPLY_STATUS_NOT_CHARGING;
				else
				status = POWER_SUPPLY_STATUS_CHARGING;	
			}
		}
		else
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	else
	status = POWER_SUPPLY_STATUS_DISCHARGING;

out:
	if(status == POWER_SUPPLY_STATUS_UNKNOWN) {
#ifdef   CONFIG_TPS80032_USB_CHARGER
		if(di->ac_online == 1 || di->usb_online == 1)
#else
		if(di->ac_online == 1)
#endif 
			status = POWER_SUPPLY_STATUS_CHARGING;	
 		else
			status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	
	return status;
}

//Fuel Gauge Func
static int tps8003x_current_setup(struct tps80031_device_info *di,bool enable)
{
	int 	ret = 0;
	uint8_t	reg = 0;

	//Clear
	ret = tps80031_reg_write(di, SLAVE_ID2, FG_REG_00,FG_REG_00_CC_AUTOCLEAR);

	//Enable Fule Gauge

	if (enable)
		reg = TOGGLE1_FGDITHS | TOGGLE1_FGS;
	else
		reg = TOGGLE1_FGDITHR | TOGGLE1_FGR;


	ret = tps80031_update(di->dev->parent, SLAVE_ID2,
			TOGGLE1, reg, 0xF0);
	if (ret)
		return ret;

	//Enables calibration (We choose 250-ms update rate,4 times result per second)
	ret = tps80031_reg_write(di, SLAVE_ID2, FG_REG_00,FG_REG_00_CC_CAL_EN);

	return ret;
}

static void tps8003x_battery_fg_current(struct tps80031_device_info *di)
{
	int ret = 0;
	int current_now = 0;
	uint16_t offset_raw_value = 0;
	uint16_t curr_raw_value = 0;

	int16_t  temp = 0;
	int16_t  cc_offset = 0;

	/* FG_REG_08, 09 is 10 bit signed calibration offset value */
	ret = tps80031_reads(di->dev->parent, SLAVE_ID2, FG_REG_08, 2,
							(uint8_t *) &offset_raw_value);
	if (ret < 0) {
		dev_err(di->dev, "failed to read FG_REG_8: current off set\n");
		return;
	}
	cc_offset = ((int16_t)(offset_raw_value << 6) >> 6);

	/* FG_REG_10, 11 is 14 bit signed instantaneous current sample value */
	ret = tps80031_reads(di->dev->parent, SLAVE_ID2, FG_REG_10, 2,
							(uint8_t *) &curr_raw_value);
	if (ret < 0) {
		dev_err(di->dev, "failed to read FG_REG_10: current_now\n");
		return;
	}
	temp = ((int16_t)(curr_raw_value << 2) >> 2);

	current_now = temp - cc_offset;

	/* current drawn per sec */
	current_now = current_now * 1;//N = 1 for an integration period of 250 ms.
	/* current in mA*/
	current_now = (current_now * (62000/25)) >> 13; //R2 should be 20, but now is about 25

	di->chg_current = current_now;

	return;
}



#define to_tps80031_device_info_bat(x) container_of((x), \
				struct tps80031_device_info, bat);


/*
static int tps80031_battery_charge_now(struct tps80031_device_info *di,
			union power_supply_propval *val)  //This function should be calc by GasGauge to get battery remain capacity,
{
	int charge;
	if(di->ac_online == 1 || di->usb_online  == 1) {
		mutex_lock(&di->adc_lock);
		charge = tps80031_gpadc_conversion(BATTERY_CHARGING_CURRENT);
		mutex_unlock(&di->adc_lock);
		if (charge < 0)
			return charge;
		charge = charge * 1000; //uA
	}else
	charge = 0;
	return charge;
}*/

static int tps80031_battery_present(struct tps80031_device_info *di,
			union power_supply_propval *val)
{
	//To-Do Detect BATREMOVAL pin status by AP(GPADC_IN0 Doesn't work according to current design)
	if(di->battery_detect_by_ntc)
		return di->battery_present;
	else
		return 1;

}

static int tps80031_battery_health(struct tps80031_device_info *di,
					union power_supply_propval *val)
{
	uint8_t status = POWER_SUPPLY_HEALTH_GOOD;
	uint8_t chg_status;
	uint8_t chg_usb_status;
	uint8_t con_status;
	int ret;

	ret = tps80031_reg_read(di, SLAVE_ID2, CONTROLLER_STAT1, &con_status);
	if (ret < 0){
		dev_err(di->dev,"%s Get CONTROLLER_STAT1 Failed\n",__func__);
		goto  out;
	}

	ret = tps80031_reg_read(di, SLAVE_ID2, LINEAR_CHARGE_STS, &chg_status);
	if (ret < 0){
		dev_err(di->dev,"%s Get LINEAR_CHARGE_STS Failed\n",__func__);
		goto  out;
	}

	ret = tps80031_reg_read(di, SLAVE_ID2, CHARGERUSB_STATUS_INT1, &chg_usb_status);
	if (ret < 0){
		dev_err(di->dev,"%s Get LINEAR_CHARGE_STS Failed\n",__func__);
		goto  out;
	}
	if(di->battery_detect_by_ntc) {
		if((con_status & TPS80031_BAT_TEMP_OV) && di->battery_present)
			status = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if((chg_status & TPS80031_VBATOV)||(chg_usb_status & TPS80031_BAT_OVP))
			status = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else
		status = POWER_SUPPLY_HEALTH_GOOD;

	}
	else {
		if(con_status & TPS80031_BAT_TEMP_OV)
			status = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if((chg_status & TPS80031_VBATOV)||(chg_usb_status & TPS80031_BAT_OVP))
			status = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else
		status = POWER_SUPPLY_HEALTH_GOOD;
	}
		

	return status;
out:	
	
	//if(val->intval == POWER_SUPPLY_HEALTH_UNKNOWN)
	//	val->intval = POWER_SUPPLY_HEALTH_GOOD;
	return  POWER_SUPPLY_HEALTH_GOOD;
}

static int tps80031_bat_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct tps80031_device_info *di = to_tps80031_device_info_bat(psy);

	switch (psp) {

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = tps80031_battery_health(di, val);
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval =  di->capacity_sts;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW://Charger current here
		val->intval =  di->chg_current * 1000;//uA  negative value means discharging
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval =  di->bat_vol * 1000;//uV
		break;

	case POWER_SUPPLY_PROP_STATUS://Charging status
		if(di->battery_detect_by_ntc) {
			if(di->battery_present)
				val->intval = di->charging_status;
			else
				val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		}
		else
		val->intval = di->charging_status;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->bat_temp;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = tps80031_battery_present(di,val);
		break;
#ifdef CONFIG_BATTERY_FUEL_GAUGE_DETECT
	case POWER_SUPPLY_PROP_FUEL_GAUGE:
		val->intval = POWER_SUPPLY_GAUGE_NO;
		break;
#endif

	default:
		return -EINVAL;
	}
	return 0;
}

#ifdef   CONFIG_TPS80032_USB_CHARGER
#define to_tps80031_device_info_usb(x) container_of((x), \
				struct tps80031_device_info, usb);

static int tps80031_usb_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct tps80031_device_info *di = to_tps80031_device_info_usb(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->usb_online;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
#endif

#define to_tps80031_device_info_ac(x) container_of((x), \
				struct tps80031_device_info, ac);

static int tps80031_ac_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct tps80031_device_info *di = to_tps80031_device_info_ac(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->ac_online;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}


void tps80031_battery_status(enum charging_states status, void *data)
{
	struct tps80031_device_info *di = data;
	enum charging_type charger_type;
	charger_type = tps8003x_charger_type();
	if (charger_type == AC_CHARGER) {
		di->cache.capacity_plug = di->capacity_sts;
		DBG_TPS8003X_INFO("State Changed:AC plug,last unplug capacity is %d\n",di->cache.capacity_plug);
		di->ac_online  = 1;
		di->usb_online = 0;
	}
	else if(charger_type == USB_CHARGER) {
		di->cache.capacity_plug = di->capacity_sts;
		DBG_TPS8003X_INFO("State Changed:USB plug,last unplug capacity is %d\n",di->cache.capacity_plug);
		di->ac_online  = 0;
		di->usb_online = 1;
	}
	else {
		di->cache.capacity_unplug = di->capacity_sts;
		DBG_TPS8003X_INFO("State Changed:Charger unplug,last plug capacity is %d\n",di->cache.capacity_unplug);
		di->ac_online  = 0;
		di->usb_online = 0;
	}
	if(!battery_probe_done)
		return;
	di->need_updated_immediately =  true;
	cancel_delayed_work_sync(&di->monitor_battery);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_battery, HZ/2);
}

//Work Handler
static void battery_handle_intrpt(struct work_struct *work)
{
	struct tps80031_device_info *di = container_of(work,
				struct tps80031_device_info, handler);
	DBG_TPS8003X_INFO("%s %d\n",__func__,__LINE__);
	//update Battery status here
	di->need_updated_immediately =  true;
	cancel_delayed_work_sync(&di->monitor_battery);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_battery, HZ);

}


static void tps8003x_fast_poll_stack(uint32_t vol_raw)//To-Do kfifo 
{
        uint32_t array_value1,array_value2;

        if(fast_poll_array.vol[0] == 0)
        {
		//First Enter FAST POLL
		fast_poll_array.vol[0] = vol_raw;
        }
	else
	{
                fast_poll_array.update_index++;
                if(fast_poll_array.update_index >= SAMPLE_TIMES)
                {
                    //PUSH
		    array_value1 = fast_poll_array.vol[SAMPLE_TIMES - 1];
		    array_value2 = fast_poll_array.vol[SAMPLE_TIMES - 2];
		    fast_poll_array.vol[SAMPLE_TIMES - 1] = vol_raw;
                    fast_poll_array.vol[SAMPLE_TIMES - 2] = array_value1;
		    fast_poll_array.vol[SAMPLE_TIMES - 3] = array_value2;
                }
		else
                fast_poll_array.vol[fast_poll_array.update_index] = vol_raw;
	}
}


static void tps8003x_fast_poll_handler(struct tps80031_device_info *di)
{

	if(di->poll_interval == di->fast_poll)
	{
		if(di->bat_vol < di->fast_poll_vol) {
			tps8003x_fast_poll_stack(di->bat_vol);
			if( (fast_poll_array.vol[SAMPLE_TIMES -1] > 0 ) && 
                            (fast_poll_array.vol[SAMPLE_TIMES -1] <= di->shut_down_vol )){
#ifdef   CONFIG_TPS80032_USB_CHARGER
				if(di->ac_online == 0 && di->usb_online == 0 ) {
#else
				if(di->ac_online == 0) {
#endif
                        		DBG_TPS8003X_INFO("%s Let's shut down\n",__func__);
					//cancel_delayed_work_sync(&di->monitor_battery);//shoudn't call this func by itself
					if(di->capacity_sts > di->shut_down_lvl)
 					di->capacity_sts = di->shut_down_lvl;  //To-Do,Sync with Framework
					power_supply_changed(&di->bat);
					mdelay(100);//Make Sure Shut down Begin
			        }
			}
			
		}
		else {
			memset(&fast_poll_array,0,sizeof(fast_poll_array));
			DBG_TPS8003X_INFO("Exit FAST POLL\n");
			di->poll_interval = di->slow_poll;
		} 
	}
	else {
		if(di->bat_vol < di->fast_poll_vol ){
			memset(&fast_poll_array,0,sizeof(fast_poll_array));
			DBG_TPS8003X_INFO("Enter FAST POLL\n");
			tps8003x_fast_poll_stack(di->bat_vol);
			di->poll_interval = di->fast_poll;
		}   
	}
}


static bool battery_same_plug_status(struct tps80031_device_info *di)
{
	return  ((di->cache.ac_online == di->ac_online) && (di->cache.usb_online == di->usb_online));	
}

static int  battery_calc_capacity_index(struct tps80031_device_info *di,BattMapInttoInt *fixed_cap,int size)
{
 
  	int index = 0; 
  	uint32_t voltage = 0;

 	if(!fixed_cap){
  		DBG_TPS8003X_INFO("Wrong capacity table!!!\n");
 		return -1;  //Don't update capacity in main function 
  	}

  	if(di->bat_vol > fixed_cap[0].vbatt)
   	 	return 0;

  	else if(di->bat_vol < fixed_cap[size -1].vbatt)
   		return size; //Zero
  	else {
  		voltage = ((uint32_t)(di->bat_vol / 10)) *10 ;
  		index = size - (voltage - fixed_cap[size -1].vbatt)/10 -1;
  		return index;
  	}
}



static uint8_t tps8003x_batt_capacity(struct tps80031_device_info *di)
{

	int nIdx = 0;
	int nSize = 0;
	uint8_t current_capacity = 0;

        nSize = sizeof(batt_cap)/sizeof(batt_cap[0]);

        nIdx = battery_calc_capacity_index(di,batt_cap,nSize);

      	if(nIdx == 0)
        	current_capacity = 100;
        else if(nIdx == nSize)
        	current_capacity = 0;
        else {
#ifdef   CONFIG_TPS80032_USB_CHARGER
		if(di->charging_status == POWER_SUPPLY_STATUS_CHARGING) {
             		if(di->ac_online == 1)
             	 		current_capacity = batt_cap[nIdx].cbatt_ac;
              		else if(di->usb_online == 1)
                 		current_capacity = batt_cap[nIdx].cbatt_usb;
		} 
#else
             	if(di->ac_online == 1)
             	 		current_capacity = batt_cap[nIdx].cbatt_ac;
              	else if(di->usb_online == 1)
                 		current_capacity = batt_cap[nIdx].cbatt_usb;
#endif
		else
                 	current_capacity = batt_cap[nIdx].cbatt_dis;
        } 
#ifdef   CONFIG_TPS80032_USB_CHARGER
	if((di->charging_status == POWER_SUPPLY_STATUS_FULL) && ((di->ac_online == 1)  || (di->usb_online == 1))){ //To-Do,W970 will rebuild usb  status
#else
	if((di->charging_status == POWER_SUPPLY_STATUS_FULL) && (di->ac_online == 1)){ 
#endif
		//battery_full_time_check++;
		current_capacity = 100;
	}
	//else
	//	battery_full_time_check = 0;  //We using united method instead of the old way.
	
	return current_capacity;
}


static int batt_calc_voltage(int voltage) //stack operation
{
       int index = 0;
       int allvol= 0;
	
        if(batt_vol_array.initflag == 0)
        {
        	for (index=0;index<SAMPLE_TIMES;index++)
        	{
            	    batt_vol_array.vol[index] = voltage;			
        	}
		batt_vol_array.initflag = 1;
        }
	else
	{
                batt_vol_array.update_index++;
                if(batt_vol_array.update_index >= SAMPLE_TIMES)
                {
                    batt_vol_array.update_index = 0;
                }
                batt_vol_array.vol[batt_vol_array.update_index] = voltage;
	}
    

	for(index=0;index<SAMPLE_TIMES;index++)
	{
	    allvol += batt_vol_array.vol[index];
	}

	return allvol/SAMPLE_TIMES;
}


static void battery_voltage_stabilize(struct tps80031_device_info *di)
{
	tps8003x_fast_poll_handler(di);

	if(battery_ini_flag == 0){
		if(di->init_vol_from_btloader) {  
			DBG_TPS8003X_INFO("%s first volatge in kernel %u  in bootloader %u\n",__func__,di->bat_vol,cap_of_battery);
			if( ((cap_of_battery > di->bat_vol) && (cap_of_battery <= di->vbat_max))
			&&((di->ac_online  == 0) && (di->usb_online == 0))
			&&( ((tegra_power_reason & State_Charger_PlugIn) == 0)
			  &&((tegra_power_reason & State_Normal_USB_PlugIn) == 0)) ) {
#ifdef   CONFIG_BOARD_US9230
				di->bat_vol = cap_of_battery - 30;
#else
				di->bat_vol = cap_of_battery - 30;  //Same with W970
#endif
			}
			else{
				DBG_TPS8003X_INFO("%s Using kernel voltage\n",__func__);
#ifdef   CONFIG_BOARD_US9230
				di->bat_vol = di->bat_vol + di->init_adjust_vol;
#else
				if( (di->usb_online == 1) && (tegra_power_reason & State_Normal_USB_PlugIn) ) {
					if((cap_of_battery > di->bat_vol) && (cap_of_battery <= di->vbat_max)) {
						if(cap_of_battery > 4000)
							di->bat_vol = cap_of_battery + 30;
						else
							di->bat_vol = cap_of_battery + 40;
					}
				}
#endif
			}
		}
		else
		di->bat_vol = di->bat_vol + di->init_adjust_vol;

		memset(&batt_vol_array,0,sizeof(batt_vol_array));	
		memset(&di->cache,0,sizeof(di->cache));
	}
	else if(di->need_updated_immediately)
		memset(&batt_vol_array,0,sizeof(batt_vol_array));
	else{

		if( (battery_same_plug_status(di)) && (di->cache.charging_status == di->charging_status))
			di->bat_vol = batt_calc_voltage(di->bat_vol); //Only use average voltage for nothing changes situation
		else
			memset(&batt_vol_array,0,sizeof(batt_vol_array));
	}	
	
}

static uint32_t battery_calc_poll_time(void)
{
	uint32_t delta_time;

	if(battery_ini_flag != 0){
        	do_gettimeofday(&battery_current_time);
		batt_sleep_time.poll_time =  battery_current_time.tv_sec;
		if((batt_sleep_time.poll_time != batt_sleep_time.init_time) 
			&& (batt_sleep_time.last_poll_time == batt_sleep_time.init_time)){
			DBG_TPS8003X_INFO("First Get True time\n");
			batt_sleep_time.last_poll_time = batt_sleep_time.poll_time;
			
		}
	}
	else {
		batt_sleep_time.poll_time = batt_sleep_time.init_time;
		batt_sleep_time.last_poll_time = batt_sleep_time.poll_time;
	}
	delta_time = batt_sleep_time.poll_time - batt_sleep_time.last_poll_time;
	batt_sleep_time.last_poll_time = batt_sleep_time.poll_time;

	return delta_time;
}


static uint8_t  pick_small_value(uint8_t a,uint8_t b)
{
  	return ((a <= b) ? a : b);  
}

static uint8_t  pick_large_value(uint8_t a,uint8_t b)
{
  	return ((a <= b) ? b : a);  
}



//Reverse Handler During discharging  -- negative direction
static uint8_t adc_bat_discharging_handler(struct tps80031_device_info *di,uint8_t capacity)
{

  int i = 0;
  int size = 0; 
  uint32_t reverse_limited = 0;
  	
  if(di->charging_status != POWER_SUPPLY_STATUS_DISCHARGING)
	return 	capacity;
  else
  {
    size = sizeof(batt_cap)/sizeof(batt_cap[0]);	 
    for(i = size -1 ;i > 0;i--)
    { 
	if(batt_cap[i].cbatt_dis >= di->capacity_sts)
	   break;	
    }
    if(batt_cap[i].vbatt  < di->bat_vol)
    {	
	if(di->capacity_sts > 15)
		reverse_limited = BATTERY_REVERSE_LOW_POWER_BASE;//80mv
	else if(di->capacity_sts > 10)
		reverse_limited = BATTERY_REVERSE_LOW_POWER_BASE + 20;//100mv
	else if(di->capacity_sts > 5)
		reverse_limited = BATTERY_REVERSE_LOW_POWER_BASE + 40;//120mv
	else
		reverse_limited = BATTERY_REVERSE_LOW_POWER_BASE + 70;//150mv

	if((di->bat_vol - batt_cap[i].vbatt) <= reverse_limited)
	{
        	DBG_TPS8003X_INFO("%d  Don't Allow Reverse,last voltage is %u,now voltage is %u\n",__LINE__,batt_cap[i].vbatt,di->bat_vol);
        	return 	di->capacity_sts;
	}
	else
	return capacity;
    }
    else
    return capacity;
  }

}

//Charging handler with charging time factor  -- positive direction
static void adc_bat_charging_handler(struct tps80031_device_info *di)
{
  long charging_time = 0;
  long fix_cap_changes = 0;
  charging_time = adc_battery_charging_continue_time();

  if(di->charging_status != POWER_SUPPLY_STATUS_CHARGING)
	return;

  if(charging_time < 0)
	return;

  if(di->capacity_sts < di->cache.capacity_sts)
	return;

  if(di->ac_online == 1) {
	if(di->cache.capacity_sts < 50)
		fix_cap_changes = (charging_time * BATTERY_CHRGING_CAP_PER_MIN_H) / 60;
	else if(di->cache.capacity_sts < 90) {
		if((di->cache.capacity_plug < 50) && (charging_one_half_time < charging_time))
			fix_cap_changes = ((charging_one_half_time * BATTERY_CHRGING_CAP_PER_MIN_H) +
				((charging_time - charging_one_half_time) * BATTERY_CHRGING_CAP_PER_MIN_L))/ 60;
		else
			fix_cap_changes = (charging_time * BATTERY_CHRGING_CAP_PER_MIN_L) / 60;
		}
	else
	return;
  }
  else {//for usb charging
	if(di->cache.capacity_sts < 50)
		fix_cap_changes = (charging_time * BATTERY_CHRGING_CAP_PER_MIN_H_O) / 60;
	else if(di->cache.capacity_sts < 90) {
		if((di->cache.capacity_plug < 50) && (charging_one_half_time < charging_time))
			fix_cap_changes = ((charging_one_half_time * BATTERY_CHRGING_CAP_PER_MIN_H_O) +
				((charging_time - charging_one_half_time) * BATTERY_CHRGING_CAP_PER_MIN_L_O))/ 60;
		else
			fix_cap_changes = (charging_time * BATTERY_CHRGING_CAP_PER_MIN_L_O) / 60;
		}
	else
	return;
  }

  if(fix_cap_changes < 0 || fix_cap_changes > 100)
	return;


  //DBG_TPS8003X_INFO("%s di->cache.capacity_sts %u di->cache.capacity_plug %u di->capacity_sts %u fix_cap_changes %ld charging time %ld\n",
  //		__func__,di->cache.capacity_sts,di->cache.capacity_plug,di->capacity_sts,fix_cap_changes,charging_time);

  di->capacity_sts = pick_small_value((di->cache.capacity_plug + fix_cap_changes),di->capacity_sts);

  if(di->cache.capacity_sts < 50  && di->capacity_sts >= 50) {
	//The Charging time is divided  to 2 Section
	charging_one_half_time = charging_time;
	DBG_TPS8003X_INFO("%s charging_one_half_time %ld\n",__func__,charging_one_half_time);

  }

}




//Reverse capacity should not  be larger than unplug charging capacity and have more conditions for P9202 during charging with USB Cable
static uint8_t  pick_reverse_value(struct tps80031_device_info *di,uint8_t capacity)
{
	uint8_t fix_capacity = 0;
#if (defined(CONFIG_BOARD_M470)||defined(CONFIG_BOARD_M470BSD)||defined(CONFIG_BOARD_M470BSS))
	if(di->ac_online == 0 && di->usb_online == 0 ) {
		if(di->cache.capacity_unplug == 0)
			fix_capacity = capacity;
		else if(di->cache.capacity_unplug < capacity)
			fix_capacity = di->cache.capacity_unplug;
		else
			fix_capacity = capacity;
	}
	else
	fix_capacity = capacity;

#else
	if(di->cache.capacity_unplug == 0)
		fix_capacity = capacity;
	else if(di->cache.capacity_unplug < capacity)
		fix_capacity = di->cache.capacity_unplug;
	else
		fix_capacity = capacity;
#endif
	fix_capacity = adc_bat_discharging_handler(di,fix_capacity);

	return  fix_capacity;        
}



static void batt_capa_revise_shortly(struct tps80031_device_info *di,uint8_t capa,uint32_t pause_time)//90s
{

	unchargerT = chargerT = (unsigned long)run_time;

	if(battery_ini_flag != 0) {
		if(batt_sleep_time.suspend_flag) { //We suspend Before runtime reach 90s
			if(!batt_sleep_time.charging_in_s3) {
				if(capa > di->capacity_sts) {	
					if((pause_time > di->sleep_up_time) && ((capa - di->capacity_sts) > CHARGER_THROSHOLD_MID))
						di->capacity_sts = pick_reverse_value(di,(capa + di->capacity_sts) / 2);
					else                                                
                 				DBG_TPS8003X_INFO("Capacity reverse %d\n",__LINE__);
				}
				else if(pause_time > di->sleep_up_time)
                                        	di->capacity_sts = capa;
				else{ 
					if((di->capacity_sts - capa) > CHARGER_THROSHOLD_MID){
						DBG_TPS8003X_INFO("Decrease too much %d\n",__LINE__);
                           			di->capacity_sts =  pick_large_value(
							(di->capacity_sts - CHARGER_THROSHOLD_MID),((di->capacity_sts + capa)/2 -1));
                        		} 
                        		else
                        			di->capacity_sts = capa;  
                                }
                	}
			else if(capa < di->capacity_sts) 
				DBG_TPS8003X_INFO("Capacity reverse happen %d\n",__LINE__);
                        else{
				if(di->charging_status == POWER_SUPPLY_STATUS_FULL)
				   di->capacity_sts = 100;
				else if(pause_time < di->sleep_up_time) {
				   di->capacity_sts =  pick_small_value(
                          		(di->capacity_sts + CHARGER_THROSHOLD_MID), ((capa + di->capacity_sts)/2));
				}
				else
					di->capacity_sts = capa;
				adc_bat_charging_handler(di);
			}
			return;            
		}
		else {
			if(di->charging_status == POWER_SUPPLY_STATUS_FULL) 
			di->capacity_sts = 100;
                        return;
		}      
	} 
	di->capacity_sts = capa;
	DBG_TPS8003X_INFO("NOW Time = %lu\n",(unsigned long)run_time);
	return;	
}


static void batt_capa_revise_normal(struct tps80031_device_info *di,uint8_t capa,uint32_t pause_time)
{

	if(pause_time > di->sleep_up_time) {
		batt_continue_reverse_times = 0;
                unchargerT = chargerT = (unsigned long)run_time;//Update window time
                if((capa > di->capacity_sts ) && (!batt_sleep_time.charging_in_s3)) {
			if((capa - di->capacity_sts) > CHARGER_THROSHOLD_MID)
			di->capacity_sts = pick_reverse_value(di,(capa + di->capacity_sts) / 2);
			else                                                
			DBG_TPS8003X_INFO("%d Capacity reverse happen \n",__LINE__);
                 	return;                 
                }
                else if((capa < di->capacity_sts) && (batt_sleep_time.charging_in_s3)) {
			DBG_TPS8003X_INFO("%d Capacity reverse happen\n",__LINE__);
			return;  
                }   

		di->capacity_sts = capa;
		adc_bat_charging_handler(di);
		return;
	}

	if(di->charging_status == POWER_SUPPLY_STATUS_CHARGING) {	
                unchargerT = (unsigned long)run_time;
                if( (di->cache.ac_online != di->ac_online) ||  (di->cache.usb_online != di->usb_online))
                chargerT =  (unsigned long)run_time;//Chager source changed between poll interval
                 
                if((unsigned long)run_time<(chargerT+WINDOW_T)) {
                    //DBG_TPS8003X_INFO("Charge Start WINDOW\n");
                    if(capa > di->capacity_sts) {
			batt_continue_reverse_times = 0;
                       	if((capa - di->capacity_sts) > CHARGER_THROSHOLD_WINDOW) {
                          di->capacity_sts =  pick_small_value(
                          	(di->capacity_sts + CHARGER_THROSHOLD_WINDOW), ((di->capacity_sts + capa)/2));
                        }
                        else 
                        di->capacity_sts = capa; 
			adc_bat_charging_handler(di);
                    }
                    
  		}	
		else if(capa > di->capacity_sts){	
                        batt_continue_reverse_times = 0;
                        if((capa - di->capacity_sts) > CHARGER_THROSHOLD) {
                          di->capacity_sts =  pick_small_value(
				(di->capacity_sts + CHARGER_THROSHOLD),((di->capacity_sts + capa)/2));
                        }
                        else 
                        di->capacity_sts = capa;
			adc_bat_charging_handler(di);
		}
		else if(capa < di->capacity_sts) {

			if(di->capacity_sts == 100)
				di->capacity_sts = 99;
			else {
				batt_continue_reverse_times++;
				DBG_TPS8003X_INFO("batt_continue_reverse_times = %d\n",batt_continue_reverse_times);
				if(batt_continue_reverse_times > di->max_reverse_times)	
					di->capacity_sts = (di->capacity_sts + capa)/2;
		
			}
		}
		else if(capa == di->capacity_sts)
			batt_continue_reverse_times = 0;
	}
	else if((di->charging_status == POWER_SUPPLY_STATUS_NOT_CHARGING) ||  
				(di->charging_status == POWER_SUPPLY_STATUS_DISCHARGING)){ //USB on M370L will always be discharging	
         	chargerT = (unsigned long)run_time;//update charging stop time
                if(batt_sleep_time.suspend_flag) {
			if((capa < di->capacity_sts) && (!batt_sleep_time.charging_in_s3)) {
				batt_continue_reverse_times = 0;
 				if((di->capacity_sts - capa) > CHARGER_THROSHOLD_MID){
                           		DBG_TPS8003X_INFO("Decrease too much, use CHARGER_THROSHOLD_MID value\n");
                           		di->capacity_sts = pick_large_value( 
                                	(di->capacity_sts - CHARGER_THROSHOLD_MID),((di->capacity_sts + capa)/2 -1));
                        	} 
                       	        else 
                        	di->capacity_sts = capa;  

                        }

                } 
                else if((unsigned long)run_time<(unchargerT+WINDOW_T))  {
		   if(capa < di->capacity_sts) {
			batt_continue_reverse_times = 0;
                        if((di->capacity_sts - capa) > CHARGER_THROSHOLD_WINDOW)
                        {
                           //DBG_TPS8003X_INFO("Decrease too much, use CHARGER_THROSHOLD_WINDOW value\n");
                           di->capacity_sts =  pick_large_value(
				(di->capacity_sts - CHARGER_THROSHOLD_WINDOW),((di->capacity_sts + capa)/2 -1));
                        } 
                        else
                        di->capacity_sts = capa;  
		   }

  		}		
		else if(capa < di->capacity_sts) {
			batt_continue_reverse_times = 0;
			if((di->capacity_sts - capa) > CHARGER_THROSHOLD) {
                           //DBG_TPS8003X_INFO("Decrease too much, use CHARGER_THROSHOLD_WINDOW value\n");
                           di->capacity_sts =   pick_large_value(
                           	(di->capacity_sts - CHARGER_THROSHOLD_WINDOW),((di->capacity_sts + capa)/2 -1));
                        } 
                        else
                        di->capacity_sts = capa;  
		}
		else if(capa > di->capacity_sts)
		{
			batt_continue_reverse_times++;
			DBG_TPS8003X_INFO("batt_continue_reverse_times = %d\n",batt_continue_reverse_times);
			if((batt_continue_reverse_times > di->max_reverse_times)  && ((capa - di->capacity_sts) > CHARGER_THROSHOLD_MID))
				di->capacity_sts = pick_reverse_value(di,(di->capacity_sts + capa)/2);
		}	
		else if(capa == di->capacity_sts)
			batt_continue_reverse_times = 0;
	}
	else if(di->charging_status == POWER_SUPPLY_STATUS_FULL)
	{
         	chargerT = (unsigned long)run_time;//update charging stop time.FULL will stop charger
		di->capacity_sts = 100;
	}
}

static void hmct_battery_user_experience(struct tps80031_device_info *di)
{
//For charging 90% -100% user experience
	uint32_t lasting_95_time = 0; 
	
	if((((di->capacity_sts >= 94) && (di->capacity_sts < 96)) ||( ((di->capacity_sts >= 96)&&(di->capacity_sts <= 98)) && (battery_95_start_time != 0)) )
				&& (di->charging_status == POWER_SUPPLY_STATUS_CHARGING)) {
		if( battery_ini_flag != 0) {
			if(battery_95_start_time == 0) {
				if(di->capacity_sts > 94)
					di->capacity_sts = 94;
				battery_95_start_time = batt_sleep_time.poll_time;
				DBG_TPS8003X_INFO("%s  capacity lasting time begin %u s\n",__func__,battery_95_start_time);
			}
			else {
				if(batt_sleep_time.poll_time > battery_95_start_time) {
					lasting_95_time = batt_sleep_time.poll_time - battery_95_start_time;
					if(lasting_95_time > ((BATTERY_CHRGING_TIME_94_95 + BATTERY_CHRGING_TIME_95_96) + 
							 	(BATTERY_CHRGING_TIME_96_97 + BATTERY_CHRGING_TIME_97_98))) {
						if(di->capacity_sts != 98) 
						DBG_TPS8003X_INFO("%s  change capacity to 98 for lasting %u  start_times %u ,current time %u\n",
							__func__,lasting_95_time,battery_95_start_time,batt_sleep_time.poll_time);
						di->capacity_sts = 98;
					}
					else if(lasting_95_time > ( (BATTERY_CHRGING_TIME_94_95 +BATTERY_CHRGING_TIME_95_96) + BATTERY_CHRGING_TIME_96_97)) {
						if(di->capacity_sts != 97) 
						DBG_TPS8003X_INFO("%s  change capacity to 97 for lasting %u  start_times %u ,current time %u\n",
							__func__,lasting_95_time,battery_95_start_time,batt_sleep_time.poll_time);
						di->capacity_sts = 97;
					}
					else if(lasting_95_time > (BATTERY_CHRGING_TIME_94_95 + BATTERY_CHRGING_TIME_95_96)) {
						if(di->capacity_sts != 96) 
						DBG_TPS8003X_INFO("%s  change capacity to 96 for lasting %u  start_times %u ,current time %u\n",
							__func__,lasting_95_time,battery_95_start_time,batt_sleep_time.poll_time);
						di->capacity_sts = 96;
					}
					else if(lasting_95_time > BATTERY_CHRGING_TIME_94_95) {
						if(di->capacity_sts != 95) 
						DBG_TPS8003X_INFO("%s  change capacity to 95 for lasting %u  start_times %u ,current time %u\n",
							__func__,lasting_95_time,battery_95_start_time,batt_sleep_time.poll_time);
						di->capacity_sts = 95;
					}
					else
						di->capacity_sts = 94;
				} 
			}
		}
		else
		battery_95_start_time = 0; 
	}
	else
		battery_95_start_time = 0;
#ifdef   CONFIG_TPS80032_USB_CHARGER
//In Charging Don't allow  reverse changes in charging if chging current > 0 for high load with charger(current can be below 50,voltage isn't very high)
	if(((di->charging_status == POWER_SUPPLY_STATUS_CHARGING) && (di->cache.capacity_sts >= 10))
			&& (di->capacity_sts < di->cache.capacity_sts)) {
		if((di->chg_current > 0) && (di->cache.chg_current  > 0)) {
			DBG_TPS8003X_INFO("%s %d In Charging Don't allow reverse changes\n",__func__,__LINE__);
			di->capacity_sts = di->cache.capacity_sts;
		}
	}

#endif
//For 99% 100% checking
	if(di->capacity_sts == 100) {
		if(di->charging_status == POWER_SUPPLY_STATUS_CHARGING){
			battery_full_time_check = 0;
			di->capacity_sts = 99;	//Re-Charge capacity changed to 99
		}
		else if(di->charging_status == POWER_SUPPLY_STATUS_FULL) {
			battery_full_time_check++;
			if(battery_full_time_check <= 1) {
				DBG_TPS8003X_INFO("%s  change full charging to 99\n",__func__);
				di->capacity_sts = 99;
				di->charging_status = POWER_SUPPLY_STATUS_CHARGING;
			}
		}
		else
			battery_full_time_check = 0;
	}
	else
			battery_full_time_check = 0;

}


static void adc_batt_capacity_calibration(struct tps80031_device_info *di,uint8_t raw_cap)
{
	
	uint32_t  pause_time = 0;
	pause_time = battery_calc_poll_time();


	run_time = cpu_clock(smp_processor_id());
	do_div(run_time, 1000000000);

	if(((unsigned long)run_time < batt_sleep_time.start_time) &&
		((raw_cap >= 10) ||(battery_ini_flag == 0)))
	batt_capa_revise_shortly(di,raw_cap,pause_time);
	else
	batt_capa_revise_normal(di,raw_cap,pause_time);
}


static void battery_monitor(struct work_struct *work)
{
	struct tps80031_device_info *di = container_of(work,
			struct tps80031_device_info, monitor_battery.work);
	int adc_voltage,adc_temp;
#ifdef   CONFIG_BOARD_US9230
	int batt_cal_vol = 0;
#endif

	uint8_t	raw_capacity;

//Get Voltage Samples
	adc_voltage = tps80031_battery_voltage(di);
	
	if( (adc_voltage >= di->vbat_min) && (adc_voltage <= di->vbat_max) )
		di->bat_vol = adc_voltage;
	else
		dev_err(di->dev, "Wrong  Battery Voltage Detected: %d\n",adc_voltage);
	//Get Charging Current Now
	tps8003x_battery_fg_current(di);
#if (defined(CONFIG_BOARD_M470)||defined(CONFIG_BOARD_M470BSD)||defined(CONFIG_BOARD_M470BSS))  //To-Do Remove  When PWR_I2C stable
	if(adc_voltage == -1)
		di->charging_status = POWER_SUPPLY_STATUS_UNKNOWN;
	else
		di->charging_status = tps80031_battery_charging_status(di);
#else
	di->charging_status = tps80031_battery_charging_status(di);
#endif
#ifdef   CONFIG_BOARD_US9230
	if((di->charging_status == POWER_SUPPLY_STATUS_DISCHARGING)
			&&((di->bat_vol > 3700) && (battery_ini_flag != 0))){
		if(di->bat_vol > 3900)
		batt_cal_vol = di->bat_vol - (di->chg_current * (di->inter_resis + 30) /1000);
		else
		batt_cal_vol = di->bat_vol - (di->chg_current * di->inter_resis /1000);

		if( (batt_cal_vol >= di->vbat_min) && (batt_cal_vol <= di->vbat_max) )
		di->bat_vol = batt_cal_vol;
		else
		dev_err(di->dev, "Wrong  batt_cal_vol  Detected: %d\n",batt_cal_vol);

		//DBG_TPS8003X_INFO("ADC Vol: %u,Current: %d cal vol: %u,bat vol:%u cache cap: %u\n",adc_voltage, di->chg_current,
		//	batt_cal_vol,di->bat_vol,di->cache.capacity_sts);
	}
#endif

//Calc Voltage	
	battery_voltage_stabilize(di);
	di->poll_index ++;	
	if((di->need_updated_immediately || di->poll_index >= di->poll_max_times)
		|| di->poll_interval == di->fast_poll ){

		di->poll_index = 0;
		di->need_updated_immediately = false;
		//Get Raw Cap
		raw_capacity = tps8003x_batt_capacity(di);
		if((battery_ini_flag == 0) || battery_same_plug_status(di)) {
			//Calc Cap
			adc_batt_capacity_calibration(di,raw_capacity);
			//Only Update temp in real poll
			adc_temp = tps80031_battery_temp(di);
			if(adc_temp != TEMP_BAD_VALUE)  
				di->bat_temp = adc_temp;
			else
			dev_err(di->dev, "Wrong  Battery Temperature Detected: %d\n",adc_temp);
			adc_battery_temp = di->bat_temp;
		}
		hmct_battery_user_experience(di);
		DBG_TPS8003X_INFO("Vol: %u,Current: %d Raw Cap: %u,Cal Cap:%u Temp: %d Charging:%d\n",di->bat_vol, di->chg_current,
			raw_capacity,di->capacity_sts,di->bat_temp,di->charging_status);
		power_supply_changed(&di->bat);
	}

	//Update Cache ones
	di->cache.ac_online  = di->ac_online ;
        di->cache.usb_online = di->usb_online;
        di->cache.charging_status = di->charging_status;
	di->cache.capacity_sts = di->capacity_sts;
	di->cache.chg_current = di->chg_current;

	if(battery_ini_flag == 0) {  //We should give the initial value if we boot up with/without charger
		di->cache.capacity_plug = di->capacity_sts;
		di->cache.capacity_unplug = di->capacity_sts;
		DBG_TPS8003X_INFO("Fill the initial di->cache.capacity_plug values %u   di->cache.capacity_unplug %u\n",
			di->cache.capacity_plug,di->cache.capacity_unplug);
		battery_ini_flag = 1;
	}
	if(batt_sleep_time.suspend_flag)
	batt_sleep_time.suspend_flag = false;
	
	queue_delayed_work(di->monitor_wqueue, &di->monitor_battery, HZ * di->poll_interval);
}


#ifdef CONFIG_HAS_EARLYSUSPEND

void tps8003x_adc_bat_early_suspend(struct early_suspend *h)
{
	if(batt_sleep_time.last_poll_time == batt_sleep_time.init_time)
	{
		do_gettimeofday(&battery_current_time);
		batt_sleep_time.last_poll_time = battery_current_time.tv_sec;//update last_poll_time
		DBG_TPS8003X_INFO("%s now time is %u\n",__func__,batt_sleep_time.last_poll_time);
	}
}

void tps8003x_adc_bat_late_resume(struct early_suspend *h)
{
    return;
}
#endif

//#ifdef CONFIG_BATTERY_ADC_TPS8003X_DEBUG
static ssize_t
raw_vol_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tps80031_device_info *di = dev_get_drvdata(dev);
	int adc_code,raw_value;
	mutex_lock(&di->adc_lock);
	adc_code = tps80031_gpadc_conversion(BATTERY_VOLTAGE);
	mutex_unlock(&di->adc_lock);

	raw_value = ((adc_code * 5000) / 4096);//DIV4/0xFFF/mV

	return sprintf(buf, "ADC Code:%d Raw Vol:%d mv\n",adc_code,raw_value);
}

static DEVICE_ATTR(raw_vol, S_IRUGO, raw_vol_show, NULL);


static ssize_t
raw_temp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tps80031_device_info *di = dev_get_drvdata(dev);
	int adc_code,raw_value;
	mutex_lock(&di->adc_lock);
	adc_code = tps80031_gpadc_conversion(BATTERY_TEMPERATURE);
	mutex_unlock(&di->adc_lock);

	raw_value = (adc_code * 1250) /4096;//mv

	return sprintf(buf, "ADC Code:%d Raw NTC:%d mv\n",adc_code,raw_value);
}
static DEVICE_ATTR(raw_temp, S_IRUGO, raw_temp_show, NULL);


static ssize_t
raw_curr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tps80031_device_info *di = dev_get_drvdata(dev);
	int adc_code,raw_value;

	mutex_lock(&di->adc_lock);
	adc_code = tps80031_gpadc_conversion(BATTERY_CHARGING_CURRENT);
	mutex_unlock(&di->adc_lock);

	raw_value = adc_code * 1000; //uA

	return sprintf(buf, "ADC Code:%d Raw Current:%d uA\n",adc_code,raw_value);
}
static DEVICE_ATTR(raw_curr, S_IRUGO, raw_curr_show, NULL);

static struct attribute *adc_battery_sysfs_entries[] = {
	&dev_attr_raw_vol.attr,
	&dev_attr_raw_temp.attr,
	&dev_attr_raw_curr.attr,
	NULL,
};


static struct attribute_group adc_battery_attr_group = {
	.attrs	= adc_battery_sysfs_entries,
};
//#endif

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static int batt_get_temp(void *dev_data, long *temp)
{
	//TODO: Implement battery temp get function
	//printk("Get battary temp !!!!!!!!!!!!!!!!!!\n");
	if(temp)
		*temp = adc_battery_temp/10;
    return 0;
}

static void batt_init(void)
{
        struct tegra_thermal_device *batt_device;
        batt_device = kzalloc(sizeof(struct tegra_thermal_device),
                                GFP_KERNEL);
        if (!batt_device) {
                printk("unable to allocate thermal device\n");
                return;
        }
 
        batt_device->name = "batt_dev";
        batt_device->id = THERMAL_DEVICE_ID_BATT;
        batt_device->get_temp = batt_get_temp;
 
        tegra_thermal_device_register(batt_device);
}
#endif //#ifdef CONFIG_TEGRA_SKIN_THROTTLE


static irqreturn_t tps80031_bat_overtemp(int irq, void *data)
{
	struct tps80031_device_info  *di = data;
	schedule_work(&di->handler);
	return IRQ_HANDLED;
}


static int tps80031_adc_battery_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct tps80031_device_info *di;

	struct tps80031_platform_data *tps80031_pdata;
	struct tps80031_adc_bat_platform_data *pdata;
	enum charging_type charger_type;
	unsigned long long t;

	tps80031_pdata = dev_get_platdata(pdev->dev.parent);
	if (!tps80031_pdata) {
		dev_err(&pdev->dev, "no tps80031 platform_data specified\n");
		return -EINVAL;
	}

	pdata = tps80031_pdata->adc_battery_pdata;
	if (!pdata) {
		dev_err(&pdev->dev, "no adc battery platform data\n");
		return -EINVAL;
	}

	di = devm_kzalloc(&pdev->dev, sizeof *di, GFP_KERNEL);
	if (!di) {
		dev_err(dev->parent, "failed to allocate device info data\n");
		return -ENOMEM;
	}

	if (!pdata->battery_present) {
		dev_err(dev, "%s() No battery detected, exiting..\n",
				__func__);
		return -ENODEV;
	}
#ifdef CONFIG_BATTERY_BQ27x00
	else if(battery_standalone_fg()) {
		dev_err(dev, "%s() standalone fuel gauge detected,adc exiting..\n",
				__func__);
		return -ENODEV;

	}
#endif

	di->dev =  &pdev->dev;
	di->poll_index = 0;
	di->battery_present = 1;
	battery_ini_flag = 0;
	
	t = cpu_clock(smp_processor_id());
	do_div(t, 1000000000);
	batt_sleep_time.start_time = t+90;//90	

	do_gettimeofday(&battery_current_time);
	batt_sleep_time.init_time = battery_current_time.tv_sec;
	batt_sleep_time.suspend_flag = false;
	DBG_TPS8003X_INFO("StartTime = %u Real InitTime is %u\n",batt_sleep_time.start_time,batt_sleep_time.init_time);

	//Copy Data From platform data
	di->vbat_max = (pdata->vbat_max) ? pdata->vbat_max : 4300;
	di->vbat_min = (pdata->vbat_min) ? pdata->vbat_min : 2800;//ATTENTION:We won't bootup if vol too low,should update if can boot up when vol < 2.8
	di->slow_poll= (pdata->slow_poll) ? pdata->slow_poll : 20;
	di->fast_poll= (pdata->fast_poll) ? pdata->fast_poll : 10;
	di->fast_poll_vol= (pdata->fast_poll_vol) ? pdata->fast_poll_vol : 3600;
	di->shut_down_vol= (pdata->shut_down_vol) ? pdata->shut_down_vol : 3400;
	di->shut_down_lvl= (pdata->shut_down_lvl) ? pdata->shut_down_lvl : 0;
	di->poll_max_times= (pdata->poll_max_times) ? pdata->poll_max_times : 3;
	di->sleep_up_time= (pdata->sleep_up_time) ? pdata->sleep_up_time : 300;
	di->adc_retry_times= (pdata->adc_retry_times) ? pdata->adc_retry_times : 3;
	di->max_reverse_times= (pdata->max_reverse_times) ? pdata->max_reverse_times : 5;
	di->init_adjust_vol= (pdata->init_adjust_vol) ? pdata->init_adjust_vol : 0;
	di->battery_detect_by_ntc = (pdata->battery_detect_by_ntc) ? pdata->battery_detect_by_ntc : 0;
	di->init_vol_from_btloader = (pdata->init_vol_from_btloader) ? pdata->init_vol_from_btloader : 0;

	di->poll_interval = di->slow_poll;
	di->bat.name		= "battery";
	di->bat.type		= POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties	= tps80031_bat_props;
	di->bat.num_properties	= ARRAY_SIZE(tps80031_bat_props);
	di->bat.get_property	= tps80031_bat_get_property;
	di->inter_resis = 120;//US9230 Vendor's testing result

	ret = power_supply_register(dev->parent, &di->bat);
	if (ret) {
		dev_err(dev->parent, "failed to register bat power supply\n");
		return ret;
	}
#ifdef   CONFIG_TPS80032_USB_CHARGER
	di->usb.name		= "usb";
	di->usb.type		= POWER_SUPPLY_TYPE_USB;
	di->usb.properties	= tps80031_usb_props;
	di->usb.num_properties	= ARRAY_SIZE(tps80031_usb_props);
	di->usb.get_property	= tps80031_usb_get_property;

	ret = power_supply_register(dev->parent, &di->usb);
	if (ret) {
		dev_err(dev->parent, "failed to register usb power supply\n");
		goto power_supply_fail2;
	}
#endif  	
	di->ac.name		= "ac";
	di->ac.type		= POWER_SUPPLY_TYPE_MAINS;
	di->ac.properties	= tps80031_ac_props;
	di->ac.num_properties	= ARRAY_SIZE(tps80031_ac_props);
	di->ac.get_property	= tps80031_ac_get_property;

	ret = power_supply_register(dev->parent, &di->ac);
	if (ret) {
		dev_err(dev->parent, "failed to register ac power supply\n");
		goto power_supply_fail1;
	}

	dev_set_drvdata(&pdev->dev, di);

	ret = register_charging_state_callback(tps80031_battery_status, di);
	if (ret < 0)
		goto power_supply_fail0;

//Enable Fule Gauge Current
	if(tps8003x_current_setup(di,true) < 0)
		dev_err(dev->parent, "failed to setup current\n");
	//Init ADC Mutex Lock
	mutex_init(&di->adc_lock);

	/* initialize all required work */
	INIT_WORK(&di->handler, battery_handle_intrpt);
	//INIT_DELAYED_WORK_DEFERRABLE isn't corrupt API
	INIT_DELAYED_WORK(&di->monitor_battery, battery_monitor);
	di->monitor_wqueue =
			create_singlethread_workqueue("tps80031-adc-battery");
	if (!di->monitor_wqueue) {
		dev_err(dev->parent, "%s(): wqueue init failed\n", __func__);
		ret = -ESRCH;
		goto power_supply_fail0;
	}
	//Set up battery detect irq by ntc
	if(di->battery_detect_by_ntc){
		ret = request_threaded_irq(pdata->irq_base + TPS80031_INT_BAT_TEMP_OVRANGE,
				NULL, tps80031_bat_overtemp,
					IRQF_ONESHOT, "tps80031_bat_overtemp", di);

		if (ret < 0) {
			dev_err(dev->parent, "request IRQ %d fail\n", pdata->irq_base + TPS80031_INT_BAT_TEMP_OVRANGE);
			goto power_supply_fail0;
		}
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	di->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 2;
	di->early_suspend.suspend = tps8003x_adc_bat_early_suspend;
	di->early_suspend.resume = 0;
	register_early_suspend(&di->early_suspend);
#endif

//#ifdef CONFIG_BATTERY_ADC_TPS8003X_DEBUG
         // /sys/class/i2c-dev/i2c-4/device/4-004a/tps80031-adc-battery.0

	if(sysfs_create_group(&pdev->dev.kobj, &adc_battery_attr_group))
		dev_err(dev->parent, "failed to creat adc battery sysfs\n");
//#endif


	dev_info(dev->parent, "ADC battery support ver. %s enabled  shut_lvl: %u  init_vol_bt: %u battery_detect: %u\n",
		DRIVER_VERSION, di->shut_down_lvl,di->init_vol_from_btloader,di->battery_detect_by_ntc);

	//Get Real Charger Online Status(ADC Battery register after USB status)
	charger_type = tps8003x_charger_type();
	if (charger_type == AC_CHARGER) {
		di->ac_online  = 1;
		di->usb_online = 0;
	}
	else if(charger_type == USB_CHARGER) {
		di->ac_online  = 0;
		di->usb_online = 1;
	}
	else {
		di->ac_online  = 0;
		di->usb_online = 0;
	}


	di->need_updated_immediately = true;
        memset(&batt_vol_array,0,sizeof(batt_vol_array));
	queue_delayed_work(di->monitor_wqueue, &di->monitor_battery, HZ);
	//Init Capacity Status Here
	battery_probe_done = true;
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	batt_init();
#endif	
	return ret;

power_supply_fail0:
	power_supply_unregister(&di->ac);
power_supply_fail1:
#ifdef   CONFIG_TPS80032_USB_CHARGER
	power_supply_unregister(&di->usb);
power_supply_fail2:
#endif
	power_supply_unregister(&di->bat);
	return ret;
}

static int tps80031_adc_battery_remove(struct platform_device *pdev)
{
	struct tps80031_device_info *di = dev_get_drvdata(&pdev->dev);

//#ifdef CONFIG_BATTERY_ADC_TPS8003X_DEBUG
	sysfs_remove_group(&pdev->dev.kobj, &adc_battery_attr_group);
//#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&di->early_suspend);
#endif
	mutex_destroy(&di->adc_lock);
	cancel_delayed_work_sync(&di->monitor_battery);
	destroy_workqueue(di->monitor_wqueue);
	cancel_work_sync(&di->handler);

	power_supply_unregister(&di->bat);
#ifdef  CONFIG_TPS80032_USB_CHARGER
	power_supply_unregister(&di->usb);
#endif
	power_supply_unregister(&di->ac);

	return 0;
}


#ifdef CONFIG_PM

static int tps80031_adc_battery_suspend(struct platform_device *pdev,
				  pm_message_t state)
{
	struct tps80031_device_info *di = dev_get_drvdata(&pdev->dev);
	cancel_work_sync(&di->handler);
	cancel_delayed_work_sync(&di->monitor_battery);
	batt_sleep_time.suspend_flag = true;
	if((di->charging_status == POWER_SUPPLY_STATUS_CHARGING) 
	|| (di->charging_status == POWER_SUPPLY_STATUS_FULL))
	batt_sleep_time.charging_in_s3=true;
	else
	batt_sleep_time.charging_in_s3=false;

	return 0;
}

static int tps80031_adc_battery_resume(struct platform_device *pdev)
{
	struct tps80031_device_info *di = dev_get_drvdata(&pdev->dev);
	if(tps8003x_current_setup(di,true) < 0)
		dev_err(di->dev, "failed to setup current\n");
	di->need_updated_immediately =  true;
	queue_delayed_work(di->monitor_wqueue, &di->monitor_battery, HZ/100);//Should update earlier before LCD on
	return 0;
}

#else

#define tps80031_adc_battery_suspend NULL
#define tps80031_adc_battery_resume NULL

#endif

static struct platform_driver tps80031_battery_driver = {
	.driver	= {
		.name	= "tps80031-adc-battery",
		.owner	= THIS_MODULE,
	},
	.probe	= tps80031_adc_battery_probe,
	.remove = tps80031_adc_battery_remove,
	.suspend  = tps80031_adc_battery_suspend,
	.resume	  = tps80031_adc_battery_resume,
};

static int __init tps80031_battery_init(void)
{
	return platform_driver_register(&tps80031_battery_driver);
}

static void __exit tps80031_battery_exit(void)
{
	platform_driver_unregister(&tps80031_battery_driver);
}

module_init(tps80031_battery_init);
module_exit(tps80031_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrew.C.Lee <lichuan@hisensecom.com> ");
MODULE_DESCRIPTION("tps80031 adc battery  driver");
