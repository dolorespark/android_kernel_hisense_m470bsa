/*
 * drivers/power/tps80031_charger.c
 *
 * Battery charger driver for TI's tps80031
 *
 * Copyright (c) 2011-2012, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/tps80031.h>
#include <linux/tps80031-charger.h>
#include <linux/wakelock.h>
#include <linux/android_alarm.h>
#include <linux/suspend.h>

#define CONTROLLER_CTRL1	0xe1
#define CONTROLLER_STAT1	0xe3
#define CHARGERUSB_CTRL2	0xe9
#define CHARGERUSB_CTRL3	0xea
#define CHARGERUSB_VOREG	0xec
#define CHARGERUSB_VICHRG	0xed
#define CHARGERUSB_CINLIMIT	0xee
#define CHARGERUSB_CTRLLIMIT2	0xf0
#define ANTICOLLAPSE_CTRL1      0xf1
#define CHARGERUSB_CTRLLIMIT1	0xef
#define CHARGERUSB_VICHRG_PC	0xdd
#define CONTROLLER_WDG		0xe2
#define LINEAR_CHRG_STS		0xde
#define END_OF_CHARGE		BIT(5)
#define CONTROLLER_CTRL2	0xda
#define CONTROLLER_CTRL2_4200	0x5b //RESET VALUE on A2F7 is 0x1b
#define CONTROLLER_VSEL_COMP    0xdb

#define TPS80031_VBUS_DET	BIT(2)
#define TPS80031_VAC_DET	BIT(3)

#define AC_CHARGER_INPUT_CURRENT_MA	1000

#define VSYSMIN_HI_THRESHOLD		0x24
#define VSYSMIN_HI_THRESHOLD_3300	0x1a
#define VSYSMIN_HI_THRESHOLD_3350	0x1b
#define VSYSMIN_HI_THRESHOLD_3400	0x1c
#define VSYSMIN_HI_THRESHOLD_3500	0x1e

#define VSYSMIN_HI_CFG_STATE		0xca
#define VSYSMIN_HI_CFG_STATE_OFF	0x00
#define VSYSMIN_HI_CFG_STATE_ON		0x01
#define VSYSMIN_HI_CFG_STATE_SLEEP	0x03

#define INT_STS_A			0xD0
#define INT_STS_A_SYS_LOW		0x04
#define INT_MSK_LINE_A			0xD3
#define INT_MSK_LINE_A_SYS_LOW_MSK	0x04
#define INT_MSK_LINE_A_SYS_LOW_EN	0x00
#define INT_MSK_LINE_A_SYS_LOW_DIS	0x04

#define INT_MSK_STS_A			0xD6
#define INT_MSK_STS_A_SYS_LOW_MSK	0x04
#define INT_MSK_STS_A_SYS_LOW_EN	0x00
#define INT_MSK_STS_A_SYS_LOW_DIS	0x04


#define TOGGLE1			0x90
#define TOGGLE1_FGDITHS		BIT(7)
#define TOGGLE1_FGDITHR		BIT(6)
#define TOGGLE1_FGS		BIT(5)
#define	TOGGLE1_FGR		BIT(4)
#define TOGGLE1_GPADCR		BIT(1)

//Fule Gauge Func for Current Cal
#define FG_REG_00		0xC0
#define FG_REG_00_CC_AUTOCLEAR	BIT(2)
#define FG_REG_00_CC_CAL_EN	BIT(1)
#define FG_REG_08		0xC8
#define FG_REG_09		0xC9
#define FG_REG_10		0xCA
#define FG_REG_11		0xCB

#if defined(CONFIG_BATTERY_BQ27x00)
#define CHARGERUSB_STATUS_INT1	0xe6
#define TPS80031_VBUS_OVP	BIT(0)
#define TPS80031_BAT_OVP	BIT(3)
#define TPS80031_FAULT_WDG	BIT(4)
#define TPS80031_CH_GATED	BIT(6)

#define TPS80031_BAT_TEMP_OV	BIT(0)
#define TPS80031_VBATOV		BIT(4)
#define TPS80031_VSYSOV		BIT(3)
#define TPS80031_DPPM		BIT(2)
#define TPS80031_CV_STS		BIT(1)
#define TPS80031_CC_STS		BIT(0)
#define TPS80031_FAST_VOL	3000
extern int  battery_temp_by_gasgauge(void);
extern bool battery_standalone_fg(void);
extern void tps8003x_update_battery_status(void);
#endif

#ifdef CONFIG_BATTERY_ADC_TPS8003X
extern int battery_temp_by_adc(void);
#elif defined CONFIG_BATTERY_GAUGE_TPS8003X
extern int battery_temp_by_gauge(void);
#endif

#define State_Charger_PlugIn	0x00000800 //AC usb plugin
#define State_Normal_USB_PlugIn	0x40000000 //normal usb
#define USB_CHARGER_CURRENT_UA  (500*1000) //500mA
#define GET_TRUE_TIME_DELAY	15
extern unsigned int tegra_power_reason;
static bool first_plug = true;
#ifdef	CONFIG_TPS80032_USB_CHARGER
#define	CHARGE_USB_SAFETY_TIMER	57600 //16 h in seconds
#define	CHARGE_AC_SAFETY_TIMER	43200 //12 h in seconds
#endif

static struct wake_lock notifier_wake_lock;
static unsigned long long system_run_time = 0;//Check printk func


struct tps80031_charger {
	int			max_charge_current_mA;
	int			max_charge_volt_mV;
	struct device		*dev;
	struct regulator_dev	*rdev;
	struct regulator_desc	reg_desc;
	struct regulator_init_data		reg_init_data;
	struct tps80031_charger_platform_data	*pdata;
	int (*board_init)(void *board_data);
	void			*board_data;
	int			irq_base;
	int			watch_time_sec;
	enum charging_states	state;
	int			charging_term_current_mA;
	charging_callback_t	charger_cb;
	void			*charger_cb_data;
	uint8_t ac_online; //ADD For BQ27541 Fule Gauge
	uint8_t usb_online;
	int			watch_reset_time_sec;
	struct delayed_work wdg_work;//For Watchdog reset work acording to FAE's advice.
	struct delayed_work first_detect_work;//Keep Current Setting by Charger
	/*ADD Sleep wakeup function*/
	struct notifier_block	pm_notifier;
	struct alarm	alarm;
	//ktime_t last_alarm_time;//Take care of other reason waking up when Sleep with AC Charger
	//software timer func
	int safety_timer_en;
	long safety_timer_sec;
	long charge_start_sec;
	long charge_continue_sec;
	//jeita current
	int jeita_control_en;
	int in_jeita_curr_control;
	int in_jeita_vol_control;
	int jeita_limited_current;
	int jeita_limited_voltage;
	int jeita_limited_temp_low;
	int jeita_limited_temp_high;
	int jeita_limited_temp_cap;
};

static struct tps80031_charger *charger_data;
static int tps80031_lowpower_msk_en(struct tps80031_charger *charger,bool enable);
static int configure_charging_parameter(struct tps80031_charger *charger,bool init);
static int jeita_voltage_limited_control(struct tps80031_charger *charger,bool enter);

#ifdef CONFIG_CHARGER_TPS8003X_DEBUG
static struct kobject *tps80031_charger_dbg_kobj;

#define charger_debug_attr(_name) \
	static struct kobj_attribute _name##_attr = { \
	.attr = { \
	.name = __stringify(_name), \
	.mode = 0644, \
	}, \
	.show = _name##_show, \
	.store = _name##_store,\
	}



static ssize_t status_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	struct tps80031_charger *charger = charger_data;
	int ret = 0;
	uint8_t value;
	int i;
        s += sprintf(s, "TPS80032 Charger Status\n");
	for( i= 0xDA;i <= 0xF5;i++)
	{
		if(i == 0xDF)
		continue;

		ret = tps80031_read(charger->dev->parent, SLAVE_ID2, i, &value);
		if(ret >= 0)
		s += sprintf(s, "0x%2x = 0x%x\n", i,value);
		else
		s += sprintf(s, "0x%2x  Read Failed\n", i);
	}

	return (s - buf);

}

static ssize_t status_store(struct kobject *kobj, struct kobj_attribute *attr,const char *buf, size_t count)
{
	return count;
}

static ssize_t config_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	struct tps80031_charger *charger = charger_data;
        uint8_t charge_voltage_limited;
	uint8_t charge_voltage;
	uint8_t buck_vth;
	uint8_t term_i;
	uint8_t wtg_sts;
	uint8_t chg_sts;

	if(tps80031_read(charger->dev->parent, SLAVE_ID2, CHARGERUSB_CTRLLIMIT1, &charge_voltage_limited) >= 0)
	s += sprintf(s, "CHARGERUSB_CTRLLIMIT1 ****ID2 Register 0x%x value is 0x%x\n", CHARGERUSB_CTRLLIMIT1, charge_voltage_limited);//DM 4.9.7 Page 93

	if(tps80031_read(charger->dev->parent, SLAVE_ID2, CHARGERUSB_VOREG, &charge_voltage) >= 0)
	s += sprintf(s, "CHARGERUSB_VOREG ****ID2 Register 0x%x value is 0x%x\n", CHARGERUSB_VOREG, charge_voltage);

	if(tps80031_read(charger->dev->parent, SLAVE_ID2, ANTICOLLAPSE_CTRL1, &buck_vth) >= 0)
	s += sprintf(s, "ANTICOLLAPSE_CTRL1 ****ID2 Register 0x%x value is 0x%x\n", ANTICOLLAPSE_CTRL1, buck_vth);

	if(tps80031_read(charger->dev->parent, SLAVE_ID2, CHARGERUSB_CTRL2, &term_i) >= 0)
	s += sprintf(s, "CHARGERUSB_CTRL2 ****ID2 Register 0x%x value is 0x%x\n", CHARGERUSB_CTRL2, term_i);

	if(tps80031_read(charger->dev->parent, SLAVE_ID2, CONTROLLER_STAT1, &chg_sts) >= 0)
	s += sprintf(s, "CONTROLLER_STAT1 ****ID2 Register 0x%x value is 0x%x\n", CONTROLLER_STAT1, chg_sts);

	if(tps80031_read(charger->dev->parent, SLAVE_ID2, CONTROLLER_WDG, &wtg_sts) >= 0)
	s += sprintf(s, "CONTROLLER_WDG ****ID2 Register 0x%x value is 0x%x\n", CONTROLLER_WDG, wtg_sts);

	return (s - buf);
}

static ssize_t config_store(struct kobject *kobj, struct kobj_attribute *attr,const char *buf, size_t count)
{
	int rc;
	unsigned long switcher;
	struct tps80031_charger *charger = charger_data;
	rc = strict_strtoul(buf, 0, &switcher);

	if(rc){
		printk("Get switcher value failed\n");
		return rc;
	}

	if(switcher == 0){
		printk("Do nothing for now !\n");
	}
	else if(switcher == 1){
		rc = configure_charging_parameter(charger,false);
		if (rc)
		printk("Reconfigure Charger Fail!\n");
		else
		printk("Reconfigure Charger Done!\n");
	}
	else{
		printk("Wrong Value for Charger config!\n");
	}

	return count;
}


static ssize_t safety_timer_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	struct tps80031_charger *charger = charger_data;
	if (!charger)
	s += sprintf(s, "Driver Not ready\n");
	else
	s += sprintf(s, "safety_timer now is %ld\n", charger->safety_timer_sec);

	return (s - buf);
}

static ssize_t safety_timer_store(struct kobject *kobj, struct kobj_attribute *attr,const char *buf, size_t count)
{
	int  ret;
	long timer_value;
	struct tps80031_charger *charger = charger_data;
	if (!charger)
	return count;

	ret = strict_strtol(buf, 0, &timer_value);

	if(ret){
		printk("Get timer value failed\n");
		return ret;
	}

	if(timer_value < 0)
		timer_value = 0;

	if(timer_value < 21600 || timer_value > 86400)
		printk("WARNING:Charger Saftey Timer Should Be 21600~86400!\n");
	else
	charger->safety_timer_sec = timer_value;

	return count;
}

static ssize_t r2_det_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	bool flag = false;
#ifdef CONFIG_BATTERY_BQ27x00
	flag = battery_standalone_fg();
#endif
	s += sprintf(s, "gasgauge: %d\n",flag);
	return (s - buf);
}

static ssize_t r2_det_store(struct kobject *kobj, struct kobj_attribute *attr,const char *buf, size_t count)
{
	struct tps80031_charger *charger = charger_data;
	int  ret = 0;
	uint8_t	reg = 0;
	unsigned long switcher;
	ret = strict_strtoul(buf, 0, &switcher);
	if(ret){
		printk("Get switcher value failed\n");
		return ret;
	}

	if(switcher == 0){
		printk("Do nothing here!\n");
	}
	else if(switcher == 1){
#ifdef CONFIG_BATTERY_BQ27x00
		if(battery_standalone_fg()) {
#endif
			//Clear
			ret = tps80031_write(charger->dev->parent, SLAVE_ID2, FG_REG_00,FG_REG_00_CC_AUTOCLEAR);
			if (ret)
			dev_err(charger->dev, "failed to write FG_REG_00: CC_AUTOCLEAR\n");
			//Enable Fule Gauge

			reg = TOGGLE1_FGDITHS | TOGGLE1_FGS;

			ret = tps80031_update(charger->dev->parent, SLAVE_ID2,
				TOGGLE1, reg, 0xF0);
			if (ret)
				dev_err(charger->dev, "failed to write TOGGLE1: TOGGLE1_FGS\n");

			//Enables calibration (We choose 250-ms update rate,4 times result per second)
			ret = tps80031_write(charger->dev->parent, SLAVE_ID2, FG_REG_00,FG_REG_00_CC_CAL_EN);
			if (ret)
				dev_err(charger->dev, "failed to write FG_REG_00: CC_CAL_EN\n");
#ifdef CONFIG_BATTERY_BQ27x00
		}
		else
			dev_info(charger->dev, "FG Already Enable\n");
#endif
	}
	else
		dev_err(charger->dev,"Wrong Config for r2 det!\n");

	return count;
}

static ssize_t r2_curr_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	/*int charging_mA = 0;
	charging_mA = tps80031_gpadc_conversion(BATTERY_CHARGING_CURRENT);
	s += sprintf(s, "%d\n",charging_mA);*/  //By ADC
	struct tps80031_charger *charger = charger_data;
	int ret = 0;
	int current_now = 0;
	uint16_t offset_raw_value = 0;
	uint16_t curr_raw_value = 0;

	int16_t  temp = 0;
	int16_t  cc_offset = 0;

	/* FG_REG_08, 09 is 10 bit signed calibration offset value */
	ret = tps80031_reads(charger->dev->parent, SLAVE_ID2, FG_REG_08, 2,
							(uint8_t *) &offset_raw_value);
	if (ret)
		dev_err(charger->dev, "failed to read FG_REG_8: current off set\n");
	cc_offset = ((int16_t)(offset_raw_value << 6) >> 6);

	/* FG_REG_10, 11 is 14 bit signed instantaneous current sample value */
	ret = tps80031_reads(charger->dev->parent, SLAVE_ID2, FG_REG_10, 2,
							(uint8_t *) &curr_raw_value);
	if (ret)
		dev_err(charger->dev, "failed to read FG_REG_10: current_now\n");
	temp = ((int16_t)(curr_raw_value << 2) >> 2);

	current_now = temp - cc_offset;

	/* current drawn per sec */
	current_now = current_now * 1;//N = 1 for an integration period of 250 ms.
	/* current in mA*/
	current_now = (current_now * (62000/25)) >> 13; //R2 should be 20, but now is about 25
	if((charger->usb_online == 1)||(charger->ac_online == 1))
	current_now = 0;//Fail If Charger Plug for FT

	s += sprintf(s, "%d\n",current_now);//By FG

	return (s - buf);
}

static ssize_t r2_curr_store(struct kobject *kobj, struct kobj_attribute *attr,const char *buf, size_t count)
{
	return count;
}

static ssize_t enable_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	struct tps80031_charger *charger = charger_data;
	int ret = 0;
	char * s = buf;
	uint8_t chg_sts;
	bool status = false;
	if(charger->ac_online || charger->usb_online) {
		ret = tps80031_read(charger_data->dev->parent, SLAVE_ID2, CONTROLLER_CTRL1, &chg_sts);
		if (ret < 0)
			dev_err(charger->dev, "%s(): Failed in reading register 0x%02x\n",__func__, CONTROLLER_CTRL1);
		else if(chg_sts == 0x30)
			status = true;
	}

	s += sprintf(s, "%d\n",status);
	return (s - buf);
}

static ssize_t enable_store(struct kobject *kobj, struct kobj_attribute *attr,const char *buf, size_t count)
{
	struct tps80031_charger *charger = charger_data;
	int ret = 0;
	uint8_t chg_reg;
	int charge_current;
	unsigned long onoff;
	ret = strict_strtoul(buf, 0, &onoff);

	if(ret){
		dev_err(charger->dev, "%s(): Failed in get charger enable command \n",__func__);
		return ret;
	}

	if(onoff == 0){//VBUS && LINEAR
		if(charger->ac_online || charger->usb_online) {
			ret = tps80031_read(charger_data->dev->parent, SLAVE_ID2, CONTROLLER_CTRL1, &chg_reg);
			if (ret < 0)
				dev_err(charger->dev, "%s(): Failed in reading register 0x%02x\n",__func__, CONTROLLER_CTRL1);
			else if(chg_reg == 0x00)
				dev_info(charger->dev, "%s():Charger Already Disable\n",__func__);
			else{
				//cancel watchdog writing work
				cancel_delayed_work(&charger->wdg_work);

				ret = tps80031_write(charger->dev->parent, SLAVE_ID2,CONTROLLER_CTRL1, 0x0);
				if (ret < 0)
				dev_err(charger->dev, "%s():Failed in Disable Charger\n",__func__);
				else
				dev_info(charger->dev, "%s():Disable Charger\n",__func__);
			}
		}
		else
		dev_info(charger->dev, "%s():No Charger Online\n",__func__);
	}
	else if(onoff == 1) {//VBUS && LINEAR
		if(charger->ac_online || charger->usb_online) {
			ret = tps80031_read(charger_data->dev->parent, SLAVE_ID2, CONTROLLER_CTRL1, &chg_reg);
			if (ret < 0)
				dev_err(charger->dev, "%s(): Failed in reading register 0x%02x\n",__func__, CONTROLLER_CTRL1);
			else if(chg_reg == 0x30)
				dev_info(charger->dev, "%s():Charger Already Enable\n",__func__);
			else {

				/* Enable watchdog timer */
				ret = tps80031_write(charger->dev->parent, SLAVE_ID2,CONTROLLER_WDG, charger->watch_time_sec);
				if (ret < 0) {
					dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",__func__, CONTROLLER_WDG);
					return ret;
				}
				/*Reconfigure Charger*/
				ret = configure_charging_parameter(charger,false);
				if (ret < 0) {
					dev_err(charger->dev, "%s(): configure_charging_parameter failed\n",__func__);
					return ret;
				}
				/*VBUS limited*/
				if(charger->ac_online)
					chg_reg = 0x20;
				else
					chg_reg = 0x09;

				ret = tps80031_update(charger->dev->parent, SLAVE_ID2,CHARGERUSB_CINLIMIT, chg_reg, 0x3F);
				if (ret < 0) {
					dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",__func__, CHARGERUSB_CINLIMIT);
					return ret;
				}
				/*Charging limited*/
				if(charger->ac_online)
					charge_current = 1500;
				else
					charge_current = 1000;

				if (charge_current <= 100)
					charge_current = 0;
				else
					charge_current = (charge_current - 100) / 100;  //POP = 1

				ret = tps80031_update(charger->dev->parent, SLAVE_ID2,CHARGERUSB_VICHRG, (uint8_t)charge_current, 0xF);
				if (ret < 0) {
					dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",__func__, CHARGERUSB_VICHRG);
					return ret;
				}

				/* Enable the charging */
				ret = tps80031_write(charger->dev->parent, SLAVE_ID2,CONTROLLER_CTRL1, 0x30);
				if (ret < 0) {
					dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",__func__, CONTROLLER_CTRL1);
					return ret;
				}
				//Clear previous work
				cancel_delayed_work_sync(&charger->wdg_work);
				//Start Watchdog Writing Work
				schedule_delayed_work(&charger->wdg_work, charger->watch_reset_time_sec * HZ);

			}
		}
		else
		dev_info(charger->dev, "%s():No Charger Online\n",__func__);
	}

	return count;
}

charger_debug_attr(status);
charger_debug_attr(config);
charger_debug_attr(safety_timer);
charger_debug_attr(r2_det);
charger_debug_attr(r2_curr);
charger_debug_attr(enable);

static struct attribute *tps80031_charger_attributes[] = {
	&status_attr.attr,
	&config_attr.attr,
	&safety_timer_attr.attr,
	&r2_det_attr.attr,
	&r2_curr_attr.attr,
	&enable_attr.attr,
	NULL
};

static const struct attribute_group tps80031_charger_attr_group = {
	.attrs = tps80031_charger_attributes,
};
#endif

static uint8_t tps80031_get_vbus_input_current_limit_code(int max_uA)
{
	const uint8_t current_to_code[] = {
		0x0,				    /* 0 - 50 mA */
		0x0,  0x1,  0x2,  0x3,  0x4,  0x5,  /* 50,  100,  ..., 300mA */
		0x6,  0x7,  0x8,  0x9,  0xA,  0xB,  /* 350, 400,  ..., 600mA */
		0xC,  0xD,  0xE,  0x27, 0x37, 0x28, /* 650, 700,  ..., 900mA */
		0x38, 0x29, 0x39, 0x2A, 0x3A, 0x2B, /* 950, 1000,  ..., 1200mA */
		0x3B, 0x2C, 0x3C, 0x2D, 0x3D, 0x2E, /* 1200,1250, ..., 1500mA */
	};
	int charge_mA;
	uint8_t code;

	charge_mA = max_uA / 1000;
	if (charge_mA < 0)
		BUG();
	else if (charge_mA < 1800)
		code = current_to_code[charge_mA / 50];
	else if (charge_mA < 2100)
		code = 0x20; /* use 1800mA code */
	else if (charge_mA < 2250)
		code = 0x21; /* use 2100mA code */
	else
		code = 0x22; /* use 2250mA code */

	return code;
};


//ADD vbus Status Detect by Charger IC
bool tps8003x_vbus_status(void)
{
	int ret = 0;
	uint8_t chg_sts;
	if (!charger_data)
		return false;

	ret = tps80031_read(charger_data->dev->parent, SLAVE_ID2, CONTROLLER_STAT1, &chg_sts);
	if(ret < 0)
	{
		dev_err(charger_data->dev, "%s(): Failed in reading register 0x%02x\n",__func__, CONTROLLER_STAT1);
		return false;
	}
	if((chg_sts & 0x04) != 0)
	return true;
	else
	return false;
}
EXPORT_SYMBOL_GPL(tps8003x_vbus_status);


static bool tps8003x_end_of_charge(struct tps80031_charger *charger)
{
	uint8_t linch_status;
	int ret;

	ret = tps80031_read(charger->dev->parent, SLAVE_ID2,LINEAR_CHRG_STS, &linch_status);
	if(ret < 0)
		return false;
	else if(linch_status & END_OF_CHARGE)
		return true;
	else
		return false;
}

static int safety_timer_check(struct tps80031_charger *charger,bool start,bool stop)//TPS80032 doesn't have such function with SW_WTD
{
	struct timeval charger_current_time;
	charger_current_time = ktime_to_timeval(alarm_get_elapsed_realtime());
        //do_gettimeofday(&charger_current_time);//user may changes system time during charging
	if(start) {
		system_run_time = cpu_clock(smp_processor_id());
		do_div(system_run_time, 1000000000);
		if(system_run_time  <= GET_TRUE_TIME_DELAY)
			charger->charge_start_sec = 0;
		else
			charger->charge_start_sec = charger_current_time.tv_sec;
		printk("***charge start time is %ld sec  system_run_time is %llu\n",charger->charge_start_sec,system_run_time);

	}
	else if(stop) {

		printk("***charge stop time is %ld sec\n",charger_current_time.tv_sec);
		charger->charge_start_sec = -1;
		charger->charge_continue_sec = 0;
	}
	else if((charger->usb_online == 1)||(charger->ac_online == 1)){
		if(charger->charge_start_sec >= 0) {
			if((charger->charge_start_sec == 0)||tps8003x_end_of_charge(charger))//Update charge_start_sec  if end of charge
				charger->charge_start_sec = charger_current_time.tv_sec;
			charger->charge_continue_sec = charger_current_time.tv_sec - charger->charge_start_sec;
			//printk("***charge continue time is %ld sec in_jeita_vol_control %d in_jeita_curr_control %d\n",
			//	charger->charge_continue_sec,charger->in_jeita_vol_control,charger->in_jeita_curr_control);
		}
	}
	return 0;
}


static int safety_timer_expired(struct tps80031_charger *charger)//Stop Charger if safety timer expired
{
	int ret = 0;
	dev_info(charger->dev, "%s() safety timer expired %ld,STOP\n", __func__,charger->charge_continue_sec);

	ret = tps80031_write(charger->dev->parent, SLAVE_ID2,CONTROLLER_WDG, 0x0);
	if (ret < 0)
		dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",__func__, CONTROLLER_WDG);

	return ret;
}


static int jeita_vichrg_reg(int curr) //POP =1
{
	if (curr <= 100)
		return  0;
	else
		return ((curr - 100) / 100);
}

static int jeita_voreg_reg(int vol)
{
	vol = min(4760, vol);
	vol = max(3500, vol);

	vol -= 3500;
	return vol/20;

}

static int jeita_current_limited_control(struct tps80031_charger *charger,bool enter)
{
	int ret = 0;
	int current_limited = 0;
	int current_limited_orig = 0;

	current_limited = jeita_vichrg_reg(charger->jeita_limited_current);
	if(charger->ac_online == 1)
		current_limited_orig = jeita_vichrg_reg(min(charger->max_charge_current_mA,1500));
	else
		current_limited_orig = jeita_vichrg_reg(USB_CHARGER_CURRENT_UA / 1000);

	if(charger->in_jeita_vol_control) {
		jeita_voltage_limited_control(charger,false);
		charger->in_jeita_vol_control = 0;
	}

	if(enter)
	//Set Current to 0.1C
		ret = tps80031_update(charger->dev->parent, SLAVE_ID2,CHARGERUSB_VICHRG, (uint8_t)current_limited, 0xF);
	else
		ret = tps80031_update(charger->dev->parent, SLAVE_ID2,CHARGERUSB_VICHRG, (uint8_t)current_limited_orig, 0xF);

	return ret;
}

static int jeita_voltage_limited_control(struct tps80031_charger *charger,bool enter)
{
	int ret = 0;
	int vol_limited = 0;
	int vol_limited_orig = 0;

	vol_limited = jeita_voreg_reg(charger->jeita_limited_voltage);
	vol_limited_orig = jeita_voreg_reg(charger->max_charge_volt_mV);

	if(charger->in_jeita_curr_control) {
		jeita_current_limited_control(charger,false);
		charger->in_jeita_curr_control = 0;
	}

	if(enter)
	//Set Voltage to 4.1C
		ret = tps80031_write(charger->dev->parent, SLAVE_ID2,CHARGERUSB_VOREG, (uint8_t)vol_limited);
	else
		ret = tps80031_write(charger->dev->parent, SLAVE_ID2,CHARGERUSB_VOREG, (uint8_t)vol_limited_orig);

	return ret;
}



static int jeita_limited_detect(struct tps80031_charger *charger)//
{
	//Get Battery Temperature here
	int battery_temp_now = 0;
	int ret = 0;
#ifdef CONFIG_BATTERY_BQ27x00
//To-Do Check Do we have independent fuel gauge
	if(battery_standalone_fg()) {
	battery_temp_now = battery_temp_by_gasgauge();
	}
	else {
#endif
#ifdef CONFIG_BATTERY_ADC_TPS8003X
	battery_temp_now = battery_temp_by_adc();
#elif defined CONFIG_BATTERY_GAUGE_TPS8003X
	battery_temp_now = battery_temp_by_gauge();
#else
	battery_temp_now = 200;//Fake vaule
#endif
#ifdef CONFIG_BATTERY_BQ27x00
	}
#endif
	if(charger->in_jeita_curr_control) {
		if(battery_temp_now >= charger->jeita_limited_temp_high ) {//shouldn't happen for real user case
			ret = jeita_voltage_limited_control(charger,true);
			if(ret < 0)
			dev_err(charger->dev, "%s(): Enter JEITA VOL Fail\n",__func__);
			else {
				dev_info(charger->dev, "%s():JEITA CURR->VOL Control\n",__func__);
				charger->in_jeita_vol_control = 1;
			}
		}
		else if(battery_temp_now >= (charger->jeita_limited_temp_low + charger->jeita_limited_temp_cap)){
			ret = jeita_current_limited_control(charger,false);
			if(ret < 0)
			dev_err(charger->dev, "%s(): Exit JEITA CURR Fail\n",__func__);
			else{
				charger->in_jeita_curr_control = 0;
				dev_info(charger->dev, "%s(): Exit JEITA CURR Control\n",__func__);
			}
		}
	}
	else if(charger->in_jeita_vol_control) {
		if(battery_temp_now <= charger->jeita_limited_temp_low) {//shouldn't happen for real user case
			ret = jeita_current_limited_control(charger,true);
			if(ret < 0)
			dev_err(charger->dev, "%s(): Enter JEITA CURR Fail\n",__func__);
			else {
				dev_info(charger->dev, "%s():JEITA VOL->CURR Control\n",__func__);
				charger->in_jeita_curr_control = 1;
			}
		}
		else if(battery_temp_now <= (charger->jeita_limited_temp_high - charger->jeita_limited_temp_cap)){
			ret = jeita_voltage_limited_control(charger,false);
			if(ret < 0)
			dev_err(charger->dev, "%s(): Exit JEITA VOL Fail\n",__func__);
			else{
				charger->in_jeita_vol_control = 0;
				dev_info(charger->dev, "%s(): Exit JEITA VOL Control\n",__func__);
			}
		}

	}
	else{
		if(battery_temp_now <= charger->jeita_limited_temp_low) {
			ret = jeita_current_limited_control(charger,true);
			if(ret < 0)
			dev_err(charger->dev, "%s(): Enter JEITA CURR Fail\n",__func__);
			else {
				dev_info(charger->dev, "%s(): Enter JEITA CURR Control\n",__func__);
				charger->in_jeita_curr_control = 1;
			}
		}
		else if(battery_temp_now >= charger->jeita_limited_temp_high ) {
			ret = jeita_voltage_limited_control(charger,true);
			if(ret < 0)
			dev_err(charger->dev, "%s(): Enter JEITA VOL Fail\n",__func__);
			else {
				dev_info(charger->dev, "%s(): Enter JEITA VOL Control\n",__func__);
				charger->in_jeita_vol_control = 1;
			}
		}
	}
	return 0;
}


static int set_charge_current_limit(struct regulator_dev *rdev,
		int min_uA, int max_uA)
{
	struct tps80031_charger *charger = rdev_get_drvdata(rdev);
	int max_charge_current = 1500;
	uint8_t code;
	int ret = 0;

	cancel_delayed_work_sync(&charger->first_detect_work);

	dev_info(charger->dev, "%s(): Min curr %dmA and max current %dmA\n",
		__func__, min_uA/1000, max_uA/1000);

	if (!max_uA) {

		if(tps8003x_vbus_status())
		{
			dev_info(charger->dev, "%s():Charger still online,Keep it\n",__func__);
			return ret;
		}
		//if alarm pending,cancel it
		alarm_try_to_cancel(&charger->alarm);

		//wake_lock_timeout(&notifier_wake_lock, 3*HZ);//dealt by otg wake lock
		//Set Current back to  500mA
		ret = tps80031_update(charger->dev->parent, SLAVE_ID2,
			CHARGERUSB_CINLIMIT, 0x09, 0x3F);
		if (ret < 0) {
			dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
					__func__, CHARGERUSB_CINLIMIT);
			return ret;
		}

		ret = tps80031_write(charger->dev->parent, SLAVE_ID2,
						CONTROLLER_CTRL1, 0x0);
		if (ret < 0)
			dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
				__func__, CONTROLLER_CTRL1);

		ret = tps80031_write(charger->dev->parent, SLAVE_ID2,
						CONTROLLER_WDG, 0x0);
		if (ret < 0)
			dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
				__func__, CONTROLLER_WDG);
		charger->state = charging_state_charging_stopped;
		charger->ac_online =  0;
		charger->usb_online = 0;

#ifdef CONFIG_BATTERY_BQ27x00//We could not use charger callback for compatible solution
	if(battery_standalone_fg()) {
		tps8003x_update_battery_status();
	}
	else {
#endif
		if (charger->charger_cb)
			charger->charger_cb(charger->state,
					charger->charger_cb_data);
#ifdef CONFIG_BATTERY_BQ27x00
	}
#endif
		//cancel watchdog writing work
		cancel_delayed_work_sync(&charger->wdg_work);

		//safety_timer_check stop
		safety_timer_check(charger,false,true);
		if(charger->in_jeita_curr_control)
			charger->in_jeita_curr_control = 0;
		if(charger->in_jeita_vol_control)
			charger->in_jeita_vol_control = 0;
#ifdef CONFIG_TPS80032_USB_CHARGER
		if(charger->safety_timer_en)//Back to Default Settings
			charger->safety_timer_sec = CHARGE_AC_SAFETY_TIMER;
#endif
		return ret;
	}

	if(charger->ac_online == 1)
	{

		dev_info(charger->dev, "%s():AC Charger online,do nothing\n",__func__);
		return ret;

	}

	//wake_lock_timeout(&notifier_wake_lock, 3*HZ);//dealt by otg wake lock

	/* Enable watchdog timer */
	ret = tps80031_write(charger->dev->parent, SLAVE_ID2,
				CONTROLLER_WDG, charger->watch_time_sec);
	if (ret < 0) {
		dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
				__func__, CONTROLLER_WDG);
		return ret;
	}

        /*Reconfigure Charger*/
	/*According to RMAP,Wring 0 to CONTROLLER_WDG will Stop charging and go to Watchdog Fault
	Also Refer to DM 4.9.13.1.1 Charger Controller Interrupts
	FAULT_WDG: The charging watchdog has expired. Host processor must reset the timer in the CONTROLLER_WDG register. In addition,
	host must initialize the charging and restart it.
	*/
	ret = configure_charging_parameter(charger,false);
	if (ret < 0) {
		dev_err(charger->dev, "%s(): configure_charging_parameter failed\n",
				__func__);
		return ret;
	}

	if((max_uA == USB_CHARGER_CURRENT_UA)  &&
		((tegra_power_reason & State_Charger_PlugIn) && first_plug) ) {
			dev_info(charger->dev, "USB Charger first plug\n");
			first_plug = false;
			//Start first plug check
			schedule_delayed_work(&charger->first_detect_work, 3 * HZ);
	}
	else
	{
		code = tps80031_get_vbus_input_current_limit_code(max_uA);

		ret = tps80031_update(charger->dev->parent, SLAVE_ID2,
				CHARGERUSB_CINLIMIT, code, 0x3F);
		if (ret < 0) {
			dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
					__func__, CHARGERUSB_CINLIMIT);
			return ret;
		}
	}
	max_charge_current = min(max_uA/1000, max_charge_current);

	if(max_uA == USB_CHARGER_CURRENT_UA)
		max_charge_current = 1000; //it's safe setting with USB cable with CHARGERUSB_CINLIMIT

	if (max_charge_current <= 100)
		max_charge_current = 0;
	else
		max_charge_current = (max_charge_current - 100) / 100;  //POP = 1

	ret = tps80031_update(charger->dev->parent, SLAVE_ID2,
			CHARGERUSB_VICHRG, (uint8_t)max_charge_current, 0xF);/*R2  sense resitor should be 20mΩ,but now we believe it's about 23-25 mΩ
									      for 1000/820  1200/1025  1500/1280 factor*/
	if (ret < 0) {
		dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
				__func__, CHARGERUSB_VICHRG);
		return ret;
	}

	/* Enable the charging */
	ret = tps80031_write(charger->dev->parent, SLAVE_ID2,
				CONTROLLER_CTRL1, 0x30);
	if (ret < 0) {
		dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
				__func__, CONTROLLER_CTRL1);
		return ret;
	}
	charger->state = charging_state_charging_in_progress;

	if (max_uA >= (AC_CHARGER_INPUT_CURRENT_MA *1000)) {
		charger->ac_online = 1;
		charger->usb_online = 0;
#ifdef CONFIG_TPS80032_USB_CHARGER
		if(charger->safety_timer_en)
			charger->safety_timer_sec = CHARGE_AC_SAFETY_TIMER;
#endif
	}
	else {
		charger->ac_online = 0;
		charger->usb_online = 1;
#ifdef CONFIG_TPS80032_USB_CHARGER
		if(charger->safety_timer_en)
			charger->safety_timer_sec = CHARGE_USB_SAFETY_TIMER;
#endif
	}
#ifdef CONFIG_BATTERY_BQ27x00//We could not use charger callback for compatible solution
	if(battery_standalone_fg()) {
		tps8003x_update_battery_status();
	}
	else {
#endif
	if (charger->charger_cb)
		charger->charger_cb(charger->state,
				charger->charger_cb_data);  //For TSP80031_ADC/TPS80031_GAUGE
#ifdef CONFIG_BATTERY_BQ27x00
	}
#endif
	//Clear previous work
	cancel_delayed_work_sync(&charger->wdg_work);
	//Start Watchdog Writing Work
	schedule_delayed_work(&charger->wdg_work, charger->watch_reset_time_sec * HZ);
	//safety_timer_check start
	safety_timer_check(charger,true,false);
	//JEITA Control
	if(charger->jeita_control_en)
		jeita_limited_detect(charger);

	return 0;
}

static struct regulator_ops tegra_regulator_ops = {
	.set_current_limit = set_charge_current_limit,
};

int register_charging_state_callback(charging_callback_t cb, void *args)
{
	struct tps80031_charger *charger = charger_data;
	if (!charger_data)
		return -ENODEV;

	charger->charger_cb = cb;
	charger->charger_cb_data = args;
	return 0;
}
EXPORT_SYMBOL_GPL(register_charging_state_callback);

#ifdef CONFIG_BATTERY_BQ27x00
//M470's 4000mA Battery GasGauge doesn't work well for DSG flag detect,we using Charger's detection instead.
uint8_t  fg_tps8003x_charging_status(void)
{
	uint8_t status = CHARGING_UNKNOW;
	uint8_t chg_status;
	uint8_t chg_usb_status;
	uint8_t con_status;
	int ret;


	if (!charger_data)
		goto  out;
#if defined(CONFIG_TPS80032_USB_CHARGER)
		if((charger_data->ac_online == 1) || (charger_data->usb_online == 1)){
#else
		if(charger_data->ac_online == 1){
#endif
		//CC/CV status doesn't work well
		ret = tps80031_read(charger_data->dev->parent, SLAVE_ID2, CONTROLLER_STAT1, &con_status);
		if (ret < 0){
			dev_err(charger_data->dev,"%s Get CONTROLLER_STAT1 Failed\n",__func__);
			goto  out;
		}
		if(con_status & TPS80031_VBUS_DET) {
			if(con_status & TPS80031_BAT_TEMP_OV)
			status = CHARGING_OVER_HEAT;
			else if(con_status & TPS80031_FAULT_WDG)
			status = CHARGING_GATED;
			else
			{

				ret = tps80031_read(charger_data->dev->parent, SLAVE_ID2, CHARGERUSB_STATUS_INT1, &chg_usb_status);
				if (ret < 0){
					dev_err(charger_data->dev,"%s Get CHARGERUSB_STATUS_INT1 Failed\n",__func__);
					goto  out;
				}

				ret = tps80031_read(charger_data->dev->parent, SLAVE_ID2, LINEAR_CHRG_STS, &chg_status);
				if (ret < 0){
					dev_err(charger_data->dev,"%s Get LINEAR_CHRG_STS Failed\n",__func__);
					goto  out;
				}
				if(chg_status & END_OF_CHARGE)
				status = CHAEGING_DONE;
				else if( ((chg_status & TPS80031_VBATOV)||(chg_status & TPS80031_VSYSOV)) ||
					 ((chg_usb_status & TPS80031_VBUS_OVP)|| (chg_usb_status & TPS80031_BAT_OVP)))
				status = CHARGING_OVER_VOLTAGE;
				else
				status = CHAEGING_CHGING;
			}
		}
		else
		status = CHARGING_VBUS_NO_PRESENT;
	}
	else
	status = CHAEGING_DISCHG;

out:

	return status;
}
EXPORT_SYMBOL_GPL(fg_tps8003x_charging_status);
#endif


//Chargin Continue time factor for ADC Based Battery
long adc_battery_charging_continue_time(void)
{
	struct tps80031_charger *charger = charger_data;
	if (!charger_data)
		return 0;
	else
		return charger->charge_continue_sec;
}
EXPORT_SYMBOL_GPL(adc_battery_charging_continue_time);


//ADD Charging type detect by Charger IC

enum charging_type tps8003x_charger_type(void)
{
	enum charging_type charger_type = NONE_CHARGER;
	if (!charger_data)
		return charger_type;

	if(charger_data->ac_online == 1)
		charger_type = AC_CHARGER;
	else if(charger_data->usb_online == 1)
		charger_type = USB_CHARGER;
	else
		charger_type = NONE_CHARGER;
	return charger_type;
}
EXPORT_SYMBOL_GPL(tps8003x_charger_type);


static irqreturn_t tps80031_sys_vlow(int irq, void *data)
{
	struct tps80031_charger *charger = data;
	dev_info(charger->dev, "%s  enter",__func__);
	wake_lock_timeout(&notifier_wake_lock, 3*HZ);//wait a bit for battery capacity status update
	tps80031_lowpower_msk_en(charger,false);//Only Need Call Once Every Wakeup
	return IRQ_HANDLED;
}

static int tps80031_set_lowpower(struct tps80031_charger *charger)
{
	int ret;
	ret = tps80031_write(charger->dev->parent, SLAVE_ID1, VSYSMIN_HI_THRESHOLD, VSYSMIN_HI_THRESHOLD_3300);
	if(ret < 0)
	goto out;

	ret = tps80031_write(charger->dev->parent, SLAVE_ID1, VSYSMIN_HI_CFG_STATE, VSYSMIN_HI_CFG_STATE_ON);
out:
	return ret;

}
//INT_MSK_LINE_A     --- A2 SYS_VLOW
static int tps80031_lowpower_msk_en(struct tps80031_charger *charger,bool enable)
{
	int ret;
	int i;
	uint8_t sts_a;
	if(enable) {
		ret = tps80031_update(charger->dev->parent, SLAVE_ID2, INT_MSK_LINE_A, INT_MSK_LINE_A_SYS_LOW_EN, INT_MSK_LINE_A_SYS_LOW_MSK);
		if(ret < 0)
		goto out;
		ret = tps80031_update(charger->dev->parent, SLAVE_ID2, INT_MSK_STS_A, INT_MSK_STS_A_SYS_LOW_EN, INT_MSK_STS_A_SYS_LOW_MSK);
	}
	else{
		ret = tps80031_update(charger->dev->parent, SLAVE_ID2, INT_MSK_LINE_A, INT_MSK_LINE_A_SYS_LOW_DIS, INT_MSK_LINE_A_SYS_LOW_MSK);
		if(ret < 0)
		goto out;
		ret = tps80031_update(charger->dev->parent, SLAVE_ID2, INT_MSK_STS_A, INT_MSK_STS_A_SYS_LOW_DIS, INT_MSK_STS_A_SYS_LOW_MSK);
		for(i = 0;i< 10;i++) {
			tps80031_read(charger->dev->parent, SLAVE_ID2,INT_STS_A, &sts_a);
			//dev_info(charger->dev, "%s %d INT_STS_A 0x%x\n",__func__,__LINE__,sts_a);
			if(sts_a == INT_STS_A_SYS_LOW)
			tps80031_write(charger->dev->parent, SLAVE_ID2,INT_STS_A, 0);
			else
			break;
		}
	}
out:
	return ret;
}


static int configure_charging_parameter(struct tps80031_charger *charger,bool init)
{
	int ret;
	int max_charge_current;
	int max_charge_volt;
	int term_current;
	bool vbus_status;
	uint8_t current_setting;
	uint8_t chg_sts;
	current_setting = 0x9;//500mA default value
	if(init){
		//Get VBUS  Status
		ret = tps80031_read(charger->dev->parent, SLAVE_ID2, CONTROLLER_STAT1, &chg_sts);
		if(ret < 0) {
			dev_err(charger->dev, "%s(): Failed in reading register 0x%02x\n",
				__func__, CONTROLLER_STAT1);
			return ret;
		}
		if((chg_sts & 0x04) != 0)
		vbus_status = true;
		else
		vbus_status = false;
		printk("*** %s %d CONTROLLER_STAT1 chg_sts  is 0x%x***\n",__func__,__LINE__,chg_sts);

		if(!vbus_status) {
			/* Disable watchdog timer */
			ret = tps80031_write(charger->dev->parent, SLAVE_ID2,
						CONTROLLER_WDG, 0x0);
			if (ret < 0) {
				dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
					__func__, CONTROLLER_WDG);
				return ret;
			}

			/* Disable the charging if any */
			ret = tps80031_write(charger->dev->parent, SLAVE_ID2,
					CONTROLLER_CTRL1, 0x0);
			if (ret < 0) {
				dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
						__func__, CONTROLLER_CTRL1);
				return ret;
			}
		}

		if (charger->board_init) {
			ret = charger->board_init(charger->board_data);
			if (ret < 0) {
				dev_err(charger->dev, "%s(): Failed in board init\n",
					__func__);
				return ret;
			}
		}

		if(vbus_status) {
			if(tegra_power_reason & State_Charger_PlugIn) {
				current_setting = 0x20;	 //1.8A
			}
		}

		//set current limited init to  500mA
		ret = tps80031_update(charger->dev->parent, SLAVE_ID2,
			CHARGERUSB_CINLIMIT, current_setting, 0x3F);
		if (ret < 0) {
			dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
					__func__, CHARGERUSB_CINLIMIT);
			return ret;
		}

		//Trickle charge VSYS set to be 4.2V
		ret = tps80031_write(charger->dev->parent, SLAVE_ID2,
					CONTROLLER_CTRL2, CONTROLLER_CTRL2_4200);
		if (ret < 0) {
			dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
						__func__, CONTROLLER_CTRL2);
			return ret;
		}

	}

	/* Unlock value */
	ret = tps80031_write(charger->dev->parent, SLAVE_ID2,
			CHARGERUSB_CTRLLIMIT2, 0);
	if (ret < 0) {
		dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
				__func__, CHARGERUSB_CTRLLIMIT2);
		return ret;
	}

	/* Set max current limit */
	max_charge_current = min(1500, charger->max_charge_current_mA);
	if (max_charge_current < 100)
		max_charge_current = 0;
	else
		max_charge_current = (max_charge_current - 100)/100;
	max_charge_current &= 0xF;
	ret = tps80031_write(charger->dev->parent, SLAVE_ID2,
		CHARGERUSB_CTRLLIMIT2, (uint8_t)max_charge_current);
	if (ret < 0) {
		dev_err(charger->dev, "%s(): Failed in writing register "
				"0x%02x\n", __func__, CHARGERUSB_CTRLLIMIT2);
		return ret;
	}

	/* Set max voltage limit */
	max_charge_volt = min(4760, charger->max_charge_volt_mV);
	max_charge_volt = max(3500, max_charge_volt);
	max_charge_volt -= 3500;
	max_charge_volt = max_charge_volt/20;
	ret = tps80031_write(charger->dev->parent, SLAVE_ID2,
		CHARGERUSB_CTRLLIMIT1, (uint8_t)max_charge_volt);//Default charging voltage level for the limit register

	if (ret < 0) {
		dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
				__func__, CHARGERUSB_CTRLLIMIT1);
		return ret;
	}

	ret = tps80031_write(charger->dev->parent, SLAVE_ID2,
		CHARGERUSB_VOREG, (uint8_t)max_charge_volt);

	if (ret < 0) {
		dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
				__func__, CHARGERUSB_VOREG);//Default battery charging voltage

		return ret;
	}


        //Set BUCK_VTH  to 4.36v  //To-Do Update in V2 board

	ret = tps80031_write(charger->dev->parent, SLAVE_ID2,
		ANTICOLLAPSE_CTRL1, 0x44);

	if (ret < 0) {
		dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
				__func__, ANTICOLLAPSE_CTRL1);

		return ret;
	}

	//Set DLIN to 100 mv Short:2.45 Full CHG:3.05
	ret = tps80031_write(charger->dev->parent, SLAVE_ID2,
		CONTROLLER_VSEL_COMP, 0x31);

	if (ret < 0) {
		dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
				__func__, CONTROLLER_VSEL_COMP);

		return ret;
	}

	/* Lock value *///these code doesn't work according to DM
        /*LOCK_LIMIT Bit register to lock the value on the register bits VICHRGL [3:0], VOREGL[5:0]:
	0: VOREGL and VICHRGL are readable and writable.
	1: VOREGL and VICHRGL are read only.
	LOCK_LIMIT bit is writable only once.
	To unlock the register bit, a NRESPWRON reset must be performed.
	*/
	ret = tps80031_set_bits(charger->dev->parent, SLAVE_ID2,
			CHARGERUSB_CTRLLIMIT2, (1 << 4));
	if (ret < 0) {
		dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
				__func__, CHARGERUSB_CTRLLIMIT2);
		return ret;
	}

	/* set Pre Charge current to 300mA */
        //Pre Charge Current <= 0.1C = 0.1x4000  Should not be larger than 400 mA,We using 300mA
	ret = tps80031_write(charger->dev->parent, SLAVE_ID2, CHARGERUSB_VICHRG_PC, 0x2);
	if (ret < 0) {
		dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
				__func__, 0xDD);
		return ret;
	}

	/* set charging termination current*/
	if (charger->charging_term_current_mA > 400)
		term_current =  7;
	else
		term_current = (charger->charging_term_current_mA - 50)/50;
	term_current = term_current << 5;
	ret = tps80031_write(charger->dev->parent, SLAVE_ID2,
			CHARGERUSB_CTRL2, (uint8_t)term_current);
	if (ret < 0) {
		dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
				__func__, CHARGERUSB_CTRL2);
		return ret;
	}

	return 0;
}

static bool tps80031_check_charging_completed(struct tps80031_charger *charger)
{
	int ret;
	uint8_t linch_status;
	uint8_t wtg_sts;
	uint8_t chg_sts;

	ret = tps80031_read(charger->dev->parent, SLAVE_ID2,
			LINEAR_CHRG_STS, &linch_status);
	if (ret < 0) {
		dev_err(charger->dev, "%s(): Failed in reading register 0x%02x\n",
				__func__, LINEAR_CHRG_STS);
		return false;
	}
	else {
		tps80031_read(charger->dev->parent, SLAVE_ID2, CONTROLLER_STAT1, &chg_sts);
		tps80031_read(charger->dev->parent, SLAVE_ID2, CONTROLLER_WDG, &wtg_sts);

		dev_info(charger->dev, "%s():The status of LINEAR_CHRG_STS is 0x%02x  CONTROLLER_STAT1 is 0x%x  CONTROLLER_WDG is 0x%x\n",
				 __func__, linch_status,chg_sts,wtg_sts);

	}

	if (linch_status & 0x20) {
		charger->state = charging_state_charging_completed;
		ret = true;
	} else {
		charger->state = charging_state_charging_in_progress;
		ret = false;
	}

	return ret;
}

static irqreturn_t linch_status_isr(int irq, void *dev_id)
{
	struct tps80031_charger *charger = dev_id;

	//dev_info(charger->dev, "%s() got called\n", __func__);

	if (tps80031_check_charging_completed(charger)) {
		charger->state = charging_state_charging_completed;
		if (charger->charger_cb)
			charger->charger_cb(charger->state,
					charger->charger_cb_data);
	}

	return IRQ_HANDLED;
}

static irqreturn_t watchdog_expire_isr(int irq, void *dev_id)
{
/*Refer DM OTP 4.9.6 Safety Timer and Charging Watchdog:During software-controlled(AUTO_CHAGER = 0) charging the safety timer
is replaced by charging watchdog (SW WDT), and If the safety timer or watchdog expires, the battery charging is gated and interrupt
is sent to host processor.So it's complicated to charging in suspend mode */
	struct tps80031_charger *charger = dev_id;
	int ret;

	if (charger->state != charging_state_charging_in_progress)
		return IRQ_HANDLED;//Return if unplug

	if(charger->safety_timer_en) {
		//safety timer check
		safety_timer_check(charger,false,false);
		//Return if safety timer expired
		if(charger->charge_continue_sec  > charger->safety_timer_sec )
			dev_info(charger->dev, "%s():safety timer expired:%ld\n",__func__,charger->charge_continue_sec);
		else {

			/* Enable watchdog timer again*/
			ret = tps80031_write(charger->dev->parent, SLAVE_ID2, CONTROLLER_WDG,
					charger->watch_time_sec);
			if (ret < 0)
				dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
					__func__, CONTROLLER_WDG);
			else
				dev_info(charger->dev, "***unexpected  %s()   happen ret %d\n",
					__func__,ret);

			/* Rewrite to enable the charging */
			if (!ret) {
				ret = configure_charging_parameter(charger,false);
				if (ret < 0) {
					dev_err(charger->dev, "%s(): configure_charging_parameter failed\n",
						__func__);
				}

				ret = tps80031_write(charger->dev->parent, SLAVE_ID2,
				CONTROLLER_CTRL1, 0x30);
				if (ret < 0)
				dev_err(charger->dev, "%s(): Failed in writing "
					"register 0x%02x\n",
					__func__, CONTROLLER_CTRL1);
			}
		}
	}
	else {
		/* Enable watchdog timer again*/
		ret = tps80031_write(charger->dev->parent, SLAVE_ID2, CONTROLLER_WDG,
				charger->watch_time_sec);
		if (ret < 0)
			dev_err(charger->dev, "%s(): Failed in writing register 0x%02x\n",
				__func__, CONTROLLER_WDG);
		else
			dev_info(charger->dev, "***unexpected  %s()   happen ret %d\n",
				 __func__,ret);

		/* Rewrite to enable the charging */
		if (!ret) {
			ret = configure_charging_parameter(charger,false);
			if (ret < 0) {
				dev_err(charger->dev, "%s(): configure_charging_parameter failed\n",
					__func__);
			}

			ret = tps80031_write(charger->dev->parent, SLAVE_ID2,
				CONTROLLER_CTRL1, 0x30);
			if (ret < 0)
				dev_err(charger->dev, "%s(): Failed in writing "
					"register 0x%02x\n",
					__func__, CONTROLLER_CTRL1);
		}
	}
	return IRQ_HANDLED;
}


static void tps8003x_watchdog_work(struct work_struct *work)
{
	struct tps80031_charger *charger =
		container_of(work, struct tps80031_charger, wdg_work.work);
	int ret;

	if (!tps8003x_vbus_status())
	{
		dev_info(charger->dev, "%s() vbus isn't on line,don't reset charger watchdog\n", __func__);
		return;
	}

	if(charger->jeita_control_en)
		jeita_limited_detect(charger);

	if(charger->safety_timer_en) {
		//safety timer check
		safety_timer_check(charger,false,false);
		if(charger->charge_continue_sec  > charger->safety_timer_sec )
		    safety_timer_expired(charger);
		else {
			//Reset WDG Timer(Writing into one of the following registers: CONTROLLER_CTRL1 , CONTROLLER_WDG , VOREG , VICHRG)
			ret = tps80031_write(charger->dev->parent, SLAVE_ID2, CONTROLLER_WDG,
				charger->watch_time_sec);
			if(ret < 0)
			dev_err(charger->dev, "%s() Reset Charger Watchdog Failed\n", __func__);


			if (charger->watch_reset_time_sec > 0)
				schedule_delayed_work(&charger->wdg_work, charger->watch_reset_time_sec * HZ);
		}
	}
	else {
		//Reset WDG Timer(Writing into one of the following registers: CONTROLLER_CTRL1 , CONTROLLER_WDG , VOREG , VICHRG)
		ret = tps80031_write(charger->dev->parent, SLAVE_ID2, CONTROLLER_WDG,
			charger->watch_time_sec);
		if(ret < 0)
		dev_err(charger->dev, "%s() Reset Charger Watchdog Failed\n", __func__);

		if (charger->watch_reset_time_sec > 0)
			schedule_delayed_work(&charger->wdg_work, charger->watch_reset_time_sec * HZ);
	}
}


static void tps8003x_program_alarm(struct tps80031_charger *charger)
{
	//We don't  have to be so accurate in order to reduce conflict.(The expired time is 120s, the alarm will be about 85-95s )
	int ret;
	ktime_t low_interval = ktime_set(charger->watch_time_sec - 35, 0);
	ktime_t slack = ktime_set(10, 0);
	ktime_t next  = ktime_add(alarm_get_elapsed_realtime(), low_interval);
	ret = alarm_try_to_cancel(&charger->alarm);
	//dev_info(charger->dev, "%s() Start  alarm in  90s cancel ret %d\n", __func__,ret);
	alarm_start_range(&charger->alarm, next, ktime_add(next, slack)); //Make sure Alarm will trigger
}

static int tps8003x_pm_notifier(struct notifier_block *notifier,
		unsigned long pm_event,
		void *unused)
{
	struct tps80031_charger *charger =
		container_of(notifier, struct tps80031_charger, pm_notifier);
	int ret;

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		if (tps8003x_vbus_status()) {
			//Let's Cancel any work here
			cancel_delayed_work_sync(&charger->wdg_work);
			if(charger->jeita_control_en)
				jeita_limited_detect(charger);
			if(charger->safety_timer_en) {
				//safety timer check
				safety_timer_check(charger,false,false);
				if(charger->charge_continue_sec  > charger->safety_timer_sec )
					safety_timer_expired(charger);
				else {
				//Reset Watchdog before Suspend
					ret = tps80031_write(charger->dev->parent, SLAVE_ID2, CONTROLLER_WDG,
								charger->watch_time_sec);
					if(ret < 0)
					dev_err(charger->dev, "%s() Reset Watchdog Failed\n", __func__);
					tps8003x_program_alarm(charger);
				}

			}
			else {
				//Reset Watchdog before Suspend
				ret = tps80031_write(charger->dev->parent, SLAVE_ID2, CONTROLLER_WDG,
							charger->watch_time_sec);
				if(ret < 0)
				dev_err(charger->dev, "%s() Reset Watchdog Failed\n", __func__);
				tps8003x_program_alarm(charger);
			}
		}
		else
		cancel_delayed_work_sync(&charger->wdg_work);//If no charging,also need cancel this work
		break;

	case PM_POST_SUSPEND:
		if (tps8003x_vbus_status()) {
			if(charger->jeita_control_en)
				jeita_limited_detect(charger);

			if(charger->safety_timer_en) {
				//safety timer check
				safety_timer_check(charger,false,false);
				if(charger->charge_continue_sec  > charger->safety_timer_sec )
					safety_timer_expired(charger);
				else {
					//Reset Watchdog after Suspend
					ret = tps80031_write(charger->dev->parent, SLAVE_ID2, CONTROLLER_WDG,
						charger->watch_time_sec);
					if(ret < 0)
					dev_err(charger->dev, "%s() Reset Watchdog Failed\n", __func__);
					//else
					//dev_info(charger->dev, "%s() Reset Watchdog Done After S3\n", __func__);
					//if alarm pending,cancel it
					alarm_try_to_cancel(&charger->alarm);
					//Let's Start the watchdog writing work
					schedule_delayed_work(&charger->wdg_work, charger->watch_reset_time_sec * HZ);
				}
			}
			else {
				//Reset Watchdog after Suspend
				ret = tps80031_write(charger->dev->parent, SLAVE_ID2, CONTROLLER_WDG,
						charger->watch_time_sec);
				if(ret < 0)
					dev_err(charger->dev, "%s() Reset Watchdog Failed\n", __func__);
				//else
				//dev_info(charger->dev, "%s() Reset Watchdog Done After S3\n", __func__);
				//if alarm pending,cancel it
				alarm_try_to_cancel(&charger->alarm);
				//Let's Start the watchdog writing work
				schedule_delayed_work(&charger->wdg_work, charger->watch_reset_time_sec * HZ);
			}
		}
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block tps8003x_pm_notifier_block = {
	.notifier_call = tps8003x_pm_notifier,
};


static void tps8003x_charger_alarm(struct alarm *alarm)
{
	struct tps80031_charger *charger =
		container_of(alarm, struct tps80031_charger, alarm);

	dev_info(charger->dev, "***%s()  expired\n", __func__);

}


static void tps8003x_first_detect_work(struct work_struct *work)
{
	struct tps80031_charger *charger =
		container_of(work, struct tps80031_charger, first_detect_work.work);
	int ret;

	if(charger->ac_online == 0)
	{
		printk("****TPS8003X:%s USB Confirm\n",__func__);
		ret = tps80031_update(charger->dev->parent, SLAVE_ID2,
			CHARGERUSB_CINLIMIT, 0x9, 0x3F);
		if(ret < 0)
		dev_err(charger->dev,"TPS8003X:%s Set USB Charging Current Failed\n",__func__);

	}
}


static int tps80031_charger_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct tps80031_charger *charger;
	struct tps80031_platform_data *tps80031_pdata;
	struct tps80031_charger_platform_data *pdata;

	dev_info(dev, "%s()\n", __func__);

	tps80031_pdata = dev_get_platdata(pdev->dev.parent);
	if (!tps80031_pdata) {
		dev_err(&pdev->dev, "no tps80031 platform_data specified\n");
		return -EINVAL;
	}

	pdata = tps80031_pdata->battery_charger_pdata;
	if (!pdata) {
		dev_err(dev, "%s() No platform data, exiting..\n", __func__);
		return -ENODEV;
	}

	if (!pdata->num_consumer_supplies) {
		dev_err(dev, "%s() No consumer supply list, exiting..\n",
				__func__);
		return -ENODEV;
	}

	charger = kzalloc(sizeof(*charger), GFP_KERNEL);
	if (!charger) {
		dev_err(dev, "failed to allocate memory status\n");
		return -ENOMEM;
	}

	charger->dev =  &pdev->dev;

	charger->max_charge_current_mA = (pdata->max_charge_current_mA) ?
					pdata->max_charge_current_mA : 1000;
	charger->max_charge_volt_mV = (pdata->max_charge_volt_mV) ?
					pdata->max_charge_volt_mV : 4200;
	charger->irq_base = pdata->irq_base;
	charger->watch_time_sec = min(pdata->watch_time_sec, 127);
	if (!charger->watch_time_sec)
		charger->watch_time_sec = 127;

	charger->watch_reset_time_sec = charger->watch_time_sec / 2 ;//Temp using 120/2 = 60s
//ADD safety timer function by SW
	charger->safety_timer_en = pdata->safety_timer_en;
	if(charger->safety_timer_en){
	   charger->safety_timer_sec = (pdata->safety_timer_sec) ?
					pdata->safety_timer_sec : 28800;
	   dev_info(&pdev->dev,"TPS80032-Charger:SW safety timer %ld sec enable!!\n",charger->safety_timer_sec);
	}

	charger->charge_start_sec =  0;
	charger->charge_continue_sec = 0;

//jeita control by SW
	charger->jeita_control_en = (pdata->jeita_current_en) ? pdata->jeita_current_en : 0;
	charger->jeita_limited_temp_low = (pdata->jeita_limited_temp_low) ? pdata->jeita_limited_temp_low : 0;
	charger->jeita_limited_temp_high = (pdata->jeita_limited_temp_high) ? pdata->jeita_limited_temp_high : 500;
	charger->jeita_limited_temp_cap = (pdata->jeita_limited_temp_cap) ? pdata->jeita_limited_temp_cap : 50;
	charger->jeita_limited_current = (pdata->jeita_limited_current) ? pdata->jeita_limited_current : 100;
	charger->jeita_limited_voltage = (pdata->jeita_limited_voltage) ? pdata->jeita_limited_voltage : 4100;
	charger->in_jeita_curr_control = 0;
	charger->in_jeita_vol_control = 0;

	if(charger->jeita_control_en){
		dev_info(&pdev->dev,"JEITA ENABLE  low temp: %d  high temp %d temp cap: %d curr: %d vol:%d\n",
		charger->jeita_limited_temp_low,charger->jeita_limited_temp_high,charger->jeita_limited_temp_cap,
		charger->jeita_limited_current,charger->jeita_limited_voltage);
	}

	charger->charging_term_current_mA =
			max(50, pdata->charging_term_current_mA);
	if (charger->charging_term_current_mA < 50)
		charger->charging_term_current_mA = 50;

	charger->reg_desc.name = "vbus_charger";
	charger->reg_desc.id = pdata->regulator_id;
	charger->reg_desc.ops = &tegra_regulator_ops;
	charger->reg_desc.type = REGULATOR_CURRENT;
	charger->reg_desc.owner = THIS_MODULE;

	charger->reg_init_data.supply_regulator = NULL;
	charger->reg_init_data.num_consumer_supplies =
					pdata->num_consumer_supplies;
	charger->reg_init_data.consumer_supplies = pdata->consumer_supplies;
	charger->reg_init_data.regulator_init = NULL;
	charger->reg_init_data.driver_data = charger;
	charger->reg_init_data.constraints.name = "vbus_charger";
	charger->reg_init_data.constraints.min_uA = 0;
	charger->reg_init_data.constraints.max_uA =
					pdata->max_charge_current_mA * 1000;
	charger->reg_init_data.constraints.valid_modes_mask =
					REGULATOR_MODE_NORMAL |
					REGULATOR_MODE_STANDBY;
	charger->reg_init_data.constraints.valid_ops_mask =
					REGULATOR_CHANGE_MODE |
					REGULATOR_CHANGE_STATUS |
					REGULATOR_CHANGE_CURRENT;

	charger->board_init = pdata->board_init;
	charger->board_data = pdata->board_data;
	charger->state = charging_state_idle;

	charger->rdev = regulator_register(&charger->reg_desc, &pdev->dev,
					&charger->reg_init_data, charger);
	if (IS_ERR(charger->rdev)) {
		dev_err(&pdev->dev, "failed to register %s\n",
						charger->reg_desc.name);
		ret = PTR_ERR(charger->rdev);
		goto regulator_fail;
	}

	ret = request_threaded_irq(charger->irq_base + TPS80031_INT_LINCH_GATED,
			NULL, linch_status_isr,	0, "tps80031-linch", charger);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register irq %d; error %d\n",
			charger->irq_base + TPS80031_INT_LINCH_GATED, ret);
		goto irq_linch_fail;
	}

	ret = request_threaded_irq(charger->irq_base + TPS80031_INT_FAULT_WDG,
			NULL, watchdog_expire_isr, 0, "tps80031-wdg", charger);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register irq %d; error %d\n",
			charger->irq_base + TPS80031_INT_FAULT_WDG, ret);
		goto irq_wdg_fail;
	}

	ret = configure_charging_parameter(charger,true);
	if (ret)
		goto config_fail;


	//Give System a bit time to get plug/unplug event
	wake_lock_init(&notifier_wake_lock, WAKE_LOCK_SUSPEND,"notifier_lock");

	//ADD  Low Power Wakeup by PMU
	ret = tps80031_set_lowpower(charger);
	if (ret) {
		dev_err(&pdev->dev, "tps80031_set_lowpower failed\n");
		goto config_fail;
	}

	ret = request_threaded_irq(charger->irq_base + TPS80031_INT_SYS_VLOW,
				NULL, tps80031_sys_vlow,IRQF_ONESHOT, "tps80031-low-power", charger);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register irq %d;error %d\n",
			charger->irq_base + TPS80031_INT_SYS_VLOW,ret);
		goto config_fail;
	}

	//Only enable during LP0  This Code doesn't work,irq_sync func will open it during other module init.
	//tps80031_lowpower_msk_en(charger,false);

#ifdef CONFIG_CHARGER_TPS8003X_DEBUG
//ADD Debug
	tps80031_charger_dbg_kobj = kobject_create_and_add("charger", NULL);
	if (tps80031_charger_dbg_kobj == NULL)
		dev_err(&pdev->dev,"TPS80032-Charger:kobj creat Failed!!\n");
	else
	{
		ret = sysfs_create_group(tps80031_charger_dbg_kobj, &tps80031_charger_attr_group);
		if(ret)
			dev_err(&pdev->dev,"TPS80032-Charger:sys group creat Failed!\n");
	}
#endif

	//RTC can't be operated in suspend/resume so move to PREPARE/POST SUSPEND
	charger->pm_notifier = tps8003x_pm_notifier_block;
	ret = register_pm_notifier(&charger->pm_notifier);
	if (ret) {
		dev_err(&pdev->dev, "failed: register pm notifier\n");
		goto config_fail;
	}

	//Init Watchdog work//deferrable sometimes will cause this work totally be removed
	INIT_DELAYED_WORK(&charger->wdg_work, tps8003x_watchdog_work);

	INIT_DELAYED_WORK(&charger->first_detect_work, tps8003x_first_detect_work);
	//ADD ALARM for AC Sleep
	alarm_init(&charger->alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,tps8003x_charger_alarm);


	dev_set_drvdata(&pdev->dev, charger);
	charger_data = charger;
	return ret;

config_fail:
	free_irq(charger->irq_base + TPS80031_INT_FAULT_WDG, charger);
irq_wdg_fail:
	free_irq(charger->irq_base + TPS80031_INT_LINCH_GATED, charger);
irq_linch_fail:
	regulator_unregister(charger->rdev);
regulator_fail:
	kfree(charger);
	return ret;
}

static int tps80031_charger_remove(struct platform_device *pdev)
{
	struct tps80031_charger *charger = dev_get_drvdata(&pdev->dev);
	wake_lock_destroy(&notifier_wake_lock);
	alarm_cancel(&charger->alarm);
	cancel_delayed_work_sync(&charger->wdg_work);
	regulator_unregister(charger->rdev);
	kfree(charger);
	return 0;
}

#ifdef CONFIG_PM

static int tps80031_charger_suspend(struct platform_device *pdev,
				  pm_message_t state)
{
	struct tps80031_charger *charger = dev_get_drvdata(&pdev->dev);
	//Open Low Power Wake up
	tps80031_lowpower_msk_en(charger,true);

	return 0;
}

static int tps80031_charger_resume(struct platform_device *pdev)
{
	struct tps80031_charger *charger = dev_get_drvdata(&pdev->dev);
	//Close Low Power Wake up
	tps80031_lowpower_msk_en(charger,false);

	return 0;
}

#else

#define tps80031_charger_suspend NULL
#define tps80031_charger_resume NULL

#endif


static struct platform_driver tps80031_charger_driver = {
	.driver	= {
		.name	= "tps80031-charger",
		.owner	= THIS_MODULE,
	},
	.probe	= tps80031_charger_probe,
	.remove = tps80031_charger_remove,
	.suspend  = tps80031_charger_suspend,
	.resume	  = tps80031_charger_resume,
};

static int __init tps80031_charger_init(void)
{
	return platform_driver_register(&tps80031_charger_driver);
}

static void __exit tps80031_charger_exit(void)
{
	platform_driver_unregister(&tps80031_charger_driver);
}

subsys_initcall(tps80031_charger_init);
module_exit(tps80031_charger_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("tps80031 battery charger driver");
