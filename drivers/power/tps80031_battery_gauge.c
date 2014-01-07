/*
 * drivers/power/tps80031_battery_gauge.c
 *
 * Gas Gauge driver for TI's tps80031
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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

#include <linux/mfd/core.h>
#include <linux/mfd/tps80031.h>
#include <linux/tps80031-charger.h>
#include <linux/workqueue.h>
#include <linux/power/ti-fg.h>
#include <linux/delay.h>
#include "fg/fg.h"
#include <linux/wakelock.h>

#define CHARGERUSB_CINLIMIT	0xee
#define CONTROLLER_CTRL1	0xe1
#define CONTROLLER_CTRL1_EN_LINCH	BIT(5)
#define CONTROLLER_STAT1	0xe3
#define LINEAR_CHARGE_STS	0xde
#define STS_HW_CONDITIONS	0x21
#define TOGGLE1			0x90
#define TOGGLE1_FGDITHS		BIT(7)
#define TOGGLE1_FGS		BIT(5)
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
#define FG_REG_00		0xc0
#define FG_REG_00_CC_AUTOCLEAR	BIT(2)
#define FG_REG_00_CC_CAL_EN	BIT(1)
#define FG_REG_00_CC_PAUSE	BIT(0)
#define FG_REG_01		0xc1	/* CONV_NR (unsigned) 0 - 7 */
#define FG_REG_02		0xc2	/* CONV_NR (unsigned) 8 - 15 */
#define FG_REG_03		0xc3	/* CONV_NR (unsigned) 16 - 23 */
#define FG_REG_04		0xc4	/* ACCM	(signed) 0 - 7 */
#define FG_REG_05		0xc5	/* ACCM	(signed) 8 - 15 */
#define FG_REG_06		0xc6	/* ACCM	(signed) 16 - 23 */
#define FG_REG_07		0xc7	/* ACCM	(signed) 24 - 31 */
#define FG_REG_08		0xc8	/* OFFSET (signed) 0 - 7 */
#define FG_REG_09		0xc9	/* OFFSET (signed) 8 - 9 */
#define FG_REG_10		0xca	/* LAST_READ (signed) 0 - 7 */
#define FG_REG_11		0xcb	/* LAST_READ (signed) 8 - 13 */

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

#define DRIVER_VERSION		"1.3.0" //Porting OMAP4-TWL6030 Fuel Gauge 1.0 Kernel Driver to T3
/*To-Do:lc@0315,2013
1.HIGH/MID Load optimize
2.Bootupvoltage Handler(OCV voltage tuning,node->BS)
3.EDV SOC JUMP Fix(Implement low voltage ops instead of current EDV)
3.Enhance Battery Service(soc adjust during bootup(init)/store last soc during powersupply changed(data partion/fcc/cc/recovery lose)
4.PowerClass SYS changed to be connected with User Space
*/
extern unsigned int cap_of_battery;//Battery voltage from bootloader
extern unsigned int tegra_power_reason;
#define State_Charger_PlugIn	0x00000800//AC usb plugin
#define State_Normal_USB_PlugIn	0x40000000//normal usb

#ifdef CONFIG_BATTERY_BQ27x00
extern bool battery_standalone_fg(void);
#endif



static int gauge_battery_temp = 200;
static const unsigned int fuelgauge_rate[4] = {1, 4, 16, 64};
extern enum charging_type tps8003x_charger_type(void);

static int tps80031_temp_table[] = {   //using M370's data
	/* adc code for temperature in degree C */
	783, 781, 778, 776, 774, 771, 769, 766, 763, 760, /* -20,-11 */
	757, 754, 751, 748, 744, 741, 737, 733, 730, 726, /* -10, -1 */
	721, 717, 713, 709, 704, 699, 695, 690, 685, 680, /* 00 - 09 */
	674, 669, 664, 658, 653, 647, 641, 635, 629, 623, /* 10 - 19 */
	617, 611, 605, 598, 592, 585, 579, 572, 565, 559, /* 20 - 29 */
	552, 545, 538, 531, 524, 517, 510, 503, 496, 489, /* 30 - 39 */
	482, 475, 468, 461, 454, 448, 441, 434, 427, 420, /* 40 - 49 */
	413, 407, 400, 393, 387, 380, 374, 367, 361, 354, /* 50 - 59 */
	348, 342, 336, 330, 324, 318, 312, 307, 301, 295, /* 60 - 69 */
};

struct tps80031_battery_cache {
	int usb_online;
	int ac_online;
};


struct tps80031_device_info {
	struct device		*dev;
	struct i2c_client	*client;
	struct power_supply	bat;
	struct power_supply	ac;
#ifdef	CONFIG_TPS80032_USB_CHARGER
	struct power_supply	usb;
#endif
	int usb_online;
	int ac_online;
	struct 	wake_lock ac_chrg_lock; //SW FG Must ADD this lock or SOC can't be calculated well during charging
	struct 	tps80031_battery_cache cache;
	/* Battery Power Supply */
	int battery_voltage_uV;
	int battery_current_uA;
	int battery_current_avg_uA;
	int battery_charge_status;
	int battery_capacity;
	int battery_boot_capacity_mAh;
	int battery_capacity_max;
	int battery_prev_capacity;
	int battery_capacity_debounce;
	int battery_health;
	int battery_online;
	int battery_timer_n2;
	int battery_timer_n1;
	s32 battery_charge_n1;
	s32 battery_charge_n2;
	int battery_current_avg_interval;
	struct delayed_work battery_current_avg_work;
	int battery_termperature_tenthC;
	/* Fuelgauge */
	int fuelgauge_mode;
	int cc_offset;
	int current_max_scale;
	int accumulated_charge;
	struct cell_state cell;
};

static enum power_supply_property tps80031_bat_props[] = {
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_PRESENT,
#ifdef CONFIG_BATTERY_FUEL_GAUGE_DETECT
	POWER_SUPPLY_PROP_FUEL_GAUGE,
#endif
};
#ifdef	CONFIG_TPS80032_USB_CHARGER
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
	int voltage;

	voltage = tps80031_gpadc_conversion(BATTERY_VOLTAGE);
	if (voltage < 0)
		return voltage;
	voltage = ((voltage * 5000) / 4096);//DIV4/0xFFF/mV

	di->battery_voltage_uV = voltage * 1000;

	return di->battery_voltage_uV;
}



static int tps80031_battery_temp(struct tps80031_device_info *di,
					union power_supply_propval *val)
{
	int adc_code, temp;

	adc_code = tps80031_gpadc_conversion(BATTERY_TEMPERATURE);
	if (adc_code < 0)
		return adc_code;

	adc_code = (adc_code * 1250) /4096;//mv

	for (temp = 0; temp < ARRAY_SIZE(tps80031_temp_table); temp++) {
		if (adc_code >= tps80031_temp_table[temp])
			break;
	}
	/* first 2 values are for negative temperature */
	val->intval = (temp - 20) * 10; /* in tenths of degree Celsius */

	di->battery_termperature_tenthC =  val->intval;

	gauge_battery_temp = val->intval;

	return  val->intval;
}

static void tps80032_battery_current_now(struct tps80031_device_info *di)
{
	int ret = 0;
	uint8_t reg = 0;
	s16 temp = 0;
	int current_now = 0;

	/* pause FG updates to get consistant data */
	ret = tps80031_update(di->dev->parent, SLAVE_ID2,FG_REG_00, FG_REG_00_CC_PAUSE, FG_REG_00_CC_PAUSE);
	if (ret < 0) {
		dev_err(di->dev, "Error pause FG updates FG_REG_00\n");
		return;
	}

	ret = tps80031_reg_read(di, SLAVE_ID2, FG_REG_10, &reg);
	if (ret < 0) {
		dev_err(di->dev, "failed to read FG_REG_10\n");
		return;
	}

	temp = reg;

	ret = tps80031_reg_read(di, SLAVE_ID2, FG_REG_11, &reg);
	if (ret < 0) {
		dev_err(di->dev, "failed to read FG_REG_11\n");
		return;
	}

	temp |= reg << 8;

	/* resume FG updates */
	ret = tps80031_update(di->dev->parent, SLAVE_ID2,FG_REG_00, 0, FG_REG_00_CC_PAUSE);
	if (ret < 0) {
		dev_err(di->dev, "Error resume FG updates FG_REG_00\n");
		return;
	}

	/* sign extend the result */
	temp = ((s16)(temp << 2) >> 2);
	current_now = temp - di->cc_offset;

	/* current drawn per sec */
	current_now = current_now * fuelgauge_rate[di->fuelgauge_mode];
	/* current in mAmperes */
	current_now = (current_now * di->current_max_scale) >> 13;
	/* current in uAmperes */
	current_now = current_now * 1000;
	di->battery_current_uA = current_now;

	return;
}


static int tps80031_battery_present(struct tps80031_device_info *di,
			union power_supply_propval *val)
{
	//To-Do Detect BATREMOVAL pin status by AP(GPADC_IN0 Doesn't work according to current design)
	return 1;
}


//Get Battery Temperature for Charger
int battery_temp_by_gauge(void)
{
	return gauge_battery_temp;
}
EXPORT_SYMBOL_GPL(battery_temp_by_gauge);


#define to_tps80031_device_info_bat(x) container_of((x), \
				struct tps80031_device_info, bat);

static int tps80031_bat_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct tps80031_device_info *di = to_tps80031_device_info_bat(psy);

	switch (psp) {

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = di->battery_health;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval =  di->cell.soc;//Palmas Beta Driver isn't implement yet!!!
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval =  tps80031_battery_voltage(di);
		break;

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = di->battery_charge_status;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = tps80031_battery_temp(di, val);
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		tps80032_battery_current_now(di);
		val->intval = di->battery_current_uA;
		break;

	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = di->battery_current_avg_uA;
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = di->cell.cycle_count;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = di->cell.fcc * 1000;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = 4000 * 1000;  //Fix Value,sync with pdate cell_cfg
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

static int tps80031_bat_set_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       const union power_supply_propval *val)
{
	struct tps80031_device_info *di = to_tps80031_device_info_bat(psy);

	switch (psp)
	{
		case POWER_SUPPLY_PROP_CHARGE_FULL:
			if (val->intval > 500) {
				di->cell.fcc = val->intval;
				di->cell.nac = (di->cell.soc * di->cell.fcc) / 100;
			} else {
				pr_err("FCC is too low, igrnoring it\n");
			}
			break;

		case POWER_SUPPLY_PROP_CYCLE_COUNT:
			di->cell.cycle_count = val->intval;
			break;
		default:
			return -EPERM;
	}

	return 0;
}

#ifdef	CONFIG_TPS80032_USB_CHARGER
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


static int tps80031_battery_charging_status(struct tps80031_device_info *di)
{
	int status = POWER_SUPPLY_STATUS_UNKNOWN;
	uint8_t chg_status;
	uint8_t chg_usb_status;
	uint8_t con_status;
	int ret;
#ifdef	CONFIG_TPS80032_USB_CHARGER
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
	return status;
}

static int tps80031_battery_health(struct tps80031_device_info *di)
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

	if(con_status & TPS80031_BAT_TEMP_OV)
		status = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if((chg_status & TPS80031_VBATOV)||(chg_usb_status & TPS80031_BAT_OVP))
		status = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else
		status = POWER_SUPPLY_HEALTH_GOOD;

out:
	return status;
}


void tps80031_battery_status(enum charging_states status, void *data)
{
	struct tps80031_device_info *di = data;
	enum charging_type charger_type;
	charger_type = tps8003x_charger_type();
	if (charger_type == AC_CHARGER) {
		wake_lock(&di->ac_chrg_lock);
		di->ac_online  = 1;
		di->usb_online = 0;
	}
	else if(charger_type == USB_CHARGER) {
		wake_unlock(&di->ac_chrg_lock);//s. p.
		di->ac_online  = 0;
		di->usb_online = 1;
	}
	else {
		wake_unlock(&di->ac_chrg_lock);
		di->ac_online  = 0;
		di->usb_online = 0;
	}
	//update battery charging status here
	di->battery_charge_status = tps80031_battery_charging_status(di);

	//report to user space
	if(di->cache.ac_online != di->ac_online)
		power_supply_changed(&di->ac);
	else if(di->cache.usb_online != di->usb_online) {
#ifdef	CONFIG_TPS80032_USB_CHARGER
		power_supply_changed(&di->usb);
#endif
	}
	else //shouldn't come here
		power_supply_changed(&di->bat);
	di->cache.ac_online  = di->ac_online ;
	di->cache.usb_online = di->usb_online;
}


static void tps80031_battery_status_init(struct tps80031_device_info *di)
{
	enum charging_type charger_type;
	charger_type = tps8003x_charger_type();
	if (charger_type == AC_CHARGER) {
		wake_lock(&di->ac_chrg_lock);
		di->ac_online  = 1;
		di->usb_online = 0;
	}
	else if(charger_type == USB_CHARGER) {
		wake_unlock(&di->ac_chrg_lock);//s. p.
		di->ac_online  = 0;
		di->usb_online = 1;
	}
	else {
		wake_unlock(&di->ac_chrg_lock);
		di->ac_online  = 0;
		di->usb_online = 0;
	}
	//update battery charging status here
	di->battery_charge_status = tps80031_battery_charging_status(di);

	//report to user space
	if(di->cache.ac_online != di->ac_online)
		power_supply_changed(&di->ac);
	else if(di->cache.usb_online != di->usb_online) {
#ifdef	CONFIG_TPS80032_USB_CHARGER
		power_supply_changed(&di->usb);
#endif
	}
	else //shouldn't come here
		power_supply_changed(&di->bat);
	di->cache.ac_online  = di->ac_online ;
	di->cache.usb_online = di->usb_online;
}


static void tps80032_gasgauge_calibrate(struct tps80031_device_info *di)
{
	int ret;
	unsigned int reg = 0;
	/*
	 * Enable the AUTOCLEAR so that any FG is in known state, and
	 * enabled the FG
	 */
	reg = FG_REG_00_CC_AUTOCLEAR;

	ret = tps80031_update(di->dev->parent, SLAVE_ID2,FG_REG_00, reg, reg);
	if (ret < 0)
		dev_err(di->dev, "Error AUTOCLEAR FG FG_REG_00\n");
	di->accumulated_charge = 0;
	//Enables calibration
	reg = FG_REG_00_CC_CAL_EN;

	ret = tps80031_update(di->dev->parent, SLAVE_ID2,FG_REG_00, reg, reg);
	if (ret < 0)
		dev_err(di->dev, "Error Enables calibration FG_REG_00\n");
}

static void tps80032_calculate_capacity(struct tps80031_device_info *di)
{
	int accumulated_charge;

	/*
	 * 3000 mA maps to a count of 4096 per sample
	 * We have 4 samples per second
	 * Charge added in one second == (acc_value * 3000 / (4 * 4096))
	 * mAh added == (Charge added in one second / 3600)
	 * mAh added == acc_value * (3000/3600) / (4 * 4096)
	 * mAh added == acc_value * (5/6) / (2^14)
	 * Using 5/6 instead of 3000 to avoid overflow
	 * FIXME: revisit this code for overflows
	 * FIXME: Take care of different value of samples/sec
	 */

	accumulated_charge = (((di->battery_charge_n1 -
		(di->cc_offset * di->battery_timer_n1)) * 5) / 6) >> 14;

	accumulated_charge = accumulated_charge * 10 /
			(int)di->cell.config->r_sense;

	fg_process(&di->cell, accumulated_charge - di->accumulated_charge,
			di->battery_voltage_uV / 1000,
			(int16_t)(di->battery_current_avg_uA / 1000),
			di->battery_termperature_tenthC);

	di->accumulated_charge = accumulated_charge;

	/* Gas gauge requested CC autocalibration */
	if (di->cell.calibrate) {
		di->cell.calibrate = false;
		tps80032_gasgauge_calibrate(di);
		di->battery_timer_n1 = 0;
		di->battery_charge_n1 = 0;
	}

	/* Battery state changes needs to be sent to the OS */
	if (di->cell.updated) {
		di->cell.updated = 0;
		power_supply_changed(&di->bat);
	}

	dev_info(di->dev, "di->battery_charge_n1 %d di->cell.calibrate %d di->cell.updated %d\n",
			di->battery_charge_n1,di->cell.calibrate,di->cell.updated);

	return;
}


static void tps80032_battery_current_avg(struct work_struct *work)
{
	struct tps80031_device_info *di = container_of(work,
		struct tps80031_device_info,
		battery_current_avg_work.work);

	s32 samples = 0;
	s16 cc_offset = 0;
	unsigned int reg = 0;
	int current_avg_uA = 0, ret;
	u8 temp[4];

	di->battery_charge_n2 = di->battery_charge_n1;
	di->battery_timer_n2 = di->battery_timer_n1;

	/* pause FG updates to get consistant data */
	reg = FG_REG_00_CC_PAUSE;

	ret = tps80031_update(di->dev->parent, SLAVE_ID2,FG_REG_00, reg, reg);
	if (ret < 0) {
		dev_err(di->dev, "Error pause FG updates FG_REG_00\n");
		return;
	}

	/* FG_REG_01, 02, 03 is 24 bit unsigned sample counter value */
	ret = tps80031_reads(di->dev->parent,SLAVE_ID2,FG_REG_01,3,temp);
	if (ret < 0) {
		dev_err(di->dev, "Error reading FG_REG_01-03\n");
		return;
	}

	temp[3] = 0;

	di->battery_timer_n1 = le32_to_cpup((u32 *)temp);

	/*
	 * FG_REG_04, 5, 6, 7 is 32 bit signed accumulator value
	 * accumulates instantaneous current value
	 */
	ret = tps80031_reads(di->dev->parent,SLAVE_ID2,FG_REG_04,4,temp);
	if (ret < 0) {
		dev_err(di->dev, "Error reading FG_REG_04-07\n");
		return;
	}

	di->battery_charge_n1 = le32_to_cpup((u32 *)temp);

	/* FG_REG_08, 09 is 10 bit signed calibration offset value */
	ret = tps80031_reads(di->dev->parent,SLAVE_ID2,FG_REG_08,2,temp);
	if (ret < 0) {
		dev_err(di->dev, "Error reading FG_REG_08-09\n");
		return;
	}

	cc_offset = le16_to_cpup((u16 *)temp);
	cc_offset = ((s16)(cc_offset << 6) >> 6);
	di->cc_offset = cc_offset;

	/* resume FG updates */
	reg = FG_REG_00_CC_PAUSE;

	ret = tps80031_update(di->dev->parent, SLAVE_ID2,FG_REG_00, 0, reg);
	if (ret < 0) {
		dev_err(di->dev, "Error resume FG updates FG_REG_00\n");
		return;
	}

	samples = di->battery_timer_n1 - di->battery_timer_n2;
	/* check for timer overflow */
	if (di->battery_timer_n1 < di->battery_timer_n2)
		samples = samples + (1 << 24);

	/* offset is accumulative over number of samples */
	cc_offset = cc_offset * samples;

	current_avg_uA = ((di->battery_charge_n1 - di->battery_charge_n2
					- cc_offset)
					* di->current_max_scale) /
					fuelgauge_rate[di->fuelgauge_mode];
	/* clock is a fixed 32Khz */
	current_avg_uA >>= 15;

	/* Correct for the fuelguage sampling rate */
	samples /= fuelgauge_rate[di->fuelgauge_mode] * 4;

	/*
	 * Only update the current average if we have had a valid number
	 * of samples in the accumulation.
	 */
	if (samples) {
		current_avg_uA = current_avg_uA / samples;
		di->battery_current_avg_uA = current_avg_uA * 1000;
	}
	//update battery charging status
	di->battery_charge_status = tps80031_battery_charging_status(di);
	//update battery health status
	di->battery_health = tps80031_battery_health(di);

	tps80032_calculate_capacity(di);

	schedule_delayed_work(&di->battery_current_avg_work,
		msecs_to_jiffies(1000 * di->battery_current_avg_interval));
	return;
}

static int tps80032_current_setup(struct tps80031_device_info *di,
		struct tps80031_bg_platform_data *pdata)
{
	//Differnet from Ti's Palmas Platform  FG_REG_00  TOGGLE1
	int ret = 0;
	unsigned int reg = 0;
	u8 temp[4];
	/*
	 * Enable the AUTOCLEAR so that any FG is in known state, and
	 * enabled the FG
	 */
	reg = FG_REG_00_CC_AUTOCLEAR;

	ret = tps80031_update(di->dev->parent, SLAVE_ID2,FG_REG_00, reg, reg);
	if (ret < 0) {
		dev_err(di->dev, "Error AUTOCLEAR FG FG_REG_00\n");
		return ret;
	}
	//Enable Fule Gauge and DITH
	reg = TOGGLE1_FGDITHS | TOGGLE1_FGS;

	ret = tps80031_update(di->dev->parent, SLAVE_ID2,TOGGLE1, reg, reg);
	if (ret < 0) {
		dev_err(di->dev, "Error Enable Fule Gauge and DITH TOGGLE1\n");
		return ret;
	}
	//Enables calibration
	reg = FG_REG_00_CC_CAL_EN;

	ret = tps80031_update(di->dev->parent, SLAVE_ID2,FG_REG_00, reg, reg);
	if (ret < 0) {
		dev_err(di->dev, "Error Enables calibration FG_REG_00\n");
		return ret;
	}
	/* initialise the current average values */
	di->battery_current_avg_interval = pdata->current_avg_interval;

	/* pause FG updates to get consistant data */
	reg = FG_REG_00_CC_PAUSE;

	ret = tps80031_update(di->dev->parent, SLAVE_ID2,FG_REG_00, reg, reg);
	if (ret < 0) {
		dev_err(di->dev, "Error pause FG updates FG_REG_00\n");
		return ret;
	}

	/* FG_REG_01, 02, 03 is 24 bit unsigned sample counter value */
	ret = tps80031_reads(di->dev->parent,SLAVE_ID2,FG_REG_01,3,temp);
	if (ret < 0) {
		dev_err(di->dev, "Error reading FG_REG_01-03\n");
		return ret;
	}

	temp[3] = 0;

	di->battery_timer_n1 = le32_to_cpup((u32 *)temp);

	/*
	 * FG_REG_04, 5, 6, 7 is 32 bit signed accumulator value
	 * accumulates instantaneous current value
	 */
	ret = tps80031_reads(di->dev->parent,SLAVE_ID2,FG_REG_04,4,temp);
	if (ret < 0) {
		dev_err(di->dev, "Error reading FG_REG_04-07\n");
		return ret;
	}

	di->battery_charge_n1 = le32_to_cpup((u32 *)temp);

	/* FG_REG_08, 09 is 10 bit signed calibration offset value */
	ret = tps80031_reads(di->dev->parent,SLAVE_ID2,FG_REG_08,2,temp);
	if (ret < 0) {
		dev_err(di->dev, "Error reading FG_REG_08-09\n");
		return ret;
	}

	di->cc_offset = le16_to_cpup((u16 *)temp);
	di->cc_offset = ((s16)(di->cc_offset << 6) >> 6);

	INIT_DELAYED_WORK_DEFERRABLE(&di->battery_current_avg_work,
						tps80032_battery_current_avg);//diff with 3.7 version

	schedule_delayed_work(&di->battery_current_avg_work,
		msecs_to_jiffies(1000 * di->battery_current_avg_interval));

	return ret;
}


static int tps80031_bat_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp)
	{
		case POWER_SUPPLY_PROP_CHARGE_FULL:
		case POWER_SUPPLY_PROP_CYCLE_COUNT:
			return 1;

		default:
			break;
	}

	return 0;
}

static int tps80031_battery_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	uint8_t retval;
	struct tps80031_device_info *di;
	struct tps80031_platform_data *tps80031_pdata;
	struct tps80031_bg_platform_data *pdata;

	tps80031_pdata = dev_get_platdata(pdev->dev.parent);
	if (!tps80031_pdata) {
		dev_err(&pdev->dev, "no tps80031 platform_data specified\n");
		return -EINVAL;
	}

	pdata = tps80031_pdata->bg_pdata;
	if (!pdata) {
		dev_err(&pdev->dev, "no battery_gauge platform data\n");
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
		dev_err(dev, "%s() standalone fuel gauge detected,board gauge exiting..\n",
				__func__);
		return -ENODEV;

	}
#endif

	//Fuel Gauge Settings
	di->cell.config = pdata->cell_cfg;

	/* Start with battery health good until we get wrong state*/
	di->battery_health = POWER_SUPPLY_HEALTH_GOOD;
	di->dev =  &pdev->dev;
	di->cell.charge_status = &di->battery_charge_status;//USB/AC  wrong status diff from palmas
	di->cell.dev = di->dev;//Copy for FG
	dev_set_drvdata(&pdev->dev, di);
	/* calculate current max scale from sense */
	di->current_max_scale = (62000) / di->cell.config->r_sense;
	/* Initialise the Coloumb Counter */
	tps80032_current_setup(di, pdata);

	di->bat.name		= "battery";
	di->bat.type		= POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties	= tps80031_bat_props;
	di->bat.num_properties	= ARRAY_SIZE(tps80031_bat_props);
	di->bat.get_property	= tps80031_bat_get_property;
	di->bat.set_property	= tps80031_bat_set_property;
	di->bat.property_is_writeable =
				  tps80031_bat_property_is_writeable;

	ret = power_supply_register(dev->parent, &di->bat);
	if (ret) {
		dev_err(dev->parent, "failed to register bat power supply\n");
		return ret;
	}
#ifdef	CONFIG_TPS80032_USB_CHARGER
	di->usb.name		= "usb";
	di->usb.type		= POWER_SUPPLY_TYPE_USB;
	di->usb.properties	= tps80031_usb_props;
	di->usb.num_properties	= ARRAY_SIZE(tps80031_usb_props);
	di->usb.get_property	= tps80031_usb_get_property;

	ret = power_supply_register(dev->parent, &di->usb);
	if (ret) {
		dev_err(dev->parent, "failed to register ac power supply\n");
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

	wake_lock_init(&di->ac_chrg_lock, WAKE_LOCK_SUSPEND, "ac_chrg_wake_lock");

	//Init charger status
	di->ac_online = 0;
	di->usb_online = 0;
	di->cache.ac_online  = di->ac_online;
        di->cache.usb_online = di->usb_online;

	ret = register_charging_state_callback(tps80031_battery_status, di);
	if (ret < 0)
		goto power_supply_fail0;

//Fill Initial State To-do  Clean
	tps80031_battery_status_init(di);

	/* Initial boot voltage */
	ret = tps80031_reg_read(di, SLAVE_ID2, CONTROLLER_STAT1, &retval);
	if (ret < 0)
		return ret;
	//Beta Driver don't do any voltage adjust,just add some initial code here(Final Driver still have this iisue)
	if ( ( (retval & TPS80031_VBUS_DET)||(tegra_power_reason & State_Charger_PlugIn) )
		|| (tegra_power_reason & State_Normal_USB_PlugIn) ) {
		tps80031_battery_voltage(di);
		dev_info(dev->parent, "STEP0 voltage from kernel %d",di->battery_voltage_uV/1000);
		//DISABLE LINEAR CHARGER
		ret = tps80031_update(di->dev->parent, SLAVE_ID2,CONTROLLER_CTRL1, 0, CONTROLLER_CTRL1_EN_LINCH);
		if (ret < 0)
			return ret;
		msleep(100);
		tps80031_battery_voltage(di);
		//RE_ENABLE LINEAR CHARGER
		ret = tps80031_update(di->dev->parent, SLAVE_ID2,CONTROLLER_CTRL1,CONTROLLER_CTRL1_EN_LINCH, CONTROLLER_CTRL1_EN_LINCH);
		if (ret < 0)
			return ret;
		dev_info(dev->parent, "Init voltage from kernel %d",di->battery_voltage_uV/1000);
		if(di->ac_online == 1)
			di->battery_voltage_uV = di->battery_voltage_uV - 50*1000;//50mV -- Need Fix
	} else {
		di->battery_voltage_uV = cap_of_battery * 1000;
		dev_info(dev->parent, "Init voltage from bootloader %d",di->battery_voltage_uV/1000);
	}
	/* Initialise the Fuel Guage */
	fg_init(&di->cell, di->battery_voltage_uV / 1000);

	dev_info(dev->parent, "support ver. %s enabled r2: %d\n", DRIVER_VERSION,di->cell.config->r_sense);

	return ret;

power_supply_fail0:
	power_supply_unregister(&di->ac);
power_supply_fail1:
#ifdef	CONFIG_TPS80032_USB_CHARGER
	power_supply_unregister(&di->usb);
power_supply_fail2:
#endif
	power_supply_unregister(&di->bat);
	return ret;
}

static int tps80031_battery_remove(struct platform_device *pdev)
{
	struct tps80031_device_info *di = dev_get_drvdata(&pdev->dev);

	power_supply_unregister(&di->bat);
#ifdef	CONFIG_TPS80032_USB_CHARGER
	power_supply_unregister(&di->usb);
#endif
	power_supply_unregister(&di->ac);

	return 0;
}
//To-DO,Implement Here(Cancel the work during suspend and re-schedule it in resume)
static int tps80031_battery_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tps80031_device_info *di = platform_get_drvdata(pdev);
	cancel_delayed_work_sync(&di->battery_current_avg_work);

	return 0;
}

static int tps80031_battery_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tps80031_device_info *di = platform_get_drvdata(pdev);
	schedule_delayed_work(&di->battery_current_avg_work, 50);

	return 0;
}

static const struct dev_pm_ops pm_ops = {
	.suspend	=  tps80031_battery_suspend,
	.resume		=  tps80031_battery_resume,
};


static struct platform_driver tps80031_battery_driver = {
	.driver	= {
		.name	= "tps80031-battery-gauge",
		.owner	= THIS_MODULE,
		.pm = &pm_ops,
	},
	.probe	= tps80031_battery_probe,
	.remove = tps80031_battery_remove,
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
MODULE_AUTHOR("Syed Rafiuddin <srafiuddin@nvidia.com> ");
MODULE_DESCRIPTION("tps80031 battery gauge driver");
