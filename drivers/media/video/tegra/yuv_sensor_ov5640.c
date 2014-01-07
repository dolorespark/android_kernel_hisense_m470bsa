/*
 * kernel/drivers/media/video/tegra
 *
 * Aptina MT9D115 sensor driver
 *
 * Copyright (C) 2010 NVIDIA Corporation
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
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <media/yuv_sensor.h>
#include "yuv_ov5640_init_tab.h"


////////
#if 0
#include <linux/debugfs.h>
#include <linux/rtc.h>
#endif
///////
#include <linux/gpio.h>  
#include "../../../../arch/arm/mach-tegra/board-enterprise.h"
#include "../../../../arch/arm/mach-tegra/gpio-names.h"

extern int lm3642_flash(int flash_mode);

#define SENSOR_NAME	"ov5640"

#define FLASH_THRESHOLD 0x18

#define XVCLK (2400UL)

#define MT_NULL      (0)
#define MT_TRUE      (1)
#define MT_FALSE     (0)

#define CAM_FLASH_T1 (300)
#define CAM_FLASH_T2 (100)
#define CAM_FLASH_T3 (100)
#define CAM_FLASH_T4 (100)


u32 preview_sysclk;
u32 preview_HTS, preview_VTS;
/////////
#if 0 
static struct dentry *ov5640_debugfs_root;
struct i2c_client *client_reg;
#endif
////////
static struct delayed_work ov5640_delayed_work; 
static int flash_on_off; 

static u8 SceneModeValue_ov5640 = SceneMode_Value_Auto;
static u8 ExposureValue_ov5640 = Exposure_Value_0;
static u8 Is_AF_Need_Init = 0;
static int g_isVideoFlash = MT_FALSE;
static int g_isFlash = MT_FALSE; 
static int g_flashAuto = MT_FALSE;
static int g_af_status = 0;

#define SIZEOF_I2C_TRANSBUF (64)
u8 i2c_trans_buf[SIZEOF_I2C_TRANSBUF];
////static u8 Is_CAPTURE_COMMAND_FLAG =0;
u8 Is_CAPTURE_COMMAND_FLAG =0;

//static u8 limunance_diff=0;

////////
#if 0
int dump_enable=0;
static DEFINE_MUTEX(ov5640_regs_dump_lock);
#endif
//////////
static u8 sensor_ov5640_read_reg_8bit(struct i2c_client *client, u16 addr, u8 *val)
{
     int err;
     struct i2c_msg msg[2];
     unsigned char data[3];
     
     if (!client->adapter)
     	return -ENODEV;
     
     msg[0].addr = client->addr;
     msg[0].flags = 0;
     msg[0].len = 2;
     msg[0].buf = data;
     
     
     data[0] = (u8) (addr >> 8);
     data[1] = (u8) (addr & 0xff);
     
     msg[1].addr = client->addr;
     msg[1].flags = I2C_M_RD;
     msg[1].len = 1;
     msg[1].buf = data + 2;
     
     err = i2c_transfer(client->adapter, msg, 2);
     
     if (err != 2)
     	return -EINVAL;
     
     *val = data[2];
     
     return *val;
}

static int sensor_ov5640_read_reg_16bit(struct i2c_client *client, u16 addr, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

    swap(*(data+2),*(data+3)); //swap high and low byte to match table format
	memcpy(val, data+2, 2);

	return 0;
}

static int sensor_ov5640_write_reg_8bit(struct i2c_client *client, u16 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[3];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);
        
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("ov5640: i2c transfer failed, retrying %x %x\n",
		       addr, val);
	} while (retry <= SENSOR_MAX_RETRIES);

	return err;
}

static int sensor_ov5640_write_reg_16bit(struct i2c_client *client, u16 addr, u16 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[4];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
    data[2] = (u8) (val >> 8);
	data[3] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 4;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("yuv_sensor : i2c transfer failed, retrying %x %x\n",
		       addr, val);
	} while (retry <= SENSOR_MAX_RETRIES);

	return err;
}

static int sensor_ov5640_poll(struct i2c_client *client, u16 addr, u16 value,
			u16 mask)
{
	u16 data;
	int try, err;

	for (try = 0; try < SENSOR_POLL_RETRIES; try++) {
		err = sensor_ov5640_read_reg_16bit(client, addr, &data);
		if (err)
			return err;
		pr_info("poll reg %x: %x == %x & %x %d times\n", addr,
			value, data, mask, try);
		if (value == (data & mask)) {
			return 0;
		}
	}
	pr_err("%s: poll for %d times %x == ([%x]=%x) & %x failed\n",__func__, try, value,
	       addr, data, mask);

	return -EIO;

}


static int sensor_ov5640_poll_bit_set(struct i2c_client *client, u16 addr, u16 bit)
{
	return sensor_ov5640_poll(client, addr, bit, bit);
}

static int sensor_ov5640_write_table(struct i2c_client *client,
			      const struct sensor_reg table[])
{
	const struct sensor_reg *next;
	int err;
    next = table ;
        
	for (next = table; next->op != SENSOR_TABLE_END; next++) {

		switch (next->op) {
		case WRITE_REG_DATA8:
		{
			err = sensor_ov5640_write_reg_8bit(client, next->addr,
				next->val);
			if (err)
				return err;
			break;
		}
		case WRITE_REG_DATA16:
		{
			err = sensor_ov5640_write_reg_16bit(client, next->addr,
				next->val);
			if (err)
				return err;
			break;
		}
		case SENSOR_WAIT_MS:
		{
			msleep(next->val);
			break;
		}
		case POLL_REG_BIT:
		{
			err = sensor_ov5640_poll_bit_set(client, next->addr,
				next->val);
			if (err)
				return err;
			break;
		}
		default:
			pr_err("%s: invalid operation 0x%x\n", __func__,
				next->op);
			return err;
		}
	}
	return 0;
}


static int ov5640_write_bulk_reg(struct i2c_client *client, u8 *data, int len)
{
	int err;
	struct i2c_msg msg;

	if (!client->adapter)
		return -ENODEV;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err == 1)
		return 0;

	dev_err(&client->dev, "ov5640: i2c transfer failed at %x\n",
		(int)data[0] << 8 | data[1]);

	return err;
}


static int sensor_ov5640_write_table_8bit(struct i2c_client *client,
			      struct sensor_reg table[])
{
	int err;
	struct sensor_reg *next, *n_next;
	u8 *b_ptr = i2c_trans_buf;
	unsigned int buf_filled = 0;
	u16 val;

	for (next = table; next->op != SENSOR_TABLE_END; next++) {
		if (next->op == SENSOR_WAIT_MS) {
			msleep(next->val);
			continue;
		}

		val = next->val;

		if (!buf_filled) {
			b_ptr = i2c_trans_buf;
			*b_ptr++ = next->addr >> 8;
			*b_ptr++ = next->addr & 0xff;
			buf_filled = 2;
		}
		*b_ptr++ = val;
		buf_filled++;

		n_next = next + 1;
		if (n_next->addr != SENSOR_TABLE_END &&
			n_next->addr != SENSOR_WAIT_MS &&
			buf_filled < SIZEOF_I2C_TRANSBUF &&
			n_next->addr == next->addr + 1) {
			continue;
		}

		err = ov5640_write_bulk_reg(client,
			i2c_trans_buf, buf_filled);
		if (err)
			return err;

		buf_filled = 0;
	}
	return 0;
}






u32 sensor_ov5640_get_sysclk(struct sensor_info *info)
{
	u8 val;
	u32 temp1, temp2, temp3;
	u32 Multiplier, PreDiv,SysDiv, Pll_rdiv, Bit_div2x, sclk_rdiv;
	u32 VCO,sysclk;
	u32 sclk_rdiv_map[] = {1, 2, 4, 8};

	temp1 = sensor_ov5640_read_reg_8bit(info->i2c_client,0x3034,&val); 
	temp2 = temp1 & 0x0f;
	if (temp2 == 8 || temp2 == 10) {
		Bit_div2x = (u32)(temp2 / 2);
	}
	else
	{
		Bit_div2x = 1;
	}
	temp1 = sensor_ov5640_read_reg_8bit(info->i2c_client,0x3035,&val); 
	if(0==temp1)
		printk("ERROR!!! reg0x3035 is zero .\n");
	SysDiv = temp1>>4;
	if(SysDiv == 0) {
		SysDiv = 16;
	}

	temp1 = sensor_ov5640_read_reg_8bit(info->i2c_client,0x3036,&val); 
	if(0==temp1)
	printk("ERROR!!! reg0x3036 is zero .\n");
	Multiplier = temp1;
	temp1 = sensor_ov5640_read_reg_8bit(info->i2c_client,0x3037,&val); 
//	temp1 = 0x13;//test 0x3037 occur zero  issue 
	temp3 = temp1;

	PreDiv = temp1 & 0x0f;
	Pll_rdiv = ((temp1 >> 4) & 0x01) + 1;

	temp1 = sensor_ov5640_read_reg_8bit(info->i2c_client,0x3108,&val); 
	temp2 = temp1 & 0x03;
	sclk_rdiv = sclk_rdiv_map[temp2];
	VCO =(u32) (XVCLK * Multiplier / PreDiv);
	if( VCO==0 || SysDiv==0 || Pll_rdiv==0 || Bit_div2x==0 || sclk_rdiv==0 || PreDiv==0)
		{
	printk("!!! WARNING !!! VCO=%x, SysDiv=%x,Pll_rdiv=%x, Bit_div2x=%x, sclk_rdiv=%x, PreDiv=%x reg 0x3037 %x\n", VCO, SysDiv, Pll_rdiv, Bit_div2x, sclk_rdiv, PreDiv, temp3);
		}
	sysclk = (u32)(VCO / SysDiv / Pll_rdiv * 2 / Bit_div2x / sclk_rdiv);
	return sysclk;
}

u32 sensor_ov5640_get_hts(struct sensor_info *info)
{
	u8 val;
	u32 HTS;
	HTS= sensor_ov5640_read_reg_8bit(info->i2c_client,0x380c,&val); 
	HTS = (HTS<<8) + sensor_ov5640_read_reg_8bit(info->i2c_client,0x380d,&val); 
	return HTS;
}

u32 sensor_ov5640_get_vts(struct sensor_info *info)
{
	u8 val;
	u32 VTS;
	VTS = sensor_ov5640_read_reg_8bit(info->i2c_client,0x380e,&val);
	VTS = (VTS<<8) + sensor_ov5640_read_reg_8bit(info->i2c_client,0x380f,&val);
	return VTS;
}

int sensor_ov5640_set_vts(struct sensor_info *info,u32 VTS)
{
	int err;
	u32 temp;
	temp = VTS & 0xff;
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x380f, temp);
	temp = VTS>>8;
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x380e, temp);
	return 0;
}

u32 sensor_ov5640_get_shutter(struct sensor_info *info)
{
	u8 val;
	u32 shutter;
	
	shutter = sensor_ov5640_read_reg_8bit(info->i2c_client,0x3500,&val);
	shutter = (shutter<<8) + sensor_ov5640_read_reg_8bit(info->i2c_client,0x3501,&val);
	shutter = (shutter<<4) + (sensor_ov5640_read_reg_8bit(info->i2c_client,0x3502,&val)>>4);

	return shutter;
}

u32 sensor_ov5640_set_shutter(struct sensor_info *info,u32 shutter)
{
	int err;
	u32 temp;
	
	shutter = shutter & 0xffff;
	temp = shutter & 0x0f;
	temp = temp<<4;
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3502, temp);
	temp = shutter & 0xfff;
	temp = temp>>4;
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3501, temp);
	temp = shutter>>12;
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3500, temp);
	
	return 0;
}

u32 sensor_ov5640_get_gain16(struct sensor_info *info)
{
	u8 val;
	u32 gain16;
	gain16 =  sensor_ov5640_read_reg_8bit(info->i2c_client,0x350a,&val) & 0x03;
	gain16 = (gain16<<8) + sensor_ov5640_read_reg_8bit(info->i2c_client,0x350b,&val);
	return gain16;
}

u32 sensor_ov5640_set_gain16(struct sensor_info *info,u32 gain16)
{
	int err;
	u32 temp;
	gain16 = gain16 & 0x3ff;
	temp = gain16 & 0xff; 
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x350b, temp);
	temp = gain16>>8;
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x350a, temp);
	return 0;
}

u32 sensor_ov5640_get_light_frequency(struct sensor_info *info)
{
	u8 val;
	// get banding filter value
	u32 temp, temp1, light_frequency;
	temp = sensor_ov5640_read_reg_8bit(info->i2c_client,0x3c01,&val);
	if (temp & 0x80) {		
	     // manual
	     temp1 = sensor_ov5640_read_reg_8bit(info->i2c_client,0x3c00,&val);
	     if (temp1 & 0x04) {
	            // 50Hz
	            light_frequency = 50;
	     }
	     else {
	            // 60Hz
	            light_frequency = 60;
	     }
	}
	else {
	     // auto

		 temp1 = sensor_ov5640_read_reg_8bit(info->i2c_client,0x3c0c,&val);
	     if (temp1 & 0x01) {
	            // 50Hz
	            light_frequency = 50;
	     }
	     else {
	            // 60Hz
	             light_frequency = 60;
	     }
	}
	return light_frequency;
}

void sensor_ov5640_set_bandingfilter(struct sensor_info *info)
{
	int err;
	u32 preview_VTS;
	u32 band_step60, max_band60, band_step50, max_band50;
	// read preview PCLK
	preview_sysclk = sensor_ov5640_get_sysclk(info);
	// read preview HTS
	preview_HTS = sensor_ov5640_get_hts(info);
	// read preview VTS
	preview_VTS = sensor_ov5640_get_vts(info);
	// calculate banding filter
	// 60Hz
	band_step60 = preview_sysclk * 100/preview_HTS * 100/120;
	
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a0a, (band_step60 >> 8));
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a0b, (band_step60 & 0xff));
	max_band60 = (u32)((preview_VTS-4)/band_step60);
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a0d, max_band60);
	// 50Hz
	band_step50 = preview_sysclk * 100/preview_HTS;

	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a08, (band_step50 >> 8));
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a09, (band_step50 & 0xff));
	max_band50 = (u32)((preview_VTS-4)/band_step50);
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a0e, max_band50);
}


u32 sensor_ov5640_set_ae_target(struct sensor_info *info,u32 target)
{
	int err;
	u32 AE_high, AE_low;
	// stable in high
	u32 fast_high, fast_low;
	AE_low = (u32)(target * 23 / 25); // 0.92
	AE_high = (u32)(target * 27 / 25); // 1.08
	fast_high = AE_high<<1;
	if(fast_high>255)
	     fast_high = 255;
	fast_low = AE_low>>1;

	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a0f, AE_high);
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a10, AE_low);
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a1b, AE_high);
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a1e, AE_low);
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a11, fast_high);
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a1f, fast_low);	  
	return 0;
}


static void sensor_flash_delay_work(struct work_struct *w)
{

	if (flash_on_off) {
        flash_on_off = 0;	
		gpio_direction_output(TEGRA_GPIO_CAM_TORCH_EN, 0);  
		gpio_direction_output(TEGRA_GPIO_CAM_FLASH_EN, 0);

	} 
	else {
		pr_info("FLASH OVER!!!!!!!!!!!!!\n");
		gpio_direction_output(TEGRA_GPIO_CAM_FLASH_EN, 0); //shut off flash
		gpio_direction_output(TEGRA_GPIO_CAM_TORCH_EN, 0); 

    }
	
}

int sensor_ov5640_capture(struct sensor_info *info)
{
////	u8 val;
	int err;
//	u32 preview_shutter, preview_gain16, average;
	u32 preview_shutter, preview_gain16;
	u32 capture_shutter;
	u32 capture_gain16;
	u32 capture_HTS, capture_VTS;
	u32 capture_sysclk;
	u32 light_frequency, capture_bandingfilter;
	u32 capture_max_band;
	u32 capture_gain16_shutter;
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a00, 0x38);
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a05, 0x30);
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a08, 0x01);
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a09, 0x27);
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a0a, 0x00);
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a0b, 0xf6);	 

	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a0d, 0x04);
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a0e, 0x03);
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a02, 0x03);
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a03, 0xd8);

	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a14, 0x03);
	err =sensor_ov5640_write_reg_8bit(info->i2c_client,0x3a15, 0xd8);

/*
	{WRITE_REG_DATA8,0x3a00 ,0x38},
	{WRITE_REG_DATA8,0x3a05 ,0x30},
	{WRITE_REG_DATA8,0x3a08 ,0x01},
	{WRITE_REG_DATA8,0x3a09 ,0x27},
	{WRITE_REG_DATA8,0x3a0a ,0x00},
	{WRITE_REG_DATA8,0x3a0b ,0xf6},
	{WRITE_REG_DATA8,0x3a0d ,0x04},
	{WRITE_REG_DATA8,0x3a0e ,0x03},
	{WRITE_REG_DATA8,0x3a02 ,0x03},
	{WRITE_REG_DATA8,0x3a03 ,0xd8},
	{WRITE_REG_DATA8,0x3a14 ,0x03},
	{WRITE_REG_DATA8,0x3a15 ,0xd8},
*/

	preview_shutter = sensor_ov5640_get_shutter(info);
	preview_gain16 = sensor_ov5640_get_gain16(info);
//	average = sensor_ov5640_read_reg_8bit(info->i2c_client,0x56a1,&val);

	err = sensor_ov5640_write_table(info->i2c_client, mode_2592x1944);
	// read capture VTS
	capture_VTS = sensor_ov5640_get_vts(info);
	capture_HTS = sensor_ov5640_get_hts(info);
	capture_sysclk = sensor_ov5640_get_sysclk(info);
	// calculate capture banding filter
	light_frequency = sensor_ov5640_get_light_frequency(info);
	if (light_frequency == 60) {
	     // 60Hz
	     capture_bandingfilter =(u32)( capture_sysclk * 100 / capture_HTS * 100 / 120);
	}
	else {
	     // 50Hz
	     capture_bandingfilter = (u32)(capture_sysclk * 100 / capture_HTS);
	}

	capture_max_band = (u32)((capture_VTS - 4)/capture_bandingfilter);
	// calculate capture shutter/gain16

	capture_gain16_shutter = preview_gain16 * preview_shutter * capture_sysclk/preview_sysclk *preview_HTS/capture_HTS;

//	if(limunance_diff > 220)
//	    capture_gain16_shutter = preview_gain16 * preview_shutter * capture_sysclk/preview_sysclk *preview_HTS/capture_HTS/8;
//	else if (limunance_diff > 200)
//		capture_gain16_shutter = preview_gain16 * preview_shutter * capture_sysclk/preview_sysclk *preview_HTS/capture_HTS/6;
//    else if (limunance_diff > 150)
//		capture_gain16_shutter = preview_gain16 * preview_shutter * capture_sysclk/preview_sysclk *preview_HTS/capture_HTS/4;
//	else if(limunance_diff > 100)
//		capture_gain16_shutter = preview_gain16 * preview_shutter * capture_sysclk/preview_sysclk *preview_HTS/capture_HTS/2;
//	else
//		capture_gain16_shutter = preview_gain16 * preview_shutter * capture_sysclk/preview_sysclk *preview_HTS/capture_HTS;
//	limunance_diff =0;


	// gain to shutter
	if(capture_gain16_shutter < (capture_bandingfilter * 16)) {
	     // shutter < 1/100
	     capture_shutter = (u32)(capture_gain16_shutter/16);
	     if(capture_shutter <1)
	            capture_shutter = 1;
	     capture_gain16 = (u32)(capture_gain16_shutter/capture_shutter);
	     if(capture_gain16 < 16)
	            capture_gain16 = 16;
	}
	else {
	     if(capture_gain16_shutter > (capture_bandingfilter*capture_max_band*16)) {
	            // exposure reach max
	            capture_shutter = capture_bandingfilter*capture_max_band;
	            capture_gain16 = (u32)(capture_gain16_shutter / capture_shutter);
	     }
	     else {
	            // 1/100 < capture_shutter =< max, capture_shutter = n/100
	            capture_shutter = ((u32)(capture_gain16_shutter/16/capture_bandingfilter)) * capture_bandingfilter;
	            capture_gain16 = capture_gain16_shutter / capture_shutter;
	     }
	}
	// write capture gain
	sensor_ov5640_set_gain16(info,capture_gain16);
	// write capture shutter
	if (capture_shutter > (capture_VTS - 4)) {
	     capture_VTS = capture_shutter + 4;
	     sensor_ov5640_set_vts(info,capture_VTS);
	}

	sensor_ov5640_set_shutter(info,capture_shutter);

	// skip 2 vysnc
	// start capture at 3rd vsync
	return 0;
}

static int sensor_ov5640_set_mode(struct sensor_info *info, struct sensor_mode *mode)
{
	int sensor_table;
	struct sensor_reg * exposure_table;
	struct sensor_reg * scene_table;
	int err =0;
	u8 val;
	u16 rc = 0;
	//u8 limunance_fir, limunance_snd;
	//u32 temp1;

	if (mode->xres == 2592 && mode->yres == 1944)
		sensor_table = SENSOR_MODE_2592x1944;
	else if (mode->xres == 1920 && mode->yres == 1080)
		sensor_table = SENSOR_MODE_1920x1080;
	else if (mode->xres == 1280 && mode->yres == 960)
		sensor_table = SENSOR_MODE_1280x960;
	else if (mode->xres == 1280 && mode->yres == 720)
		sensor_table = SENSOR_MODE_1280x720;
		else if (mode->xres == 1024 && mode->yres == 768)
		sensor_table = SENSOR_MODE_1024x768;
	else if (mode->xres == 640 && mode->yres == 480)
		sensor_table = SENSOR_MODE_640x480;
	else if (mode->xres == 176 && mode->yres == 144)
	sensor_table = SENSOR_MODE_176x144;
	else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

    pr_err("First time initial check value %x\n",val);

    //check already program the sensor mode, Aptina support Context B fast switching capture mode back to preview mode
    //we don't need to re-program the sensor mode for 640x480 table
    
    if(sensor_table == SENSOR_MODE_2592x1944)
    {
	    pr_info("initial cts tab sensor_mode_2592x1944 %d\n",sensor_table);

        if (MT_TRUE == g_flashAuto) {
		 	rc = sensor_ov5640_read_reg_8bit(info->i2c_client,0x56a1,&val); 
			
			if (rc < 0)
				return rc;
			printk("the threshold of flashauto is %d\n",val);

			if (val < FLASH_THRESHOLD) {   //flash threshold
				g_isFlash = MT_TRUE;
			}
			else{
				g_isFlash = MT_FALSE;
			}
		}
		sensor_ov5640_capture(info);

        msleep(360); //modify ... give up over 2 frame

	if(Is_CAPTURE_COMMAND_FLAG ==1){
		
		Is_CAPTURE_COMMAND_FLAG =0;
		if(MT_TRUE == g_isFlash){
			flash_on_off = 1;	
			//gpio_direction_output(HS_GPIO_CAM_TORCH_EN, 0); //open torch
		////	msleep(200);// preflash 200ms
			gpio_direction_output(TEGRA_GPIO_CAM_FLASH_EN, 1); //open flash	
			err = lm3642_flash(2);
			schedule_delayed_work(&ov5640_delayed_work, msecs_to_jiffies(CAM_FLASH_T1));
		}
	}

	if (err) {
		return err;
	}

    }
	else
    {

        if (MT_TRUE == g_flashAuto) {
		 	rc = sensor_ov5640_read_reg_8bit(info->i2c_client,0x56a1,&val); 
			
			if (rc < 0)
				return rc;
			printk("the threshold of flashauto is %d\n",val);

			if (val < FLASH_THRESHOLD) {   //flash threshold
				g_isFlash = MT_TRUE;
			}
			else{
				g_isFlash = MT_FALSE;
			}
		}


	
		if(MT_TRUE == g_isVideoFlash){
			 gpio_direction_output(TEGRA_GPIO_CAM_TORCH_EN, 1);
			 err = lm3642_flash(1);
		}	

		if(MT_FALSE == g_isVideoFlash){
			gpio_direction_output(TEGRA_GPIO_CAM_FLASH_EN, 0); //shut off flash
			gpio_direction_output(TEGRA_GPIO_CAM_TORCH_EN, 0);
		}

		printk("initial tab %d\n",sensor_table);		

		sensor_ov5640_write_reg_8bit(info->i2c_client,0x4202, 0x00); //enable mipi
		sensor_ov5640_write_reg_8bit(info->i2c_client,0x3003, 0x00);  
		sensor_ov5640_write_reg_8bit(info->i2c_client,0x4202, 0x0f); //  disenable mipi

		err = sensor_ov5640_write_table(info->i2c_client, ov5640_init);

		g_af_status =0;			

		err = sensor_ov5640_write_table(info->i2c_client, mode_table[sensor_table]);

        if(Is_AF_Need_Init)
        {
     //   if(1)
	//		err = sensor_ov5640_write_table(info->i2c_client, af_load_fw);//download af firmware
     //   else
			err = sensor_ov5640_write_table_8bit(info->i2c_client, af_load_fw);//download af firmware
			Is_AF_Need_Init = 0;
		}

		sensor_ov5640_set_bandingfilter(info);

//		sensor_ov5640_write_reg_8bit(info->i2c_client,0x4202, 0x00); //enable mipi

		switch(ExposureValue_ov5640){
			case Exposure_Value_0:
				exposure_table = exp_zero;
				break;
			case Exposure_Value_1:
				exposure_table = exp_one;
				break;
			case Exposure_Value_2:
				exposure_table = exp_two;
				break;
			case Exposure_Value_Negative_1:
				exposure_table = exp_negative1;
				break;
			case Exposure_Value_Negative_2:
				exposure_table = exp_negative2;
				break;
			default:
				exposure_table = NULL;
				break;
		}

		if (exposure_table)
		{
			err = sensor_ov5640_write_table(info->i2c_client, exposure_table);
		}

		switch(SceneModeValue_ov5640){
			case SceneMode_Value_Auto:
				scene_table = scene_auto;
				break;
			case SceneMode_Value_Night:
				scene_table = scene_night;
				break;
			case SceneMode_Value_Action:
				scene_table = scene_action;
				break;
			case SceneMode_Value_Sunset:
				scene_table = scene_sunset;
				break;
			default :
				scene_table = NULL;
				break;
		}

		if (scene_table)
		{
			err = sensor_ov5640_write_table(info->i2c_client, scene_table);
		}

        if (err) {
            return err; 
        } 
		sensor_ov5640_write_reg_8bit(info->i2c_client,0x4202, 0x00); //enable mipi

        msleep(200); //modify ... give up over 2 frame

		
if(Is_CAPTURE_COMMAND_FLAG)

{
		Is_CAPTURE_COMMAND_FLAG =0;
				//huang
		 if(MT_TRUE == g_isFlash){
		 flash_on_off = 1;  
	//	  gpio_direction_output(HS_GPIO_CAM_TORCH_EN, 0); //open torch
		 msleep(200);// preflash 200ms
		 gpio_direction_output(TEGRA_GPIO_CAM_FLASH_EN, 1); //open flash	
		 err = lm3642_flash(2);
		 schedule_delayed_work(&ov5640_delayed_work, msecs_to_jiffies(CAM_FLASH_T1));
//huang
    }
		 
}

/*
	while(1) {
		temp1 = sensor_ov5640_read_reg_8bit(info->i2c_client,0x3037,&val); 
		printk("temp1 = %d\n",temp1);
		if(0==temp1)
			BUG_ON(1);
	}
	*/
		}

	info->mode = sensor_table;

	return 0;
}
static int sensor_ov5640_set_af_mode(struct sensor_info *info, u8 mode){
	sensor_ov5640_write_reg_8bit(info->i2c_client,0x3022, 0x08); //release focus
	// re-launch auto focus zones
	sensor_ov5640_write_reg_8bit(info->i2c_client,0x3022, 0x12); 
	mdelay(20); 
	printk("%s  \n" ,__func__);
	// focus
	sensor_ov5640_write_reg_8bit(info->i2c_client,0x3023, 0x01);  
	sensor_ov5640_write_reg_8bit(info->i2c_client,0x3022, 0x03);
	//msleep(50);	 // modify....	
 	return 0;
}

static int sensor_ov5640_get_af_status(struct sensor_info *info, u8 *val)
{
	int err;
	err = sensor_ov5640_read_reg_8bit(info->i2c_client, 0x3023, val);
	if (err)
		return -EINVAL;
	return 0;
}

static long sensor_ov5640_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)

{
	struct sensor_info *info = file->private_data;
	int err=0;
	u16 val;

	struct sensor_mode mode;
	u8 coloreffect;
	u8 scenemode;
	int Exposure_value;
	int ISO_value;
	int Contrast_value;
	int Saturation_value;

    struct sensor_reg * reg_table = NULL;

	pr_info("%s: cmd %u \n",__func__, cmd);
	switch (cmd) {
	case SENSOR_IOCTL_SET_MODE:
	{
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct sensor_mode))) {
			return -EFAULT;
		}
		return sensor_ov5640_set_mode(info, &mode);
	}
	break;
	case SENSOR_IOCTL_GET_STATUS:
	{	
		pr_info("sensor ioctl SENSOR_IOCTL_GET_STATUS  comein \n");
		return 0;
	}
	break;
	case SENSOR_IOCTL_GET_BRIGHTNESS:
	{
	    err = sensor_ov5640_write_reg_16bit(info->i2c_client, 0x098E, 0xB80C);
	    if (err)
	        return err;
           
        err = sensor_ov5640_read_reg_16bit(info->i2c_client, 0xB80C, &val);

	    pr_info("%s: value %x \n",__func__, val);

        if (copy_to_user((void __user *)arg, &val,2)) 
            pr_info("%s %d\n", __func__, __LINE__);
	  
        return 0;
	}
	break;
	case SENSOR_IOCTL_SET_FLASH_MODE:
	{

		u8 flash_mode_val;
	    int err;
		if (copy_from_user(&flash_mode_val,
				   (const void __user *)arg,
				   sizeof(flash_mode_val))) {
			return -EFAULT;
		}
		switch(flash_mode_val){
			
			case YUV_FlashAuto:
				g_flashAuto = MT_TRUE; 	
				gpio_direction_output(TEGRA_GPIO_CAM_FLASH_EN, 0); //shut off flash
			    gpio_direction_output(TEGRA_GPIO_CAM_TORCH_EN, 0);
				 break;
			case YUV_FlashOn:				
                g_flashAuto = MT_FALSE; 	
				g_isFlash = MT_TRUE;
				gpio_direction_output(TEGRA_GPIO_CAM_FLASH_EN, 0); //shut off flash
				gpio_direction_output(TEGRA_GPIO_CAM_TORCH_EN, 0);
				 break;
			case YUV_FlashOff:
				g_flashAuto = MT_FALSE; 	
				g_isFlash = MT_FALSE;
				g_isVideoFlash = MT_FALSE;

				gpio_direction_output(TEGRA_GPIO_CAM_FLASH_EN, 0); 	 
				gpio_direction_output(TEGRA_GPIO_CAM_TORCH_EN, 0);
				break;
			case YUV_FlashTorch:
                pr_info("debug:Torch flash being on\n");
				g_flashAuto = MT_FALSE; 
                g_flashAuto = MT_FALSE; 	
				g_isVideoFlash = MT_TRUE;

			    gpio_direction_output(TEGRA_GPIO_CAM_TORCH_EN, 1);
	            err = lm3642_flash(1);
				break;

		    case YUV_FlashRedEye:
				pr_info("flash redeye \n");
				break;
			default:
				pr_info("flash being 4\n");
				break;
		}

        if(arg == YUV_FlashAuto)
			g_flashAuto = MT_TRUE;

        if(arg == YUV_FlashOn)
        {
			pr_info("flash mode is coming in flash_on");
			g_flashAuto = MT_FALSE;	
			g_isFlash = MT_TRUE;
        }

        pr_info("Set flash mode as %lu reg value %x\n",arg, val);

        return 0;
	}
	break;
	
	case SENSOR_IOCTL_GET_AF_STATUS:
	{
		 int err;
		 u8 val;
		 err = sensor_ov5640_get_af_status(info, &val);
		 if (copy_to_user((void __user *) arg,
		 &val, sizeof(val))) {
		 return -EFAULT;
		 }

		 if (err)
		 return err;
		 break;
		
	}
	break;
	
	case SENSOR_IOCTL_SET_AF_MODE:
	{
		printk("sensor ioctl set af mode. \n");
		return sensor_ov5640_set_af_mode(info, arg);
	}
	break;
	case SENSOR_IOCTL_CAPTURE_CMD:
	{
		err = 0;
		  pr_info("send capture command finish arg 3 huangyongheng********* \n");		
#if YUV_SENSOR_STROBE
        if(arg == 1)
        {
		    sensor_ov5640_write_table(info->i2c_client, mode_capture_cmd);

        }
        if(arg == 2)
        {
			printk("send capture command finish arg 2 \n");

        }
        if(arg == 3)
        {
			Is_CAPTURE_COMMAND_FLAG =1;
		    printk("send capture command finish arg 3 \n");		
		/*	  
		//huang
		 if(MT_TRUE == g_isFlash){
		 flash_on_off = 1;  
	//	  gpio_direction_output(HS_GPIO_CAM_TORCH_EN, 0); //open torch
		 msleep(200);// preflash 200ms
		 gpio_direction_output(TEGRA_GPIO_CAM_FLASH_EN, 1); //open flash	
		 err = lm3642_flash(2);
		 schedule_delayed_work(&ov5640_delayed_work, msecs_to_jiffies(CAM_FLASH_T1));
//huang
    }
		 */
        }
#endif
        return err;
	}
	break;
    case SENSOR_IOCTL_SET_COLOR_EFFECT:
    {
	 
        if (copy_from_user(&coloreffect,
		   (const void __user *)arg,
		   sizeof(coloreffect))) {
	         return -EFAULT;
        }

        switch(coloreffect)
        {
            case YUV_ColorEffect_None:
                reg_table = ColorEffect_None;
                break;
            case YUV_ColorEffect_Mono:
                reg_table = ColorEffect_Mono;
                break;
            case YUV_ColorEffect_Sepia:
                reg_table = ColorEffect_Sepia;
                break;
            case YUV_ColorEffect_Negative:
                reg_table = ColorEffect_Negative;
                break;
            case YUV_ColorEffect_Solarize:
                reg_table = ColorEffect_Solarize;
                break;
            case YUV_ColorEffect_Posterize:
                reg_table = ColorEffect_Posterize;
                break;
            default:
				reg_table = ColorEffect_None;
                break;
        }

		err = sensor_ov5640_write_table(info->i2c_client, reg_table);

        return err;
    }
	break;
    case SENSOR_IOCTL_SET_WHITE_BALANCE:
    {
		
        u8 whitebalance;

		
		if (copy_from_user(&scenemode,
			  (const void __user *)arg,
			  sizeof(scenemode))) {
			return -EFAULT;
		}


		reg_table = Whitebalance_Auto;

		if (scenemode != YUV_SceneMode_Sunset)
		{
			if (copy_from_user(&whitebalance,
				   (const void __user *)arg,
				   sizeof(whitebalance))) {
			   return -EFAULT;
			}
            switch(whitebalance)
			{
                case YUV_Whitebalance_Auto:				
					reg_table = Whitebalance_Auto;
					break;
                case YUV_Whitebalance_Incandescent:					
					reg_table = Whitebalance_Incandescent;	 
					break;
                case YUV_Whitebalance_Daylight:
					reg_table = Whitebalance_Daylight;
					break;
                case YUV_Whitebalance_Fluorescent:
					reg_table = Whitebalance_Fluorescent;
					break;
                case YUV_Whitebalance_CloudyDaylight: 
					reg_table = Whitebalance_CloudyDaylight;
					break;
                default:
					reg_table = Whitebalance_Auto;
					break;
			}
		}
		
		err = sensor_ov5640_write_table(info->i2c_client, reg_table);

        return err;
    }
	break;
    case SENSOR_IOCTL_SET_SCENE_MODE:
    {
		if (copy_from_user(&scenemode,
			 (const void __user *)arg,
			 sizeof(scenemode)))
		{
		    return -EFAULT;
		}

        switch(scenemode)
        {
			case YUV_SceneMode_Auto:
				SceneModeValue_ov5640 = SceneMode_Value_Auto;
				reg_table = scene_auto;
				break;
            case YUV_SceneMode_Action:
				SceneModeValue_ov5640 = SceneMode_Value_Action;
				reg_table = scene_action;
				break;
            case YUV_SceneMode_Portrait:
				SceneModeValue_ov5640 = SceneMode_Value_Portrait;
				reg_table = scene_portrait;
				break;
            case YUV_SceneMode_Landscape:
				SceneModeValue_ov5640 = SceneMode_Value_Landscape;
				reg_table = scene_landscape;
				break;
            case YUV_SceneMode_Night:
				SceneModeValue_ov5640 = SceneMode_Value_Night;
				reg_table = scene_night;
				break;
            case YUV_SceneMode_NightPortrait:
				SceneModeValue_ov5640 = SceneMode_Value_NightPortrait;
				reg_table = scene_nightportrait;
				break;
            case YUV_SceneMode_Theatre:
				SceneModeValue_ov5640 = SceneMode_Value_Theatre;
				reg_table = scene_theatre;
				break;
            case YUV_SceneMode_Beach:
				SceneModeValue_ov5640 = SceneMode_Value_Beach;
				reg_table = scene_beach;
				break;
            case YUV_SceneMode_Snow:
				SceneModeValue_ov5640 = SceneMode_Value_Snow;
				reg_table = scene_snow;
				break;
            case YUV_SceneMode_Sunset:
				SceneModeValue_ov5640 = SceneMode_Value_Sunset;
				reg_table = scene_sunset;
				break;
            case YUV_SceneMode_SteadyPhoto:
				SceneModeValue_ov5640 = SceneMode_Value_SteadyPhoto;
				reg_table = scene_steadyphoto;
				break;
            case YUV_SceneMode_Fireworks:
				SceneModeValue_ov5640 = SceneMode_Value_Fireworks;
				reg_table = scene_fireworks;
				break;
            default:
				SceneModeValue_ov5640 = SceneMode_Value_Auto;
				reg_table = scene_auto;
				break;
        }

		err = sensor_ov5640_write_table(info->i2c_client, reg_table);

        return err;
    }
	break;

	
    case SENSOR_IOCTL_SET_EXPOSURE:
    {

		if (copy_from_user(&Exposure_value,
				 (const void __user *)arg,
				 sizeof(Exposure_value)))
		{
			return -EFAULT;
		}

	    switch(Exposure_value)
        {
            case YUV_Exposure_0:
				ExposureValue_ov5640 = Exposure_Value_0;
				reg_table = exp_zero;
				break;
            case YUV_Exposure_1:
				ExposureValue_ov5640 = Exposure_Value_1;
				reg_table = exp_one;
				break;
            case YUV_Exposure_2:
				ExposureValue_ov5640 = Exposure_Value_2;
				reg_table = exp_two;
				break;
            case YUV_Exposure_Negative_1:
				ExposureValue_ov5640 = Exposure_Value_Negative_1;
				reg_table = exp_negative1;
				break;
            case YUV_Exposure_Negative_2:
				ExposureValue_ov5640 = Exposure_Value_Negative_2;
				reg_table = exp_negative2;
				break;
            default:
				reg_table = exp_zero;
                break;
        }

		return sensor_ov5640_write_table(info->i2c_client, reg_table);
    }
	break;

	case SENSOR_IOCTL_SET_ISO:
	{
		if (copy_from_user(&ISO_value,
					 (const void __user *)arg,
					 sizeof(ISO_value)))
		{
			return -EFAULT;
		}

		switch(ISO_value)
        {
            case YUV_ISO_0:               
				reg_table = ISO_auto;
				break;
            case YUV_ISO_1:               
				reg_table = ISO_100;
				break;
            case YUV_ISO_2:               
				reg_table = ISO_200;
				break;
            case YUV_ISO_Negative_1:            
				reg_table = ISO_400;
				break;
            case YUV_ISO_Negative_2:              
				reg_table = ISO_800;
				break;
            default:
				reg_table = ISO_auto;
				break;
        }
		
		return sensor_ov5640_write_table(info->i2c_client, reg_table);
    }
	break;

    case SENSOR_IOCTL_SET_Contrast:
	{
		if (copy_from_user(&Contrast_value,
				   (const void __user *)arg,
				   sizeof(Contrast_value))) {
				   
		  return -EFAULT;
		}

		switch(Contrast_value)
        {
            case YUV_Contrast_0:              
				reg_table = contrast_0;
				break;
            case YUV_Contrast_1:            
				reg_table = contrast_1;
				break;
            case YUV_Contrast_2:            
				reg_table = contrast_2;
				break;
            case YUV_Contrast_Negative_1:         
				reg_table = contrast_negative_1;
				break;
            case YUV_Contrast_Negative_2:    
				reg_table = contrast_negative_2;
				break;
            default:
				reg_table = contrast_0;
                break;
        }

		return sensor_ov5640_write_table(info->i2c_client, reg_table);
    }
	break;

    case SENSOR_IOCTL_SET_Saturation:
	{
		if (copy_from_user(&Saturation_value,
			(const void __user *)arg,
			sizeof(Saturation_value)))
		{
			return -EFAULT;
		}

		switch(Saturation_value)
        {
            case YUV_Saturation_0:
				reg_table = saturation_0;
				break;
            case YUV_Saturation_1:  
				reg_table = saturation_1;
				break;
            case YUV_Saturation_2:
				reg_table = saturation_2;
				break;
            case YUV_Saturation_3:
				reg_table = saturation_3;
				break;
            case YUV_Saturation_4:
				reg_table = saturation_4;
				break;
            default:
				reg_table = saturation_0;
				break;
        }

		return sensor_ov5640_write_table(info->i2c_client, reg_table);
    }
	break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sensor_ov5640_open(struct inode *inode, struct file *file)
{ 
	file->private_data = info;
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();
	g_isVideoFlash = MT_FALSE;

    Is_AF_Need_Init = 1;	
	#if 0
		dump_enable = 1;
	#endif
	return 0;
}

int sensor_ov5640_release(struct inode *inode, struct file *file)
{
	pr_info("yuv ov5640 %s\n",__func__);
	#if 0
		dump_enable = 0;
	#endif
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();
	file->private_data = NULL;
	return 0;
}


static const struct file_operations sensor_fileops = {
	.owner = THIS_MODULE,
	.open = sensor_ov5640_open,
	.unlocked_ioctl = sensor_ov5640_ioctl,
	.release = sensor_ov5640_release,
};

static struct miscdevice sensor_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = SENSOR_NAME,
	.fops = &sensor_fileops,
};

static int sensor_ov5640_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;
	INIT_DELAYED_WORK(&ov5640_delayed_work, sensor_flash_delay_work); 

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
		printk("ov5640_probe failed!\n");
		return -ENODEV;
	}
	
	info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);

	if (!info) {
		pr_err("yuv_sensor : Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&sensor_device);
	if (err) {
		pr_err("yuv_sensor : Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;
	info->i2c_client->addr = (0x78>>1);// two sensors i2c slave address be same

pr_info("the temptest for i2c irq value is %d.\n",client->irq);
	i2c_set_clientdata(client, info);
#if 0
	    sensor_ov5640_debug_init();
        client_reg=client;
#endif
	return 0;
}

static int sensor_ov5640_remove(struct i2c_client *client)
{
	struct sensor_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&sensor_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ SENSOR_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name = SENSOR_NAME,
		.owner = THIS_MODULE,
	},
	.probe = sensor_ov5640_probe,
	.remove = sensor_ov5640_remove,
	.id_table = sensor_id,
};

static int __init sensor_init(void)
{
	return i2c_add_driver(&sensor_i2c_driver);
	
}

static void __exit sensor_exit(void)
{
	i2c_del_driver(&sensor_i2c_driver);
}

module_init(sensor_init);
module_exit(sensor_exit);

