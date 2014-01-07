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
#include "yuv_ov2655_init_tab.h"


#include <linux/gpio.h>  
#include "../../../../arch/arm/mach-tegra/board-enterprise.h"
#include "../../../../arch/arm/mach-tegra/gpio-names.h"


#define Default_SVGA_Line_Width       1940
#define Default_UXGA_Line_Width       1940
#define Default_Reg0x3028             0x07
#define Default_Reg0x3029             0x93
#define Default_Reg0x302a             0x04
#define Default_Reg0x302b             0xd4

#define Default_UXGA_maximum_shutter  1236
#define Capture_dummy_pixel           00
#define Capture_dummy_line            00
#define Capture_PCLK_Frequency        18  //Unit is MHz 36
#define Preview_PCLK_Frequency        36  //Unit is MHz


#define SENSOR_NAME	"ov2655"
//#define SIZEOF_I2C_TRANSBUF (32)
//static u8 i2c_trans_buf[SIZEOF_I2C_TRANSBUF];

static u8  ColorEffectValue_ov2655=0, WhiteBalanceValue_ov2655=0 , SceneModeValue_ov2655=0 ;
static u8 ExposureValue_ov2655;
static u8 coloreffect;
static u8 whitebalance;
static u8 scenemode;
u8 Sensor_ov2655_Init_Flag =0;



static u8 sensor_ov2655_read_reg_8bit(struct i2c_client *client, u16 addr, u8 *val)
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

static int sensor_ov2655_write_reg_8bit(struct i2c_client *client, u16 addr, u8 val) 
{
	int err;
	struct i2c_msg msg;
	unsigned char data[3];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);;
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
		pr_err("ov2655: i2c transfer failed, retrying %x %x\n",
		       addr, val);
		
	} while (retry <= 3);

	return err;
}



static int sensor_ov2655_read_reg(struct i2c_client *client, u16 addr, u16 *val)
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

	// high byte goes out first 
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

static int sensor_ov2655_write_reg8(struct i2c_client *client, u16 addr, u8 val)
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
		pr_err("yuv_sensor : i2c transfer failed, retrying %x %x\n",
		       addr, val);
		
	} while (retry <= SENSOR_MAX_RETRIES);

	return err;
}

static int sensor_ov2655_poll(struct i2c_client *client, u16 addr, u16 value,
			u16 mask)
{
	u16 data;
	int try, err;

	for (try = 0; try < SENSOR_POLL_RETRIES; try++) {
		err = sensor_ov2655_read_reg(client, addr, &data);
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
static int sensor_ov2655_poll_bit_set(struct i2c_client *client, u16 addr, u16 bit)
{
	return sensor_ov2655_poll(client, addr, bit, bit);
}

static int sensor_ov2655_write_reg16(struct i2c_client *client, u16 addr, u16 val)
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
		msleep(3);
	} while (retry <= SENSOR_MAX_RETRIES);

	return err;
}

static int sensor_ov2655_write_table(struct i2c_client *client,
			      const struct sensor_reg table[])
{
	const struct sensor_reg *next;
	int err;
        next = table ;
  
	pr_info("yuv %s \n",__func__);
	for (next = table; next->op != SENSOR_TABLE_END; next++) {

		switch (next->op) {
		case WRITE_REG_DATA8:
		{
			err = sensor_ov2655_write_reg8(client, next->addr,
				next->val);
			if (err)
				return err;
			break;
		}
		case WRITE_REG_DATA16:
		{
			err = sensor_ov2655_write_reg16(client, next->addr,
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
			err = sensor_ov2655_poll_bit_set(client, next->addr,
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

/*
static int ov2655_write_bulk_reg(struct i2c_client *client, u8 *data, int len)
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

	dev_err(&client->dev, "ov2655: i2c transfer failed at %x\n",
		(int)data[0] << 8 | data[1]);

	return err;
}

static int sensor_ov2655_write_table_8bit(struct i2c_client *client,
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



		err = ov2655_write_bulk_reg(client,
			i2c_trans_buf, buf_filled);
		if (err)
			return err;

		buf_filled = 0;
	}
	return 0;
}
*/
/*
static int ov2655_CalGainExposure(struct sensor_info *info)
{
	u8 val;
	int err;
    unsigned char Preview_Reg0x3000, Preview_Reg0x3002, Preview_Reg0x3003, Preview_Reg0x3013; 
	unsigned char Preview_Reg0x3028, Preview_Reg0x3029, Preview_Reg0x302d, Preview_Reg0x302e;
	unsigned char Reg0x3013, Reg0x3000, Reg0x3002, Reg0x3003, Reg0x302d, Reg0x302e; 
	unsigned char Reg0x3028, Reg0x3029;
	unsigned short Extra_lines, Preview_dummy_pixel, Capture_Banding_Filter, Capture_Gain16, Preview_Gain16;
    unsigned int Preview_Exposure, Shutter, Capture_Max_Gain16, Preview_Line_Width; 
	unsigned int Capture_Line_Width, Capture_Exposure, Capture_Maximum_Shutter, Gain_Exposure,Gain;

	//Stop AE/AG
	Reg0x3013= sensor_ov2655_read_reg_8bit(info->i2c_client,0x3013,&val); 
	Preview_Reg0x3013 = Reg0x3013;
	Reg0x3013 = Reg0x3013 & 0xfa;
	err =sensor_ov2655_write_reg_8bit(info->i2c_client,0x3013, Reg0x3013);

  
	//Read back preview shutter
	Reg0x3002= sensor_ov2655_read_reg_8bit(info->i2c_client,0x3002,&val); 
	Reg0x3003= sensor_ov2655_read_reg_8bit(info->i2c_client,0x3003,&val); 

	Preview_Reg0x3002 = Reg0x3002;
	Preview_Reg0x3003 = Reg0x3003;

	Shutter=Reg0x3002;
	Shutter=Shutter<<8;
	Shutter += Reg0x3003;
	
	//Read back extra line
	Reg0x302d= sensor_ov2655_read_reg_8bit(info->i2c_client,0x302d,&val); 
	Reg0x302e= sensor_ov2655_read_reg_8bit(info->i2c_client,0x302e,&val); 

	Preview_Reg0x302d = Reg0x302d;
	Preview_Reg0x302e = Reg0x302e;  

	Extra_lines =Reg0x302d;
	Extra_lines =Extra_lines<<8;
	Extra_lines +=Reg0x302e;

	Preview_Exposure = Shutter + Extra_lines;

	//Read Back Gain for preview
	Reg0x3000= sensor_ov2655_read_reg_8bit(info->i2c_client,0x3000,&val); 

	Preview_Reg0x3000 = Reg0x3000;  
	Preview_Gain16 = (((Reg0x3000 & 0xf0)>>4) + 1) * (16 + (Reg0x3000 & 0x0f));

	//Read back dummy pixels
	Reg0x3028= sensor_ov2655_read_reg_8bit(info->i2c_client,0x3028,&val); 
	Reg0x3029= sensor_ov2655_read_reg_8bit(info->i2c_client,0x3029,&val); 

	Preview_Reg0x3028 = Reg0x3028;
	Preview_Reg0x3029 = Reg0x3029;

	Preview_dummy_pixel = (Reg0x3028 -Default_Reg0x3028)&0xf0 ;
	Preview_dummy_pixel = Preview_dummy_pixel <<8;
	Preview_dummy_pixel += (Reg0x3029-Default_Reg0x3029);

  
	//Maximum gain limitation for capture, Capture_max_gain16 = capture_maximum_gain * 16
	Capture_Max_Gain16 = 1024;//for maxmium 8x gain

	Preview_Line_Width = Default_SVGA_Line_Width + Preview_dummy_pixel;
	Capture_Line_Width = Default_UXGA_Line_Width + Capture_dummy_pixel;
	Capture_Maximum_Shutter = Default_UXGA_maximum_shutter + Capture_dummy_line;
	Capture_Exposure = Preview_Exposure*Capture_PCLK_Frequency/Preview_PCLK_Frequency*Preview_Line_Width/Capture_Line_Width;

	Capture_Banding_Filter = Capture_PCLK_Frequency*1000000/ 100/(2*Capture_Line_Width);

	//printk("$$$$$$$Capture_Exposure is:%d$$$$$$$$\n",Capture_Exposure); 

	//	Gain_Exposure = Preview_Gain16 * Capture_Exposure*95/100; //yangbin modify for test 20111129
	Gain_Exposure = Preview_Gain16 * Capture_Exposure; //huang modify 12-12

    if (Gain_Exposure < Capture_Banding_Filter * 16) {//1200
        // Exposure < 1/100
        Capture_Exposure = Gain_Exposure /16;
        Capture_Gain16 = (Gain_Exposure*2 + 1)/Capture_Exposure/2;
    }
    else {
        if (Gain_Exposure > Capture_Maximum_Shutter * 16) {//16720
            // Exposure > Capture_Maximum_Shutter
            Capture_Exposure = Capture_Maximum_Shutter;
            Capture_Gain16 = (Gain_Exposure*2 + 1)/Capture_Maximum_Shutter/2;
            if (Capture_Gain16 > Capture_Max_Gain16) {
                // gain reach maximum, insert extra line
                Capture_Exposure = Gain_Exposure*11/10/Capture_Max_Gain16;
                // Exposure = n/100
              // Capture_Exposure = Gain_Exposure/Capture_Banding_Filter;
		 Capture_Exposure = Gain_Exposure/16/Capture_Banding_Filter;//add 12-13
                Capture_Exposure = Capture_Exposure * Capture_Banding_Filter;
                Capture_Gain16 = (Gain_Exposure *2+1)/ Capture_Exposure/2;
            }
            else{
            //      Capture_Exposure = Capture_Exposure/16/Capture_Banding_Filter;
		  Capture_Exposure = Capture_Exposure/Capture_Banding_Filter;//add 12-13
                  Capture_Exposure = Capture_Exposure * Capture_Banding_Filter;
                  Capture_Gain16 = (Gain_Exposure*2 +1) / Capture_Exposure/2;        
            }
        }
        else {
            // 1/100 < Exposure < Capture_Maximum_Shutter, Exposure = n/100
        Capture_Exposure = Gain_Exposure/16/Capture_Banding_Filter;
        Capture_Exposure = Capture_Exposure * Capture_Banding_Filter;
        Capture_Gain16 = (Gain_Exposure *2+1)/ Capture_Exposure/2;
        }
    }

    //Write Exposure
    if (Capture_Exposure > Capture_Maximum_Shutter) {
        Shutter = Capture_Maximum_Shutter;
        Extra_lines = Capture_Exposure - Capture_Maximum_Shutter;
    }
    else {
        Shutter = Capture_Exposure;
        Extra_lines = 0;
    }

	err = sensor_ov2655_write_table(info->i2c_client, mode_1600x1200);
	
	Reg0x3003 = Shutter & 0x00ff;
	Reg0x3002 = (Shutter >>8) & 0x00ff;
	err =sensor_ov2655_write_reg_8bit(info->i2c_client,0x3003, Reg0x3003);
	err =sensor_ov2655_write_reg_8bit(info->i2c_client,0x3002, Reg0x3002);


    // Write extra line
	Reg0x302e= Extra_lines & 0x00ff;
	Reg0x302d = Extra_lines >> 8;

	err =sensor_ov2655_write_reg_8bit(info->i2c_client,0x302d, Reg0x302d);
	err =sensor_ov2655_write_reg_8bit(info->i2c_client,0x302e, Reg0x302e);
	
    // Write Gain
    Gain = 0;
    if (Capture_Gain16 > 31) {
        Capture_Gain16 = Capture_Gain16 /2;
        Gain = 0x10;
    }
    if (Capture_Gain16 > 31) {
        Capture_Gain16 = Capture_Gain16 /2;
        Gain = Gain | 0x20;
    }
    if (Capture_Gain16 > 31) {
        Capture_Gain16 = Capture_Gain16 /2;
        Gain = Gain | 0x40;
    }
    if (Capture_Gain16 > 31) {
        Capture_Gain16 = Capture_Gain16 /2;
        Gain = Gain | 0x80;
    }
    if (Capture_Gain16 > 16) {
        Gain = Gain | (Capture_Gain16 -16);
    }
	err =sensor_ov2655_write_reg_8bit(info->i2c_client,0x3000, Gain);

	return 0;
}
*/



static int OV2655_get_sysclk(struct sensor_info *info)
{
	int XVCLK = 2400; // real clock/10000
	// calculate sysclk
	int temp1, temp2;
	u8 val;
	int Indiv2x, Bit8Div, FreqDiv2x, PllDiv, SensorDiv, ScaleDiv, DvpDiv, ClkDiv, VCO, sysclk;
	int Indiv2x_map[] = {2, 3, 4, 6, 4, 6, 8, 12};
	int Bit8Div_map[] = {1, 1, 4, 5};
	int FreqDiv2x_map[] = {2, 3, 4, 6};
	int DvpDiv_map[] = {1, 2, 8, 16};
	temp1 = sensor_ov2655_read_reg_8bit(info->i2c_client, 0x300e, &val);
	// bit[5:0] PllDiv
	PllDiv = 64 - (temp1 & 0x3f);
	temp1 = sensor_ov2655_read_reg_8bit(info->i2c_client, 0x300f, &val);
	// bit[2:0] Indiv
	temp2 = temp1 & 0x07;
	Indiv2x = Indiv2x_map[temp2];
	// bit[5:4] Bit8Div
	temp2 = (temp1 >> 4) & 0x03;
	Bit8Div = Bit8Div_map[temp2];
	// bit[7:6] FreqDiv
	temp2 = temp1 >> 6;
	FreqDiv2x = FreqDiv2x_map[temp2];

	temp1 = sensor_ov2655_read_reg_8bit(info->i2c_client, 0x3010, &val);
	//bit[3:0] ScaleDiv
	temp2 = temp1 & 0x0f;
	if(temp2==0) {
	ScaleDiv = 1;
	}
	else {
	ScaleDiv = temp2 * 2;
	}
	// bit[4] SensorDiv
	if(temp1 & 0x10) {
	SensorDiv = 2;
	}
	else {
	SensorDiv = 1;
	}
	// bit[5] LaneDiv
	// bit[7:6] DvpDiv
	temp2 = temp1 >> 6;
	DvpDiv = DvpDiv_map[temp2];
	temp1 = sensor_ov2655_read_reg_8bit(info->i2c_client, 0x3011, &val);
	// bit[5:0] ClkDiv
	temp2 = temp1 & 0x3f;
	ClkDiv = temp2 + 1;
	VCO = XVCLK * Bit8Div * FreqDiv2x * PllDiv / Indiv2x ;
	sysclk = VCO / Bit8Div / SensorDiv / ClkDiv / 4;
	return sysclk;
}
static int ov2655_CalGainExposure(struct sensor_info *info)
{

	int temp;
	u8 val;
	int err;
	int gain =0 ;
	unsigned char Reg0x3013, Reg0x3000, Reg0x3002, Reg0x3003,Reg0x3014; 
	unsigned char Reg0x300b, Reg0x3028, Reg0x3029, Reg0x302a, Reg0x302b, Reg0x302d, Reg0x302e, Reg0x302c;
	unsigned int preview_shutter, preview_gain16, preview_binning, preview_sysclk, preview_HTS;
	unsigned int capture_bandingfilter, capture_max_band, capture_sysclk;
	unsigned int capture_gain16, capture_shutter;
	unsigned int HTS, VTS, extra_HTS, extra_VTS, capture_VTS, capture_HTS;
	unsigned long int capture_gain16_shutter;

	
	// read preview PCLK
	preview_sysclk = OV2655_get_sysclk(info) ;
	
	//read preview HTS, extra_HTS,
	Reg0x3028 = sensor_ov2655_read_reg_8bit(info->i2c_client, 0x3028, &val); 
	Reg0x3029 = sensor_ov2655_read_reg_8bit(info->i2c_client, 0x3029, &val); 
	Reg0x302c = sensor_ov2655_read_reg_8bit(info->i2c_client, 0x302c, &val); 
	HTS = (Reg0x3028<<8)+Reg0x3029;
	extra_HTS = Reg0x302c;
	preview_HTS = HTS+extra_HTS;

	//Read back preview shutter
	Reg0x3002= sensor_ov2655_read_reg_8bit(info->i2c_client,0x3002,&val); 
	Reg0x3003= sensor_ov2655_read_reg_8bit(info->i2c_client,0x3003,&val); 
	Reg0x302d = sensor_ov2655_read_reg_8bit(info->i2c_client,0x302d,&val);
	Reg0x302e = sensor_ov2655_read_reg_8bit(info->i2c_client,0x302e,&val);
	
	preview_shutter=((Reg0x3002<<8)+Reg0x3003) + ((Reg0x302d<<8)+Reg0x302e);

	//Read Back Gain for preview
	Reg0x3000= sensor_ov2655_read_reg_8bit(info->i2c_client,0x3000,&val); 
	preview_gain16 = (((Reg0x3000 & 0xf0)>>4) + 1) * (16 + (Reg0x3000 & 0x0f));
	Reg0x300b = sensor_ov2655_read_reg_8bit(info->i2c_client, 0x300b, &val);
	if (Reg0x300b == 0x52) 
		preview_binning = 2;
	else
		preview_binning = 1;
	
	//Stop AE/AG
	Reg0x3013= sensor_ov2655_read_reg_8bit(info->i2c_client,0x3013,&val); 
	
	Reg0x3014= sensor_ov2655_read_reg_8bit(info->i2c_client,0x3014,&val); 
	Reg0x3013 = Reg0x3013&0xfa;
	err =sensor_ov2655_write_reg_8bit(info->i2c_client,0x3013, Reg0x3013);
	
	//write capture setting
	err = sensor_ov2655_write_table(info->i2c_client, mode_1600x1200);

	//read capture VTS=shutter, extra_VTS=extra_lines
	Reg0x302a = sensor_ov2655_read_reg_8bit(info->i2c_client, 0x302a, &val);
	Reg0x302b = sensor_ov2655_read_reg_8bit(info->i2c_client, 0x302b, &val);
	Reg0x302d = sensor_ov2655_read_reg_8bit(info->i2c_client, 0x302d, &val);
	Reg0x302e = sensor_ov2655_read_reg_8bit(info->i2c_client, 0x302e, &val);
	VTS = (Reg0x302a<<8)+Reg0x302b;
	extra_VTS = (Reg0x302d<<8)+Reg0x302e;
	
	//read capture HTS, extra_HTS,
	Reg0x3028 = sensor_ov2655_read_reg_8bit(info->i2c_client, 0x3028, &val); 
	Reg0x3029 = sensor_ov2655_read_reg_8bit(info->i2c_client, 0x3029, &val); 
	Reg0x302c = sensor_ov2655_read_reg_8bit(info->i2c_client, 0x302c, &val); 
	HTS = (Reg0x3028<<8)+Reg0x3029;
	extra_HTS = Reg0x302c;

	//capture_VTS=Preview_Exposure, 
	capture_VTS = VTS+extra_VTS;
	capture_HTS = HTS+extra_HTS;
	capture_sysclk = OV2655_get_sysclk(info);
	capture_bandingfilter = capture_sysclk * 100 / capture_HTS *100/120;
	capture_max_band = (capture_VTS )/capture_bandingfilter;
	// calculate capture shutter/gain16
	capture_gain16_shutter = preview_gain16 * preview_shutter * capture_sysclk/preview_sysclk * preview_HTS/capture_HTS*preview_binning ;

	// gain to shutter
	if(capture_gain16_shutter < (capture_bandingfilter * 16)) {
	// shutter < 1/100
	capture_shutter = capture_gain16_shutter/16;
	if(capture_shutter <1)
		capture_shutter = 1;
		capture_gain16 = capture_gain16_shutter/capture_shutter;
	}
	else {
		if(capture_gain16_shutter > (capture_bandingfilter*capture_max_band*16)) {
			// exposure reach max
			capture_shutter = capture_bandingfilter*capture_max_band;
			capture_gain16 = capture_gain16_shutter / capture_shutter;
		}
		else {
			// 1/100 < capture_shutter =< max, capture_shutter = n/100
			capture_shutter =  (capture_gain16_shutter/16/capture_bandingfilter) * capture_bandingfilter;
			capture_gain16 = capture_gain16_shutter / capture_shutter;
		}
	}
	
	// write gain, 16 = 1x
	gain = 0;
	if( capture_gain16 > 31)
	{
		capture_gain16 = capture_gain16 >> 1;
		gain  = gain | 0x10;
	}
	if( capture_gain16 > 31)
	{
		capture_gain16 = capture_gain16 >> 1;
		gain  = gain | 0x20;
	}
	if( capture_gain16 > 31)
	{
		capture_gain16 = capture_gain16 >> 1;
		gain  = gain | 0x40;
	}
	if( capture_gain16 > 31)
	{
		capture_gain16 = capture_gain16 >> 1;
		gain  = gain | 0x80;
	}
   	if (capture_gain16 > 16) {
        	gain = gain | (capture_gain16 -16);
    	}
	
	sensor_ov2655_write_reg_8bit(info->i2c_client, 0x3000, gain);
	// write capture shutter
	if (capture_shutter > (capture_VTS )) {
		capture_VTS = capture_shutter;	
		// write VTS to registers
		temp = capture_VTS & 0xff;
		sensor_ov2655_write_reg_8bit(info->i2c_client, 0x302b, temp);
		temp = capture_VTS>>8;
		sensor_ov2655_write_reg_8bit(info->i2c_client, 0x302a, temp);
	}

	// write shutter, in number of line period
	capture_shutter = capture_shutter & 0xffff;
	temp = capture_shutter & 0x00ff;
	sensor_ov2655_write_reg_8bit(info->i2c_client, 0x3003, temp);
	temp = capture_shutter>>8;
	sensor_ov2655_write_reg_8bit(info->i2c_client, 0x3002, temp);
	
	
	return 0;
}

static int sensor_ov2655_set_mode(struct sensor_info *info, struct sensor_mode *mode)
{
	int sensor_table;
	int err;
  
	pr_info("%s: xres %u yres %u\n",__func__, mode->xres, mode->yres);	
	
	if (mode->xres == 1600 && mode->yres == 1200)
		sensor_table = SENSOR_MODE_1600x1200;
	else if (mode->xres == 1280 && mode->yres == 720)
		sensor_table = SENSOR_MODE_1280x720;
	else if (mode->xres == 800 && mode->yres == 600)
		sensor_table = SENSOR_MODE_800x600;
	else if (mode->xres == 640 && mode->yres == 480)
		sensor_table = SENSOR_MODE_640x480;
	else if (mode->xres == 176 && mode->yres == 144)
		sensor_table = SENSOR_MODE_176x144;
	else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

    if (sensor_table == SENSOR_MODE_1600x1200)
    {
			pr_info("initial cts tab sensor_mode_1600x1200 %d\n",sensor_table);
			ov2655_CalGainExposure(info);
	msleep(540); //modify ... give up over 2 frame

	}


	else
    {
		pr_info("initial tab %d\n",sensor_table);
		if(Sensor_ov2655_Init_Flag ){
			
		Sensor_ov2655_Init_Flag =0;
		err = sensor_ov2655_write_table(info->i2c_client, ov2655_init);
	}

        err = sensor_ov2655_write_table(info->i2c_client, mode_table[sensor_table]);
//	err = sensor_ov2655_write_table(info->i2c_client, ov2655_reg_mipi_control_tab);

	switch(WhiteBalanceValue_ov2655)
	{
		case Whitebalance_Value_Auto:
			err = sensor_ov2655_write_table(info->i2c_client, Whitebalance_Auto);
	             break;
		case Whitebalance_Value_Incandescent:
			err = sensor_ov2655_write_table(info->i2c_client, Whitebalance_Incandescent);
	             break;
		case Whitebalance_Value_Daylight:
			err = sensor_ov2655_write_table(info->i2c_client, Whitebalance_Daylight);
	             break;
		case Whitebalance_Value_Fluorescent:
			err = sensor_ov2655_write_table(info->i2c_client, Whitebalance_Fluorescent);
	             break;
		case Whitebalance_Value_CloudyDaylight:
		    err = sensor_ov2655_write_table(info->i2c_client, Whitebalance_CloudyDaylight);
			     break;
	    default:
	             break;
	}
	err = sensor_ov2655_write_table(info->i2c_client, ov2655_reg_mipi_control_tab);
		info->mode = sensor_table;
	msleep(700); 

    if (err)
        return err;
    }																				

    
	return 0;
}


static long sensor_ov2655_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	struct sensor_info *info = file->private_data;
    int err=0;
	int Saturation_value;
	int Contrast_value;
	int Exposure_value;

	pr_info("%s: cmd %u \n",__func__, cmd);
	switch (cmd) {
	case SENSOR_IOCTL_SET_MODE:
	{
		struct sensor_mode mode;
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct sensor_mode))) {
			return -EFAULT;
		}

		return sensor_ov2655_set_mode(info, &mode);
	}
	break;
	case SENSOR_IOCTL_GET_STATUS:
	{
		return 0;
	}
	break;
	case SENSOR_IOCTL_GET_BRIGHTNESS:
	{
        return 0;
	}
	break;
	case SENSOR_IOCTL_SET_FLASH_MODE:
	{
        return 0;
	}
	break;
	case SENSOR_IOCTL_GET_AF_STATUS:
	{
        return 0;  
	}
	break;
	case SENSOR_IOCTL_SET_AF_MODE:
	{
        return 0;
			
	}
	break;
	case SENSOR_IOCTL_CAPTURE_CMD:
	{
#if YUV_SENSOR_STROBE
		if(arg == 1)
		{
		   pr_info("send capture command finish arg == 1\n");
		}
		if(arg == 2)
		{
		   pr_info("send capture command finish arg == 2\n");

		}
		if(arg == 3)
		{
		   pr_info("send capture command finish arg == 3\n");    
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
				ColorEffectValue_ov2655=ColorEffect_Value_None;
		     err = sensor_ov2655_write_table(info->i2c_client, ColorEffect_None);
		         break;
		    case YUV_ColorEffect_Mono:
				ColorEffectValue_ov2655=ColorEffect_Value_Mono;
		     err = sensor_ov2655_write_table(info->i2c_client, ColorEffect_Mono);
		         break;
		    case YUV_ColorEffect_Sepia:
				ColorEffectValue_ov2655=ColorEffect_Value_Sepia;
		     err = sensor_ov2655_write_table(info->i2c_client, ColorEffect_Sepia);
		         break;
		    case YUV_ColorEffect_Negative:
				ColorEffectValue_ov2655=ColorEffect_Value_Negative;
		     err = sensor_ov2655_write_table(info->i2c_client, ColorEffect_Negative);
		         break;
		    case YUV_ColorEffect_Solarize:
				ColorEffectValue_ov2655=ColorEffect_Value_Solarize;
		     err = sensor_ov2655_write_table(info->i2c_client, ColorEffect_Solarize);
		         break;
		    case YUV_ColorEffect_Posterize:
				ColorEffectValue_ov2655=ColorEffect_Value_Posterize;
		     err = sensor_ov2655_write_table(info->i2c_client, ColorEffect_Posterize);
		         break;
		    default:
				ColorEffectValue_ov2655=ColorEffect_Value_None;
		         break;
		}

	   if (err)
	   return err;

	   return 0;
	}
	break;
	case SENSOR_IOCTL_SET_WHITE_BALANCE:
	{
		if (copy_from_user(&whitebalance,
			   (const void __user *)arg,
			   sizeof(whitebalance))) {
		return -EFAULT;
		}
		pr_info("yuv whitebalance %d\n",whitebalance);

	    switch(whitebalance)
	    {
	        case YUV_Whitebalance_Auto:
				WhiteBalanceValue_ov2655=Whitebalance_Value_Auto;
	         err = sensor_ov2655_write_table(info->i2c_client, Whitebalance_Auto);
	             break;
	        case YUV_Whitebalance_Incandescent:
				WhiteBalanceValue_ov2655=Whitebalance_Value_Incandescent;
	         err = sensor_ov2655_write_table(info->i2c_client, Whitebalance_Incandescent);
	             break;
	        case YUV_Whitebalance_Daylight:
				WhiteBalanceValue_ov2655=Whitebalance_Value_Daylight;
	         err = sensor_ov2655_write_table(info->i2c_client, Whitebalance_Daylight);
	             break;
	        case YUV_Whitebalance_Fluorescent:
				WhiteBalanceValue_ov2655=Whitebalance_Value_Fluorescent;
	         err = sensor_ov2655_write_table(info->i2c_client, Whitebalance_Fluorescent);
	             break;

			case YUV_Whitebalance_CloudyDaylight: 
				WhiteBalanceValue_ov2655=Whitebalance_Value_CloudyDaylight;				
			 err = sensor_ov2655_write_table(info->i2c_client, Whitebalance_CloudyDaylight);
				 
                 break;
	        default:
				WhiteBalanceValue_ov2655=Whitebalance_Value_Auto;
	             break;
	    }

	if (err)
	return err;

	    return 0;
	}
	break;
	case SENSOR_IOCTL_SET_SCENE_MODE:
	{
		if (copy_from_user(&scenemode,
		 (const void __user *)arg,
		 sizeof(scenemode))) {
		return -EFAULT;
		}

		switch(scenemode)
		{
			case YUV_SceneMode_Auto:
				SceneModeValue_ov2655 =SceneMode_Value_Auto;
			 err = sensor_ov2655_write_table(info->i2c_client, scene_auto);
			     break;
			case YUV_SceneMode_Action:
				SceneModeValue_ov2655 =SceneMode_Value_Action;
			 err = sensor_ov2655_write_table(info->i2c_client, scene_action);
			     break;
			case YUV_SceneMode_Portrait:
				SceneModeValue_ov2655 =SceneMode_Value_Portrait;
			 err = sensor_ov2655_write_table(info->i2c_client, scene_portrait);
			     break;
			case YUV_SceneMode_Landscape:
				SceneModeValue_ov2655 =SceneMode_Value_Landscape;
			 err = sensor_ov2655_write_table(info->i2c_client, scene_landscape);
			     break;
			case YUV_SceneMode_Night:
				SceneModeValue_ov2655 =SceneMode_Value_Night;
			 err = sensor_ov2655_write_table(info->i2c_client, scene_night);
			     break;
			case YUV_SceneMode_NightPortrait:
				SceneModeValue_ov2655 =SceneMode_Value_NightPortrait;
			 err = sensor_ov2655_write_table(info->i2c_client, scene_nightportrait);
			     break;
			case YUV_SceneMode_Theatre:
				SceneModeValue_ov2655 =SceneMode_Value_Theatre;
			 err = sensor_ov2655_write_table(info->i2c_client, scene_theatre);
			     break;
			case YUV_SceneMode_Beach:
				SceneModeValue_ov2655 =SceneMode_Value_Beach;
			 err = sensor_ov2655_write_table(info->i2c_client, scene_beach);
			     break;
			case YUV_SceneMode_Snow:
				SceneModeValue_ov2655 =SceneMode_Value_Snow;
			 err = sensor_ov2655_write_table(info->i2c_client, scene_snow);
			     break;
			case YUV_SceneMode_Sunset:
				SceneModeValue_ov2655 =SceneMode_Value_Sunset;
			 err = sensor_ov2655_write_table(info->i2c_client, scene_sunset);
			     break;
			case YUV_SceneMode_SteadyPhoto:
				SceneModeValue_ov2655 =SceneMode_Value_SteadyPhoto;
			 err = sensor_ov2655_write_table(info->i2c_client, scene_steadyphoto);
			     break;
			case YUV_SceneMode_Fireworks:
				SceneModeValue_ov2655 =SceneMode_Value_Fireworks;
			 err = sensor_ov2655_write_table(info->i2c_client, scene_fireworks);
			     break;

			default:
				SceneModeValue_ov2655 =SceneMode_Value_Auto;
			     break;
		}
		return 0;
	}
	break;
	case SENSOR_IOCTL_SET_EXPOSURE:
	{	
		if (copy_from_user(&Exposure_value,
				 (const void __user *)arg,
				 sizeof(Exposure_value))) {
				 
		 return -EFAULT;

		}

	    switch(Exposure_value)
	    {
	        case YUV_Exposure_0:
				ExposureValue_ov2655 =Exposure_Value_0;
				err = sensor_ov2655_write_table(info->i2c_client, exp_zero);
	             break;
	        case YUV_Exposure_1:
				ExposureValue_ov2655 =Exposure_Value_1;
				err = sensor_ov2655_write_table(info->i2c_client, exp_one);
	             break;
	        case YUV_Exposure_2:
				ExposureValue_ov2655 =Exposure_Value_2;
				err = sensor_ov2655_write_table(info->i2c_client, exp_two);
	             break;
	        case YUV_Exposure_Negative_1:
				ExposureValue_ov2655 =Exposure_Value_Negative_1;
				 err = sensor_ov2655_write_table(info->i2c_client, exp_negative1);
	             break;
	        case YUV_Exposure_Negative_2:
				ExposureValue_ov2655 =Exposure_Value_Negative_2;
				err = sensor_ov2655_write_table(info->i2c_client, exp_negative2);			
	             break;
	        default:
	             break;
	        }
		
	}
	break;


	case SENSOR_IOCTL_SET_Contrast:
	{
		if (copy_from_user(&Contrast_value,
				   (const void __user *)arg,
				   sizeof(Contrast_value))) {
				   
		  return -EFAULT;
		}

	}
	break;

	case SENSOR_IOCTL_SET_Saturation:
	{
		if (copy_from_user(&Saturation_value,
			(const void __user *)arg,
			sizeof(Saturation_value))) {

				  return -EFAULT;
		}

	}
	break;

	default:
		return -EINVAL;
	}
	return 0;
}

static int sensor_ov2655_open(struct inode *inode, struct file *file)
{

	file->private_data = info;
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();

	gpio_direction_output(TEGRA_GPIO_CAM_AF_EN, 0);
	Sensor_ov2655_Init_Flag =1;
	return 0;
}

int sensor_ov2655_release(struct inode *inode, struct file *file)
{
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();
	file->private_data = NULL;
	return 0;
}


static const struct file_operations sensor_fileops = {
	.owner = THIS_MODULE,
	.open = sensor_ov2655_open,
	.unlocked_ioctl = sensor_ov2655_ioctl,
	.release = sensor_ov2655_release,
};

static struct miscdevice sensor_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = SENSOR_NAME,
	.fops = &sensor_fileops,
};

static int sensor_ov2655_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;
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

	info->i2c_client->addr = (0x60>>1);

	i2c_set_clientdata(client, info);
    	
	return 0;
}

static int sensor_ov2655_remove(struct i2c_client *client)
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
	.probe = sensor_ov2655_probe,
	.remove = sensor_ov2655_remove,
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


