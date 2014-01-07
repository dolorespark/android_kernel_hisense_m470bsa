/* drivers/input/touchscreen/ft5x06_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
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

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/i2c/ft5x06_ts.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
//add
#include <linux/input/mt.h>
#ifdef CONFIG_CHARGER_TPS8003X
#include <linux/tps80031-charger.h>
#endif
#include <mach/hardware.h>
#include "../../../arch/arm/mach-tegra/gpio-names.h"
#include "../../../arch/arm/mach-tegra/board-enterprise.h"

//#define SCREEN_MAX_X  720
//#define SCREEN_MAX_Y  1280
#define FT5x0x_TX_NUM	26
#define FT5x0x_RX_NUM   16

#ifdef CONFIG_BATTERY_BQ27x00
extern bool battery_standalone_fg(void);
static bool already_in_suspend = false;
#if (defined(CONFIG_MACH_M470)||defined(CONFIG_MACH_M470BSD)||defined(CONFIG_MACH_M470BSS))
extern unsigned int his_board_version;
#endif
#endif

#if defined(CONFIG_BATTERY_BQ27x00) && (defined(CONFIG_MACH_M470) || defined(CONFIG_MACH_M470BSD)||defined(CONFIG_MACH_M470BSS))
static bool gen2_i2c_3v3_in_use(void)
{
	bool in_use = false;
	if(his_board_version == M470_REVISION_2A_TS_3V3
		|| his_board_version == M470_REVISION_2BC_TS_3V3) {
		if(battery_standalone_fg())
			in_use = true;
	}
	return in_use;
}
#endif

static struct i2c_client *ft5x06_client;

/* For ignoring the first interrupt after resuming, heqi */
static u8 first_point_after_resume =0;

static unsigned int ft5x06_irq= 0;

/* For anti-interference, heqi */
static u8 ft5x06_has_probe = 0;

#ifdef CFG_SUPPORT_TOUCH_KEY
static int touch_key_sensitivity = 35;   //The region of keypad
static int first_keypad_flag = 0;
static int current_key = 0;

int tsp_keycodes[CFG_NUMOFKEYS] ={
    KEY_MENU,KEY_HOMEPAGE,KEY_BACK,KEY_SEARCH
};

char *tsp_keyname[CFG_NUMOFKEYS] ={
    "Menu",
    "Home",
    "Back",
    "Search"
};

static bool tsp_keystatus[CFG_NUMOFKEYS];
#endif

static int debug_mask;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR);

struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];	/*touch event:
					0 -- down; 1-- contact; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];	/*touch ID */
	u16 pressure;
	u8 touch_point;
};

struct ft5x0x_ts_data {
	unsigned int irq;
	unsigned int x_max;
	unsigned int y_max;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
	struct mutex device_mode_mutex; 
	struct ts_event event;
	struct early_suspend early_suspend;
};

#ifdef CONFIG_CHARGER_TPS8003X
extern enum charging_type tps8003x_charger_type(void);

#endif

static u8 ft5x0x_enter_factory(struct ft5x0x_ts_data *ft5x0x_ts);
static u8 ft5x0x_enter_work(struct ft5x0x_ts_data *ft5x0x_ts);

/*
*ft5x0x_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
int ft5x0x_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}


/*write data by i2c*/
int ft5x0x_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);

	return ret;
}

static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= ft5x06_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= ft5x06_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

    //msleep(1);
	ret = i2c_transfer(ft5x06_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	
	return ret;
}
/***********************************************************************************************
Name	:	 
Input	:	
Output	:	
function	:	
***********************************************************************************************/
static int ft5x0x_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= ft5x06_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

   	//msleep(1);
	ret = i2c_transfer(ft5x06_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static int ft5x0x_write_reg(u8 addr, u8 para)
{
    u8 buf[3];
    int ret = -1;

    buf[0] = addr;
    buf[1] = para;
    ret = ft5x0x_i2c_txdata(buf, 2);
    if (ret < 0) {
        pr_err("write reg failed! %#x ret: %d", buf[0], ret);
        return -1;
    }
    
    return 0;
}

static int ft5x0x_read_reg(u8 addr, u8 *pdata)
{
	int ret;
	u8 buf[2];
	struct i2c_msg msgs[2];

	buf[0] = addr;    //register address
	
	msgs[0].addr = ft5x06_client->addr;
	msgs[0].flags = 0;    //write mode
	msgs[0].len = 1;
	msgs[0].buf = buf;
	msgs[1].addr = ft5x06_client->addr;
	msgs[1].flags = I2C_M_RD;  //read mode
	msgs[1].len = 1;
	msgs[1].buf = buf;

	ret = i2c_transfer(ft5x06_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	*pdata = buf[0];
	return ret;
  
}

/***********************************************************************************************
Name	:    ft5x0x_read_fw_ver
Input   :    void
Output  :    firmware version   
function:    read TP firmware version
***********************************************************************************************/
static unsigned char ft5x0x_read_fw_ver(void)
{
	unsigned char ver;
	ft5x0x_read_reg(FT5X0X_REG_FIRMID, &ver);
	return(ver);
}


//upgrade related
typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

typedef struct _FTS_CTP_PROJECT_SETTING_T
{
    unsigned char uc_i2C_addr;             //I2C slave address (8 bit address)
    unsigned char uc_io_voltage;           //IO Voltage 0---3.3v;	1----1.8v
    unsigned char uc_panel_factory_id;     //TP panel factory ID
}FTS_CTP_PROJECT_SETTING_T;

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE              0x0

#define I2C_CTPM_ADDRESS       0x70


void delay_qt_ms(unsigned long  w_ms)
{
    unsigned long i;
    unsigned long j;

    for (i = 0; i < w_ms; i++)
    {
        for (j = 0; j < 1000; j++)
        {
            udelay(1);
        }
    }
}

/*
[function]: 
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    
    ret=i2c_master_recv(ft5x06_client, pbt_buf, dw_lenth);

    if(ret<=0)
    {
        printk("[FTS]i2c_read_interface error\n");
        return FTS_FALSE;
    }
  
    return FTS_TRUE;
}

/*
[function]: 
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    ret=i2c_master_send(ft5x06_client, pbt_buf, dw_lenth);
    if(ret<=0)
    {
        printk("[FTS]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

/*
[function]: 
    send a command to ctpm.
[parameters]:
    btcmd[in]        :command code;
    btPara1[in]    :parameter 1;    
    btPara2[in]    :parameter 2;    
    btPara3[in]    :parameter 3;    
    num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
    FTS_BYTE write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
}

/*
[function]: 
    write data to ctpm , the destination address is 0.
[parameters]:
    pbt_buf[in]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
    
    return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

/*
[function]: 
    read out data from ctpm,the destination address is 0.
[parameters]:
    pbt_buf[out]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
    return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}


/*
[function]: 
    burn the FW to ctpm.
[parameters]:(ref. SPEC)
    pbt_buf[in]    :point to Head+FW ;
    dw_lenth[in]:the length of the FW + 6(the Head length);    
    bt_ecc[in]    :the ECC of the FW
[return]:
    ERR_OK        :no error;
    ERR_MODE    :fail to switch to UPDATE mode;
    ERR_READID    :read id fail;
    ERR_ERASE    :erase chip fail;
    ERR_STATUS    :status error;
    ERR_ECC        :ecc error.
*/

#define    FTS_PACKET_LENGTH        256

static unsigned char CTPM_FW_QT[] = 
{
	#include "ft_app.i"
};


E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    FTS_BYTE reg_val[2] = {0};
    FTS_DWRD i = 0;

    FTS_DWRD  packet_number;
    FTS_DWRD  j;
    FTS_DWRD  temp;
    FTS_DWRD  lenght;
    FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
    FTS_BYTE  auc_i2c_write_buf[10];
    FTS_BYTE bt_ecc;
    int      i_ret;

    /*********Step 1:Reset  CTPM *****/
    /*write 0xaa to register 0xfc*/
    ft5x0x_write_reg(0xfc,0xaa);
    msleep(50);
    /*write 0x55 to register 0xfc*/
    ft5x0x_write_reg(0xfc,0x55);
    printk("[FTS] Step 1: Reset CTPM test\n");
   
    msleep(30);   


    /*********Step 2:Enter upgrade mode *****/
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        i_ret = ft5x0x_i2c_txdata(auc_i2c_write_buf, 2);
        msleep(5);
    }while(i_ret <= 0 && i < 5 );

    /*********Step 3:check READ-ID***********************/        
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
    if (reg_val[0] == 0x79 && (reg_val[1] == 0x7||reg_val[1] == 0x3))
    {
        printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
        return ERR_READID;
        //i_is_new_protocol = 1;
    }

    cmd_write(0xcd,0x0,0x00,0x00,1);
    byte_read(reg_val,1);
    printk("[FTS] bootloader version = 0x%x\n", reg_val[0]);

     /*********Step 4:erase app and panel paramenter area ********************/
    cmd_write(0x61,0x00,0x00,0x00,1);  //erase app area
    msleep(1500); 
    cmd_write(0x63,0x00,0x00,0x00,1);  //erase panel parameter area
    msleep(100);
    printk("[FTS] Step 4: erase. \n");

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    printk("[FTS] Step 5: start upgrade. \n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
	
    for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght>>8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }
        
        byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
        msleep(20);//(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
			printk("[FTS] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],temp+6);    
        msleep(20);
    }

    //send the last six byte
    for (i = 0; i<6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp =1;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i]; 
        bt_ecc ^= packet_buf[6];

        byte_write(&packet_buf[0],7);  
        msleep(20);
    }

    /*********Step 6: read out checksum***********************/
    /*send the opration head*/
    cmd_write(0xcc,0x00,0x00,0x00,1);
    byte_read(reg_val,1);
    printk("[FTS] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
        return ERR_ECC;
    }

    /*********Step 7: reset the new FW***********************/
    cmd_write(0x07,0x00,0x00,0x00,1);

    msleep(300);  //make sure CTP startup normally
    
    return ERR_OK;
}


static int fts_Get_RawData(u16 RawData[][FT5x0x_RX_NUM])
{
	int retval  = 0;
	int i       = 0;
	u8  devmode = 0x00;
	u8  rownum  = 0x00;

	u8 read_buffer[FT5x0x_RX_NUM * 2];
	u8 read_len = 0;
	//u8 write_buffer[2];
	struct ft5x0x_ts_data * ft5x0x_ts =  i2c_get_clientdata(ft5x06_client);

	struct i2c_msg msgs[] = { 
		{
		.addr	= ft5x06_client->addr,
		.flags	= 1,
		.len	= FT5x0x_RX_NUM * 2,
		.buf	= read_buffer,
		}
	};

	if(ft5x0x_enter_factory(ft5x0x_ts)<0)
	{
		pr_err("%s ERROR: could not enter factory mode", __FUNCTION__);
		retval = -1;
		goto error_return;
	}
	//scan
	if(ft5x0x_read_reg(0x00, &devmode)<0)
	{
		pr_err("%s %d ERROR: could not read register 0x00", __FUNCTION__, __LINE__);
		retval = -1;
		goto error_return;
	}
	devmode |= 0x80;
	if(ft5x0x_write_reg(0x00, devmode)<0)
	{
		pr_err("%s %d ERROR: could not read register 0x00", __FUNCTION__, __LINE__);
		retval = -1;
		goto error_return;
	}
	msleep(20);
	if(ft5x0x_read_reg(0x00, &devmode)<0)
	{
		pr_err("%s %d ERROR: could not read register 0x00", __FUNCTION__, __LINE__);
		retval = -1;
		goto error_return;
	}
	if(0x00 != (devmode&0x80))
	{
		pr_err("%s %d ERROR: could not scan", __FUNCTION__, __LINE__);
		retval = -1;
		goto error_return;
	}
	pr_info("Read rawdata .......\n");
	for(rownum=0; rownum<FT5x0x_TX_NUM; rownum++)
	{
		memset(read_buffer, 0x00, (FT5x0x_RX_NUM * 2));

		if(ft5x0x_write_reg(0x01, rownum)<0)
		{
			pr_err("%s ERROR:could not write rownum", __FUNCTION__);
			retval = -1;
			goto error_return;
		}
		msleep(1);
		read_len = FT5x0x_RX_NUM * 2;
		if(ft5x0x_write_reg(0x10, read_len)<0)
		{
			pr_err("%s ERROR:could not write rownum", __FUNCTION__);
			retval = -1;
			goto error_return;
		}

		retval = i2c_transfer(ft5x06_client->adapter, msgs, 1);
		if (retval < 0) 
		{
			pr_err("%s ERROR:Could not read row %u raw data", __FUNCTION__, rownum);
			retval = -1;
			goto error_return;
		}
		for(i=0; i<FT5x0x_RX_NUM; i++)
		{
			RawData[rownum][i] = (read_buffer[i<<1]<<8) + read_buffer[(i<<1)+1];
		}
	}
error_return:
	if(ft5x0x_enter_work(ft5x0x_ts)<0)
	{
		pr_err("%s ERROR:could not enter work mode ", __FUNCTION__);
		retval = -1;
	}
	return retval;
}

int fts_ctpm_auto_clb(void)
{
    unsigned char uc_temp;
    unsigned char i ;

    printk("[FTS] start auto CLB.\n");
    //msleep(200);
    ft5x0x_write_reg(0, 0x40);  
    msleep(50);   //make sure already enter factory mode
    ft5x0x_write_reg(2, 0x4);  //write command to start calibration
    msleep(300);
    for(i=0;i<100;i++)
    {
        ft5x0x_read_reg(0,&uc_temp);
        if ( ((uc_temp&0x70)>>4) == 0x0)  //return to normal mode, calibration finish
        {
            break;
        }
        msleep(20);
        printk("[FTS] waiting calibration %d\n",i);
        
    }
    printk("[FTS] calibration OK.\n");
    
    msleep(300);
    ft5x0x_write_reg(0, 0x40);  //goto factory mode
    msleep(100);   //make sure already enter factory mode
    ft5x0x_write_reg(2, 0x5);  //store CLB result
    msleep(300);
    ft5x0x_write_reg(0, 0x0); //return to normal mode 
    msleep(300);
    printk("[FTS] store CLB result OK.\n");
    return 0;
}

int fts_ctpm_fw_upgrade_with_i_file(void)
{
	FTS_BYTE*     pbt_buf = FTS_NULL;
	int i_ret;
	int fw_size = 0;

	//=========FW upgrade========================*/
	
	fw_size = sizeof(CTPM_FW_QT);
	pbt_buf = CTPM_FW_QT;
	
	/*call the upgrade function*/
	i_ret =  fts_ctpm_fw_upgrade(pbt_buf,fw_size);
	if (i_ret != 0)
	{
		printk("[FTS] upgrade failed i_ret = %d.\n", i_ret);
		//error handling ...
		//TBD
	}
	else
	{
		printk("[FTS] upgrade successfully.\n");
		fts_ctpm_auto_clb();  //start auto CLB
	}

	return i_ret;
}

unsigned char fts_ctpm_get_i_file_ver(void)
{
    unsigned int ui_sz;
	FTS_BYTE*     pbt_buf = FTS_NULL;

	ui_sz = sizeof(CTPM_FW_QT);
	pbt_buf = CTPM_FW_QT;
	
    if (ui_sz > 2)
    {
        return pbt_buf[ui_sz - 2];
    }
    else
    {
        //TBD, error handling?
        return 0xff; //default value
    }
}

#ifdef CFG_SUPPORT_AUTO_UPG


int fts_ctpm_auto_upg(void)
{
    unsigned char uc_host_fm_ver;
    unsigned char uc_tp_fm_ver;
    int           i_ret;

    uc_tp_fm_ver = ft5x0x_read_fw_ver();
    uc_host_fm_ver = fts_ctpm_get_i_file_ver();
    if ( uc_tp_fm_ver == 0xa6  ||   //the firmware in touch panel maybe corrupted
         uc_tp_fm_ver < uc_host_fm_ver //the firmware in host flash is new, need upgrade
        )
    {
        //msleep(100);
        printk("[FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",
            uc_tp_fm_ver, uc_host_fm_ver);
        i_ret = fts_ctpm_fw_upgrade_with_i_file();    
        if (i_ret == 0)
        {
            msleep(300);
            uc_host_fm_ver = fts_ctpm_get_i_file_ver();
            printk("[FTS] upgrade to new version 0x%x\n", uc_host_fm_ver);
        }
        else
        {
            printk("[FTS] upgrade failed ret=%d.\n", i_ret);
        }
    }

    return 0;
}

#endif
void ft5x0x_anti_interference_open(void)
{
     u8 devmode = 0x1;
  	 //gpio_direction_output(TEGRA_GPIO_TP_VDD_EN, 1);
  	 //mdelay(150);
#if defined(CONFIG_BATTERY_BQ27x00) && (defined(CONFIG_MACH_M470) || defined(CONFIG_MACH_M470BSD)||defined(CONFIG_MACH_M470BSS))
	if(gen2_i2c_3v3_in_use()) {
		if((!already_in_suspend) && ft5x06_has_probe){
			ft5x0x_write_reg(FT5X0X_REG_INTERFERENCE, devmode);
			printk("[FTS]: start anti-interference.\n");
		}
	}
	else {
		if((gpio_get_value(TEGRA_GPIO_TP_VDD_EN)) && ft5x06_has_probe){
			ft5x0x_write_reg(FT5X0X_REG_INTERFERENCE, devmode);
			printk("[FTS]: start anti-interference.\n");
		}
	}
#else
  	 if((gpio_get_value(TEGRA_GPIO_TP_VDD_EN)) && ft5x06_has_probe){
	     ft5x0x_write_reg(FT5X0X_REG_INTERFERENCE, devmode);
	     printk("[FTS]: start anti-interference.\n");
  	 }
#endif
}

EXPORT_SYMBOL_GPL(ft5x0x_anti_interference_open);

void ft5x0x_anti_interference_close(void)
{
	u8 devmode = 0x0;
	//gpio_direction_output(TEGRA_GPIO_TP_VDD_EN, 1);
  	//mdelay(150);
#if defined(CONFIG_BATTERY_BQ27x00) && (defined(CONFIG_MACH_M470) || defined(CONFIG_MACH_M470BSD)||defined(CONFIG_MACH_M470BSS))
	if(gen2_i2c_3v3_in_use()) {
		if((!already_in_suspend) && ft5x06_has_probe){
			ft5x0x_write_reg(FT5X0X_REG_INTERFERENCE, devmode);
			printk("[FTS]: stop anti-interference.\n");
		}
	}
	else {
		if((gpio_get_value(TEGRA_GPIO_TP_VDD_EN)) && ft5x06_has_probe){
			ft5x0x_write_reg(FT5X0X_REG_INTERFERENCE, devmode);
			printk("[FTS]: stop anti-interference.\n");
		}
	}
#else
  	if((gpio_get_value(TEGRA_GPIO_TP_VDD_EN)) && ft5x06_has_probe){
		ft5x0x_write_reg(FT5X0X_REG_INTERFERENCE, devmode);
		printk("[FTS]: stop anti-interference.\n");
  	}
#endif
}

EXPORT_SYMBOL_GPL(ft5x0x_anti_interference_close);

/*Read touch point information when the interrupt  is asserted.*/
static int ft5x0x_read_Touchdata(void)
{
    struct ft5x0x_ts_data *data = i2c_get_clientdata(ft5x06_client);
	struct ts_event *event = &data->event;
	u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FT_MAX_ID;

	ret = ft5x0x_i2c_Read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}
	memset(event, 0, sizeof(struct ts_event));

	event->touch_point = 0;
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			break;
		else
			event->touch_point++;
		event->au16_x[i] =
		    (s16) (buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i];
		event->au16_y[i] =
		    (s16) (buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i];
		event->au8_touch_event[i] =
		    buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		event->au8_finger_id[i] =
		    (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
	}

	event->pressure = FT_PRESS;

	return 0;
}


#ifdef CFG_SUPPORT_TOUCH_KEY
int ft5x0x_touch_key_process(struct input_dev *dev, int x, int y, int touch_event)
{
    int i;
    int key_id;
    
    if (x < 223&&x > 153)
    {
        key_id = 1;
    }
    else if ( x < 123&&x > 53)
    {
        key_id = 0;
    }
    
    else if ( x< 333&&x > 263)
    {
        key_id = 2;
    }  
    else if (x <433&&x > 363)
    {
        key_id = 3;
    }
    else
    {
        key_id = 0xf;
    }
    
    for(i = 0; i <CFG_NUMOFKEYS; i++ )
    {	
    	if(touch_event == 1)
		{	
			input_mt_report_slot_state(dev, MT_TOOL_FINGER,false);
			if(tsp_keystatus[i] )
        	{	
            	input_report_key(dev, tsp_keycodes[i], 0);
				
				if(debug_mask)
            		printk("[FTS] %s key is release. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);
            	tsp_keystatus[i] = KEY_RELEASE;
        	}
		}
        else if( touch_event == 0)
        {	
        	
            if( key_id == i )                               // detect
            {	
                input_report_key(dev, tsp_keycodes[i], 1);
				
				if(debug_mask)
                	printk( "[FTS] %s key is pressed. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);
                tsp_keystatus[i] = KEY_PRESS;
            }
        }
    }
    return 0;
    
}    
#endif


/*
*report the point information
*/
static void ft5x0x_report_value(void)
{   
	struct ft5x0x_ts_data *data = i2c_get_clientdata(ft5x06_client);
	struct ts_event *event = &data->event;
	int i;
	int uppoint = 0;

	for (i = 0; i < event->touch_point; i++)
	{	
		input_mt_slot(data->input_dev, event->au8_finger_id[i]);
		if (event->au16_x[i] <= data->x_max && event->au16_y[i] <= data->y_max)
		{	
			if(debug_mask)
				printk("x=%d,y=%d\n",event->au16_x[i],event->au16_y[i]);

			if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2)
			{
				if (event->au16_y[i] <= data->y_max)
				{
					input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,true);
					input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,event->pressure);
					input_report_abs(data->input_dev, ABS_MT_POSITION_X,event->au16_x[i]);
					input_report_abs(data->input_dev, ABS_MT_POSITION_Y,event->au16_y[i]);	
				}
			}
			else
			{
				uppoint++;
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,false);
			}

		}
		
#ifdef CFG_SUPPORT_TOUCH_KEY	
		else 
			
			if(event->au16_y[i] > data->y_max+28)
			{
				ft5x0x_touch_key_process(data->input_dev, event->au16_x[i], event->au16_y[i], event->au8_touch_event[i]);
			}	

#endif
	}
		
	if(event->touch_point == uppoint){
		/* Slot release */
		for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++)
		{
			input_mt_slot(data->input_dev, i);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,false);
		}

#ifdef CFG_SUPPORT_TOUCH_KEY	
		/* Key release */
		for (i = 0; i < CFG_NUMOFKEYS; i++)
		{
			if(tsp_keystatus[i] )
			{	
				input_report_key(data->input_dev, tsp_keycodes[i], 0);

				if(debug_mask)
					printk("[FTS] %s key is release. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);
				
				tsp_keystatus[i] = KEY_RELEASE;
			}
		}

		input_report_key(data->input_dev, BTN_TOUCH, 0);
#endif		
	}
#ifdef CFG_SUPPORT_TOUCH_KEY	
	else
		input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
#endif

	input_sync(data->input_dev);
	
	
}

static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{
	int ret = -1;
	
	ret = ft5x0x_read_Touchdata();	
	if (ret == 0) 
	{	
		ft5x0x_report_value();
	}

	enable_irq(ft5x06_irq);
}

/*The ft5x0x device will signal the host about TRIGGER_FALLING.
*Processed when the interrupt is asserted.
*/
static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{

	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;

	//escape useless interrupt from resume
	if(1 == first_point_after_resume)
	{
		printk(KERN_ERR "[FTS]: drop the first INT when resuming\n");
		first_point_after_resume = 0;
		return IRQ_HANDLED;
	}

	disable_irq_nosync(ft5x06_irq);
	if(debug_mask)
		printk("[FTS] -------------------------INT______-\n");
	
	if (!work_pending(&ft5x0x_ts->pen_event_work)) {
		queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND

static void ft5x0x_ts_suspend(struct early_suspend *handler)
{	
	struct ft5x0x_ts_data *ts;
	printk("==ft5x0x_ts_suspend=\n");
#ifdef CONFIG_BATTERY_BQ27x00
	already_in_suspend = true;
#endif
	
	ts =  container_of(handler, struct ft5x0x_ts_data, early_suspend);
	
	tegra_gpio_enable(TEGRA_GPIO_BL_PWM);
	gpio_direction_output(TEGRA_GPIO_BL_PWM, 0);
	msleep(20);
	gpio_direction_output(TEGRA_GPIO_EN_VDD_BL, 0);
	
	free_irq(ft5x06_irq, ts);
	cancel_work_sync(&ts->pen_event_work);
	flush_workqueue(ts->ts_workqueue);
#if defined(CONFIG_BATTERY_BQ27x00) && (defined(CONFIG_MACH_M470) || defined(CONFIG_MACH_M470BSD)||defined(CONFIG_MACH_M470BSS))
	if(gen2_i2c_3v3_in_use())
		printk("****KEEP GEN2_I2C PULL UP VOLTAGE\n");
	else
	gpio_direction_output(TEGRA_GPIO_TP_VDD_EN, 0);//For 2.0A
#else
	gpio_direction_output(TEGRA_GPIO_TP_VDD_EN, 0);//For V1,V2
#endif
}

static void ft5x0x_ts_resume(struct early_suspend *handler)
{
	int err = 0;
	int i = 0;
	struct ft5x0x_ts_data *ts;
	printk("==ft5x0x_ts_resume=\n");
	
	first_point_after_resume = 1;
	
	ts =  container_of(handler, struct ft5x0x_ts_data, early_suspend);

	/* Power supply */
	gpio_direction_output(TEGRA_GPIO_TS_WAKE, 1);
	msleep(20);
	gpio_direction_output(TEGRA_GPIO_TP_VDD_EN, 1);
	msleep(150);
	#ifdef CONFIG_CHARGER_TPS8003X
	if(tps8003x_charger_type() == AC_CHARGER || tps8003x_charger_type() == USB_CHARGER)
	{	
		ft5x0x_write_reg(FT5X0X_REG_INTERFERENCE, 0x1);
		printk("[FTS]: start anti-interference.\n");
	}
	#endif
	/* Interrupt apply */
	err = request_irq(ft5x06_irq, ft5x0x_ts_interrupt, IRQF_TRIGGER_FALLING, ft5x06_client->dev.driver->name, ts);
	if (err < 0) {
		printk("ft5x0x_resume: request irq failed\n");
	}
	disable_irq(ft5x06_irq);

	/* Slot release */
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++)
	{
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER,false);
	}
	input_sync(ts->input_dev);
	msleep(80);
	gpio_direction_output(TEGRA_GPIO_EN_VDD_BL, 1);
	msleep(20);
	tegra_gpio_disable(TEGRA_GPIO_BL_PWM);
	msleep(50);

	/* Interrupt enable */
	enable_irq(ft5x06_irq);

#ifdef CONFIG_BATTERY_BQ27x00
	already_in_suspend = false;
#endif

}
#endif

/* sysfs */
static u8 ft5x0x_enter_factory(struct ft5x0x_ts_data *ft5x0x_ts)
{
	u8 regval;
	
	flush_workqueue(ft5x0x_ts->ts_workqueue);
	disable_irq_nosync(ft5x06_irq);
	ft5x0x_write_reg(0x00, 0x40);  //goto factory mode
	msleep(100);   //make sure already enter factory mode
	if(ft5x0x_read_reg(0x00, &regval)<0)
		pr_err("%s ERROR: could not read register\n", __FUNCTION__);
	else
	{
		if((regval & 0x70) != 0x40)
		{
			pr_err("%s() - ERROR: The Touch Panel was not put in Factory Mode. The Device Mode register contains 0x%02X\n", __FUNCTION__, regval);
			return -1;
		}
	}
	return 0;
}
static u8 ft5x0x_enter_work(struct ft5x0x_ts_data *ft5x0x_ts)
{
	u8 regval;
	
	ft5x0x_write_reg(0x00, 0x00); //return to normal mode 
	msleep(100);
	
	if(ft5x0x_read_reg(0x00, &regval)<0)
		pr_err("%s ERROR: could not read register\n", __FUNCTION__);
	else
	{
		if((regval & 0x70) != 0x00)
		{
			pr_err("%s() - ERROR: The Touch Panel was not put in Work Mode. The Device Mode register contains 0x%02X\n", __FUNCTION__, regval);
			enable_irq(ft5x06_irq);
			return -1;
		}
	}
	enable_irq(ft5x06_irq);
	return 0;
}


static int ft5x0x_GetFirmwareSize(char * firmware_name)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic; 
	off_t fsize = 0; 
	char filepath[128];memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "/sdcard/%s", firmware_name);
	pr_info("filepath=%s\n", filepath);
	if(NULL == pfile){
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if(IS_ERR(pfile)){
		pr_err("error occured while opening file %s.\n", filepath);
		return -1;
	}
	inode=pfile->f_dentry->d_inode; 
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size; 
	filp_close(pfile, NULL);
	return fsize;
}
static int ft5x0x_ReadFirmware(char * firmware_name, unsigned char * firmware_buf)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic; 
	off_t fsize; 
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "/sdcard/%s", firmware_name);
	pr_info("filepath=%s\n", filepath);
	if(NULL == pfile){
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if(IS_ERR(pfile)){
		pr_err("error occured while opening file %s.\n", filepath);
		return -1;
	}
	inode=pfile->f_dentry->d_inode; 
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size; 
	//char * buf;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	vfs_read(pfile, firmware_buf, fsize, &pos);

	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}

int fts_ctpm_fw_upgrade_with_app_file(char * firmware_name)
{
  	FTS_BYTE*     pbt_buf = 0x0;
   	int i_ret; u8 fwver;
   	int fwsize = ft5x0x_GetFirmwareSize(firmware_name);
	
   	if(fwsize <= 0)
   	{
   		pr_err("%s ERROR:Get firmware size failed\n", __FUNCTION__);
		return -1;
   	}
    //=========FW upgrade========================*/
  	 pbt_buf = (unsigned char *) kmalloc(fwsize+1,GFP_ATOMIC);
	if(ft5x0x_ReadFirmware(firmware_name, pbt_buf))
	{
       	pr_err("%s() - ERROR: request_firmware failed\n", __FUNCTION__);
        	kfree(pbt_buf);
		return -1;
	}
   	/*call the upgrade function*/
   	i_ret =  fts_ctpm_fw_upgrade(pbt_buf, fwsize);
   	if (i_ret != 0)
   	{
       	pr_err("%s() - ERROR:[FTS] upgrade failed i_ret = %d.\n",__FUNCTION__,  i_ret);
       //error handling ...
       //TBD
   	}
  	 else
   	{
       	pr_info("[FTS] upgrade successfully.\n");
		if(ft5x0x_read_reg(FT5x0x_REG_FW_VER, &fwver)>=0)
			pr_info("the new fw ver is 0x%02x\n", fwver);
  		fts_ctpm_auto_clb();  //start auto CLB
   	}
	kfree(pbt_buf);
   	return i_ret;
}

static ssize_t ft5x0x_tpfwiver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	unsigned char uc_host_fm_ver;
	
	uc_host_fm_ver = fts_ctpm_get_i_file_ver();

	num_read_chars = snprintf(buf, PAGE_SIZE, "%02X\n", uc_host_fm_ver);

	return num_read_chars;
}

static ssize_t ft5x0x_tpfwiver_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

static ssize_t ft5x0x_tpfwver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ft5x0x_ts_data *data = NULL;
	struct i2c_client *client;
	ssize_t num_read_chars = 0;
	u8	   fwver = 0;

	client = container_of(dev, struct i2c_client, dev);
	data = (struct ft5x0x_ts_data *) i2c_get_clientdata( client );
	mutex_lock(&data->device_mode_mutex);
	if(ft5x0x_read_reg(FT5x0x_REG_FW_VER, &fwver) < 0)
		num_read_chars = snprintf(buf, PAGE_SIZE, "get tp fw version fail!\n");
	else
		num_read_chars = snprintf(buf, PAGE_SIZE, "%02X\n", fwver);

	mutex_unlock(&data->device_mode_mutex);
	return num_read_chars;
}

static ssize_t ft5x0x_tpfwver_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

static ssize_t ft5x0x_tprwreg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	struct ft5x0x_ts_data *data = NULL;
	struct i2c_client *client;
	ssize_t num_read_chars = 0;
	u8	   fwreg = 0;

	client = container_of(dev, struct i2c_client, dev);
	data = (struct ft5x0x_ts_data *) i2c_get_clientdata( client );
	mutex_lock(&data->device_mode_mutex);
	if(ft5x0x_read_reg(FT5X0X_REG_INTERFERENCE, &fwreg) < 0)
		num_read_chars = snprintf(buf, PAGE_SIZE, "get tp fw reg fail!\n");
	else
		num_read_chars = snprintf(buf, PAGE_SIZE, "%02X\n", fwreg);

	mutex_unlock(&data->device_mode_mutex);
	return num_read_chars;
}

static ssize_t ft5x0x_tprwreg_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct ft5x0x_ts_data *data = NULL;
	struct i2c_client *client;
	ssize_t num_read_chars = 0;
	int retval;
	u16 wmreg=0;
	u8 regaddr=0xff,regvalue=0xff;
	u8 valbuf[5];

	memset(valbuf, 0, sizeof(valbuf));
	client = container_of(dev, struct i2c_client, dev);
	data = (struct ft5x0x_ts_data *) i2c_get_clientdata( client );

	mutex_lock(&data->device_mode_mutex);
	num_read_chars = count - 1;

	if(num_read_chars!=2)
	{
		if(num_read_chars!=4)
		{
			pr_info("please input 2 or 4 character\n");
			goto error_return;
		}
	}
	
	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, &wmreg);

	if (0 != retval)
	{
    	pr_err("%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n", __FUNCTION__, buf);
    	goto error_return;
	}

	if(2 == num_read_chars)
	{
		//read register
		regaddr = wmreg;
		if(ft5x0x_read_reg(regaddr, &regvalue) < 0)
			pr_err("Could not read the register(0x%02x)\n", regaddr);
		else
			pr_info("the register(0x%02x) is 0x%02x\n", regaddr, regvalue);
	}
	else
	{
		regaddr = wmreg>>8;
		regvalue = wmreg;
		if(ft5x0x_write_reg(regaddr, regvalue)<0)
			pr_err("Could not write the register(0x%02x)\n", regaddr);
		else
			pr_err("Write 0x%02x into register(0x%02x) successful\n", regvalue, regaddr);
	}
error_return:
	mutex_unlock(&data->device_mode_mutex);

	return count;
}


static ssize_t ft5x0x_fwupdate_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
    return -EPERM;
}
//upgrade from *.i
static ssize_t ft5x0x_fwupdate_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct ft5x0x_ts_data *data = NULL;
	struct i2c_client *client;
	u8 uc_host_fm_ver;
	int i_ret;

	client = container_of(dev, struct i2c_client, dev);
	data = (struct ft5x0x_ts_data *) i2c_get_clientdata( client );
	mutex_lock(&data->device_mode_mutex);

	disable_irq(ft5x06_irq);
	i_ret = fts_ctpm_fw_upgrade_with_i_file();    
    if (i_ret == 0)
    {
        msleep(300);
        uc_host_fm_ver = fts_ctpm_get_i_file_ver();
        pr_info("%s [FTS] upgrade to new version 0x%x\n", __FUNCTION__, uc_host_fm_ver);
    }
    else
    {
        pr_err("%s ERROR:[FTS] upgrade failed ret=%d.\n", __FUNCTION__, i_ret);
    }
	enable_irq(ft5x06_irq);
	
	mutex_unlock(&data->device_mode_mutex);

	return count;
}

static ssize_t ft5x0x_fwupgradeapp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
    return -EPERM;
}
//upgrade from app.bin
static ssize_t ft5x0x_fwupgradeapp_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct ft5x0x_ts_data *data = NULL;
	struct i2c_client *client;
	char fwname[128];

	client = container_of(dev, struct i2c_client, dev);
	data = (struct ft5x0x_ts_data *) i2c_get_clientdata( client );
	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count-1] = '\0';

	mutex_lock(&data->device_mode_mutex);
	disable_irq(ft5x06_irq);
	
	fts_ctpm_fw_upgrade_with_app_file(fwname);
	
	enable_irq(ft5x06_irq);


	mutex_unlock(&data->device_mode_mutex);

	return count;
}

static ssize_t ft5x0x_rawdata_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x0x_ts_data *data = NULL;
	struct i2c_client *client;
	ssize_t num_read_chars = 0;
	int i=0, j=0;
	u16	RawData[FT5x0x_TX_NUM][FT5x0x_RX_NUM];

	client = container_of(dev, struct i2c_client, dev);
	data = (struct ft5x0x_ts_data *) i2c_get_clientdata( client );
	mutex_lock(&data->device_mode_mutex);
	
	if(fts_Get_RawData(RawData)<0)
		sprintf(buf, "%s", "could not get rawdata\n");
	else
	{
		for(i=0; i<FT5x0x_TX_NUM; i++)
		{
			for(j=0; j<FT5x0x_RX_NUM; j++)
			{
				num_read_chars += sprintf(&(buf[num_read_chars]), "%u ", RawData[i][j]);
			}
			buf[num_read_chars-1] = '\n';
		}
	}

	mutex_unlock(&data->device_mode_mutex);	
	return num_read_chars;
}

//upgrade from app.bin
static ssize_t ft5x0x_rawdata_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{

	return -EPERM;
}


/* sysfs */
static DEVICE_ATTR(ftstpfwiver, S_IRUGO|S_IWUSR, ft5x0x_tpfwiver_show, ft5x0x_tpfwiver_store);
static DEVICE_ATTR(ftstpfwver, S_IRUGO|S_IWUSR, ft5x0x_tpfwver_show, ft5x0x_tpfwver_store);
//upgrade from *.i
static DEVICE_ATTR(ftsfwupdate, S_IRUGO|S_IWUSR, ft5x0x_fwupdate_show, ft5x0x_fwupdate_store);
static DEVICE_ATTR(ftstprwreg, S_IRUGO|S_IWUSR, ft5x0x_tprwreg_show, ft5x0x_tprwreg_store);
//upgrade from app.bin 
static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO|S_IWUSR, ft5x0x_fwupgradeapp_show, ft5x0x_fwupgradeapp_store);
static DEVICE_ATTR(ftsrawdatashow, S_IRUGO|S_IWUSR, ft5x0x_rawdata_show, ft5x0x_rawdata_store);


static struct attribute *ft5x0x_attributes[] = {
	&dev_attr_ftstpfwiver.attr,
	&dev_attr_ftstpfwver.attr,
	&dev_attr_ftsfwupdate.attr,
	&dev_attr_ftstprwreg.attr,
	&dev_attr_ftsfwupgradeapp.attr,
	&dev_attr_ftsrawdatashow.attr,
	NULL
};

static struct attribute_group ft5x0x_attribute_group = {
	.attrs = ft5x0x_attributes,
	.name = "ft5x06",
};

static int ft5x0x_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ft5x06_platform_data *pdata =
	    (struct ft5x06_platform_data *)client->dev.platform_data;
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char uc_reg_value;
	
#ifdef CFG_SUPPORT_TOUCH_KEY
	int i;
#endif

	printk("=====[FTS] ft5x0x_ts_probe=====\n");

	ft5x06_irq = client->irq;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft5x0x_ts = kzalloc(sizeof(struct ft5x0x_ts_data), GFP_KERNEL);

	if (!ft5x0x_ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
	
	ft5x06_client = client;

	/*get some register information */
	if(ft5x0x_read_reg(FT5x0x_REG_FW_VER, &uc_reg_value) < 0)
		goto exit_read_device_failed;
	printk("[FTS] Firmware version = 0x%x\n", uc_reg_value);
	ft5x0x_read_reg(FT5x0x_REG_POINT_RATE, &uc_reg_value);
	printk("[FTS] report rate is %dHz.\n", uc_reg_value * 10);
	ft5x0x_read_reg(FT5X0X_REG_THGROUP, &uc_reg_value);
	printk("[FTS] touch threshold is %d.\n", uc_reg_value * 4);

	i2c_set_clientdata(client, ft5x0x_ts);
	ft5x0x_ts->irq = client->irq;
	ft5x0x_ts->client = client;
	ft5x0x_ts->x_max = pdata->x_max - 1;
	ft5x0x_ts->y_max = pdata->y_max - 1;

	mutex_init(&ft5x0x_ts->device_mode_mutex);
	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);
	ft5x0x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	err = request_irq(client->irq, ft5x0x_ts_interrupt, IRQF_TRIGGER_FALLING, client->dev.driver->name, ft5x0x_ts);
	//err = request_threaded_irq(client->irq, NULL, ft5x0x_ts_interrupt,IRQF_TRIGGER_FALLING, client->dev.driver->name, ft5x0x_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	
	disable_irq(client->irq);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ft5x0x_ts->input_dev = input_dev;
	
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
    

	input_mt_init_slots(input_dev, CFG_MAX_TOUCH_POINTS);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ft5x0x_ts->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ft5x0x_ts->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, CFG_MAX_TOUCH_POINTS, 0, 0);

	input_dev->name = "ft5x06_input";
	
#ifdef CFG_SUPPORT_TOUCH_KEY
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	
	input_dev->keycode = tsp_keycodes;
	for(i = 0; i < CFG_NUMOFKEYS; i++) {
		input_set_capability(input_dev, EV_KEY, ((int*)input_dev->keycode)[i]);
		tsp_keystatus[i] = KEY_RELEASE;
	}
#endif
	
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
			"ft5x0x_ts_probe: failed to register input device: %s\n",
			dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
	/*make sure CTP already finish startup process */
	msleep(150);
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	printk("==register_early_suspend =\n");
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ft5x0x_ts->early_suspend.resume	= ft5x0x_ts_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif

#ifdef CFG_SUPPORT_AUTO_UPG
	fts_ctpm_auto_upg();
#endif


	enable_irq(client->irq);
	
	
	//create sysfs
	err = sysfs_create_group(&client->dev.kobj, &ft5x0x_attribute_group);
	if (0 != err)
	{
		dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed: %d\n", __FUNCTION__, err);
		sysfs_remove_group(&client->dev.kobj, &ft5x0x_attribute_group);
	}
	else
	{
		printk("ft5x0x:%s() - sysfs_create_group() succeeded.\n", __FUNCTION__);
	}
#ifdef CONFIG_CHARGER_TPS8003X

	/* If USB/AC is plugged in, start anti-interference, heqi */
	if(tps8003x_charger_type() == AC_CHARGER || tps8003x_charger_type() == USB_CHARGER)
	{	
		ft5x0x_write_reg(FT5X0X_REG_INTERFERENCE, 0x1);
		printk("[FTS]: start anti-interference.\n");
	}
#endif
	ft5x06_has_probe = 1;
	
	printk("[FTS] ====probe over ====\n");
	return 0;
	
exit_input_register_device_failed:
	input_free_device(input_dev);
	
exit_input_dev_alloc_failed:
	free_irq(client->irq, ft5x0x_ts);

exit_irq_request_failed:
exit_create_singlethread:
	printk("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
exit_read_device_failed:
	kfree(ft5x0x_ts);

exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int __devexit ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	ft5x0x_ts = i2c_get_clientdata(client);
	ft5x06_has_probe = 0;
	input_unregister_device(ft5x0x_ts->input_dev);
	free_irq(client->irq, ft5x0x_ts);
	kfree(ft5x0x_ts);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{FT5X0X_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static struct i2c_driver ft5x0x_ts_driver = {
	.probe = ft5x0x_ts_probe,
	.remove = __devexit_p(ft5x0x_ts_remove),
	.id_table = ft5x0x_ts_id,
	//.suspend = ft5x0x_ts_suspend,
	//.resume = ft5x0x_ts_resume,
	.driver = {
	   .name = FT5X0X_NAME,
	   .owner = THIS_MODULE,
   },
};

static int __init ft5x0x_ts_init(void)
{
	int ret;
	ret = i2c_add_driver(&ft5x0x_ts_driver);
	debug_mask = 0;
	if (ret) {
		printk(KERN_WARNING "Adding ft5x0x driver failed "
		       "(errno = %d)\n", ret);
	} else {
		pr_info("Successfully added driver %s\n",
			ft5x0x_ts_driver.driver.name);
	}
	return ret;
}

static void __exit ft5x0x_ts_exit(void)
{
	i2c_del_driver(&ft5x0x_ts_driver);
}

module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<luowj>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
