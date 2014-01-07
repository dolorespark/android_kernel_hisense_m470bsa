/*
 * lm3642.c - lm3642 flash/torch kernel driver
 *
 * Copyright (C) 2013 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 *
 * Author: Nathan Zhang 
 * Create date: 2013 -01 -17
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/switch.h>
#include <linux/sysfs.h>
#include <mach/gpio.h>
#include <linux/device.h>
#include <linux/leds.h>

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <media/lm3642.h>

#define LM3642_LOG 1
#ifdef LM3642_LOG
#define LM3642_DEBUG(fmt,arg...) \
            printk("\e[31;40;1m" "LM3642:" "\t %s():%d\t\t" fmt  "\e[m\n", __func__,__LINE__, ##arg)
#else
#define LM3642_DEBUG(stuff...)  do{} while(0)
#endif


struct flashled_driver_data
{	
    	struct i2c_client *i2c_client;
	struct lm3642_platform_data *pdata;
    int torch_gpio_state;
	int torch_gpio;	
	int flash_gpio_state;  
	int flash_gpio;
};
struct i2c_client *tst_client;

static u8 torch_level_state=0;
static u8 flash_level_state=0;

static struct flashled_driver_data *dev_info=NULL;
#define POWER_ON 			1
#define POWER_OFF 			0
#define LED_ON				1
#define LED_OFF				0

//LM3642 REGS
#define REVISION_AND_FILTER_TIME_REG0 	0x00

//REG01
#define IVFM_MODE_REG1 					0X01
#define UVLO_ENABLE						0x80
#define IVM_D_THRESHOLD_BIT 				0X1C
#define IVM_D_THRESHOLD_SHIFT 			2
#define IVM_D_THRESHOLD_DEFAUL 			(0X06<<IVM_D_THRESHOLD_SHIFT)  //(3.6V)

//REG08    
#define FLASH_FEATURES_REG8 				0X08
#define INDUCTOR_CRRENT_LIMIT 			1<<6//(0=1.6A;1=1.88A)
#define FLASH_TIMER_OUT_BIT 				0X07
#define FLASH_TIMER_OUT_DEFAUL_VALUE 	0X06 //(000=50ms;001=100ms;010=150ms.......)

//REG09   
#define CURRENT_CONTROL_REG9 			0X09
#define TORCH_CURRENT_LEVEL_BIT 			0X70
#define TORCH_CURRENT_LEVEL_SHIFT 		4
#define TORCH_CURRENT_DEAULT_LEVEL  		0 <<TORCH_CURRENT_LEVEL_SHIFT //48.4mA

#define FLASH_CURRENT_LEVEL_BIT 			0X0F
#define FLASH_CURRENT_LEVEL_SHIFT 		0
#define FLASH_CURRENT_DEFUALT_LEVEL 		0xF<<TORCH_CURRENT_LEVEL_SHIFT //1500mA

//REG10   
#define ENABLE_REGA 						0X0A
#define STORBE_ENABLE 					1<<5
#define TORCH_ENABLE 					1<<4
#define MODE_BIT 							0X03 
#define STANDBY_MODE 					0x00
#define INDICATOR_MODE 					0x01
#define TORCH_MODE 						0x02
#define FLASH_MODE 						0x03

//REG11    
#define FLAGS_STATE_REGB 					0X0B
#define IVFM_STATE 						1<<5
#define UVLO_STATE 						1<<4
#define OVP_STATE 						1<<3
#define LED_VOUT_SHORT_STATE 			1<<2
#define THERMAL_SHUTDOWN 				1<<1
#define TIMEOUT							1<<0

#define MAX_TORCH_LEVEL 					8
#define MAX_FLASH_LEVEL 					16
#define MAX_FLASH_TIMER_LEVEL 			7
    

static int lm3642_read(struct i2c_client *client, u8 addr, u8 *value)
{
	struct i2c_msg msg[2];
	unsigned char data[3];

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = data;

	data[0] = (u8)addr;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 2;
	if (i2c_transfer(client->adapter, msg, 2) != 2)
		return -1;
	*value = data[2];
	return 0;
}

static int lm3642_write(struct i2c_client *client, u8 addr, u8 value)
{
	int count;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = addr;
	data[1] = value;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = ARRAY_SIZE(data);
	msg[0].buf = data;
	do {
		count = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (count == ARRAY_SIZE(msg))
			return 0;
		retry++;
		pr_err("lm3642: i2c transfer failed, retrying %x\n", value);
		msleep(3);
	} while (retry <= 3);

	return -EIO;
}

static int lm3642_power_en(struct flashled_driver_data *info,bool power_enable)
{
	u8 value;
	int err=0;

	LM3642_DEBUG("lm3642 power_enable=%d",power_enable);

	if(power_enable==POWER_ON) {
		err= lm3642_read(info->i2c_client,IVFM_MODE_REG1,&value);
		if (err<0)
			goto fail;

		value&=(~IVM_D_THRESHOLD_BIT);
		value&=(~UVLO_ENABLE);
		value|=IVM_D_THRESHOLD_DEFAUL;
		err=lm3642_write(info->i2c_client, IVFM_MODE_REG1, value);
		if (err<0)
			goto fail;
		
		err=lm3642_read(info->i2c_client, FLASH_FEATURES_REG8, &value);
		if (err<0)
			goto fail;
		value&=(~FLASH_TIMER_OUT_BIT);
		value|=(FLASH_TIMER_OUT_DEFAUL_VALUE);
		err=lm3642_write(info->i2c_client,FLASH_FEATURES_REG8, value);
		if (err<0)
			goto fail;

		err=lm3642_read(info->i2c_client,CURRENT_CONTROL_REG9,&value);
		if (err<0)
			goto fail;

		value&=(~(FLASH_CURRENT_LEVEL_BIT|TORCH_CURRENT_LEVEL_BIT));
		value|=FLASH_CURRENT_DEFUALT_LEVEL|TORCH_CURRENT_DEAULT_LEVEL;
		err=lm3642_write(info->i2c_client,CURRENT_CONTROL_REG9,value);
		if (err<0)
			goto fail;

		err=lm3642_read(info->i2c_client, ENABLE_REGA,&value);
		if (err<0)
			goto fail;

		value&=(~MODE_BIT);
		err=lm3642_write(info->i2c_client, ENABLE_REGA,value);  
		if (err<0)
			goto fail;

		err=lm3642_read(info->i2c_client, FLAGS_STATE_REGB, &value);
		if (err<0)
			goto fail;
	} else { 
		msleep(50);
	}
	torch_level_state=0;
	flash_level_state=0;

	return 0;   

fail:
	return -1;   

}

int lm3642_flash(int flash_mode)
{
	    int err=0;
		if(flash_mode ==0){
			err=lm3642_write(tst_client,0x0a,0x00);
			}			
		else if(flash_mode ==1){  //torch mode
			err=lm3642_write(tst_client,0x01,0x00);
		   err=lm3642_write(tst_client,0x09,0x24); //set flash current 560mA, torch:140mA.
			   err=lm3642_write(tst_client,0x0a,0x12);
			}
		else if(flash_mode ==2){  //flash mode
			err=lm3642_write(tst_client,0x01,0x00);
			err=lm3642_write(tst_client,0x09,0x24); //set flash current 560mA, torch: 140mA.
			 err=lm3642_write(tst_client,0x0a,0x13);
			}
		else
			{
				err=lm3642_write(tst_client,0x0a,0x00);
			}

		return 0;
}

static int lm3642_torch_level(struct flashled_driver_data *info,u8 level)
{
	u8 value=0;
	int err=0;

	LM3642_DEBUG("lm3642_torch_level=%d",level);
	if((level>=0)&&(level<=MAX_TORCH_LEVEL)) {
		if(level==0) {
		//   err= lm3642_read(info->i2c_client,ENABLE_REGA,&value);
		//   if (err<0)
		//      return err;   
		value=0;
		value&=(~MODE_BIT);
		err= lm3642_write(info->i2c_client,ENABLE_REGA,value);
		if (err<0)
			return err;   
		} else {

        pr_info("huangyongheng test flash/torch comeinto flag .\n");

		err = lm3642_flash(2);
//torch light		
		err=lm3642_write(info->i2c_client,0x01,0x00);

		err=lm3642_write(info->i2c_client,0x0a,0x02);
		msleep(3000);
		err=lm3642_write(info->i2c_client,0x0a,0x00);

//flash light
		err=lm3642_write(info->i2c_client,0x01,0x00);

		err=lm3642_write(info->i2c_client,0x0a,0x03);
		msleep(3000);
		err=lm3642_write(info->i2c_client,0x0a,0x00);
		
			//level=level-1;
			//    err=lm3642_read(info->i2c_client,CURRENT_CONTROL_REG9,&value);
			// if (err<0)
			//  return err;   
			value=0;
			value&=(~TORCH_CURRENT_LEVEL_BIT);
			value|=((level<<TORCH_CURRENT_LEVEL_SHIFT)&TORCH_CURRENT_LEVEL_BIT);
			err=lm3642_write(info->i2c_client,CURRENT_CONTROL_REG9,value);
			if (err<0)
				return err;   

			// err=lm3642_read(info->i2c_client,ENABLE_REGA,&value);
			// if (err<0)
			//    return err;   
			value=0;
			value&=(~MODE_BIT);
			value|=TORCH_MODE | TORCH_ENABLE;
			err=lm3642_write(info->i2c_client,ENABLE_REGA,value);
			if (err<0)
				return err;   
			
	//		err=lm3642_read(info->i2c_client,ENABLE_REGA,&value);
	//		if (err<0)
	//		    return err;   
	//		LM3642_DEBUG("Read REGA=0x%x",value);
		}
	} else {
		printk("lm3642_torch_level limit for%d\r\n",level);
	}
		return 0;   
}

static int lm3642_flash_level(struct flashled_driver_data *info,u8 level)
{
	u8 value=0;
	int err=0;

	LM3642_DEBUG("lm3642_flash_level=%d",level);
	if((level>=0)&&(level<=MAX_FLASH_LEVEL)) {
		if(level==0) {
			//  err=lm3642_read(info->i2c_client,ENABLE_REGA,&value);
			// if (err<0)
			//      return err;   
			value=0;
			value&=(~MODE_BIT);
			err=lm3642_write(info->i2c_client,ENABLE_REGA,value);
			if (err<0)
				return err;   
		} else {
			//   err=lm3642_read(info->i2c_client,CURRENT_CONTROL_REG9,&value);
			// if (err<0)
			//      return err;   
			value=0;
			value&=(~FLASH_CURRENT_LEVEL_BIT);
			value|=((level<<FLASH_CURRENT_LEVEL_SHIFT)&FLASH_CURRENT_LEVEL_BIT);
			err=lm3642_write(info->i2c_client,CURRENT_CONTROL_REG9,value);
			if (err<0)
				return err;   

			// err=lm3642_read(info->i2c_client,ENABLE_REGA,&value);
			//  if (err<0)
			//    return err;   
			value=0;  
			value&=(~MODE_BIT);
			value|=FLASH_MODE | STORBE_ENABLE;
			err=lm3642_write(info->i2c_client, ENABLE_REGA, value);      
			if (err<0)
				return err;      
			
		//	err=lm3642_read(info->i2c_client, ENABLE_REGA, &value);
		//	if (err<0)
		//		return err;  
		//	LM3642_DEBUG("Read REGA=0x%x",value);
		}
	} else  {
		printk("lm3642_flash_level limit for%d\r\n",level);
	}
	return 0;   
}

static long lm3642_ioctl(struct file *file,unsigned int cmd,unsigned long arg)
{
	int error;
	u8 data;
	u32 level;

	struct flashled_driver_data *info = file->private_data;

	switch (cmd) {
		case TORCH_LED_GET_STATE:
			data=torch_level_state;
			error = copy_to_user((int *)arg,&data,sizeof(data));
			if(error) {
				printk("Torch led: Can not save argument to user space!");
				return 0;
			}
			break;

		case TORCH_LED_SET_STATE:	
			error = copy_from_user(&data, (void *)arg, sizeof(data));
			if(error) {
				printk("Torch led: Can not get argument from user space!");
				return 0;
			} else {
				error=lm3642_torch_level(info,data);
				if(error<0) {
					lm3642_power_en(info,POWER_OFF);
					torch_level_state=0;
					return 0;
				}
			}
			torch_level_state=data;
			break;
		
		case FLASH_LED_GET_STATE:
			data=flash_level_state;
			error = copy_to_user((int *)arg,&data,sizeof(data));
			if(error) {
				printk("Flash led: Can not save argument to user space!");
				return -1;
			}
			break;

		case FLASH_LED_SET_STATE:		
			error = copy_from_user(&data, (void *)arg, sizeof(data));
			if(error) {
				printk("Flash led: Can not get argument from user space!");
				return -1;
			} else {
				error=lm3642_flash_level(info,data);
				if(error<0) {
					lm3642_power_en(info,POWER_OFF);
					flash_level_state=0;
					return 0;
				}
				flash_level_state=data;  
			}
			break;		

		case LM3556_IOCTL_MODE_SHUTDOWN:
			lm3642_power_en(info,POWER_OFF);
			break;

		case LM3556_IOCTL_MODE_STANDBY:
			lm3642_power_en(info,POWER_ON);
			break;
		
		case LM3556_IOTCL_MODE_TORCH:
			error = copy_from_user(&level, (void *)arg, sizeof(level));
			if(error) {
				printk("Torch led: Can not get argument from user space!");
				return -1;
			} else {
				error=lm3642_torch_level(info,level);
				if(error<0)
					return 0;
				torch_level_state=level;
			}
			break;
		
		case LM3556_IOCTL_MODE_FLASH:			
			error = copy_from_user(&level, (void *)arg, sizeof(level));
			if(error) {
				printk("Flash led: Can not get argument from user space!");
				return -1;
			} else {
				error=lm3642_flash_level(info,level);
				if(error<0)
					return 0;
				flash_level_state=level;  
			}
			break;	

		default:
			break;
	}
	return 0;
}

//sys file for debug mode 
//path :/sys/bus/i2c/devices/2-0063/lm3642_attr
static ssize_t torch_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 state;
	LM3642_DEBUG("torch_level_show");

	state = torch_level_state;
	return sprintf(buf, "%d\n", state);
}

static ssize_t torch_level_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	long unsigned int state = 0;
	int err = 0;
	
	LM3642_DEBUG();

	if((state>=0)&&(state<=MAX_TORCH_LEVEL)) {
		err = strict_strtoul(buf, 0, &state);
		if (err)
			return err;

		if(dev_info!=NULL) {
			if(state==0) {
				lm3642_torch_level(dev_info,state);
				lm3642_power_en(dev_info,POWER_OFF);
				gpio_direction_output(dev_info->torch_gpio, LED_OFF);
				gpio_direction_output(dev_info->flash_gpio, LED_OFF);
			} else {
				gpio_direction_output(dev_info->torch_gpio, LED_ON);
				gpio_direction_output(dev_info->flash_gpio, LED_OFF);
				lm3642_power_en(dev_info,POWER_ON);
				lm3642_torch_level(dev_info,state);
			}
		}
		torch_level_state=state;
	}

	return count;
}
static ssize_t flash_level_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	u8 state;
	LM3642_DEBUG();

	state = flash_level_state;
	return sprintf(buf, "%d\n", state);
}

static ssize_t flash_level_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	long unsigned int state = 0;
	int err = 0;

	LM3642_DEBUG();

	if((state>=0)&&(state<=MAX_FLASH_LEVEL)) {
		err = strict_strtoul(buf, 0, &state);
		if (err)
			return err;

		if(dev_info!=NULL) {
			if(state==0) {
				lm3642_flash_level(dev_info,state);
				lm3642_power_en(dev_info,POWER_OFF);
				gpio_direction_output(dev_info->torch_gpio, LED_OFF);
				gpio_direction_output(dev_info->flash_gpio, LED_OFF);
			} else {
				gpio_direction_output(dev_info->torch_gpio, LED_OFF);
				gpio_direction_output(dev_info->flash_gpio, LED_ON);
				lm3642_power_en(dev_info,POWER_ON);
				lm3642_flash_level(dev_info,state);
			}
		}
		flash_level_state=state;
	}

	return count;
}

static ssize_t torch_max_level_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	u8 value;
	
	LM3642_DEBUG();
	value = MAX_TORCH_LEVEL;
	return sprintf(buf, "%d\n", value);
}
static ssize_t flash_max_level_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	u8 value;
	LM3642_DEBUG();

	value =MAX_FLASH_LEVEL;
	return sprintf(buf, "%d\n", value);
}


#if 0
static void lm3642_flash_timer_level(struct i2c_client *client,u8 level)
{
     u8 value;

#if defined(debug_mode)
    printk("lm3642_flash_timer_level=%d\r\n",level);
#endif

    if((level>=0)&&(level<=MAX_FLASH_TIMER_LEVEL))
   {
        lm3642_read(client,FLASH_FEATURES_REG8,&value);
        value&=(~FLASH_TIMER_OUT_BIT);
        value|=(level);
        lm3642_write(client,FLASH_FEATURES_REG8,value);
    }
    else 
    {
        printk("lm3642_flash_timer_level limit for%d\r\n",level);
    }
}
static u8 get_flash_timer_level(struct i2c_client *client)
{
     u8 value;

        lm3642_read(client,FLASH_FEATURES_REG8,&value);
        value&=FLASH_TIMER_OUT_BIT;
        return value;
}

static ssize_t flash_timer_level_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
    long unsigned int state = 0;
    int err = 0;
    struct i2c_client *client = to_i2c_client(dev);
    
     if((state>=0)&&(state<=MAX_FLASH_TIMER_LEVEL))
    {
        err = strict_strtoul(buf, 0, &state);
        if (err)
       	return err;

            lm3642_flash_timer_level(client,state);
            flash_level_state=state;
    }
    return count;
}

static ssize_t flash_timer_level_show(struct device *dev,struct device_attribute *attr,char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
   u8 state;

    state = get_flash_timer_level(client);
    return sprintf(buf, "%d\n", state);
}
static ssize_t flash_timer_max_level_show(struct device *dev,struct device_attribute *attr,char *buf)
{
    u8 value;

    value = MAX_FLASH_TIMER_LEVEL;
    return sprintf(buf, "%d\n", value);
}

static DEVICE_ATTR(flash_timer_level, S_IWUSR | S_IRUGO, flash_timer_level_show, flash_timer_level_store);
static DEVICE_ATTR(flash_timer_max_level,  S_IRUGO, flash_timer_max_level_show, NULL);
#endif

static DEVICE_ATTR(torch_level, S_IWUSR | S_IRUGO, torch_level_show, torch_level_store);
static DEVICE_ATTR(flash_level, S_IWUSR | S_IRUGO, flash_level_show, flash_level_store);
static DEVICE_ATTR(torch_max_level, S_IRUGO, torch_max_level_show, NULL);
static DEVICE_ATTR(flash_max_level,  S_IRUGO, flash_max_level_show, NULL);

static struct attribute *dev_flashled_attributes[] = {
	&dev_attr_torch_level.attr,
	&dev_attr_flash_level.attr,	
	&dev_attr_torch_max_level.attr,	
	&dev_attr_flash_max_level.attr,
#if 0
	&dev_attr_flash_timer_level.attr,
	&dev_attr_flash_timer_max_level.attr,	
#endif
	NULL,
};

static struct attribute_group dev_flashled_attr_group = {
	.name="lm3642_attr",
	.attrs = dev_flashled_attributes,
};

static int lm3642_open(struct inode *inode, struct file *file)
{
	LM3642_DEBUG();
	file->private_data = dev_info;
	lm3642_power_en(dev_info,POWER_ON);
	return 0;
}

int lm3642_release(struct inode *inode, struct file *file)
{
	file->private_data=NULL;
	return 0;
}

static const struct file_operations lm3642_fileops = {
	.owner = THIS_MODULE,
	.open = lm3642_open,
	.unlocked_ioctl = lm3642_ioctl,
	.release = lm3642_release,
};

static struct miscdevice lm3642_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lm3642",
	.fops = &lm3642_fileops,
};

static int lm3642_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err=0;
	int i = 0;

	LM3642_DEBUG();
pr_info("huangyongheng add comeinto lm3642_probe .\n");
	dev_info = kzalloc(sizeof(struct flashled_driver_data), GFP_KERNEL);
	if (!dev_info)  {
		pr_err("lm3642: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&lm3642_device);
	if (err)  {
		pr_err("lm3642: Unable to register misc device!\n");
		kfree(dev_info);
		return err;
	}

	dev_info->pdata = client->dev.platform_data;
	dev_info->i2c_client = client;

	for(i=0;i<dev_info->pdata->num_leds;i++) {
		if(!strcmp(dev_info->pdata->leds[i].name,"torch_enable")) {
			dev_info->torch_gpio_state = dev_info->pdata->leds[i].init_state;
			dev_info->torch_gpio = dev_info->pdata->leds[i].gpio;
			gpio_direction_output(dev_info->torch_gpio, dev_info->torch_gpio_state);
			gpio_export(dev_info->torch_gpio, false);
		}
		if(!strcmp(dev_info->pdata->leds[i].name,"flash_enable")) {
			dev_info->flash_gpio_state = dev_info->pdata->leds[i].init_state;
			dev_info->flash_gpio = dev_info->pdata->leds[i].gpio;
			gpio_direction_output(dev_info->flash_gpio, dev_info->flash_gpio_state);
			gpio_export(dev_info->flash_gpio, false);
		}
	}

	i2c_set_clientdata(client, dev_info);
	
	err =sysfs_create_group(&dev_info->i2c_client->dev.kobj,&dev_flashled_attr_group);
	if (err)
		goto sysfs_error;

	tst_client=client;//huangyongheng add 2013.1.28
	return 0;

sysfs_error:
	printk("sysfs creat error\r\n");
	sysfs_remove_group(&dev_info->i2c_client->dev.kobj,&dev_flashled_attr_group);

	kfree(dev_info);

	return err;
}

static int lm3642_remove(struct i2c_client *client)
{
	int i = 0;
	struct flashled_driver_data *info;
	
	info = i2c_get_clientdata(client);
	misc_deregister(&lm3642_device);
	kfree(info);
	sysfs_remove_group(&client->dev.kobj, &dev_flashled_attr_group);

	for(i=0;i< dev_info->pdata->num_leds;i++)
		gpio_free(dev_info->pdata->leds[i].gpio);
	
	kfree(dev_info);
	return 0;
}

static const struct i2c_device_id lm3642_id[] = {
	{ "lm3642", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, lm3642_id);

static struct i2c_driver lm3642_i2c_driver = {
	.driver = {
		.name = "lm3642",
		.owner = THIS_MODULE,
	},
	.probe = lm3642_probe,
	.remove = lm3642_remove,
	.id_table = lm3642_id,
};

static int __init LM3642_ModuleInit(void)
{
	LM3642_DEBUG();
	return i2c_add_driver(&lm3642_i2c_driver);
}
static void __exit LM3642_ModuleExit(void)
{
	i2c_del_driver(&lm3642_i2c_driver);
}
module_init(LM3642_ModuleInit);
module_exit(LM3642_ModuleExit);

MODULE_DESCRIPTION("Camera Flash/Torch driver for lm3642");
MODULE_AUTHOR("nathanz@nvidia.com");
MODULE_LICENSE("GPL v2");
