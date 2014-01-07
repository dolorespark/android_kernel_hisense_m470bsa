/*
 * arch/arm/mach-tegra/board-enterprise-sensors.c
 *
 * Copyright (c) 2011-2012, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/i2c/pca954x.h>
#ifndef CONFIG_SENSORS_TMP401
#include <linux/nct1008.h>
#endif
#include <linux/err.h>
#include <linux/mpu.h>
#include <linux/akm8963.h>//akm8963
#include <linux/platform_data/ina230.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <asm/mach-types.h>
#include <mach/gpio.h>
#include <media/ar0832_main.h>
#include <media/tps61050.h>
#include <media/ov9726.h>
#include <mach/edp.h>
#include <mach/thermal.h>
#include <mach/clk.h>
#include "cpu-tegra.h"
#include "gpio-names.h"
#include "board-enterprise.h"
#include "board.h"
#include "clock.h"
#ifdef CONFIG_SENSORS_TMP401
#include <linux/tmp401.h>
#endif

static struct board_info board_info;
#include <media/yuv_sensor.h>

#include <media/lm3642.h>

extern unsigned int his_hw_ver;
extern unsigned int his_board_version;

#ifndef CONFIG_SENSORS_TMP401
static int nct_get_temp(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp(data, temp);
}

static int nct_get_temp_low(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp_low(data, temp);
}

static int nct_set_limits(void *_data,
			long lo_limit_milli,
			long hi_limit_milli)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_limits(data,
					lo_limit_milli,
					hi_limit_milli);
}

static int nct_set_alert(void *_data,
				void (*alert_func)(void *),
				void *alert_data)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_alert(data, alert_func, alert_data);
}

static int nct_set_shutdown_temp(void *_data, long shutdown_temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_shutdown_temp(data,
						shutdown_temp);
}

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static int nct_get_itemp(void *dev_data, long *temp)
{
       struct nct1008_data *data = dev_data;
       return nct1008_thermal_get_temps(data, NULL, temp);
}
#endif

static void nct1008_probe_callback(struct nct1008_data *data)
{
	struct tegra_thermal_device *thermal_device;

	thermal_device = kzalloc(sizeof(struct tegra_thermal_device),
					GFP_KERNEL);
	if (!thermal_device) {
		pr_err("unable to allocate thermal device\n");
		return;
	}

	thermal_device->name = "nct1008";
	thermal_device->data = data;
	thermal_device->id = THERMAL_DEVICE_ID_NCT_EXT;
	thermal_device->offset = TDIODE_OFFSET;
	thermal_device->get_temp = nct_get_temp;
	thermal_device->get_temp_low = nct_get_temp_low;
	thermal_device->set_limits = nct_set_limits;
	thermal_device->set_alert = nct_set_alert;
	thermal_device->set_shutdown_temp = nct_set_shutdown_temp;

	tegra_thermal_device_register(thermal_device);

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
        {
               struct tegra_thermal_device *int_nct;
               int_nct = kzalloc(sizeof(struct tegra_thermal_device),
                               GFP_KERNEL);
               if (!int_nct) {
                       kfree(int_nct);
                       pr_err("unable to allocate thermal device\n");
                       return;
               }

               int_nct->name = "nct_int";
               int_nct->id = THERMAL_DEVICE_ID_NCT_INT;
               int_nct->data = data;
               int_nct->get_temp = nct_get_itemp;

               tegra_thermal_device_register(int_nct);
       }
#endif	
}

static struct nct1008_platform_data enterprise_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.offset = 8, /* 4 * 2C. Bug 844025 - 1C for device accuracies */
	.probe_callback = nct1008_probe_callback,
};

static struct i2c_board_info enterprise_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_TEMP_ALERT_N),
		.platform_data = &enterprise_nct1008_pdata,
	}
};

static void enterprise_nct1008_init(void)
{
	int ret;

	//ret = gpio_request(TEGRA_GPIO_TEMP_ALERT_N, "temp_alert");
	//if (ret < 0) {
	//	pr_err("%s: gpio_request failed %d\n", __func__, ret);
	//	return;
	//}

	ret = gpio_direction_input(TEGRA_GPIO_TEMP_ALERT_N);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_TEMP_ALERT_N);
		return;
	}

	i2c_register_board_info(4, enterprise_i2c4_nct1008_board_info,
				ARRAY_SIZE(enterprise_i2c4_nct1008_board_info));
}
#else
static int tmp_get_temp(void *_data, long *temp)
{
	struct tmp401_data *data = _data;
	return tmp401_thermal_get_temp(data, temp);
}
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static int tmp_get_temp_i(void *_data, long *temp)
{
	struct tmp401_data *data = _data;
	return tmp401_thermal_get_temp_i(data, temp);
}
#endif
static int tmp_get_temp_low(void *_data, long *temp)
{
	struct tmp401_data *data = _data;
	return tmp401_thermal_get_temp_low(data, temp);
}

static int tmp_set_limits(void *_data,
			long lo_limit_milli,
			long hi_limit_milli)
{
	struct tmp401_data *data = _data;
	return tmp401_thermal_set_limits(data,
					lo_limit_milli,
					hi_limit_milli);
}

static int tmp_set_alert(void *_data,
				void (*alert_func)(void *),
				void *alert_data)
{
	struct tmp401_data *data = _data;
	return tmp401_thermal_set_alert(data, alert_func, alert_data);
}

static int tmp_set_shutdown_temp(void *_data, long shutdown_temp)
{
	struct tmp401_data *data = _data;
	return tmp401_thermal_set_shutdown_temp(data,
						shutdown_temp);
}

static void tmp401_probe_callback(struct tmp401_data *data)
{
	struct tegra_thermal_device *thermal_device;

	thermal_device = kzalloc(sizeof(struct tegra_thermal_device),
					GFP_KERNEL);
	//printk("tmp401_probe_callback 1\n");
	if (!thermal_device) {
		pr_err("unable to allocate thermal device\n");
		return;
	}
	//printk("tmp401_probe_callback 2\n");
	thermal_device->name = "tmp401";//tmp401
	thermal_device->data = data;
	thermal_device->id = THERMAL_DEVICE_ID_NCT_EXT;
	thermal_device->offset = TDIODE_OFFSET;
	thermal_device->get_temp = tmp_get_temp;
	thermal_device->get_temp_low = tmp_get_temp_low;
	thermal_device->set_limits = tmp_set_limits;
	thermal_device->set_alert = tmp_set_alert;
	thermal_device->set_shutdown_temp = tmp_set_shutdown_temp;

	tegra_thermal_device_register(thermal_device);
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
        {
               struct tegra_thermal_device *int_nct;
               int_nct = kzalloc(sizeof(struct tegra_thermal_device),
                               GFP_KERNEL);
               if (!int_nct) {
                       kfree(int_nct);
                       pr_err("unable to allocate thermal device\n");
                       return;
               }

               int_nct->name = "nct_int";
               int_nct->id = THERMAL_DEVICE_ID_NCT_INT;
               int_nct->data = data;
               int_nct->get_temp = tmp_get_temp_i;

               tegra_thermal_device_register(int_nct);
       }
#endif	
}

static struct tmp401_platform_data enterprise_tmp401_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.offset = 8, /* 4 * 2C. Bug 844025 - 1C for device accuracies */
	.probe_callback = tmp401_probe_callback,
};
static struct i2c_board_info enterprise_i2c4_tmp401_board_info[] = {
	{
		I2C_BOARD_INFO("tmp401", 0x4C),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_TEMP_ALERT_N),
		.platform_data = &enterprise_tmp401_pdata,
	}
};

static void enterprise_tmp401_init(void)
{
	int ret;

	//ret = gpio_request(TEGRA_GPIO_TEMP_ALERT_N, "temp_alert");
	//if (ret < 0) {
	//	pr_err("%s: gpio_request failed %d\n", __func__, ret);
	//	return;
	//}

	ret = gpio_direction_input(TEGRA_GPIO_TEMP_ALERT_N);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_TEMP_ALERT_N);
		return;
	}

	i2c_register_board_info(4, enterprise_i2c4_tmp401_board_info,
				ARRAY_SIZE(enterprise_i2c4_tmp401_board_info));
}

#endif
/* MPU board file definition	*/
 
#if (MPU_GYRO_TYPE == MPU_TYPE_MPU3050)
 
#define MPU_GYRO_NAME           "mpu3050"
 
static struct mpu_platform_data mpu_gyro_data = {
 
        .int_config  = 0x10,
 
        .level_shifter = 0,
 
        .orientation = MPU_GYRO_ORIENTATION,    /* Located in
board_[platformname].h    */
 
        .sec_slave_type = SECONDARY_SLAVE_TYPE_ACCEL,
 
        .sec_slave_id   = ACCEL_ID_BMA250,
 
        .secondary_i2c_addr = MPU_ACCEL_ADDR,
 
        .secondary_orientation = MPU_ACCEL_ORIENTATION, /* Located in
board_[platformname].h    */
 
        .key = {},            //Key需要替换成各个customer自己的key
 
};
 
#endif
 
#if (MPU_GYRO_TYPE == MPU_TYPE_MPU6050)
 
#define MPU_GYRO_NAME           "mpu6050"
 
static struct mpu_platform_data mpu_gyro_data = {
 
        .int_config  = 0x10,
 
        .level_shifter = 0,
 
        .orientation = MPU_GYRO_ORIENTATION,    /* Located in
board_[platformname].h    */
 #if defined(CONFIG_BOARD_P9202)
        .sec_slave_type = SECONDARY_SLAVE_TYPE_NONE,
       // .sec_slave_id = COMPASS_ID_AK8963,
      //  .secondary_i2c_addr =MPU_COMPASS_ADDR,
      //  .secondary_orientation =MPU_COMPASS_ORIENTATION,
 #else
	 .sec_slave_type = SECONDARY_SLAVE_TYPE_COMPASS,
	 .sec_slave_id = COMPASS_ID_AK8963,
	 .secondary_i2c_addr =MPU_COMPASS_ADDR,
	 .secondary_orientation =MPU_COMPASS_ORIENTATION,
 #endif
        .key = {0xf1, 0x9e, 0x11, 0xc4, 0xc8, 0x77, 0xe0, 0xc8, 
			0x4f, 0xa7, 0x1a, 0x8a, 0x72, 0xc0, 0x68, 0x69}          //Key需要替换成各个customer自己的key
 
};
 
#endif
 
//static struct mpu_platform_data mpu_compass_data = {
 
        //.orientation = MPU_COMPASS_ORIENTATION, /* Located in
//board_[platformname].h    */
 
//};
 
 
static struct i2c_board_info __initdata inv_mpu_i2c2_board_info[] = {
 
        {
 
                I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
 
                .irq = TEGRA_GPIO_TO_IRQ(MPU_GYRO_IRQ_GPIO),
 
                .platform_data = &mpu_gyro_data,
 
        },
 
        //{
 
                //I2C_BOARD_INFO(MPU_COMPASS_NAME, MPU_COMPASS_ADDR),
 
                //.platform_data = &mpu_compass_data,
 
       // },
 
};
 
 
 
//Interrupt configuration in board-xxx-sensor.c:
 
 
 
static void mpuirq_init(void)
 
{
 
        int ret = 0;
 
 
 
        printk("*** MPU START *** mpuirq_init...\n");
 
 
 
        /* MPU-IRQ assignment */
 
        //tegra_gpio_enable(MPU_GYRO_IRQ_GPIO);
 
       // ret = gpio_request(MPU_GYRO_IRQ_GPIO, MPU_GYRO_NAME);
 
        //if (ret < 0) {
 
        //        pr_err("%s: gpio_request failed %d\n", __func__, ret);
 
       //         return;
 
        //}
 
 
 
        ret = gpio_direction_input(MPU_GYRO_IRQ_GPIO);
 
        if (ret < 0) {
 
                pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
 
                gpio_free(MPU_GYRO_IRQ_GPIO);
 
                return;
 
        }
 
        printk("*** MPU END *** mpuirq_init...\n");
 
 
 
        i2c_register_board_info(MPU_GYRO_BUS_NUM, inv_mpu_i2c2_board_info,
 
                ARRAY_SIZE(inv_mpu_i2c2_board_info));
 
}


static inline void enterprise_msleep(u32 t)
{
	/*
	If timer value is between ( 10us - 20ms),
	usleep_range() is recommended.
	Please read Documentation/timers/timers-howto.txt.
	*/
	usleep_range(t*1000, t*1000 + 500);
}
#ifdef CONFIG_ISL29023
static struct i2c_board_info enterprise_i2c0_isl_board_info[] = {
	{
		I2C_BOARD_INFO("isl29023", 0x44),
	}
};

static void enterprise_isl_init(void)
{
	i2c_register_board_info(0, enterprise_i2c0_isl_board_info,
				ARRAY_SIZE(enterprise_i2c0_isl_board_info));
}
#endif







static struct nvc_torch_pin_state enterprise_tps61050_pinstate = {
	.mask		= 0x0008, /*VGP3*/
	.values		= 0x0008,
};

static struct tps61050_platform_data enterprise_tps61050_pdata = {
	.dev_name	= "torch",
	.pinstate	= &enterprise_tps61050_pinstate,
};

//add start 2013.1.25
////#ifdef CONFIG_TORCH_LM3642
static struct lm3642_pin lm3642[] = {
	{
		.name			= "flash_enable",
        		.gpio				= TEGRA_GPIO_CAM_FLASH_EN,
        		.init_state 		= 0,
	},
	{
		.name			= "torch_enable",
        		.gpio				= TEGRA_GPIO_CAM_TORCH_EN,
        		.init_state 		= 0,
	},
};

static struct lm3642_platform_data lm3642_info = {
	.leds			= lm3642,
	.num_leds	= ARRAY_SIZE(lm3642),
};
////#endif
//add end 2013.1.25

struct enterprise_cam_gpio {
	int gpio;
	const char *label;
	int value;
};

#define TEGRA_CAMERA_GPIO(_gpio, _label, _value)	\
	{						\
		.gpio = _gpio,				\
		.label = _label,			\
		.value = _value,			\
	}

static struct pca954x_platform_mode enterprise_pca954x_modes[] = {
	{ .adap_id = PCA954x_I2C_BUS0, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS1, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS2, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS3, .deselect_on_exit = true, },
};

static struct pca954x_platform_data enterprise_pca954x_data = {
	.modes    = enterprise_pca954x_modes,
	.num_modes      = ARRAY_SIZE(enterprise_pca954x_modes),
};

static const struct i2c_board_info enterprise_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("pca9546", 0x70),
		.platform_data = &enterprise_pca954x_data,
	},
	{
		I2C_BOARD_INFO("tps61050", 0x33),
		.platform_data = &enterprise_tps61050_pdata,
	},

};

/*
 * Since ar0832 driver should support multiple devices, slave
 * address should be changed after it is open. Default slave
 * address of ar0832 is 0x36. It will be changed to alternate
 * address defined below when device is open.
 */



static int enterprise_ov2655_power_on(void)
{		
	mdelay(10); 
	gpio_direction_output(TEGRA_GPIO_CAM_PWR_EN , 1);
	pr_info(" OV2655 AVDD POWER ON. \n" );
	mdelay(10); 
        if((his_board_version == 0) || (his_board_version == 2) || (his_board_version == 3))
            gpio_direction_output(TEGRA_GPIO_CAM_AVDD_PWR_EN , 1);

	gpio_direction_output(TEGRA_GPIO_REAR_CAM_PWDN, 1);//m370 back camera ov5640 pwdn  : enable :1, disable:0.
	gpio_direction_output(TEGRA_GPIO_FRONT_CAM_PWDN, 0);//front camera pwdn is disable	: enable:1, disable:0
	mdelay(10); 
	gpio_direction_output(TEGRA_GPIO_FRONT_CAM_RST, 1);//front camera rst . enable:0 , disable:1.
	return 0;
}
static int enterprise_ov2655_power_off(void)
{	
	pr_info(" OV2655 AVDD POWER OFF .\n" );
	gpio_direction_output(TEGRA_GPIO_FRONT_CAM_PWDN, 1);//front camera pwdn is disable	: enable:1, disable:0
	mdelay(10);
	gpio_direction_output(TEGRA_GPIO_FRONT_CAM_RST, 0);	//front camera rst . enable:0 , disable:1.
	gpio_direction_output(TEGRA_GPIO_REAR_CAM_PWDN, 0);// us9230 prj back camera ov8825 pwdn   : enable :0, disable:1.
        if((his_board_version == 0) || (his_board_version == 2) || (his_board_version == 3))
            gpio_direction_output(TEGRA_GPIO_CAM_AVDD_PWR_EN , 0);
	mdelay(10); 
	gpio_direction_output(TEGRA_GPIO_CAM_PWR_EN , 0);
	return 0;
	}
struct yuv_sensor_platform_data enterprise_ov2655_data = {	
	.power_on = enterprise_ov2655_power_on,	
	.power_off = enterprise_ov2655_power_off,
};

static int enterprise_ov5640_power_on(void)
{		
	gpio_direction_output(TEGRA_GPIO_CAM_PWR_EN , 1);
        mdelay(10); 
        gpio_direction_output(TEGRA_GPIO_CAM_AF_EN, 1);//AF power on
        if((his_board_version == 0) || (his_board_version == 2) || (his_board_version == 3))
            gpio_direction_output(TEGRA_GPIO_CAM_AVDD_PWR_EN , 1);
	pr_info(" OV5640 AVDD POWER ON .\n" );	
	mdelay(10);	
	gpio_direction_output(TEGRA_GPIO_CAM_FLASH_EN, 0); //flash disable
	gpio_direction_output(TEGRA_GPIO_CAM_TORCH_EN, 0); //torch disable 
	gpio_direction_output(TEGRA_GPIO_FRONT_CAM_PWDN, 1);//front camera pwdn
	gpio_direction_output(TEGRA_GPIO_REAR_CAM_PWDN, 0);//rear camera pwdn disable 
	gpio_direction_output(TEGRA_GPIO_REAR_CAM_RST_N, 1);//rear camera 	rst disable	
	mdelay(10); 	
	return 0;
}
static int enterprise_ov5640_power_off(void)
{		
	pr_info(" OV5640 AVDD POWER OFF .\n" );		
	gpio_direction_output(TEGRA_GPIO_CAM_FLASH_EN, 0); //flash disable
	gpio_direction_output(TEGRA_GPIO_CAM_TORCH_EN, 0); //torch disable	
	gpio_direction_output(TEGRA_GPIO_REAR_CAM_PWDN, 1);//rear camera pwdn enable 
	mdelay(10);
	gpio_direction_output(TEGRA_GPIO_REAR_CAM_RST_N, 0); //back camera rst enable	
	gpio_direction_output(TEGRA_GPIO_FRONT_CAM_PWDN, 1);//front camera pwdn enable	
        if((his_board_version == 0) || (his_board_version == 2) || (his_board_version == 3))
            gpio_direction_output(TEGRA_GPIO_CAM_AVDD_PWR_EN , 0);
	mdelay(10);
	gpio_direction_output(TEGRA_GPIO_CAM_PWR_EN , 0);
	gpio_direction_output(TEGRA_GPIO_CAM_AF_EN, 0);//AF power off
	return 0;
	}
struct yuv_sensor_platform_data enterprise_ov5640_data = {	
	.power_on = enterprise_ov5640_power_on,	
	.power_off = enterprise_ov5640_power_off,
	};



static  struct i2c_board_info enterprise_i2c3_boardinfo[] = {
	{
		I2C_BOARD_INFO("ov5640", 0x3c),
		.platform_data = &enterprise_ov5640_data,
		.irq = -1,
	},

	{
		I2C_BOARD_INFO("ov2655", 0x30),
		.platform_data = &enterprise_ov2655_data,
		.irq = -1,
	},
//add start 2013.1.25
//#ifdef CONFIG_TORCH_LM3642
	{
	I2C_BOARD_INFO("lm3642", 0x63),
	.platform_data = &lm3642_info,
	},
//#endif
};
//add end 2013.1.25

static int enterprise_cam_init(void)
{
	gpio_direction_output(TEGRA_GPIO_FRONT_CAM_PWDN, 0);
	gpio_export(TEGRA_GPIO_FRONT_CAM_PWDN, false);
	
	gpio_direction_output(TEGRA_GPIO_REAR_CAM_PWDN, 0);
	gpio_export(TEGRA_GPIO_REAR_CAM_PWDN, false);
		   
	gpio_direction_output(TEGRA_GPIO_FRONT_CAM_RST, 0);
	gpio_export(TEGRA_GPIO_FRONT_CAM_RST, false);
			
	gpio_direction_output(TEGRA_GPIO_REAR_CAM_RST_N, 0);
	gpio_export(TEGRA_GPIO_REAR_CAM_RST_N, false);

	gpio_direction_output(TEGRA_GPIO_CAM_FLASH_EN, 0);

	gpio_direction_output(TEGRA_GPIO_CAM_TORCH_EN, 0);


    	gpio_direction_output(TEGRA_GPIO_CAM_PWR_EN , 0);
    	gpio_export(TEGRA_GPIO_CAM_PWR_EN, false);

        if((his_board_version == 0) || (his_board_version == 2) || (his_board_version == 3))
        {
            gpio_direction_output(TEGRA_GPIO_CAM_AVDD_PWR_EN , 0);
            gpio_export(TEGRA_GPIO_CAM_AVDD_PWR_EN, false);
        }

	gpio_direction_output(TEGRA_GPIO_CAM_AF_EN, 0);
	gpio_export(TEGRA_GPIO_CAM_AF_EN, false);

	return 0;
}



#define ENTERPRISE_INA230_ENABLED 0

static struct ina230_platform_data ina230_platform = {
	.rail_name = "VDD_AC_BAT",
	.current_threshold = TEGRA_CUR_MON_THRESHOLD,
	.resistor = TEGRA_CUR_MON_RESISTOR,
	.min_cores_online = TEGRA_CUR_MON_MIN_CORES,
};

#if ENTERPRISE_INA230_ENABLED
static struct i2c_board_info enterprise_i2c0_ina230_info[] = {
	{
		I2C_BOARD_INFO("ina230", 0x42),
		.platform_data = &ina230_platform,
		.irq = -1,
	},
};

static int __init enterprise_ina230_init(void)
{
	return i2c_register_board_info(0, enterprise_i2c0_ina230_info,
			ARRAY_SIZE(enterprise_i2c0_ina230_info));
}
#endif

static struct i2c_board_info tai_i2c4_ina230_info[] = {
	{
		I2C_BOARD_INFO("ina230", 0x40),
		.platform_data = &ina230_platform,
		.irq = -1, /* connected to SPI2_CS1_N(PX3) */
	},
};

static int __init tai_ina230_init(void)
{
	return i2c_register_board_info(4, tai_i2c4_ina230_info,
			ARRAY_SIZE(tai_i2c4_ina230_info));
}

int __init enterprise_sensors_init(void)
{
	int ret = 0;

	tegra_get_board_info(&board_info);
#ifdef CONFIG_ISL29023	
	enterprise_isl_init();
#endif
	#ifndef CONFIG_SENSORS_TMP401
		enterprise_nct1008_init();
	#else
		enterprise_tmp401_init();
	#endif
	//if (board_info.board_id != BOARD_E1239)
		mpuirq_init();
//#if ENTERPRISE_INA230_ENABLED
	//if (machine_is_tegra_enterprise())
	//	enterprise_ina230_init();
//#endif
	if (machine_is_tai())
		tai_ina230_init();
 	
    	ret = enterprise_cam_init();

	i2c_register_board_info(2, enterprise_i2c3_boardinfo,
				ARRAY_SIZE(enterprise_i2c3_boardinfo));
	return ret;
}

