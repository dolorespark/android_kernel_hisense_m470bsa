/*
 * isl29023.c - Intersil ISL29023  ALS & Proximity Driver
 *
 * By Intersil Corp
 * Michael DiGioia
 *
 * Based on isl29011.c
 *	by Michael DiGioia <michaelx.digioia@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/hwmon.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/pm_runtime.h>

/* Insmod parameters */
//I2C_CLIENT_INSMOD_1(isl29023);

#define MODULE_NAME	"isl29023"

/* registers */
#define ISL29023_REG_VENDOR_REV                 0x06
#define ISL29023_VENDOR                         1
#define ISL29023_VENDOR_MASK                    0x0F
#define ISL29023_REV                            4
#define ISL29023_REV_SHIFT                      4
#define ISL29023_REG_DEVICE                     0x23
#define ISL29023_DEVICE                         23


#define REG_CMD_1		0x00
#define REG_CMD_2		0x01
#define REG_DATA_LSB		0x02
#define REG_DATA_MSB		0x03
#define ISL_MOD_MASK		0xE0
#define ISL_MOD_POWERDOWN	0
#define ISL_MOD_ALS_ONCE	1
#define ISL_MOD_IR_ONCE		2
#define ISL_MOD_PS_ONCE		3
#define ISL_MOD_RESERVED	4
#define ISL_MOD_ALS_CONT	5
#define ISL_MOD_IR_CONT		6
#define ISL_MOD_PS_CONT		7
#define IR_CURRENT_MASK		0xC0
#define IR_FREQ_MASK		0x30
#define SENSOR_RANGE_MASK	0x03
#define ISL_RES_MASK		0x0C

static int last_mod;

static DEFINE_MUTEX(mutex);

static int isl_set_range(struct i2c_client *client, int range)
{
	int ret_val;

	ret_val = i2c_smbus_read_byte_data(client, REG_CMD_2);
	if (ret_val < 0)
		return -EINVAL;
	ret_val &= ~SENSOR_RANGE_MASK;	/*reset the bit */
	ret_val |= range;
	ret_val = i2c_smbus_write_byte_data(client, REG_CMD_2, ret_val);

 printk(KERN_INFO MODULE_NAME ": %s isl29023 set_range call, \n", __func__);
	if (ret_val < 0)
		return ret_val;
	return range;
}
static int isl_get_range(struct i2c_client *client)
{
	int ret_val,range=0;

	ret_val = i2c_smbus_read_byte_data(client, REG_CMD_2);
	if (ret_val < 0)
		return -EINVAL;
	range =(1 << (2 * (ret_val & 3)))*1000;
	return range;
}

static int isl_set_mod(struct i2c_client *client, int mod)
{
	int ret, val, freq;

	switch (mod) {
	case ISL_MOD_POWERDOWN:
	case ISL_MOD_RESERVED:
		goto setmod;
	case ISL_MOD_ALS_ONCE:
	case ISL_MOD_ALS_CONT:
		freq = 0;
		break;
	case ISL_MOD_IR_ONCE:
	case ISL_MOD_IR_CONT:
	case ISL_MOD_PS_ONCE:
	case ISL_MOD_PS_CONT:
		freq = 1;
		break;
	default:
		return -EINVAL;
	}
	/* set IR frequency */
	val = i2c_smbus_read_byte_data(client, REG_CMD_2);
	if (val < 0)
		return -EINVAL;
	val &= ~IR_FREQ_MASK;
	if (freq)
		val |= IR_FREQ_MASK;
	ret = i2c_smbus_write_byte_data(client, REG_CMD_2, val);
	if (ret < 0)
		return -EINVAL;

setmod:
	/* set operation mod */
	val = i2c_smbus_read_byte_data(client, REG_CMD_1);
	if (val < 0)
		return -EINVAL;
	val &= ~ISL_MOD_MASK;
	val |= (mod << 5);
	ret = i2c_smbus_write_byte_data(client, REG_CMD_1, val);
	if (ret < 0)
		return -EINVAL;

	if (mod != ISL_MOD_POWERDOWN)
		last_mod = mod;

	return mod;
}

static int isl_get_res(struct i2c_client *client)
{
	int val;

 printk(KERN_INFO MODULE_NAME ": %s isl29023 get_res call, \n", __func__);
	val = i2c_smbus_read_byte_data(client, REG_CMD_2);

	if (val < 0)
		return -EINVAL;

	val &= ISL_RES_MASK;
	val >>= 2;

	switch (val) {
	case 0:
		return 65536;
	case 1:
		return 4096;
	case 2:
		return 256;
	case 3:
		return 16;
	default:
		return -EINVAL;
	}
}

static int isl_get_mod(struct i2c_client *client)
{
	int val;

	val = i2c_smbus_read_byte_data(client, REG_CMD_1);
	if (val < 0)
		return -EINVAL;
	return val >> 5;
}

static ssize_t
isl_sensing_range_show(struct device *dev,
		       struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int val;

	mutex_lock(&mutex);
	//pm_runtime_get_sync(dev);
	val = i2c_smbus_read_byte_data(client, REG_CMD_2);
	//pm_runtime_put_sync(dev);
	mutex_unlock(&mutex);

	dev_dbg(dev, "%s: range: 0x%.2x\n", __func__, val);

	if (val < 0)
		return val;
	return sprintf(buf, "%d000\n", 1 << (2 * (val & 3)));
}

static ssize_t
ir_current_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int val;

	mutex_lock(&mutex);
	//pm_runtime_get_sync(dev);
	val = i2c_smbus_read_byte_data(client, REG_CMD_2);
	//pm_runtime_put_sync(dev);
	mutex_unlock(&mutex);

	dev_dbg(dev, "%s: IR current: 0x%.2x\n", __func__, val);

	if (val < 0)
		return -EINVAL;
	val >>= 6;

	switch (val) {
	case 0:
		val = 100;
		break;
	case 1:
		val = 50;
		break;
	case 2:
		val = 25;
		break;
	case 3:
		val = 0;
		break;
	default:
		return -EINVAL;
	}

	if (val)
		val = sprintf(buf, "%d\n", val);
	else
		val = sprintf(buf, "%s\n", "12.5");
	return val;
}

static ssize_t
isl_sensing_mod_show(struct device *dev,
		     struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int val;

	mutex_lock(&mutex);
	//pm_runtime_get_sync(dev);
	val = isl_get_mod(client);
	//pm_runtime_put_sync(dev);
	mutex_unlock(&mutex);

	dev_dbg(dev, "%s: mod: 0x%.2x\n", __func__, val);

	if (val < 0)
		return val;

	switch (val) {
	case ISL_MOD_POWERDOWN:
		return sprintf(buf, "%s\n", "0-Power-down");
	case ISL_MOD_ALS_ONCE:
		return sprintf(buf, "%s\n", "1-ALS once");
	case ISL_MOD_IR_ONCE:
		return sprintf(buf, "%s\n", "2-IR once");
	case ISL_MOD_PS_ONCE:
		return sprintf(buf, "%s\n", "3-Proximity once");
	case ISL_MOD_RESERVED:
		return sprintf(buf, "%s\n", "4-Reserved");
	case ISL_MOD_ALS_CONT:
		return sprintf(buf, "%s\n", "5-ALS continuous");
	case ISL_MOD_IR_CONT:
		return sprintf(buf, "%s\n", "6-IR continuous");
	case ISL_MOD_PS_CONT:
		return sprintf(buf, "%s\n", "7-Proximity continuous");
	default:
		return -EINVAL;
	}
}

static ssize_t
isl_output_data_show(struct device *dev,
		     struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret_val, val, mod;
	unsigned long int max_count, output = 0;
	int temp;
	//printk("output data start\n");
	mutex_lock(&mutex);
	//pm_runtime_get_sync(dev);

	temp = i2c_smbus_read_byte_data(client, REG_DATA_MSB);
	if (temp < 0)
		goto err_exit;
	ret_val = i2c_smbus_read_byte_data(client, REG_DATA_LSB);
	if (ret_val < 0)
		goto err_exit;
	ret_val |= temp << 8;

	dev_dbg(dev, "%s: Data: %04x\n", __func__, ret_val);

	mod = isl_get_mod(client);
	switch (mod) {
	case ISL_MOD_ALS_CONT:
	case ISL_MOD_ALS_ONCE:
		//max_count =65536;// isl_get_res(client)
		//range = 4000;//isl_get_range(client)
		output =(ret_val*30);//7*0.06*100  20*0.015*100
		//printk("als max_count = %d, range=%d,output= %d,ret_val=%d\n",max_count,range,output,ret_val);
		break;
	case ISL_MOD_IR_ONCE:
	case ISL_MOD_IR_CONT:
		output = ret_val;
		break;
	case ISL_MOD_PS_CONT:
	case ISL_MOD_PS_ONCE:
		val = i2c_smbus_read_byte_data(client, REG_CMD_2);
		if (val < 0)
			goto err_exit;
		max_count = isl_get_res(client);
		output = (((1 << (2 * (val & SENSOR_RANGE_MASK))) * 1000)
			  * ret_val) / max_count;
		break;
	default:
		goto err_exit;
	}
	//pm_runtime_put_sync(dev);
	mutex_unlock(&mutex);
	//printk("output data stop\n");
	return sprintf(buf, "%ld\n", output);

err_exit:
	//pm_runtime_put_sync(dev);
	mutex_unlock(&mutex);
	return -EINVAL;
}

static ssize_t
isl_reg_dump_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
	{
		int ii;
		char data=0;
		ssize_t bytes_printed = 0;
		struct i2c_client *client = to_i2c_client(dev);
		for (ii = 0; ii < 4; ii++) {
			data=i2c_smbus_read_byte_data(client, ii);
			bytes_printed += sprintf(buf + bytes_printed, "REG_CMD_%d: %#2x\n",ii, data);
		}
	
		return  bytes_printed;
	}

static ssize_t
isl_sensing_range_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned int ret_val;
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case 1000:
		val = 0;
		break;
	case 4000:
		val = 1;
		break;
	case 16000:
		val = 2;
		break;
	case 64000:
		val = 3;
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&mutex);
	//pm_runtime_get_sync(dev);
	ret_val = isl_set_range(client, val);
	//pm_runtime_put_sync(dev);
	mutex_unlock(&mutex);

	if (ret_val < 0)
		return ret_val;
	return count;
}

static ssize_t
ir_current_store(struct device *dev,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned int ret_val;
	unsigned long val;

	if (!strncmp(buf, "12.5", 4))
		val = 3;
	else {
		if (strict_strtoul(buf, 10, &val))
			return -EINVAL;
		switch (val) {
		case 100:
			val = 0;
			break;
		case 50:
			val = 1;
			break;
		case 25:
			val = 2;
			break;
		default:
			return -EINVAL;
		}
	}

	mutex_lock(&mutex);
	//pm_runtime_get_sync(dev);

	ret_val = i2c_smbus_read_byte_data(client, REG_CMD_2);
	if (ret_val < 0)
		goto err_exit;

	ret_val &= ~IR_CURRENT_MASK;	/*reset the bit before setting them */
	ret_val |= (val << 6);

	ret_val = i2c_smbus_write_byte_data(client, REG_CMD_2, ret_val);
	if (ret_val < 0)
		goto err_exit;

	//pm_runtime_put_sync(dev);
	mutex_unlock(&mutex);

	return count;

err_exit:
	//pm_runtime_put_sync(dev);
	mutex_unlock(&mutex);
	return -EINVAL;
}

static ssize_t
isl_sensing_mod_store(struct device *dev,
		      struct device_attribute *attr,
		      const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret_val;
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	if (val > 7)
		return -EINVAL;

	mutex_lock(&mutex);
	//pm_runtime_get_sync(dev);
	ret_val = isl_set_mod(client, val);
	// printk(KERN_INFO MODULE_NAME "zhaojing isl29023 isl_set_mod=%d   ret_val = %d\n", val,ret_val);
	//pm_runtime_put_sync(dev);
	mutex_unlock(&mutex);

	if (ret_val < 0)
		return ret_val;
	return count;
}

static DEVICE_ATTR(range, S_IRUGO | S_IWUSR,
		   isl_sensing_range_show, isl_sensing_range_store);
static DEVICE_ATTR(mod, S_IRUGO | S_IWUSR,
		   isl_sensing_mod_show, isl_sensing_mod_store);
static DEVICE_ATTR(ir_current, S_IRUGO | S_IWUSR,
		   ir_current_show, ir_current_store);
static DEVICE_ATTR(output, S_IRUGO, isl_output_data_show, NULL);
static DEVICE_ATTR(reg_dump, S_IRUGO, isl_reg_dump_show, NULL);

static struct attribute *mid_att_isl[] = {
	&dev_attr_range.attr,
	&dev_attr_mod.attr,
	&dev_attr_ir_current.attr,
	&dev_attr_output.attr,
	&dev_attr_reg_dump.attr,
	NULL
};

static struct attribute_group m_isl_gr = {
	.name = "isl29023",
	.attrs = mid_att_isl
};

static int isl_set_default_config(struct i2c_client *client)
{
	int ret;
	ret = i2c_smbus_write_byte_data(client, REG_CMD_1, 0xE0);
	if (ret < 0)
		return -EINVAL;
	ret = i2c_smbus_write_byte_data(client, REG_CMD_2, 0xC0);
	if (ret < 0)
		return -EINVAL;
 printk(KERN_INFO MODULE_NAME ": %s isl29023 set_default_config call, \n", __func__);

	return 0;
}

/* Return 0 if detection is successful, -ENODEV otherwise */
static int isl29023_detect(struct i2c_client *client, int kind,
                          struct i2c_board_info *info)
{
        struct i2c_adapter *adapter = client->adapter;

        if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
                return -ENODEV;

 printk(KERN_INFO MODULE_NAME ": %s isl29023 detact call, kind:%d type:%s addr:%x \n", __func__, kind, info->type, info->addr);

        if (kind <= 0) {
                int vendor, device, revision;

                vendor = i2c_smbus_read_word_data(client,
                                                  ISL29023_REG_VENDOR_REV);
                vendor >>= 8;
                revision = vendor >> ISL29023_REV_SHIFT;
                vendor &= ISL29023_VENDOR_MASK;
                if (vendor != ISL29023_VENDOR)
                        return -ENODEV;

                device = i2c_smbus_read_word_data(client,
                                                  ISL29023_REG_DEVICE);
                device >>= 8;
                if (device != ISL29023_DEVICE)
                        return -ENODEV;

                if (revision != ISL29023_REV)
                        dev_info(&adapter->dev, "Unknown revision %d\n",
                                 revision);
        } else
                dev_dbg(&adapter->dev, "detection forced\n");

        strlcpy(info->type, "isl29023", I2C_NAME_SIZE);

        return 0;
}

static int
isl29023_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int res;

	dev_info(&client->dev, "%s: ISL 023 chip found\n", client->name);

 printk(KERN_INFO MODULE_NAME ": %s isl29023 probe call, ID= %s\n",
                        __func__, id->name);
	res = isl_set_default_config(client);
	if (res < 0) {
		//pr_warn("isl29023: set default config failed!!\n");
 printk(KERN_INFO MODULE_NAME ": %s isl29023 set default config failed\n", __func__);
		return -EINVAL;
	}

	res = sysfs_create_group(&client->dev.kobj, &m_isl_gr);
	if (res) {
		//pr_warn("isl29023: device create file failed!!\n");
 printk(KERN_INFO MODULE_NAME ": %s isl29023 device create file failed\n", __func__);
		return -EINVAL;
	}

	last_mod = 0;
	isl_set_mod(client, ISL_MOD_POWERDOWN);
	//pm_runtime_enable(&client->dev);

	dev_dbg(&client->dev, "isl29023 probe succeed!\n");

	return res;
}

static int isl29023_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &m_isl_gr);
	__pm_runtime_disable(&client->dev, false);
 printk(KERN_INFO MODULE_NAME ": %s isl29023 remove call, \n", __func__);
	return 0;
}
static void isl29023_shutdown(struct i2c_client *client)
{
	isl_set_mod(client, ISL_MOD_POWERDOWN);

 printk(KERN_INFO MODULE_NAME ": %s isl29023 shutdown call, \n", __func__);
	//return 0;
}

static struct i2c_device_id isl29023_id[] = {
	{"isl29023", 0},
	{}
};

static int isl29023_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	dev_dbg(dev, "suspend\n");
	isl_set_mod(client, ISL_MOD_POWERDOWN);
 //printk(KERN_INFO MODULE_NAME ": %s isl29023 suspend call, \n", __func__);
	return 0;
}

static int isl29023_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	dev_dbg(dev, "resume\n");
	isl_set_mod(client, last_mod);
	msleep(100);
 //printk(KERN_INFO MODULE_NAME ": %s isl29023 resume call, \n", __func__);
	return 0;
}

MODULE_DEVICE_TABLE(i2c, isl29023_id);

static const struct dev_pm_ops isl29023_pm_ops = {
	.runtime_suspend = isl29023_runtime_suspend,
	.runtime_resume = isl29023_runtime_resume,
};

static struct i2c_driver isl29023_driver = {
	.driver = {
		   .name = "isl29023",
		   .pm = &isl29023_pm_ops,
		   },
	.probe = isl29023_probe,
	.remove = isl29023_remove,
	.shutdown = isl29023_shutdown,
	.id_table = isl29023_id,
	.detect         = isl29023_detect,
	//.address_data   = &addr_data,
};

static int __init sensor_isl29023_init(void)
{
 printk(KERN_INFO MODULE_NAME ": %s isl29023 init call, \n", __func__);
	return i2c_add_driver(&isl29023_driver);
}

static void __exit sensor_isl29023_exit(void)
{
 printk(KERN_INFO MODULE_NAME ": %s isl29023 exit call \n", __func__);
	i2c_del_driver(&isl29023_driver);
}

module_init(sensor_isl29023_init);
module_exit(sensor_isl29023_exit);

MODULE_AUTHOR("mdigioia");
MODULE_ALIAS("isl29023 ALS/PS");
MODULE_DESCRIPTION("Intersil isl29023 ALS/PS Driver");
MODULE_LICENSE("GPL v2");

