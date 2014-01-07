/*
 * tlv320aic3256-irq.c  --  Interrupt controller support for
 *			 TI OMAP44XX TLV320aic3256
 *
 * Author:      Mukund Navada <navada@ti.com>
 *              Mehar Bajwa <mehar.bajwa@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/interrupt.h>

#include <linux/mfd/tlv320aic3xxx-core.h>
#include <linux/mfd/tlv320aic3256-registers.h>
#include "../../../arch/arm/mach-tegra/board-enterprise.h"
#include "../../../arch/arm/mach-tegra/gpio-names.h"

#include <linux/delay.h>

struct aic3256_irq_data {
	int mask;
	int status;
};
#define HEADSETIN_WORKQUEUE_TIME    1000
#define HEADSETOUT_WORKQUEUE_TIME    800
static bool HeadsetPlugInEvent = false;
static bool HeadsetPlugState = false;
static bool HeadsetPlugOutEvent = false;
bool HeadsetIN = false; 
extern int rescheduled;
extern int his_hpdet;
extern bool pressdown;
extern bool pressup;
//extern bool aic325xdev_suspend;

static struct aic3256_irq_data aic3256_irqs[] = {
	{
	 .mask = AIC3256_HEADSET_IN_MASK,
	 .status = AIC3256_HEADSET_PLUG_UNPLUG_INT,
	 },
	{
	 .mask = AIC3256_BUTTON_PRESS_MASK,
	 .status = AIC3256_BUTTON_PRESS_INT,
	 },
	{
	 .mask = AIC3256_DAC_DRC_THRES_MASK,
	 .status =  AIC3256_LEFT_DRC_THRES_INT |  AIC3256_RIGHT_DRC_THRES_INT,
	 },
	{
	 .mask =  AIC3256_AGC_NOISE_MASK,
	 .status =  AIC3256_LEFT_AGC_NOISE_INT |  AIC3256_RIGHT_AGC_NOISE_INT,
	 },
	{
	 .mask =  AIC3256_OVER_CURRENT_MASK,
	 .status =  AIC3256_LEFT_OUTPUT_DRIVER_OVERCURRENT_INT
	 |  AIC3256_RIGHT_OUTPUT_DRIVER_OVERCURRENT_INT,
	 },
	{
	 .mask =  AIC3256_OVERFLOW_MASK,
	 .status =
	  AIC3256_LEFT_DAC_OVERFLOW_INT |  AIC3256_RIGHT_DAC_OVERFLOW_INT |
	  AIC3256_LEFT_ADC_OVERFLOW_INT |  AIC3256_RIGHT_ADC_OVERFLOW_INT ,
	 },

};

struct aic3256_gpio_data {

};

static inline struct aic3256_irq_data *irq_to_aic3256_irq(struct aic3xxx
							  *aic3256, int irq)
{
         //printk("%s  %d\n", __FUNCTION__,  irq - aic3256->irq_base);
	return &aic3256_irqs[irq - aic3256->irq_base];
}

static void aic3256_irq_lock(struct irq_data *data)
{
	struct aic3xxx *aic3256 = irq_data_get_irq_chip_data(data);

	mutex_lock(&aic3256->irq_lock);
}

static void aic3256_irq_sync_unlock(struct irq_data *data)
{
	struct aic3xxx *aic3256 = irq_data_get_irq_chip_data(data);
	//printk("%s: aic3256->irq_masks_cur = %d\n", __func__, aic3256->irq_masks_cur);
	/* write back to hardware any change in irq mask */
	if (aic3256->irq_masks_cur != aic3256->irq_masks_cache) {
		aic3256->irq_masks_cache = aic3256->irq_masks_cur;
		aic3xxx_reg_write(aic3256, AIC3256_INT1_CTRL,
				  aic3256->irq_masks_cur);
	}

	mutex_unlock(&aic3256->irq_lock);
}

static void aic3256_irq_unmask(struct irq_data *data)
{
	struct aic3xxx *aic3256 = irq_data_get_irq_chip_data(data);
	struct aic3256_irq_data *irq_data =
	    irq_to_aic3256_irq(aic3256, data->irq);

	aic3256->irq_masks_cur |= irq_data->mask;
	
	//printk("%s: aic3256->irq_masks_cur = %d\n", __func__, aic3256->irq_masks_cur);
}

static void aic3256_irq_mask(struct irq_data *data)
{
	struct aic3xxx *aic3256 = irq_data_get_irq_chip_data(data);
	struct aic3256_irq_data *irq_data =
	    irq_to_aic3256_irq(aic3256, data->irq);

	aic3256->irq_masks_cur &= ~irq_data->mask;
	
	//printk("%s: aic3256->irq_masks_cur = %d\n", __func__, aic3256->irq_masks_cur);
}

static struct irq_chip aic3256_irq_chip = {
	.name = "tlv320aic3256",
	.irq_bus_lock = aic3256_irq_lock,
	.irq_bus_sync_unlock = aic3256_irq_sync_unlock,
	.irq_mask = aic3256_irq_mask,
	.irq_unmask = aic3256_irq_unmask,
};
void aic325x_headsetin_work(struct work_struct *work)
{
	struct aic3xxx *aic3256 = container_of(work,
						    struct aic3xxx,
						    headsetin_delayed_work.work);
	if(his_hpdet){		
		HeadsetIN = true;
		handle_nested_irq(aic3256->irq_base);
		}
	else{
		HeadsetPlugState = true;
		HeadsetPlugInEvent= false;
	}
	//printk("***************%s HeadsetPlugInEvent =%d HeadsetPlugState=%d\n",__func__,HeadsetPlugInEvent,HeadsetPlugState);
}
EXPORT_SYMBOL_GPL(aic325x_headsetin_work);
void aic325x_headsetout_work(struct work_struct *work)
{
	struct aic3xxx *aic3256 = container_of(work,
						    struct aic3xxx,
						    headsetout_delayed_work.work);
	if(HeadsetPlugOutEvent){
		HeadsetPlugOutEvent= false;
		return;
	}
	else
		handle_nested_irq(aic3256->irq_base + 1);
	//printk("***************%s HeadsetPlugOutEvent =%d\n",__func__,HeadsetPlugOutEvent);
}
EXPORT_SYMBOL_GPL(aic325x_headsetout_work);
void aic325x_keypress_work(struct work_struct *work)
{
	struct aic3xxx *aic3256 = container_of(work,
						    struct aic3xxx,
						    keypress_delayed_work.work);
		handle_nested_irq(aic3256->irq_base + 1);
}
EXPORT_SYMBOL_GPL(aic325x_keypress_work);
void aic325x_irq_work(struct work_struct *work)
{
	struct aic3xxx *aic3256 = container_of(work,
						    struct aic3xxx,
						    irq_delayed_work.work);
	u8 status[4];
	int val;
	/* Reading the sticky bit registers acknowledges
	the interrupt to the device */
	wake_lock_timeout(&aic3256->irq_wake_lock, 4*HZ);
	aic3xxx_bulk_read(aic3256, AIC3256_INT_STICKY_FLAG1, 4, status);
	val = aic3xxx_reg_read(aic3256,AIC3256_INT_FLAG2);
	if(his_hpdet){
	////////////HP in out
	/*
		if(gpio_get_value(TEGRA_GPIO_M470_HP_DET)){
			if(!HeadsetIN){
				printk("________IN\n");
				rescheduled = 0;//just report once!!!
				queue_delayed_work(aic3256->headsetin_workqueue, &aic3256->headsetin_delayed_work,
						   msecs_to_jiffies(1000));
				return ;
			}
		}
		else{
			printk("_______OUT \n");
			HeadsetIN = false;
			cancel_delayed_work_sync(&aic3256->headsetin_delayed_work);
			handle_nested_irq(aic3256->irq_base);
			return;
		}
	*/
		if(gpio_get_value(TEGRA_GPIO_M470_KEY_DET)){
			printk("KEY  HIGH \n");
			pressdown = true;
		}
		if(!gpio_get_value(TEGRA_GPIO_M470_KEY_DET)){
			printk("KEY  LOW\n");
			pressup = true;
		}
		if(HeadsetIN){
			queue_delayed_work(aic3256->keypress_workqueue, &aic3256->keypress_delayed_work,
					   msecs_to_jiffies(400));
		}
		return;
	}
	else{
			/* report  */
			if (status[2] & aic3256_irqs[AIC3256_IRQ_HEADSET_DETECT].status){
				handle_nested_irq(aic3256->irq_base);

				if(val & 0x10){
					//printk("________IN\n");
					HeadsetPlugInEvent = true;
					queue_delayed_work(aic3256->headsetin_workqueue, &aic3256->headsetin_delayed_work,
							   msecs_to_jiffies(HEADSETIN_WORKQUEUE_TIME));
				}
				else{
					//printk("_______OUT\n");
					HeadsetPlugOutEvent = true;
					HeadsetPlugState = false;
					queue_delayed_work(aic3256->headsetout_workqueue, &aic3256->headsetout_delayed_work,
							   msecs_to_jiffies(HEADSETOUT_WORKQUEUE_TIME));
				}
					
			}
			if(HeadsetPlugInEvent)
				return;
			if (status[2] & aic3256_irqs[AIC3256_IRQ_BUTTON_PRESS].status){
				if(HeadsetPlugState)
					queue_delayed_work(aic3256->headsetout_workqueue, &aic3256->headsetout_delayed_work,
							   msecs_to_jiffies(HEADSETOUT_WORKQUEUE_TIME));
			}
		}

	if (status[2] & aic3256_irqs[AIC3256_IRQ_DAC_DRC].status)
		handle_nested_irq(aic3256->irq_base + 2);
	if (status[3] & aic3256_irqs[AIC3256_IRQ_AGC_NOISE].status)
		handle_nested_irq(aic3256->irq_base + 3);
	if (status[2] & aic3256_irqs[AIC3256_IRQ_OVER_CURRENT].status)
		handle_nested_irq(aic3256->irq_base + 4);
	if (status[0] & aic3256_irqs[AIC3256_IRQ_OVERFLOW_EVENT].status)
		handle_nested_irq(aic3256->irq_base + 5);


	
	/* ack unmasked irqs */
	/* No need to acknowledge the interrupt on AIC3262 */
}
EXPORT_SYMBOL_GPL(aic325x_irq_work);

static irqreturn_t aic3256_jack_irq_thread(int irq, void *data)
{
	struct aic3xxx  *aic3256 = data;
	/*
	queue_delayed_work(aic3256->irq_workqueue, &aic3256->irq_delayed_work,
			   msecs_to_jiffies(1));
	return IRQ_HANDLED;
	*/
	if(gpio_get_value(TEGRA_GPIO_M470_HP_DET)){
			if(!HeadsetIN){
				printk("________IN\n");
				rescheduled = 0;//just report once!!!
				queue_delayed_work(aic3256->headsetin_workqueue, &aic3256->headsetin_delayed_work,
						   msecs_to_jiffies(1000));
				return IRQ_HANDLED;
			}
		}
		else{
			printk("_______OUT \n");
			HeadsetIN = false;
			cancel_delayed_work_sync(&aic3256->headsetin_delayed_work);
			handle_nested_irq(aic3256->irq_base);
			return IRQ_HANDLED;
		}
	return IRQ_HANDLED;
	
}
static irqreturn_t aic3256_key_irq_thread(int irq, void *data)
{
	struct aic3xxx  *aic3256 = data;
	queue_delayed_work(aic3256->irq_workqueue, &aic3256->irq_delayed_work,
			   msecs_to_jiffies(10));
/*
	if(gpio_get_value(TEGRA_GPIO_M470_KEY_DET))
		printk("_______________________aic3256_key_irq_thread  HIGH  HeadsetIN = %d\n",HeadsetIN);
	else
		printk("_______________________aic3256_key_irq_thread  LOW  HeadsetIN = %d\n",HeadsetIN);
	if(HeadsetIN){
		printk("irq begin!!\n");
		handle_nested_irq(aic3256->irq_base + 1);
		printk("irq end!!\n");
	}
*/
	return IRQ_HANDLED;
}

static irqreturn_t aic3256_irq_thread(int irq, void *data)
{
	struct aic3xxx *aic3256 = data;
	if(his_hpdet){
		printk("  ____________________  aic3256_irq_thread     HeadsetIN = %d \n",HeadsetIN);
		if(HeadsetIN == false)
			return IRQ_HANDLED;
	}
	queue_delayed_work(aic3256->irq_workqueue, &aic3256->irq_delayed_work,
			   msecs_to_jiffies(100));
	return IRQ_HANDLED;
}

int aic3xxx_irq_init(struct aic3xxx *aic3256)
{
	int cur_irq, ret;

	mutex_init(&aic3256->irq_lock);
	//printk("==============> %s : enter \n", __func__);
	wake_lock_init(&aic3256->irq_wake_lock, WAKE_LOCK_SUSPEND,
			"irq_suspend_lock");
	/* mask the individual interrupt sources */
	aic3256->irq_masks_cur = 0x0;
	aic3256->irq_masks_cache = 0x0;
	aic3xxx_reg_write(aic3256, AIC3256_INT1_CTRL, 0x0);

	if (!aic3256->irq) {
		dev_warn(aic3256->dev,
			 "no interrupt specified, no interrupts\n");
		aic3256->irq_base = 0;
		return 0;
	}

	if (!aic3256->irq_base) {
		dev_err(aic3256->dev,
			"no interrupt base specified, no interrupts\n");
		return 0;
	}

	/* Register them with genirq */
	for (cur_irq = aic3256->irq_base;
	     cur_irq < aic3256->irq_base + ARRAY_SIZE(aic3256_irqs);
	     cur_irq++) {
		irq_set_chip_data(cur_irq, aic3256);
		irq_set_chip_and_handler(cur_irq, &aic3256_irq_chip,
					 handle_edge_irq);
		irq_set_nested_thread(cur_irq, 1);

		/* ARM needs us to explicitly flag the IRQ as valid
		 * and will set them noprobe when we do so. */
#ifdef CONFIG_ARM
		set_irq_flags(cur_irq, IRQF_VALID);
#else
		set_irq_noprobe(cur_irq);
#endif
	}
	//printk("%s: register irq\n", __func__);
	printk("his_hpdet = %d\n",his_hpdet);
	if(his_hpdet){
		ret = request_threaded_irq(aic3256->jack_irq, NULL, aic3256_jack_irq_thread,
					   IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,
					   "tlv320aic3256-jack", aic3256);
		if (ret) {
			dev_err(aic3256->dev, "failed to request IRQ %d: %d\n",
				aic3256->jack_irq, ret);
			return ret;
		}
		//keypress
		ret = request_threaded_irq(aic3256->key_irq, NULL, aic3256_key_irq_thread,
					   IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,
					   "tlv320aic3256-key", aic3256);
		if (ret) {
			dev_err(aic3256->dev, "failed to request IRQ %d: %d\n",
				aic3256->key_irq, ret);
			return ret;
		}
		ret = enable_irq_wake(aic3256->key_irq);
		if (ret < 0) {
			dev_err(aic3256->dev, "failed to enable  wake  IRQ %d: %d\n",
				aic3256->key_irq, ret);
			return ret;
		}
		ret = enable_irq_wake(aic3256->jack_irq);
		if (ret < 0) {
			dev_err(aic3256->dev, "failed to enable  wake  IRQ %d: %d\n",
				aic3256->jack_irq, ret);
			return ret;
		}

	}
	else{
		ret = request_threaded_irq(aic3256->irq, NULL, aic3256_irq_thread,
					   IRQF_TRIGGER_RISING,
					   "tlv320aic3256", aic3256);
		if (ret < 0) {
			dev_err(aic3256->dev, "failed to request IRQ %d: %d\n",
				aic3256->irq, ret);
			return ret;
		}
		ret = enable_irq_wake(aic3256->irq);
		if (ret < 0) {
			dev_err(aic3256->dev, "failed to enable  wake  IRQ %d: %d\n",
				aic3256->irq, ret);
			return ret;
		}
	}
		
	//printk("%s: register finish\n", __func__);

	return 0;
}
EXPORT_SYMBOL(aic3xxx_irq_init);

void aic3xxx_irq_exit(struct aic3xxx *aic3256)
{
	if (aic3256->irq)
		free_irq(aic3256->irq, aic3256);
}
EXPORT_SYMBOL(aic3xxx_irq_exit);
MODULE_AUTHOR("Mukund navada <navada@ti.com>");
MODULE_AUTHOR("Mehar Bajwa <mehar.bajwa@ti.com>");
MODULE_DESCRIPTION
	("Interrupt controller support for TI OMAP44XX TLV320aic3256");
MODULE_LICENSE("GPL");
