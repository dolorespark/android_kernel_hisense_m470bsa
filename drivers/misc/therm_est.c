/*
 * drivers/misc/therm_est.c
 *
 * Copyright (C) 2010-2012 NVIDIA Corporation.
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

#include <linux/kernel.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/therm_est.h>
 
#ifdef CONFIG_DEBUG_FS
static struct dentry *thermal_debugfs_root;

static int skin_est_tmp_show(struct seq_file *s, void *data)
{
	int i;
	struct therm_estimator *est = s->private;

	for (i = 0; i < est->ndevs; i++) 
		seq_printf(s, "dev%d: %d  ", i, 
			est->devs[i]->hist[(est->ntemp % HIST_LEN)]);
	

	seq_printf(s, "\n");

	return 0;
}

static int skin_est_tmp_open(struct inode *inode, struct file *file)
{
	return single_open(file, skin_est_tmp_show, inode->i_private);
}

static ssize_t skin_est_tmp_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	return count;
}

static const struct file_operations skin_est_tmp_fops = {
	.open		= skin_est_tmp_open,
	.read		= seq_read,
	.write		= skin_est_tmp_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#endif


int therm_est_get_temp(struct therm_estimator *est, long *temp)
{
	*temp = est->cur_temp;
	return 0;
}

int therm_est_set_limits(struct therm_estimator *est,
				long lo_limit,
				long hi_limit)
{
	est->therm_est_lo_limit = lo_limit;
	est->therm_est_hi_limit = hi_limit;
	return 0;
}

int therm_est_set_alert(struct therm_estimator *est,
			void (*cb)(void *),
			void *cb_data)
{
	if ((!cb) || est->callback)
		BUG();

	est->callback = cb;
	est->callback_data = cb_data;

	return 0;
}

static void therm_est_work_func(struct work_struct *work)
{
	int i, j, index, sum = 0;
	long temp;
	static int run_count = 0;
	int skip_count = 30;
	struct delayed_work *dwork = container_of (work,
					struct delayed_work, work);
	struct therm_estimator *est = container_of(
					dwork,
					struct therm_estimator,
					therm_est_work);

	for (i = 0; i < est->ndevs; i++) {
		if (est->devs[i]->get_temp(est->devs[i]->dev_data, &temp))
			continue;
		est->devs[i]->hist[(est->ntemp % HIST_LEN)] = temp;
			if(run_count % skip_count == 0){
				printk(KERN_INFO "%s: i=%d, "
						"est->devs[%d]->coeffs[0]=%d, "
						"est->devs[%d]->temp=%d\n",
						 __func__, i, i,
						est->devs[i]->coeffs[0], i, temp);
				}
			
	}
	run_count++;
	for (i = 0; i < est->ndevs; i++) {
		for (j = 0; j < HIST_LEN; j++) {
			index = (est->ntemp - j + HIST_LEN) % HIST_LEN;
			sum += est->devs[i]->hist[index] *
				est->devs[i]->coeffs[j];
		}
	}

	est->cur_temp = sum / 100 + est->toffset;

	est->ntemp++;

	if (est->callback && ((est->cur_temp >= est->therm_est_hi_limit) ||
			 (est->cur_temp <= est->therm_est_lo_limit)))
		est->callback(est->callback_data);

	queue_delayed_work(est->workqueue, &est->therm_est_work,
				msecs_to_jiffies(est->polling_period));
}

struct therm_estimator *therm_est_register(
			struct therm_est_subdevice **devs,
			int ndevs,
			long toffset,
			long polling_period)
{
	int i, j;
	long temp;
	struct therm_estimator *est;
	struct therm_est_subdevice *dev;

	#ifdef CONFIG_DEBUG_FS
		thermal_debugfs_root = debugfs_create_dir("tegra_thermal_est", 0);
	#endif
	
	est = kzalloc(sizeof(struct therm_estimator), GFP_KERNEL);
	if (IS_ERR_OR_NULL(est))
		return ERR_PTR(-ENOMEM);

	est->devs = devs;
	est->ndevs = ndevs;
	est->toffset = toffset;
	est->polling_period = polling_period;

	/* initialize history */
	for (i = 0; i < ndevs; i++) {
		dev = est->devs[i];

		if (dev->get_temp(dev->dev_data, &temp)) {
			kfree(est);
			return ERR_PTR(-EINVAL);
		}

		for (j = 0; j < HIST_LEN; j++) {
			dev->hist[j] = temp;
		}
	}

	est->workqueue = alloc_workqueue("therm_est",
				    WQ_HIGHPRI | WQ_UNBOUND | WQ_RESCUER, 1);
	INIT_DELAYED_WORK(&est->therm_est_work, therm_est_work_func);

	queue_delayed_work(est->workqueue,
				&est->therm_est_work,
				msecs_to_jiffies(est->polling_period));
	#ifdef CONFIG_DEBUG_FS
		debugfs_create_file("est_temp", 0444, thermal_debugfs_root,
			est, &skin_est_tmp_fops);
	#endif

	return est;
}
