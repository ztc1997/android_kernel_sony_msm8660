/*
 *
 * Bosh	BMA	250. Digital, triaxial acceleration	sensor.
 *
 * NOTE: This file contains	code from: bma250_ng.c
 *		 The orginal bma250_ng.c header	is included	below:
 *
 * Copyright (C) 2010 Sony Ericsson	Mobile Communications AB.
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Author: Marcus Bauer	<marcus.bauer@sonymobile.com>
 *		   Tadashi Kubo	<tadashi.kubo@sonymobile.com>
 *		   Takashi Shiina <takashi.shiina@sonymobile.com>
 *		   Hisakazu	Furuie <hisakazu.x.furuie@sonymobile.com>
 *		   Chikaharu Gonnokami <Chikaharu.X.Gonnokami@sonymobile.com>
 *
 */
/*
 * Motion detection	driver for Bosch BMA250	accelerometer
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/bma250_ng.h>
#include <linux/bma250_ng_common.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "bma250_motion.h"

#define	BMA250_MOTION_NAME					"bma250_motion"
#define	BMA250_MOTION_VENDORID				0x0001

#define	BMA250_STARTUP_TIME					4

#define	BMA250_TAP_DURATION_DEFAULT			(BMA250_TAP_SHOCK_50MS | BMA250_TAP_QUIET_30MS | BMA250_TAP_DUR_50MS)
#define	BMA250_TAP_THRESHOLD_DEFAULT		20
#define	BMA250_SLOPE_DURATION_DEFAULT		3
#define	BMA250_SLOPE_THRESHOLD_DEFAULT		48
#define	BMA250_LOW_G_DURATION_DEFAULT		99
#define	BMA250_LOW_G_THRESHOLD_DEFAULT		112
#define	BMA250_LOW_G_HYSTERESIS_DEFAULT		0
#define	BMA250_HIGH_G_DURATION_DEFAULT		99
#define	BMA250_HIGH_G_THRESHOLD_DEFAULT		144
#define	BMA250_HIGH_G_HYSTERESIS_DEFAULT	0
#define	BMA250_FLAT_THETA_DEFAULT			2
#define	BMA250_FLAT_HOLD_DEFAULT			BMA250_FLAT_HOLD_512MS

enum {
	BMA250_MOTION_TAP,
	BMA250_MOTION_SLOPE,
	BMA250_MOTION_LOW_G,
	BMA250_MOTION_HIGH_G,
	BMA250_MOTION_COUNT,
};

#define	BMA250_MODE_MASK				0x0F
#define	BMA250_EVENT_MASK				0x30
#define	BMA250_EVENT_POWER_TOGGLE		0x00
#define	BMA250_EVENT_POWER_UP			0x10
#define	BMA250_EVENT_POWER_DOWN			0x20
#define	BMA250_EVENT_ABS				0x30
#define	BMA250_FLAT_MASK				0x40
#define	BMA250_GAP_MASK					0x80

struct gap_work	{
	struct delayed_work			 work;
	struct driver_data			*dd;
	int							 motion;
};

struct driver_data {
	struct input_dev			*ip_dev;
	struct i2c_client			*ic_dev;
	struct bma250_platform_data	*pdata;
	struct gap_work				 gap_work[BMA250_MOTION_COUNT];
	int							 flat_motion;
	struct mutex				 mutex;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend		 early_suspend;
#endif
	int							 suspend;
	int							 power;
	unsigned char				 mode[BMA250_MOTION_COUNT];
	unsigned char				 tap_duration;
	unsigned char				 tap_threshold;
	unsigned char				 slope_duration;
	unsigned char				 slope_threshold;
	unsigned char				 low_g_duration;
	unsigned char				 low_g_threshold;
	unsigned char				 low_g_hysteresis;
	unsigned char				 high_g_duration;
	unsigned char				 high_g_threshold;
	unsigned char				 high_g_hysteresis;
	unsigned char				 flat_theta;
	unsigned char				 flat_hold;
};

struct motion_attr {
	int	irq_mask;
	int	mode_count;
	int	mode_irq_mask[8];
};
static const struct	motion_attr	bma250_motion_attr[BMA250_MOTION_COUNT]	=
{
	{
		BMA250_INT_STATUS_S_TAP	| BMA250_INT_STATUS_D_TAP,
		3,
		{ 0, BMA250_INT_STATUS_S_TAP, BMA250_INT_STATUS_D_TAP },
	},
	{
		BMA250_INT_SLOPE_MASK,
		8,
		{ 0, 1,	2, 3, 4, 5,	6, 7 },
	},
	{
		BMA250_INT_LOW_G <<	8,
		2,
		{ 0, BMA250_INT_LOW_G << 8 },
	},
	{
		BMA250_INT_HIGH_G_MASK << 8,
		8,
		{ 0, 1 << 8, 2 << 8, 3 << 8, 4 << 8, 5 << 8, 6 << 8, 7 << 8	},
	},
};


static irqreturn_t bma250_motion_thread_irq(int	irq, void *dev);

static int bma250_motion_irq(struct	driver_data	*dd, int clear,	int	set)
{
	int	rc;
	u8 int_enable;

	if ((clear & 0xFF) || (set & 0xFF))	{
		rc = bma250_ic_read(dd->ic_dev,	BMA250_INT_ENABLE1_REG,	&int_enable, 1);
		if (rc)
			return rc;

		int_enable &= ~(clear &	0xFF);
		int_enable |= (set & 0xFF);

		rc = bma250_ic_write(dd->ic_dev, BMA250_INT_ENABLE1_REG, int_enable);
		if (rc)
			return rc;
	}

	if (((clear	>> 8) &	0xFF) || ((set >> 8) & 0xFF)) {
		rc = bma250_ic_read(dd->ic_dev,	BMA250_INT_ENABLE2_REG,	&int_enable, 1);
		if (rc)
			return rc;

		int_enable &= ~((clear >> 8) & 0xFF);
		int_enable |= ((set	>> 8) &	0xFF);

		rc = bma250_ic_write(dd->ic_dev, BMA250_INT_ENABLE2_REG, int_enable);
		if (rc)
			return rc;
	}

	return rc;
}

static int bma250_motion_irq_enable(struct driver_data *dd,	int	motion)
{
	int	mode, clear, set;

	mode = dd->mode[motion];
	clear =	bma250_motion_attr[motion].irq_mask;
	set	= bma250_motion_attr[motion].mode_irq_mask[mode	& BMA250_MODE_MASK];

	return bma250_motion_irq(dd, clear,	set);
}

static int bma250_motion_irq_disable(struct	driver_data	*dd, int motion)
{
	int	clear;

	clear =	bma250_motion_attr[motion].irq_mask;

	return bma250_motion_irq(dd, clear,	0);
}

static void	bma250_hw_shutdown(struct driver_data *dd)
{
	dd->pdata->teardown(&dd->ic_dev->dev);
	dd->pdata->hw_config(0);
}

static int bma250_hw_setup(struct driver_data *dd)
{
	int	rc = 0;

	dd->pdata->hw_config(1);
	rc = dd->pdata->setup(&dd->ic_dev->dev);
	if (rc)
		dd->pdata->hw_config(0);
	return rc;
}

static int bma250_motion_power_down(struct driver_data *dd,	int	motion)
{
	struct bma250_platform_data	*pdata = dd->ic_dev->dev.platform_data;
	int	power;

	power =	dd->power &	~(1	<< motion);

	if (!power)	{
		free_irq(dd->ic_dev->irq, dd);
		bma250_ic_write(dd->ic_dev,	BMA250_INT_ENABLE1_REG,	0x00);
		bma250_ic_write(dd->ic_dev,	BMA250_INT_ENABLE2_REG,	0x00);

		pdata->vote_sleep_status(BMA250_SLAVE3,	BMA250_SLEEP);
		if (pdata->check_sleep_status()	== BMA250_SLEEP) {
			bma250_ic_write(dd->ic_dev,
				BMA250_MODE_CTRL_REG,
				BMA250_MODE_SUSPEND);
			bma250_hw_shutdown(dd);
		}
	}

	dd->power =	power;

	return 0;
}

static int bma250_motion_power_up(struct driver_data *dd, int motion)
{
	struct bma250_platform_data	*pdata = dd->ic_dev->dev.platform_data;
	int	rc = 0,	power;

	power =	dd->power |	(1 << motion);

	if (!dd->power)	{
		if (pdata->check_sleep_status()	== BMA250_SLEEP) {
			rc = bma250_hw_setup(dd);
			if (rc)
				goto hw_setup_error;
			rc = bma250_ic_write(dd->ic_dev,
				BMA250_RESET_REG, BMA250_RESET);
			if (rc)
				goto power_up_error;
			msleep(BMA250_STARTUP_TIME);
			rc = bma250_ic_write(dd->ic_dev,
				BMA250_MODE_CTRL_REG,
				BMA250_MODE_NOSLEEP);
			if (rc)
				goto power_up_error;
		}

		rc = bma250_ic_write(dd->ic_dev, BMA250_TAP_DUR_REG, dd->tap_duration);
		if (rc)
			goto power_up_error;
		rc = bma250_ic_write(dd->ic_dev, BMA250_TAP_THR_REG, dd->tap_threshold);
		if (rc)
			goto power_up_error;
		rc = bma250_ic_write(dd->ic_dev, BMA250_SLOPE_DUR_REG, dd->slope_duration);
		if (rc)
			goto power_up_error;
		rc = bma250_ic_write(dd->ic_dev, BMA250_SLOPE_THR_REG, dd->slope_threshold);
		if (rc)
			goto power_up_error;
		rc = bma250_ic_write(dd->ic_dev, BMA250_LOW_G_DUR_REG, dd->low_g_duration);
		if (rc)
			goto power_up_error;
		rc = bma250_ic_write(dd->ic_dev, BMA250_LOW_G_THR_REG, dd->low_g_threshold);
		if (rc)
			goto power_up_error;
		rc = bma250_ic_write(dd->ic_dev, BMA250_HIGH_G_DUR_REG,	dd->high_g_duration);
		if (rc)
			goto power_up_error;
		rc = bma250_ic_write(dd->ic_dev, BMA250_HIGH_G_THR_REG,	dd->high_g_threshold);
		if (rc)
			goto power_up_error;
		rc = bma250_ic_write(dd->ic_dev, BMA250_G_MODE_REG,
					dd->low_g_hysteresis | dd->high_g_hysteresis);
		if (rc)
			goto power_up_error;
		rc = bma250_ic_write(dd->ic_dev, BMA250_FLAT_THETA_REG,	dd->flat_theta);
		if (rc)
			goto power_up_error;
		rc = bma250_ic_write(dd->ic_dev, BMA250_FLAT_HOLD_REG, dd->flat_hold);
		if (rc)
			goto power_up_error;

		rc = bma250_ic_write(dd->ic_dev, BMA250_INT_CONFIG_REG,
				BMA250_INT_PIN1_LEVEL |	BMA250_INT_PIN2_LEVEL);
		if (rc)
			goto power_up_error;
		rc = bma250_ic_write(dd->ic_dev, BMA250_INT_PIN1_REG, BMA250_INT_PIN1_MASK);
		if (rc)
			goto power_up_error;
		rc = bma250_ic_write(dd->ic_dev, BMA250_INT_CTRL_REG,
				BMA250_INT_RESET | BMA250_INT_LATCHED);
		if (rc)
			goto power_up_error;

		rc = request_threaded_irq(dd->ic_dev->irq, NULL, bma250_motion_thread_irq,
				  IRQF_TRIGGER_HIGH	| IRQF_ONESHOT,	dd->ic_dev->name, dd);
		if (rc)
			goto power_up_error;

		pdata->vote_sleep_status(BMA250_SLAVE3,	BMA250_AWAKE);
	}

	dd->power =	power;
	return rc;

power_up_error:
	bma250_hw_shutdown(dd);
hw_setup_error:
	printk(KERN_ERR	"%s: Force power down.\n", __func__);
	return rc;
}

static int bma250_motion_mode(struct driver_data *dd, int motion, int mode)
{
	int	rc,	clear, set;

	if (motion >= BMA250_MOTION_COUNT)
		return -EINVAL;

	if ((mode &	BMA250_MODE_MASK) >= bma250_motion_attr[motion].mode_count)
		return -EINVAL;

	clear =	bma250_motion_attr[motion].irq_mask;
	set	= bma250_motion_attr[motion].mode_irq_mask[mode	& BMA250_MODE_MASK];

	cancel_delayed_work(&dd->gap_work[motion].work);
	if ((dd->flat_motion ==	motion)	&& dd->power) {
		rc = bma250_motion_irq(dd, BMA250_INT_FLAT,	0);
		if (rc)
			return rc;
	}

	if (mode & BMA250_FLAT_MASK)
		dd->flat_motion	= motion;

	if ((mode &	BMA250_MODE_MASK) &&
		!(((mode & BMA250_EVENT_MASK) == BMA250_EVENT_POWER_UP)	&& !dd->suspend) &&
		!(((mode & BMA250_EVENT_MASK) == BMA250_EVENT_POWER_DOWN) && dd->suspend)) {
		rc = bma250_motion_power_up(dd,	motion);
		if (rc)
			return rc;

		if (mode & BMA250_FLAT_MASK)
			rc = bma250_motion_irq(dd, 0, BMA250_INT_FLAT);
		else
			rc = bma250_motion_irq(dd, clear, set);
		if (rc)
			return rc;
	} else {
		rc = bma250_motion_irq(dd, clear, 0);
		if (rc)
			return rc;

		rc = bma250_motion_power_down(dd, motion);
		if (rc)
			return rc;
	}

	dd->mode[motion] = mode;
	return 0;
}

static void	bma250_motion_event(struct driver_data *dd,	int	motion)
{
	switch (dd->mode[motion] & BMA250_EVENT_MASK) {
	case BMA250_EVENT_POWER_TOGGLE:
power_key:
		input_report_key(dd->ip_dev, KEY_POWER,	1);
		input_sync(dd->ip_dev);
		input_report_key(dd->ip_dev, KEY_POWER,	0);
		input_sync(dd->ip_dev);
		break;
	case BMA250_EVENT_POWER_UP:
		if (dd->suspend)
			goto power_key;
		break;
	case BMA250_EVENT_POWER_DOWN:
		if (!dd->suspend)
			goto power_key;
		break;
	case BMA250_EVENT_ABS:
		input_abs_set_val(dd->ip_dev, ABS_MISC,	0);
		input_report_abs(dd->ip_dev, ABS_MISC, 1);
		input_sync(dd->ip_dev);
		break;
	}
}

static void	bma250_motion_trig(struct driver_data *dd, int motion)
{
	bma250_motion_irq_disable(dd, motion);

	if (dd->mode[motion] & BMA250_GAP_MASK)	{
		cancel_delayed_work(&dd->gap_work[motion].work);
		schedule_delayed_work(&dd->gap_work[motion].work, HZ);
	} else if (!(dd->mode[motion] &	BMA250_FLAT_MASK)) {
		bma250_motion_irq_enable(dd, motion);
	}

	bma250_motion_event(dd,	motion);
}

static irqreturn_t bma250_motion_thread_irq(int	irq, void *dev)
{
	struct driver_data *dd = (struct driver_data *)dev;
	u8 status, status2;

	mutex_lock(&dd->mutex);

	if (!bma250_ic_read(dd->ic_dev,	BMA250_INT_STATUS_REG, &status,	1))	{
		if ((status	& (BMA250_INT_STATUS_S_TAP | BMA250_INT_STATUS_D_TAP | BMA250_INT_STATUS_SLOPE)) &&
			!bma250_ic_read(dd->ic_dev,	BMA250_TAP_SLOPE_STATUS_REG, &status2, 1)) {
			if ((status	& (BMA250_INT_STATUS_S_TAP | BMA250_INT_STATUS_D_TAP)) &&
				(status2 & BMA250_TAP_FIRST_Z))	{
				pr_info(BMA250_MOTION_NAME": tap detected\n");
				bma250_motion_trig(dd, BMA250_MOTION_TAP);
			}

			if (status & BMA250_INT_STATUS_SLOPE) {
				pr_info(BMA250_MOTION_NAME": slope detected\n");
				bma250_motion_trig(dd, BMA250_MOTION_SLOPE);
			}
		}

		if (status & BMA250_INT_STATUS_LOW_G) {
			pr_info(BMA250_MOTION_NAME": low g detected\n");
			bma250_motion_trig(dd, BMA250_MOTION_LOW_G);
		}

		if ((status	& (BMA250_INT_STATUS_HIGH_G	| BMA250_INT_STATUS_FLAT)) &&
			!bma250_ic_read(dd->ic_dev,	BMA250_ORIENT_STATUS_REG, &status2,	1))	{
			if (status & BMA250_INT_STATUS_HIGH_G) {
				pr_info(BMA250_MOTION_NAME": high g	detected\n");
				bma250_motion_trig(dd, BMA250_MOTION_HIGH_G);
			}

			if (status & BMA250_INT_STATUS_FLAT) {
				if (status2	& BMA250_FLAT_STATUS) {
					pr_info(BMA250_MOTION_NAME": flat detected\n");
					bma250_motion_irq_enable(dd, dd->flat_motion);
				} else {
					bma250_motion_irq_disable(dd, dd->flat_motion);
				}
			}
		}
	}

	bma250_ic_write(dd->ic_dev,	BMA250_INT_CTRL_REG,
		BMA250_INT_RESET | BMA250_INT_LATCHED);

	mutex_unlock(&dd->mutex);

	return IRQ_HANDLED;
}

static void	bma250_motion_gap_work(struct work_struct *work)
{
	struct gap_work	*gap = (struct gap_work	*)work;

	mutex_lock(&gap->dd->mutex);
	bma250_motion_irq_enable(gap->dd, gap->motion);
	mutex_unlock(&gap->dd->mutex);
}

static ssize_t bma250_tap_mode_show(struct device *dev,
					struct device_attribute	*attr,
					char *buf)
{
	struct driver_data *dd = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE,	"%d\n",	dd->mode[BMA250_MOTION_TAP]);
}

static ssize_t bma250_tap_mode_store(struct	device *dev,
					struct device_attribute	*attr,
					const char *buf, size_t	count)
{
	struct driver_data *dd = dev_get_drvdata(dev);
	int	rc;
	unsigned long mode;

	rc = strict_strtoul(buf, 10, &mode);
	if (rc)
		return rc;

	mutex_lock(&dd->mutex);
	if (dd->mode[BMA250_MOTION_TAP]	!= mode)
		rc = bma250_motion_mode(dd,	BMA250_MOTION_TAP, (int)mode);
	mutex_unlock(&dd->mutex);

	return rc ?	rc : count;
}

static ssize_t bma250_tap_param_show(struct	device *dev,
					struct device_attribute	*attr,
					char *buf)
{
	struct driver_data *dd = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE,	"%d	%d %d %d\n",
		!!(dd->tap_duration	& BMA250_TAP_SHOCK_MASK),
		!!(dd->tap_duration	& BMA250_TAP_QUIET_MASK),
		dd->tap_duration & BMA250_TAP_DUR_MASK,
		dd->tap_threshold);
}

static ssize_t bma250_tap_param_store(struct device	*dev,
					struct device_attribute	*attr,
					const char *buf, size_t	count)
{
	struct driver_data *dd = dev_get_drvdata(dev);
	int	rc;
	unsigned int shock,	quiet, dur,	thr;

	rc = sscanf(buf, "%10u %10u	%10u %10u",
			&shock,	&quiet,	&dur, &thr);
	if ((rc	!= 4) ||
		(dur & ~BMA250_TAP_DUR_MASK) ||
		(thr & ~BMA250_TAP_THR_MASK))
		return -EINVAL;

	mutex_lock(&dd->mutex);

	dd->tap_duration = dur |
		(quiet ? BMA250_TAP_QUIET_20MS : BMA250_TAP_QUIET_30MS)	|
		(shock ? BMA250_TAP_SHOCK_75MS : BMA250_TAP_SHOCK_50MS);
	dd->tap_threshold =	thr;

	if (dd->power) {
		rc = bma250_ic_write(dd->ic_dev, BMA250_TAP_DUR_REG, dd->tap_duration);
		if (!rc)
			rc = bma250_ic_write(dd->ic_dev, BMA250_TAP_THR_REG, dd->tap_threshold);
	} else {
		rc = 0;
	}
	mutex_unlock(&dd->mutex);

	return rc ?	rc : count;
}

static ssize_t bma250_slope_mode_show(struct device	*dev,
					struct device_attribute	*attr,
					char *buf)
{
	struct driver_data *dd = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE,	"%d\n",	dd->mode[BMA250_MOTION_SLOPE]);
}

static ssize_t bma250_slope_mode_store(struct device *dev,
					struct device_attribute	*attr,
					const char *buf, size_t	count)
{
	struct driver_data *dd = dev_get_drvdata(dev);
	int	rc;
	unsigned long mode;

	rc = strict_strtoul(buf, 10, &mode);
	if (rc)
		return rc;

	mutex_lock(&dd->mutex);
	if (dd->mode[BMA250_MOTION_SLOPE] != mode)
		rc = bma250_motion_mode(dd,	BMA250_MOTION_SLOPE, (int)mode);
	mutex_unlock(&dd->mutex);

	return rc ?	rc : count;
}

static ssize_t bma250_slope_param_show(struct device *dev,
					struct device_attribute	*attr,
					char *buf)
{
	struct driver_data *dd = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE,	"%d	%d\n",
		dd->slope_duration,	dd->slope_threshold);
}

static ssize_t bma250_slope_param_store(struct device *dev,
					struct device_attribute	*attr,
					const char *buf, size_t	count)
{
	struct driver_data *dd = dev_get_drvdata(dev);
	int	rc;
	unsigned int dur, thr;

	rc = sscanf(buf, "%10u %10u", &dur,	&thr);
	if ((rc	!= 2) ||
		(dur & ~BMA250_SLOPE_DUR_MASK) ||
		(thr & ~0xFF))
		return -EINVAL;

	mutex_lock(&dd->mutex);

	dd->slope_duration = dur;
	dd->slope_threshold	= thr;

	if (dd->power) {
		rc = bma250_ic_write(dd->ic_dev, BMA250_SLOPE_DUR_REG, dur);
		if (!rc)
			rc = bma250_ic_write(dd->ic_dev, BMA250_SLOPE_THR_REG, thr);
	} else {
		rc = 0;
	}
	mutex_unlock(&dd->mutex);

	return rc ?	rc : count;
}

static ssize_t bma250_low_g_mode_show(struct device	*dev,
					struct device_attribute	*attr,
					char *buf)
{
	struct driver_data *dd = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE,	"%d\n",	dd->mode[BMA250_MOTION_LOW_G]);
}

static ssize_t bma250_low_g_mode_store(struct device *dev,
					struct device_attribute	*attr,
					const char *buf, size_t	count)
{
	struct driver_data *dd = dev_get_drvdata(dev);
	int	rc;
	unsigned long mode;

	rc = strict_strtoul(buf, 10, &mode);
	if (rc)
		return rc;

	mutex_lock(&dd->mutex);
	if (dd->mode[BMA250_MOTION_LOW_G] != mode)
		rc = bma250_motion_mode(dd,	BMA250_MOTION_LOW_G, (int)mode);
	mutex_unlock(&dd->mutex);

	return rc ?	rc : count;
}

static ssize_t bma250_low_g_param_show(struct device *dev,
					struct device_attribute	*attr,
					char *buf)
{
	struct driver_data *dd = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE,	"%d	%d %d\n",
		dd->low_g_duration,	dd->low_g_threshold, dd->low_g_hysteresis);
}

static ssize_t bma250_low_g_param_store(struct device *dev,
					struct device_attribute	*attr,
					const char *buf, size_t	count)
{
	struct driver_data *dd = dev_get_drvdata(dev);
	int	rc;
	unsigned int dur, thr, hy;

	rc = sscanf(buf, "%10u %10u	%10u", &dur, &thr, &hy);
	if ((rc	!= 3) || (dur &	~0xFF) || (thr & ~0xFF)	|| (hy & ~BMA250_LOW_G_HY_MASK))
		return -EINVAL;

	mutex_lock(&dd->mutex);

	dd->low_g_duration = dur;
	dd->low_g_threshold	= thr;
	dd->low_g_hysteresis = hy;

	if (dd->power) {
		rc = bma250_ic_write(dd->ic_dev, BMA250_LOW_G_DUR_REG, dur);
		if (!rc)
			rc = bma250_ic_write(dd->ic_dev, BMA250_LOW_G_THR_REG, thr);
		if (!rc)
			rc = bma250_ic_write(dd->ic_dev, BMA250_G_MODE_REG,
					hy | dd->high_g_hysteresis);
	} else {
		rc = 0;
	}
	mutex_unlock(&dd->mutex);

	return rc ?	rc : count;
}

static ssize_t bma250_high_g_mode_show(struct device *dev,
					struct device_attribute	*attr,
					char *buf)
{
	struct driver_data *dd = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE,	"%d\n",	dd->mode[BMA250_MOTION_HIGH_G]);
}

static ssize_t bma250_high_g_mode_store(struct device *dev,
					struct device_attribute	*attr,
					const char *buf, size_t	count)
{
	struct driver_data *dd = dev_get_drvdata(dev);
	int	rc;
	unsigned long mode;

	rc = strict_strtoul(buf, 10, &mode);
	if (rc)
		return rc;

	mutex_lock(&dd->mutex);
	if (dd->mode[BMA250_MOTION_HIGH_G] != mode)
		rc = bma250_motion_mode(dd,	BMA250_MOTION_HIGH_G, (int)mode);
	mutex_unlock(&dd->mutex);

	return rc ?	rc : count;
}

static ssize_t bma250_high_g_param_show(struct device *dev,
					struct device_attribute	*attr,
					char *buf)
{
	struct driver_data *dd = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE,	"%d	%d %d\n",
		dd->high_g_duration, dd->high_g_threshold, dd->high_g_hysteresis);
}

static ssize_t bma250_high_g_param_store(struct	device *dev,
					struct device_attribute	*attr,
					const char *buf, size_t	count)
{
	struct driver_data *dd = dev_get_drvdata(dev);
	int	rc;
	unsigned int dur, thr, hy;

	rc = sscanf(buf, "%10u %10u	%10u", &dur, &thr, &hy);
	if ((rc	!= 3) || (dur &	~0xFF) || (thr & ~0xFF)	|| (hy & ~BMA250_HIGH_G_HY_MASK))
		return -EINVAL;

	mutex_lock(&dd->mutex);

	dd->high_g_duration	= dur;
	dd->high_g_threshold = thr;
	dd->high_g_hysteresis =	hy;

	if (dd->power) {
		rc = bma250_ic_write(dd->ic_dev, BMA250_HIGH_G_DUR_REG,	dur);
		if (!rc)
			rc = bma250_ic_write(dd->ic_dev, BMA250_HIGH_G_THR_REG,	thr);
		if (!rc)
			rc = bma250_ic_write(dd->ic_dev, BMA250_G_MODE_REG,
					hy | dd->low_g_hysteresis);
	} else {
		rc = 0;
	}
	mutex_unlock(&dd->mutex);

	return rc ?	rc : count;
}

static ssize_t bma250_flat_param_show(struct device	*dev,
					struct device_attribute	*attr,
					char *buf)
{
	struct driver_data *dd = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE,	"%d	%d\n", dd->flat_theta, dd->flat_hold);
}

static ssize_t bma250_flat_param_store(struct device *dev,
					struct device_attribute	*attr,
					const char *buf, size_t	count)
{
	struct driver_data *dd = dev_get_drvdata(dev);
	int	rc;
	unsigned int flat_theta, flat_hold;

	rc = sscanf(buf, "%10u %10u", &flat_theta, &flat_hold);
	if ((rc	!= 2) ||
		(flat_theta	& ~BMA250_FLAT_THETA_MASK) ||
		(flat_hold & ~BMA250_FLAT_HOLD_MASK))
		return -EINVAL;

	dd->flat_theta = flat_theta;
	dd->flat_hold =	flat_hold;

	mutex_lock(&dd->mutex);
	if (dd->power) {
		rc = bma250_ic_write(dd->ic_dev, BMA250_FLAT_THETA_REG,	dd->flat_theta);
		if (!rc)
			rc = bma250_ic_write(dd->ic_dev, BMA250_FLAT_HOLD_REG, dd->flat_hold);
	} else {
		rc = 0;
	}
	mutex_unlock(&dd->mutex);

	return rc ?	rc : count;
}

static struct device_attribute attributes[]	= {
	__ATTR(tap_mode, 0644, bma250_tap_mode_show, bma250_tap_mode_store),
	__ATTR(tap_param, 0644,	bma250_tap_param_show, bma250_tap_param_store),
	__ATTR(slope_mode, 0644, bma250_slope_mode_show, bma250_slope_mode_store),
	__ATTR(slope_param,	0644, bma250_slope_param_show, bma250_slope_param_store),
	__ATTR(low_g_mode, 0644, bma250_low_g_mode_show, bma250_low_g_mode_store),
	__ATTR(low_g_param,	0644, bma250_low_g_param_show, bma250_low_g_param_store),
	__ATTR(high_g_mode,	0644, bma250_high_g_mode_show, bma250_high_g_mode_store),
	__ATTR(high_g_param, 0644, bma250_high_g_param_show, bma250_high_g_param_store),
	__ATTR(flat_param, 0644, bma250_flat_param_show, bma250_flat_param_store),
};

static int add_sysfs_interfaces(struct device *dev)
{
	int	i;

	for	(i = 0;	i <	ARRAY_SIZE(attributes);	i++)
		if (device_create_file(dev,	attributes + i))
			goto undo;
	return 0;
undo:
	for	(; i >=	0 ;	i--)
		device_remove_file(dev,	attributes + i);
	dev_err(dev, "%s: failed to	create sysfs interface\n", __func__);
	return -ENODEV;
}

static void	remove_sysfs_interfaces(struct device *dev)
{
	int	i;

	for	(i = 0;	i <	ARRAY_SIZE(attributes);	i++)
		device_remove_file(dev,	attributes + i);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void	bma250_motion_early_suspend(struct early_suspend *handler)
{
	struct driver_data *dd = container_of(handler,
			struct driver_data,	early_suspend);
	int	i;

	pr_info(BMA250_MOTION_NAME": early suspend\n");

	mutex_lock(&dd->mutex);
	dd->suspend	= 1;
	for	(i = 0;	i <	BMA250_MOTION_COUNT; i++) {
		if (((dd->mode[i] &	BMA250_EVENT_MASK) == BMA250_EVENT_POWER_DOWN) ||
			((dd->mode[i] &	BMA250_EVENT_MASK) == BMA250_EVENT_POWER_UP))
			bma250_motion_mode(dd, i, dd->mode[i]);
	}
	mutex_unlock(&dd->mutex);
}

static void	bma250_motion_late_resume(struct early_suspend *handler)
{
	struct driver_data *dd = container_of(handler,
			struct driver_data,	early_suspend);
	int	i;

	pr_info(BMA250_MOTION_NAME": late resume\n");

	mutex_lock(&dd->mutex);
	dd->suspend	= 0;
	for	(i = 0;	i <	BMA250_MOTION_COUNT; i++) {
		if (((dd->mode[i] &	BMA250_EVENT_MASK) == BMA250_EVENT_POWER_DOWN) ||
			((dd->mode[i] &	BMA250_EVENT_MASK) == BMA250_EVENT_POWER_UP))
			bma250_motion_mode(dd, i, dd->mode[i]);
	}
	mutex_unlock(&dd->mutex);
}
#endif

#if	defined(CONFIG_PM)
int	bma250_motion_suspend(void *dev)
{
	struct driver_data *dd = (struct driver_data *)dev;

	pr_info(BMA250_MOTION_NAME": suspend\n");

	mutex_lock(&dd->mutex);
	if (dd->power) {
		disable_irq(dd->ic_dev->irq);
		if (device_may_wakeup(&dd->ic_dev->dev))
			enable_irq_wake(dd->ic_dev->irq);
	}
	mutex_unlock(&dd->mutex);

	return 0;
}

int	bma250_motion_resume(void *dev)
{
	struct driver_data *dd = (struct driver_data *)dev;

	pr_info(BMA250_MOTION_NAME": resume\n");

	mutex_lock(&dd->mutex);
	if (dd->power) {
		if (device_may_wakeup(&dd->ic_dev->dev))
			disable_irq_wake(dd->ic_dev->irq);
		enable_irq(dd->ic_dev->irq);
	}
	mutex_unlock(&dd->mutex);

	return 0;
}
#endif /* CONFIG_PM	*/

void * __devinit bma250_motion_probe(struct	i2c_client *ic_dev)
{
	struct driver_data *dd;
	int					rc,	i;
	struct bma250_platform_data	*pdata = ic_dev->dev.platform_data;

	if (!pdata || !pdata->power_mode ||	!pdata->hw_config ||
			!pdata->setup || !pdata->teardown)
		return NULL;

	if (!pdata->check_sleep_status && !pdata->vote_sleep_status)
		return NULL;

	if (!ic_dev->irq)
		return NULL;

	dd = kzalloc(sizeof(struct driver_data), GFP_KERNEL);
	if (!dd)
		return NULL;

	dd->ic_dev = ic_dev;
	dd->pdata =	pdata;

	for	(i = 0;	i <	BMA250_MOTION_COUNT; i++) {
		dd->gap_work[i].dd = dd;
		dd->gap_work[i].motion = i;
		INIT_DELAYED_WORK(&dd->gap_work[i].work, bma250_motion_gap_work);
	}
	mutex_init(&dd->mutex);
	dd->tap_duration = BMA250_TAP_DURATION_DEFAULT;
	dd->tap_threshold =	BMA250_TAP_THRESHOLD_DEFAULT;
	dd->slope_duration = BMA250_SLOPE_DURATION_DEFAULT;
	dd->slope_threshold	= BMA250_SLOPE_THRESHOLD_DEFAULT;
	dd->low_g_duration = BMA250_LOW_G_DURATION_DEFAULT;
	dd->low_g_threshold	= BMA250_LOW_G_THRESHOLD_DEFAULT;
	dd->low_g_hysteresis = BMA250_LOW_G_HYSTERESIS_DEFAULT;
	dd->high_g_duration	= BMA250_HIGH_G_DURATION_DEFAULT;
	dd->high_g_threshold = BMA250_HIGH_G_THRESHOLD_DEFAULT;
	dd->high_g_hysteresis =	BMA250_HIGH_G_HYSTERESIS_DEFAULT;
	dd->flat_theta = BMA250_FLAT_THETA_DEFAULT;
	dd->flat_hold =	BMA250_FLAT_HOLD_DEFAULT;

	dd->ip_dev = input_allocate_device();
	if (!dd->ip_dev) {
		rc = -ENOMEM;
		goto probe_err_reg;
	}
	input_set_drvdata(dd->ip_dev, dd);
	dd->ip_dev->name	   = BMA250_MOTION_NAME;
	dd->ip_dev->id.vendor  = BMA250_MOTION_VENDORID;
	dd->ip_dev->id.product = 1;
	dd->ip_dev->id.version = 1;
	input_set_capability(dd->ip_dev, EV_KEY, KEY_POWER);
	input_set_capability(dd->ip_dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dd->ip_dev, ABS_MISC, 0, 0xFF,	0, 0);

	rc = input_register_device(dd->ip_dev);
	if (rc)	{
		input_free_device(dd->ip_dev);
		goto probe_err_reg;
	}

	rc = add_sysfs_interfaces(&dd->ip_dev->dev);
	if (rc)
		goto probe_err_sysfs;

#ifdef CONFIG_HAS_EARLYSUSPEND
	dd->early_suspend.suspend =	bma250_motion_early_suspend;
	dd->early_suspend.resume = bma250_motion_late_resume;
	register_early_suspend(&dd->early_suspend);
#endif

	device_init_wakeup(&dd->ic_dev->dev, true);

	return dd;

	remove_sysfs_interfaces(&dd->ip_dev->dev);
probe_err_sysfs:
	input_unregister_device(dd->ip_dev);
probe_err_reg:
	kfree(dd);
	return NULL;
}

int	__devexit bma250_motion_remove(void	*dev)
{
	struct driver_data *dd = (struct driver_data *)dev;
	int					rc,	i;

	device_init_wakeup(&dd->ic_dev->dev, false);

	for	(i = 0;	i <	BMA250_MOTION_COUNT; i++) {
		rc = bma250_motion_mode(dd,	i, 0);
		if (rc)
			pr_err(BMA250_MOTION_NAME":	remove failed with error %d\n",	rc);
	}
	dd->pdata->power_mode(0);

	remove_sysfs_interfaces(&dd->ip_dev->dev);
	input_unregister_device(dd->ip_dev);
	kfree(dd);
	return 0;
}

