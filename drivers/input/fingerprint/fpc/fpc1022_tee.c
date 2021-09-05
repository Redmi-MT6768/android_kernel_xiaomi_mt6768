/*
 * FPC1022 Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, enabling and disabling of platform
 * clocks, controlling GPIOs such as SPI chip select, sensor reset line, sensor
 * IRQ line, MISO and MOSI lines.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node. This makes it possible for a user
 * space process to poll the input node and receive IRQ events easily. Usually
 * this node is available under /dev/input/eventX where 'X' is a number given by
 * the event system. A user space process will need to traverse all the event
 * nodes and ask for its parent's name (through EVIOCGNAME) which should match
 * the value in device tree named input-device-name.
 *
 * This driver will NOT send any SPI commands to the sensor it only controls the
 * electrical parts.
 *
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 * Copyright (C) 2021 XiaoMi, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <gf_spi_tee.h>
#ifndef CONFIG_SPI_MT65XX
#include "mtk_spi.h"
#include "mtk_spi_hal.h"
#endif

/* begin modify for unlock speed */
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#else
#include <linux/notifier.h>
#endif
#include <linux/fb.h>
#include "../../../misc/mediatek/video/include/mtkfb.h"
/* end modify for unlock speed */

#ifndef CONFIG_SPI_MT65XX
#include "mtk_gpio.h"
#include "mach/gpio_const.h"
#endif

#ifdef CONFIG_HQ_SYSFS_SUPPORT
#include <linux/hqsysfs.h>
#endif
#include  <linux/regulator/consumer.h>

/* begin modify for unlock speed */
#include "../../../misc/mediatek/base/power/include/mtk_ppm_api.h"
#include <mt-plat/cpu_ctrl.h>
#include <linux/pm_qos.h>
#include <helio-dvfsrc-opp.h>

#define BSP_CERVINO_CLUSTER_NUMBERS  2
/* end modify for unlock speed */

#define FPC1022_RESET_LOW_US 5000
#define FPC1022_RESET_HIGH1_US 100
#define FPC1022_RESET_HIGH2_US 5000

#define FPC_IRQ_DEV_NAME         "fpc_irq"
#define FPC_TTW_HOLD_TIME 2000

#define     FPC102X_REG_HWID      252
#define FPC1022_CHIP 0x1000
#define FPC1022_CHIP_MASK_SENSOR_TYPE 0xff00

#define GPIO_GET(pin) __gpio_get_value(pin)	//get input pin value

#define GPIOIRQ 2		//XPT

/* begin modify for unlock speed */
#define FP_UNLOCK_REJECTION_TIMEOUT 1500
/* end modify for unlock speed */

struct regulator *regu_buck;

/* begin modify for unlock speed */
struct ppm_limit_data fingerprint_freq_to_set[BSP_CERVINO_CLUSTER_NUMBERS];
struct ppm_limit_data fingerprint_freq_to_release[BSP_CERVINO_CLUSTER_NUMBERS];
static struct pm_qos_request fpc_fingerprint_ddr_req;
/* end modify for unlock speed */

#ifdef CONFIG_SPI_MT65XX
extern void mt_spi_enable_master_clk(struct spi_device *spidev);
extern void mt_spi_disable_master_clk(struct spi_device *spidev);
#endif

/********xinan_bp for dual_TA begain *********/
#include "teei_fp.h"
#include "tee_client_api.h"
struct TEEC_UUID uuid_ta_fpc = { 0x7778c03f, 0xc30c, 0x4dd0,
	{0xa3, 0x19, 0xea, 0x29, 0x64, 0x3d, 0x4d, 0x4b}
};

/********xinan_bp for dual_TA end*********/

struct fpc1022_data {
	struct device *dev;
	struct platform_device *pldev;
	struct spi_device *spi;
	int irq_gpio;
	int irq_num;
	struct pinctrl *pinctrl;
	struct pinctrl_state *st_irq;	//xpt
	struct pinctrl_state *st_rst_l;
	struct pinctrl_state *st_rst_h;
	struct pinctrl_state *st_spi_cs_l;
	struct pinctrl_state *st_spi_cs_h;

	struct input_dev *idev;
	char idev_name[32];
	int event_type;
	int event_code;
	struct mutex lock;
	bool prepared;
	bool wakeup_enabled;
	struct wakeup_source *ttw_wl;

/* begin modify for unlock speed */
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#else
	struct notifier_block fb_notifier;
#endif
	bool fb_black;
	bool wait_finger_down;
	struct work_struct work;
/* end modify for unlock speed */

};
int fp_idx_ic_exist;

extern bool goodix_fp_exist;
extern struct spi_device *spi_fingerprint;
bool fpc1022_fp_exist;

static struct fpc1022_data *fpc1022;

#ifndef CONFIG_SPI_MT65XX
static const struct mt_chip_conf spi_mcc = {
	.setuptime = 20,
	.holdtime = 20,
	.high_time = 50,	/* 1MHz */
	.low_time = 50,
	.cs_idletime = 5,
	.ulthgh_thrsh = 0,

	.cpol = SPI_CPOL_0,
	.cpha = SPI_CPHA_0,

	.rx_mlsb = SPI_MSB,
	.tx_mlsb = SPI_MSB,

	.tx_endian = SPI_LENDIAN,
	.rx_endian = SPI_LENDIAN,

	.com_mod = FIFO_TRANSFER,
	/* .com_mod = DMA_TRANSFER, */

	.pause = 1,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
#endif

#ifdef CONFIG_SPI_MT65XX
u32 spi_speed = 1 * 1000000;
#endif

static void fpc1022_get_irqNum(struct fpc1022_data *fpc1022)
{
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mediatek,fpc1022_irq");

	if (node) {

		fpc1022->irq_num = irq_of_parse_and_map(node, 0);	//xpt
		fpc1022->irq_gpio = of_get_named_gpio(node, "fpc,gpio_irq", 0);
	}
}

static int hw_reset(struct fpc1022_data *fpc1022)
{
	struct device *dev = fpc1022->dev;

	pinctrl_select_state(fpc1022->pinctrl, fpc1022->st_rst_h);
	usleep_range(FPC1022_RESET_HIGH1_US, FPC1022_RESET_HIGH1_US + 100);

	pinctrl_select_state(fpc1022->pinctrl, fpc1022->st_rst_l);
	usleep_range(FPC1022_RESET_LOW_US, FPC1022_RESET_LOW_US + 100);

	pinctrl_select_state(fpc1022->pinctrl, fpc1022->st_rst_h);
	usleep_range(FPC1022_RESET_HIGH1_US, FPC1022_RESET_HIGH1_US + 100);

	return 0;
}

static ssize_t hw_reset_set(struct device *dev,
			    struct device_attribute *attr, const char *buf,
			    size_t count)
{
	int ret;
	struct fpc1022_data *fpc1022 = dev_get_drvdata(dev);

	if (!strncmp(buf, "reset", strlen("reset")))
		ret = hw_reset(fpc1022);
	else
		return -EINVAL;
	return ret ? ret : count;
}

static DEVICE_ATTR(hw_reset, 0200, NULL, hw_reset_set);

/**
* sysfs node for controlling whether the driver is allowed
* to wake up the platform on interrupt.
*/
static ssize_t wakeup_enable_set(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct fpc1022_data *fpc1022 = dev_get_drvdata(dev);

	if (!strncmp(buf, "enable", strlen("enable"))) {
		fpc1022->wakeup_enabled = true;
		smp_wmb();
	} else if (!strncmp(buf, "disable", strlen("disable"))) {
		fpc1022->wakeup_enabled = false;
		smp_wmb();
	} else
		return -EINVAL;

	return count;
}

static DEVICE_ATTR(wakeup_enable, 0200, NULL, wakeup_enable_set);

/**
* sysfs node for sending event to make the system interactive,
* i.e. waking up
*/
static ssize_t do_wakeup_set(struct device *dev,
			     struct device_attribute *attr, const char *buf,
			     size_t count)
{
	struct fpc1022_data *fpc1022 = dev_get_drvdata(dev);

	if (count > 0) {
		/* Sending power key event creates a toggling
		   effect that may be desired. It could be
		   replaced by another event such as KEY_WAKEUP. */
		input_report_key(fpc1022->idev, KEY_POWER, 1);
		input_report_key(fpc1022->idev, KEY_POWER, 0);
		input_sync(fpc1022->idev);
	} else {
		return -EINVAL;
	}

	return count;
}

static DEVICE_ATTR(do_wakeup, 0200, NULL, do_wakeup_set);

static ssize_t clk_enable_set(struct device *dev,
			      struct device_attribute *attr, const char *buf,
			      size_t count)
{
	struct fpc1022_data *fpc1022 = dev_get_drvdata(dev);

	if (fpc1022->spi) {
		//update spi clk
		if (*buf == 49)
			mt_spi_enable_master_clk(fpc1022->spi);

		if (*buf == 48)
			mt_spi_disable_master_clk(fpc1022->spi);
			return 1;
	} else
		return 0;
}

static DEVICE_ATTR(clk_enable, 0200, NULL, clk_enable_set);

/**
* sysf node to check the interrupt status of the sensor, the interrupt
* handler should perform sysf_notify to allow userland to poll the node.
*/
static ssize_t irq_get(struct device *device,
		       struct device_attribute *attribute, char *buffer)
{
	struct fpc1022_data *fpc1022 = dev_get_drvdata(device);
	int irq = __gpio_get_value(fpc1022->irq_gpio);
	return scnprintf(buffer, PAGE_SIZE, "%i\n", irq);
}

/**
* writing to the irq node will just drop a printk message
* and return success, used for latency measurement.
*/
static ssize_t irq_ack(struct device *device,
		       struct device_attribute *attribute,
		       const char *buffer, size_t count)
{
	struct fpc1022_data *fpc1022 = dev_get_drvdata(device);
	return count;
}

static DEVICE_ATTR(irq, 0600, irq_get, irq_ack);

static ssize_t fpc_ic_is_exist(struct device *device,
			       struct device_attribute *attribute, char *buffer)
{
	int fpc_exist = 0;
	if (fpc1022_fp_exist) {
		fpc_exist = 1;
	} else {
		fpc_exist = 0;
	}
	return scnprintf(buffer, PAGE_SIZE, "%i\n", fpc_exist);
}

static DEVICE_ATTR(fpid_get, 0600, fpc_ic_is_exist, NULL);

/* begin modify for unlock speed */
static ssize_t fingerdown_wait_set(struct device *device,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
		struct fpc1022_data *fpc1022 = dev_get_drvdata(device);

		dev_dbg(fpc1022->dev, "%s\n", __func__);
		if (!strncmp(buf, "enable", strlen("enable"))) {
			printk("fingerdown_wait_set enable\n");
			fpc1022->wait_finger_down = true;
		dev_dbg(fpc1022->dev, "%s set wait_finger_down true\n", __func__);
		} else if (!strncmp(buf, "disable", strlen("disable"))) {
			fpc1022->wait_finger_down = false;
		dev_dbg(fpc1022->dev, "%s\n set wait_finger_down false", __func__);
		} else {
		dev_dbg(fpc1022->dev, "%s set wait_finger_down error\n", __func__);
			return -EINVAL;
		}

		return count;
}

static DEVICE_ATTR(fingerdown_wait, 0200, NULL, fingerdown_wait_set);
/* end modify for unlock speed */


static struct attribute *attributes[] = {
	&dev_attr_hw_reset.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_do_wakeup.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_irq.attr,
	&dev_attr_fpid_get.attr,

	/* begin modify for unlock speed */
	&dev_attr_fingerdown_wait.attr,
	/* end modify for unlock speed */

	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

/* begin modify for unlock speed */
static int fpc_fingerprint_freq_set(void)
{
	int i, cluster_num;

	printk("fpc_fingerprint_freq_set\n");
	cluster_num = arch_get_nr_clusters();
	if (cluster_num > BSP_CERVINO_CLUSTER_NUMBERS)
		cluster_num = BSP_CERVINO_CLUSTER_NUMBERS;

	for (i = 0; i < BSP_CERVINO_CLUSTER_NUMBERS; i++) {
		fingerprint_freq_to_set[i].min = 2001000;
		fingerprint_freq_to_set[i].max = -1;
	}

	if (cluster_num > 0) {
		update_userlimit_cpu_freq(CPU_KIR_FINGERPRINT, cluster_num,
					fingerprint_freq_to_set);
		return 0;
	}

	return -1;
}


static int fpc_fingerprint_freq_release(void)
{
	int i, cluster_num;

	printk("fpc_fingerprint_freq_release\n");
	cluster_num = arch_get_nr_clusters();
	if (cluster_num > BSP_CERVINO_CLUSTER_NUMBERS)
		cluster_num = BSP_CERVINO_CLUSTER_NUMBERS;

	for (i = 0; i < BSP_CERVINO_CLUSTER_NUMBERS; i++) {
		fingerprint_freq_to_release[i].min = -1;
		fingerprint_freq_to_release[i].max = -1;
	}

	if (cluster_num > 0) {
		update_userlimit_cpu_freq(CPU_KIR_FINGERPRINT, cluster_num,
					fingerprint_freq_to_release);
		return 0;
	}

	return -1;
}

static int fpc_fingerprint_vcorefs_hold(void)
{
	printk("fpc_fingerprint_vcorefs_hold\n");
	pm_qos_update_request(&fpc_fingerprint_ddr_req, DDR_OPP_0);
	return 0;
}

static int fpc_fingerprint_vcorefs_release(void)
{
	printk("fpc_fingerprint_vcorefs_release\n");
	pm_qos_update_request(&fpc_fingerprint_ddr_req, DDR_OPP_UNREQ);
	return 0;
}

extern int mdss_prim_panel_fb_unblank(int timeout);
static void notification_work(struct work_struct *work)
{
	printk("notification_work fpc unblank start\n");
	fpc_fingerprint_freq_set();
	fpc_fingerprint_vcorefs_hold();
	mdss_prim_panel_fb_unblank(FP_UNLOCK_REJECTION_TIMEOUT);
	fpc_fingerprint_freq_release();
	fpc_fingerprint_vcorefs_release();
	printk("notification_work fpc unblank end\n");
}
/* end modify for unlock speed */

static irqreturn_t fpc1022_irq_handler(int irq, void *handle)
{
	struct fpc1022_data *fpc1022 = handle;

	/* Make sure 'wakeup_enabled' is updated before using it
	 ** since this is interrupt context (other thread...) */
	printk("fpc1022_irq_handler");
	smp_rmb();

	/* if (fpc1022->wakeup_enabled) { */
	__pm_wakeup_event(fpc1022->ttw_wl, msecs_to_jiffies(FPC_TTW_HOLD_TIME));
	/* } */

	/* begin modify for unlock speed */
	printk("%s fastScreenOn wait_finger_down = %d, fb_black = %d \n", __func__,
			fpc1022->wait_finger_down, fpc1022->fb_black);
	if (fpc1022->wait_finger_down && fpc1022->fb_black) {
			fpc1022->wait_finger_down = false;
			schedule_work(&fpc1022->work);
	}
	/* end modify for unlock speed */

	sysfs_notify(&fpc1022->dev->kobj, NULL, dev_attr_irq.attr.name);

	return IRQ_HANDLED;
}

/* begin modify for unlock speed */
#ifdef CONFIG_HAS_EARLYSUSPEND
static void fpc_early_suspend(struct early_suspend *handler)
{
	struct fpc1022_data *fpc1022 = container_of(handler,
		struct fpc1022_data, early_suspend);
	printk("%s enter\n", __func__);

	fpc1022->fb_black = true;
}

static void fpc_late_resume(struct early_suspend *handler)
{
	struct fpc1022_data *fpc1022 = container_of(handler,
		struct fpc1022_data, early_suspend);
	printk("%s enter\n", __func__);

	fpc1022->fb_black = false;
}
#else

static int fpc_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fpc1022_data *fpc1022 = container_of(self, struct fpc1022_data,
		fb_notifier);
	struct fb_event *evdata = data;
	unsigned int blank;
	int retval = 0;

	if (!fpc1022)
		return 0;

	if (event != FB_EVENT_BLANK)
		return 0;

	printk("[info] %s value = %d\n", __func__, (int)event);

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = *(int *)(evdata->data);
		switch (blank) {
		case FB_BLANK_POWERDOWN:
			printk("%s lcd off notify\n", __func__);
			fpc1022->fb_black = true;
			break;
		case FB_BLANK_UNBLANK:
			printk("%s lcd on notify\n", __func__);
			fpc1022->fb_black = false;
			break;
		default:
			printk("%s other notifier, ignore\n", __func__);
			break;
	}
}
	return retval;
}
#endif
/* end modify for unlock speed */

static int fpc1022_platform_probe(struct platform_device *pldev)
{
	int ret = 0;
	int irqf;
	u32 val;
	const char *idev_name;
	struct device *dev = &pldev->dev;
	struct device_node *np = dev->of_node;

	if (!np) {
		ret = -EINVAL;
		goto err_no_of_node;
	}

	fpc1022 = devm_kzalloc(dev, sizeof(*fpc1022), GFP_KERNEL);
	if (!fpc1022) {
		dev_err(dev,
			"failed to allocate memory for struct fpc1022_data\n");
		ret = -ENOMEM;
		goto err_fpc1022_malloc;
	}

	fpc1022->dev = dev;
	fpc1022->spi = spi_fingerprint;
	dev_set_drvdata(dev, fpc1022);
	fpc1022->pldev = pldev;

	fpc1022->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(fpc1022->pinctrl)) {
		ret = PTR_ERR(fpc1022->pinctrl);
		goto err_pinctrl_get;
	}

	fpc1022->st_irq = pinctrl_lookup_state(fpc1022->pinctrl, "default");
	if (IS_ERR(fpc1022->st_irq)) {
		ret = PTR_ERR(fpc1022->st_irq);
	}			/////////////////////////////////xpt

	fpc1022->st_rst_h =
	    pinctrl_lookup_state(fpc1022->pinctrl, "reset_high");
	if (IS_ERR(fpc1022->st_rst_h)) {
		ret = PTR_ERR(fpc1022->st_rst_h);
		goto err_lookup_state;
	}

	fpc1022->st_rst_l = pinctrl_lookup_state(fpc1022->pinctrl, "reset_low");
	if (IS_ERR(fpc1022->st_rst_l)) {
		ret = PTR_ERR(fpc1022->st_rst_l);
		goto err_lookup_state;
	}

	fpc1022->st_spi_cs_h = pinctrl_lookup_state(fpc1022->pinctrl, "spi_cs_high");
	if (IS_ERR(fpc1022->st_spi_cs_h)) {
		ret = PTR_ERR(fpc1022->st_spi_cs_h);
		goto err_lookup_state;
	}
	fpc1022->st_spi_cs_l = pinctrl_lookup_state(fpc1022->pinctrl, "spi_cs_low");
	if (IS_ERR(fpc1022->st_spi_cs_l)) {
		ret = PTR_ERR(fpc1022->st_spi_cs_l);
		goto err_lookup_state;
	}

	mdelay(10);
	//set cs from gpio mode to spi mode
	pinctrl_select_state(fpc1022->pinctrl, fpc1022->st_spi_cs_h);

	fpc1022_get_irqNum(fpc1022);

	ret = of_property_read_u32(np, "fpc,event-type", &val);
	fpc1022->event_type = ret < 0 ? EV_MSC : val;

	ret = of_property_read_u32(np, "fpc,event-code", &val);
	fpc1022->event_code = ret < 0 ? MSC_SCAN : val;

	fpc1022->idev = devm_input_allocate_device(dev);
	if (!fpc1022->idev) {
		dev_err(dev, "failed to allocate input device\n");
		ret = -ENOMEM;
		goto err_input_malloc;
	}

	input_set_capability(fpc1022->idev, fpc1022->event_type,
			     fpc1022->event_code);

	if (!of_property_read_string(np, "input-device-name", &idev_name)) {
		fpc1022->idev->name = idev_name;
	} else {
		snprintf(fpc1022->idev_name, sizeof(fpc1022->idev_name),
			 "fpc_irq@%s", dev_name(dev));
		fpc1022->idev->name = fpc1022->idev_name;
	}

	/* Also register the key for wake up */
	set_bit(EV_KEY, fpc1022->idev->evbit);
	set_bit(EV_PWR, fpc1022->idev->evbit);
	set_bit(KEY_WAKEUP, fpc1022->idev->keybit);
	set_bit(KEY_POWER, fpc1022->idev->keybit);
	ret = input_register_device(fpc1022->idev);
	fpc1022->wakeup_enabled = false;

	if (ret) {
		goto err_register_input;
	}

	irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
	if (of_property_read_bool(dev->of_node, "fpc,enable-wakeup")) {
		irqf |= IRQF_NO_SUSPEND;
		device_init_wakeup(dev, 1);
	}

	mutex_init(&fpc1022->lock);

	ret = devm_request_threaded_irq(dev, fpc1022->irq_num,
					NULL, fpc1022_irq_handler, irqf,
					dev_name(dev), fpc1022);

	if (ret)
		goto err_request_irq;

	/* Request that the interrupt should be wakeable */
	enable_irq_wake(fpc1022->irq_num);
	fpc1022->ttw_wl = wakeup_source_register(dev, "fpc_ttw_wl");
	/* begin modify for unlock speed */
	pm_qos_add_request(&fpc_fingerprint_ddr_req, PM_QOS_DDR_OPP, PM_QOS_DDR_OPP_DEFAULT_VALUE);
	/* end modify for unlock speed */


/* begin modify for unlock speed */
#if defined(CONFIG_HAS_EARLYSUSPEND)
	dev_info(dev, "%s : register_early_suspend\n", __func__);
	fpc1022->early_suspend.level = (EARLY_SUSPEND_LEVEL_DISABLE_FB - 1);
	fpc1022->early_suspend.suspend = fpc_early_suspend,
	fpc1022->early_suspend.resume = fpc_late_resume,
	register_early_suspend(&fpc1022->early_suspend);
#else
	/* register screen on/off callback */
	dev_info(dev, "%s : register_fpc_fb_notifier_callback\n", __func__);
	fpc1022->fb_notifier.notifier_call = fpc_fb_notifier_callback;
	fb_register_client(&fpc1022->fb_notifier);
#endif
	fpc1022->fb_black = false;
	fpc1022->wait_finger_down = false;
	INIT_WORK(&fpc1022->work, notification_work);
/* end modify for unlock speed */

	ret = sysfs_create_group(&dev->kobj, &attribute_group);
	if (ret) {
		dev_err(dev, "could not create sysfs\n");
		goto err_create_sysfs;
	}

#ifdef CONFIG_HQ_SYSFS_SUPPORT
	dev_info(dev, "%s hq_regiser_hw_info\n", __func__);
	hq_regiser_hw_info(HWID_FP, "FPC");
#endif
	hw_reset(fpc1022);

	return ret;

err_create_sysfs:
	wakeup_source_unregister(fpc1022->ttw_wl);

/* begin modify for unlock speed */
#ifdef CONFIG_HAS_EARLYSUSPEND
		if (fpc1022->early_suspend.suspend)
			unregister_early_suspend(&fpc1022->early_suspend);
#else
		fb_unregister_client(&fpc1022->fb_notifier);
#endif
/* end modify for unlock speed */

err_request_irq:
	mutex_destroy(&fpc1022->lock);

err_register_input:
	input_unregister_device(fpc1022->idev);

err_input_malloc:
err_lookup_state:
err_pinctrl_get:
	devm_kfree(dev, fpc1022);

err_fpc1022_malloc:
err_no_of_node:

	return ret;
}

static int fpc1022_platform_remove(struct platform_device *pldev)
{
	struct device *dev = &pldev->dev;
	struct fpc1022_data *fpc1022 = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);

	sysfs_remove_group(&dev->kobj, &attribute_group);

	mutex_destroy(&fpc1022->lock);
	wakeup_source_unregister(fpc1022->ttw_wl);

/* begin modify for unlock speed */
#ifdef CONFIG_HAS_EARLYSUSPEND
		if (fpc1022->early_suspend.suspend)
			unregister_early_suspend(&fpc1022->early_suspend);
#else
		fb_unregister_client(&fpc1022->fb_notifier);
#endif
/* end modify for unlock speed */

	input_unregister_device(fpc1022->idev);
	devm_kfree(dev, fpc1022);

	return 0;
}

static struct of_device_id fpc1022_of_match[] = {
	{.compatible = "mediatek,fpc1022_irq",},
	{}
};

MODULE_DEVICE_TABLE(of, fpc1022_of_match);

static struct platform_driver fpc1022_driver = {
	.driver = {
		   .name = "fpc1022_irq",
		   .owner = THIS_MODULE,
		   .of_match_table = fpc1022_of_match,
		   },
	.probe = fpc1022_platform_probe,
	.remove = fpc1022_platform_remove
};

static int spi_read_hwid(struct spi_device *spi, u8 *rx_buf)
{
	int error;
	struct spi_message msg;
	struct spi_transfer *xfer;
	u8 tmp_buf[10] = { 0 };
	u8 addr = FPC102X_REG_HWID;

	xfer = kzalloc(sizeof(*xfer) * 2, GFP_KERNEL);
	if (xfer == NULL) {
		dev_err(&spi->dev, "%s, no memory for SPI transfer\n",
			__func__);
		return -ENOMEM;
	}

	spi_message_init(&msg);

	tmp_buf[0] = addr;
	xfer[0].tx_buf = tmp_buf;
	xfer[0].len = 1;
	xfer[0].delay_usecs = 5;

#ifdef CONFIG_SPI_MT65XX
	xfer[0].speed_hz = spi_speed;
#endif

	spi_message_add_tail(&xfer[0], &msg);

	xfer[1].tx_buf = tmp_buf + 2;
	xfer[1].rx_buf = tmp_buf + 4;
	xfer[1].len = 2;
	xfer[1].delay_usecs = 5;

#ifdef CONFIG_SPI_MT65XX
	xfer[1].speed_hz = spi_speed;
#endif

	spi_message_add_tail(&xfer[1], &msg);
	error = spi_sync(spi, &msg);

	memcpy(rx_buf, (tmp_buf + 4), 2);

	kfree(xfer);
	if (xfer != NULL)
		xfer = NULL;

	return 0;
}

static int check_hwid(struct spi_device *spi)
{
	int error = 0;
	u32 time_out = 0;
	u8 tmp_buf[2] = { 0 };
	u16 hardware_id;

	do {
		spi_read_hwid(spi, tmp_buf);

		time_out++;

		hardware_id = ((tmp_buf[0] << 8) | (tmp_buf[1]));

		if ((FPC1022_CHIP_MASK_SENSOR_TYPE & hardware_id) ==
		    FPC1022_CHIP)
			error = 0;
		else
			error = -1;

		if (!error) {
			return 0;
		}
	} while (time_out < 2);

	return -1;
}

MODULE_DEVICE_TABLE(of, fpc1022_spi_of_match);

static int __init fpc1022_init(void)
{
	int error = 0;

	if (goodix_fp_exist)
		return -EINVAL;

	if (0 != platform_driver_register(&fpc1022_driver)) {
		return -EINVAL;
	}

	//workaround to solve two spi device
	if (spi_fingerprint == NULL)
		pr_debug("%s Line:%d spi device is NULL,cannot spi transfer\n",
			  __func__, __LINE__);
	else {
		error = check_hwid(spi_fingerprint);

		if (error < 0) {
			return -EINVAL;
		}
		printk(KERN_INFO
			       "fpc %s, detected sensor, error=%d\n",
			       __func__, error);
		fpc1022_fp_exist = true;
		if (NULL == fpc1022) {
			printk(KERN_INFO
			       "fpc %s,fpc1022 is NULL\n",
			       __func__, error);

		} else {
			fpc1022->spi = spi_fingerprint;
		}
		printk(KERN_INFO
			       "fpc %s,before memcpy\n",
			       __func__, error);
		/********xinan_bp for dual_TA begain *********/
		memcpy(&uuid_fp, &uuid_ta_fpc, sizeof(struct TEEC_UUID));
		printk(KERN_INFO
			       "fpc %s, after memcpy\n",
			       __func__);
	/********xinan_bp for dual_TA end *********/

	}

	return 0;
}

static void __exit fpc1022_exit(void)
{
	platform_driver_unregister(&fpc1022_driver);
}

late_initcall(fpc1022_init);
module_exit(fpc1022_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksej Makarov");
MODULE_AUTHOR("Henrik Tillman <henrik.tillman@fingerprints.com>");
MODULE_DESCRIPTION("fpc1022 Fingerprint sensor device driver.");
