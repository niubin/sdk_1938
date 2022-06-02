/*
 * Driver for FTDCC010 Temperature-To-Digital Converter Controller
 *
 * Copyright (c) 2016 Faraday Technology Corporation
 *
 * B.C. Chen <bcchen@faraday-tech.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/thermal.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/of_platform.h>

#include "fttdcc010.h"

#define DRIVER_DESC     "FTDCC010 Temperature-To-Digital Converter Controller Driver"
#define DRIVER_VERSION  "28-Sep-2017"

#define NUM_TDC         1
#define SCANMODE        SCANMODE_CONT

#define HOT_LIMIT_TEMP_ON		(120)
#define HOT_LIMIT_TEMP_OFF		(117)
#define TDC_MAGIC_NUM_SIZE  4
#define TDC_SINGLE_GROUP    4
#define TDC_NUM_MAX         20
#define TDC_GROUP_MAX       5

static u32 tdc_group_num = 0;
static u32 cool_level = 0;
static int start_cooling[TDC_GROUP_MAX];
static int stop_cooling[TDC_GROUP_MAX];
static u32 frequency_adjust[TDC_GROUP_MAX];
static u32 cores_adjust[TDC_GROUP_MAX];
static int cur_freq = 0;
static u32 thermal_critical_trips = CRITICAL_THERMAL;

static void fttdcc010_force_update_temp(struct fttdcc010_thermal_priv *priv, int id)
{
	struct fttdcc010_thermal_zone *zone;
	struct device *dev = fttdcc010_thermal_priv_to_dev(priv);
	u32 val;
	
	val = readl(&priv->regs->ctrl);
	writel(val | TDC_EN, &priv->regs->ctrl);
	
	list_for_each_entry(zone, &priv->head, list) {
		if(zone->id != id)
			continue;
		
		if(!wait_for_completion_timeout(&zone->done, msecs_to_jiffies(1000)))
			dev_err(dev, "ftddcc010 conversion timeout\n");
	}
}

static int set_cooling(u32 frequency, u32 cores)
{
	cur_freq = cpufreq_update_trip(frequency);
	if(cur_freq >= 0) {
		switch(cores) {
			case 4:
			default:
				cpu_up(1);
				cpu_up(2);
				cpu_up(3);
				break;
			case 3:
				cpu_up(1);
				cpu_up(2);
				cpu_down(3);
				break;
			case 2:
				cpu_up(1);
				cpu_down(2);
				cpu_down(3);
				break;
			case 1:
				cpu_down(1);
				cpu_down(2);
				cpu_down(3);
				break;
		}
	}

	return cur_freq;
}

static int fttdcc010_get_temp(struct thermal_zone_device *thermal,
                              int *temp)
{
	struct fttdcc010_thermal_priv *priv = thermal->devdata;
	int val;
	u32 i;
	
	if(SCANMODE == SCANMODE_SGL)
		fttdcc010_force_update_temp(priv, 0);
	
	val = readl(&priv->regs->data[0]) & 0x3FF;
	
	*temp = (val - 376)/4;

	for(i = 1; i < tdc_group_num; i++) {
		/* stop cooling */
		if((cool_level == i) && (*temp < stop_cooling[i])) {
			cool_level--;
			set_cooling(frequency_adjust[cool_level], cores_adjust[cool_level]);
		}

		/* start cooling */
		if((cool_level < i) && (*temp >= start_cooling[i])) {
			cool_level = i;
			set_cooling(frequency_adjust[cool_level], cores_adjust[cool_level]);
		}
	}

	/* mise */
	if (cur_freq != frequency_adjust[cool_level]){
		set_cooling(frequency_adjust[cool_level], cores_adjust[cool_level]);
	}
	if((0 == cool_level) && (cur_freq != frequency_adjust[0])){
		//set_cooling(frequency_adjust[0], cores_adjust[0]);
	}

	return 0;
}

static int fttdcc010_get_mode(struct thermal_zone_device *thermal,
                              enum thermal_device_mode *mode)
{
	struct fttdcc010_thermal_priv *priv = thermal->devdata;
	u32 val;

	val = readl(&priv->regs->ctrl);
	if(val & TDC_EN)
		*mode = THERMAL_DEVICE_ENABLED;
	else
		*mode = THERMAL_DEVICE_DISABLED;
	
	return 0;
}

static int fttdcc010_set_mode(struct thermal_zone_device *thermal,
                              enum thermal_device_mode mode)
{
	struct fttdcc010_thermal_priv *priv = thermal->devdata;
	struct device *dev = fttdcc010_thermal_priv_to_dev(priv);
	u32 val;
	
	switch (mode) {
		case THERMAL_DEVICE_DISABLED:
			dev_info(dev, "Disable FTTDCC010 conversion function\n");
			val = readl(&priv->regs->ctrl);
			val &= ~TDC_EN;
			writel(val, &priv->regs->ctrl);
			break;
		case THERMAL_DEVICE_ENABLED:
			dev_info(dev, "Enable FTTDCC010 conversion function\n");
			val = readl(&priv->regs->ctrl);
			val |= TDC_EN;
			writel(val, &priv->regs->ctrl);
			break;
	}
	
	return 0;
}

static int fttdcc010_get_trip_type(struct thermal_zone_device *thermal,
                                   int trip, enum thermal_trip_type *type)
{
	struct fttdcc010_thermal_priv *priv = thermal->devdata;
	struct fttdcc010_thermal_zone *zone;
	struct device *dev = fttdcc010_thermal_priv_to_dev(priv);
	
	list_for_each_entry(zone, &priv->head, list) {
		if(zone->id != thermal->id)
			continue;

		if(trip >= zone->ntrips) {
			dev_err(dev, "get trip type error\n");
			return -EINVAL;
		} else {
			*type = zone->trips[0].type;
		}
	}
	
	return 0;
}

static int fttdcc010_get_trip_temp(struct thermal_zone_device *thermal,
                                   int trip, int *temp)
{
	struct fttdcc010_thermal_priv *priv = thermal->devdata;
	struct fttdcc010_thermal_zone *zone;
	struct device *dev = fttdcc010_thermal_priv_to_dev(priv);

	list_for_each_entry(zone, &priv->head, list) {
		if(zone->id != thermal->id)
			continue;

		if(trip >= zone->ntrips) {
			dev_err(dev, "get trip temp error\n");
			return -EINVAL;
		} else {
			*temp = zone->trips[0].temperature;
		}
	}
	
	return 0;
}

static int fttdcc010_set_trip_temp(struct thermal_zone_device *thermal,
                                   int trip, int temp)
{
	struct fttdcc010_thermal_priv *priv = thermal->devdata;
	struct fttdcc010_thermal_zone *zone;
	struct device *dev = fttdcc010_thermal_priv_to_dev(priv);
	
	list_for_each_entry(zone, &priv->head, list) {
		if(zone->id != thermal->id)
			continue;
		
		if(trip >= zone->ntrips) {
			dev_err(dev, "set trip temp error\n");
			return -EINVAL;
		} else {
			zone->trips[0].temperature = temp;
		}
	}
	
	return 0;
}

static int fttdcc010_notify(struct thermal_zone_device *thermal,
                            int trip, enum thermal_trip_type type)
{
	struct fttdcc010_thermal_priv *priv = thermal->devdata;
	struct device *dev = fttdcc010_thermal_priv_to_dev(priv);
	u32 val;
	
	switch (type) {
		case THERMAL_TRIP_CRITICAL:
			/* FIXME */
			dev_warn(dev, "Thermal reached to critical temperature\n");
			/* Disable FTTDCC010 conversion function */
			val = readl(&priv->regs->ctrl);
			val &= ~TDC_EN;
			writel(val, &priv->regs->ctrl);
			break;
		default:
			break;
	}
	
	return 0;
}

static struct thermal_zone_device_ops ops = {
	.get_temp      = fttdcc010_get_temp,
	.get_mode      = fttdcc010_get_mode,
	.set_mode      = fttdcc010_set_mode,
	.get_trip_type = fttdcc010_get_trip_type,
	.get_trip_temp = fttdcc010_get_trip_temp,
	.set_trip_temp = fttdcc010_set_trip_temp,
	.notify        = fttdcc010_notify,
};

static void fttdcc010_irq_ctrl(struct fttdcc010_thermal_priv *priv, u32 flags, int enable)
{
	u32 inten;
	
	inten = readl(&priv->regs->inten);
	if(enable)
		inten |= flags;
	else
		inten &= ~flags;
	
	writel(inten, &priv->regs->inten);
}

static void fttdcc010_thermal_update_work(struct work_struct *work)
{
	struct fttdcc010_thermal_priv *priv;
	struct fttdcc010_thermal_zone *zone;
	int i;
	
	priv = container_of(work, struct fttdcc010_thermal_priv, work);
	
	for(i = 0; i < NUM_TDC; i++) {
		list_for_each_entry(zone, &priv->head, list) {
			if(zone->id != i)
				continue;
			thermal_zone_device_update(zone->thermal, THERMAL_EVENT_TEMP_SAMPLE);
		}
	}
}

static void fttdcc010_irq_th(struct fttdcc010_thermal_priv *priv)
{
	struct fttdcc010_thermal_zone *zone;
	u32 ovr_intst, undr_intst;
	int i;
	
	for(i = 0; i < NUM_TDC; i++) {
		ovr_intst = priv->pending & OVR_INTST(i);
		undr_intst = priv->pending & UNDR_INTST(i);
		if(!(ovr_intst || undr_intst))
			continue;
		
		if(ovr_intst)
			priv->pending &= ~OVR_INTST(i);
		
		if(undr_intst)
			priv->pending &= ~UNDR_INTST(i);
		
		list_for_each_entry(zone, &priv->head, list) {
			if(zone->id != i)
				continue;
			schedule_work(&priv->work);
		}
	}
}

static void fttdcc010_irq_done(struct fttdcc010_thermal_priv *priv)
{
	struct fttdcc010_thermal_zone *zone;
	int i;
	
	for(i = 0; i < NUM_TDC; i++) {
		if(!(priv->pending & CHDONE_INTST(i)))
			continue;
		
		priv->pending &= ~CHDONE_INTST(i);
		
		list_for_each_entry(zone, &priv->head, list) {
			if(zone->id != i)
				continue;
			complete(&zone->done);
			schedule_work(&priv->work);
		}
	}
}

static irqreturn_t fttdcc010_irq(int irq, void *data)
{
	struct fttdcc010_thermal_priv *priv = data;
	u32 status, enable;
	u32 ret;
	
	enable = readl(&priv->regs->inten);
	status = readl(&priv->regs->intst);

	priv->pending = status & enable;
	writel(priv->pending, &priv->regs->intst);
	
	if(status & TH_INTST) {
		fttdcc010_irq_th(priv);
		ret = IRQ_HANDLED;
	} else if(status & DONE_INTST) {
		fttdcc010_irq_done(priv);
		ret = IRQ_HANDLED;
	} else {
		dev_err(priv->dev, "Null interrupt!!!\n");
		ret = IRQ_NONE;
	}
	
	return ret;
}

static void fttdcc010_init(struct fttdcc010_thermal_priv *priv)
{
	int i;
	
	writel(SCANMODE | TDC_EN, &priv->regs->ctrl);

	// Wait for scanning done.
	for(i = 0; i < NUM_TDC; i++) {
		while(!(readl(&priv->regs->intst) & CHDONE_INTST(i))) {}
	}
}

static void fttdcc010_populate_trips(struct fttdcc010_thermal_trip *trips)
{
	trips[0].type = THERMAL_TRIP_CRITICAL;
	trips[0].temperature = thermal_critical_trips;
}

static int fttdcc010_probe(struct platform_device *pdev)
{
#ifdef CONFIG_OF
	struct device_node *np = pdev->dev.of_node;
#endif
	struct fttdcc010_thermal_priv *priv;
	struct fttdcc010_thermal_zone *zone;
	struct fttdcc010_thermal_trip *trip;
	struct resource *res, *irq;
	int ret, i;
	int data[TDC_SINGLE_GROUP];
	
#ifdef CONFIG_OF
	int size = 0;
	const __be32 *list;

	if (!np) {
		dev_err(&pdev->dev, "Failed to get device node\n");
		return -EINVAL;
	}

	of_property_read_u32(np, "thermal-critical-trips", &thermal_critical_trips);
	printk("thermal_critical_trips:%d\n", thermal_critical_trips);
	
	list = of_get_property(np, "cooling-maps", &size);
	

	if (!list)
		return -EINVAL;

	size = size / TDC_MAGIC_NUM_SIZE;
	if(0 == size || size >= TDC_NUM_MAX || size % TDC_SINGLE_GROUP) {
		dev_err(&pdev->dev, "%pOF: number error!\n", np);
		return -EINVAL;
	}

	tdc_group_num = 0;

	while(size > 0) {
		for (i = 0; i < TDC_SINGLE_GROUP; i++) {
			data[i] = be32_to_cpu(*list++);
		}

		/* start cooling: start reduce frequency and
							reduce nuclears , Max 180*/
		if(data[0] > 180)
			break;

		/* stop cooling: can't be bigger than start cooling*/
		if(data[1] > data[0])
			break;

		/* frequency: KHz referrence to operating-points */
		if(data[2] > 1800000)
			break;

		/* core_reduce:1,2,3,4 */
		if(data[3] > 4 || 0 == data[3])
			break;

		start_cooling[tdc_group_num]      = data[0];
		stop_cooling[tdc_group_num]       = data[1];
		frequency_adjust[tdc_group_num]   = data[2];
		cores_adjust[tdc_group_num]       = data[3];

		tdc_group_num++;
		if(tdc_group_num >= TDC_GROUP_MAX)
			break;

		size -= TDC_SINGLE_GROUP;
	};

	cool_level = 0;
	//set_cooling(frequency_adjust[0], cores_adjust[0]);
#endif
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get platform resource(IORESOURCE_MEM)\n");
		return -ENODEV;
	}
	
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "Failed to get platform resource(IORESOURCE_IRQ)\n");
		return -ENODEV;
	}
	
	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "Failed to allocate priv\n");
		return -ENOMEM;
	}
	
	priv->dev = &pdev->dev;
	
	priv->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!priv->regs) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto err_ioremap;
	}
	
	ret = devm_request_irq(&pdev->dev, irq->start, fttdcc010_irq, 0,
	                       "fttdcc010", priv);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed\n");
		goto err_request_irq;
	}
	
	fttdcc010_init(priv);

	INIT_LIST_HEAD(&priv->head);
	INIT_WORK(&priv->work, fttdcc010_thermal_update_work);
	
	for(i = 0; i < NUM_TDC; i++) {
		zone = devm_kzalloc(&pdev->dev, sizeof(*zone), GFP_KERNEL);
		if (!zone) {
			dev_err(&pdev->dev, "Failed to allocate zone\n");
			ret = -ENOMEM;
			goto err_alloc_zone;
		}
		
		INIT_LIST_HEAD(&zone->list);
		init_completion(&zone->done);
		
		zone->ntrips = 1;
		zone->trips = devm_kzalloc(&pdev->dev, zone->ntrips * sizeof(*trip), GFP_KERNEL);
		if (!zone->trips) {
			dev_err(&pdev->dev, "Failed to allocate zone trips\n");
			ret = -ENOMEM;
			kfree(zone);
			goto err_alloc_zone_trips;
		}
		fttdcc010_populate_trips(zone->trips);
		
		list_add_tail(&zone->list, &priv->head);
		zone->thermal = thermal_zone_device_register("fttdcc010_thermal", zone->ntrips, 1,
		                                             priv, &ops, NULL, 1000, 2000);
		if (IS_ERR(zone->thermal)) {
			dev_err(&pdev->dev, "Failed to register thermal zone device\n");
			ret = PTR_ERR(zone->thermal);
			kfree(zone->trips);
			kfree(zone);
			goto err_zone_register;
		}
		
		zone->id = zone->thermal->id;

		writel(HTHR_EN | HTHR(MCELSIUS(thermal_critical_trips)), &priv->regs->thrhold[i]);
		writel(OVR_INTEN(i) | CHDONE_INTEN(i), &priv->regs->inten);	
	}
	
	platform_set_drvdata(pdev, priv);
	
	dev_info(&pdev->dev, "version %s\n", DRIVER_VERSION);
	
	return 0;
	
err_zone_register:
err_alloc_zone_trips:
err_alloc_zone:
	list_for_each_entry(zone, &priv->head, list) {
		thermal_zone_device_unregister(zone->thermal);
		kfree(zone->trips);
		kfree(zone);
	}
err_request_irq:
err_ioremap:
	kfree(priv);
	
	return ret;
}

static int fttdcc010_remove(struct platform_device *pdev)
{
	struct thermal_zone_device *thermal = platform_get_drvdata(pdev);
	struct fttdcc010_thermal_priv *priv = thermal->devdata;
	struct fttdcc010_thermal_zone *zone;
	u32 val;
	
	/* Disable FTTDCC010 conversion function */
	val = readl(&priv->regs->ctrl);
	val &= ~TDC_EN;
	writel(val, &priv->regs->ctrl);
	
	list_for_each_entry(zone, &priv->head, list) {
		thermal_zone_device_unregister(zone->thermal);
		kfree(zone->trips);
		kfree(zone);
	}
	
	platform_set_drvdata(pdev, NULL);
	
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int fttdcc010_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct thermal_zone_device *thermal = platform_get_drvdata(pdev);
	struct fttdcc010_thermal_priv *priv = thermal->devdata;
	u32 val;
	
	/* Disable FTTDCC010 conversion function */
	val = readl(&priv->regs->ctrl);
	val &= ~TDC_EN;
	writel(val, &priv->regs->ctrl);
	
	return 0;
}

static int fttdcc010_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct thermal_zone_device *thermal = platform_get_drvdata(pdev);
	struct fttdcc010_thermal_priv *priv = thermal->devdata;
	u32 val;
	
	/* Enalbe FTTDCC010 conversion function */
	val = readl(&priv->regs->ctrl);
	val |= TDC_EN;
	writel(val, &priv->regs->ctrl);
	
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(fttdcc010_pm_ops,
                         fttdcc010_suspend, fttdcc010_resume);

#ifdef CONFIG_OF
static const struct of_device_id fttdcc010_dt_ids[] = {
	{ .compatible = "faraday,fttdcc010" },
	{ }
};

MODULE_DEVICE_TABLE(of, fttdcc010_dt_ids);
#endif

static struct platform_driver fttdcc010_driver = {
	.driver = {
		.name  = "fttdcc010",
		.owner = THIS_MODULE,
		.pm    = &fttdcc010_pm_ops,
		.of_match_table = of_match_ptr(fttdcc010_dt_ids),
	},
	.probe  = fttdcc010_probe,
	.remove = fttdcc010_remove,
};

module_platform_driver(fttdcc010_driver);

MODULE_AUTHOR("B.C. Chen <bcchen@faraday-tech.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
