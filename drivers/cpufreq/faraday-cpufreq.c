/*
 * Faraday CPU CPUFreq Driver Framework
 *
 * (C) Copyright 2020 Faraday Technology
 * Bo-Cun Chen <bcchen@faraday-tech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>

#include <asm/io.h>
#include <asm/smp_plat.h>

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/notifier.h>
#include <linux/uaccess.h>

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/thermal.h>
#include <linux/cpu.h>


#include "faraday-cpufreq.h"

#define PU_SOC_VOLTAGE_NORMAL	1100000
#define VOLTAGE_BASE_VALUE   712500
#define VOLTAGE_MAGIC_VALUE  12500
#define PMIC_BUCK1_ON_VSEL_REG 0x2F
#define PMIC_ADDR 0x18

extern int plat_cpufreq_init(struct faraday_cpu_dvfs_info *);

static struct faraday_cpu_dvfs_info *info;
static struct cpufreq_freqs freqs;
static u32 *scm801_soc_volt;
static u32 soc_opp_count = 0;
static int pmic_i2c_num;
static DEFINE_SPINLOCK(cpufreq_lock);

static int faraday_voltage_target(u32 volt, int i2cnum)
{
	struct i2c_client *client;
	struct i2c_adapter *adap;
	u8 val;
	u8 buffer[2];
	struct i2c_msg msg;

	adap = i2c_get_adapter(i2cnum);
	if (!adap)
		return -ENODEV;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client) {
		i2c_put_adapter(adap);
		return -ENOMEM;
	}
	snprintf(client->name, I2C_NAME_SIZE, "i2c-dev %d", adap->nr);

	client->adapter = adap;
	client->addr = PMIC_ADDR;
	
	
	val = (volt - VOLTAGE_BASE_VALUE) / VOLTAGE_MAGIC_VALUE;

	buffer[0] = PMIC_BUCK1_ON_VSEL_REG;
	buffer[1] = val;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = sizeof(buffer);
	msg.buf = buffer;

	i2c_transfer(client->adapter, &msg, 1);

	i2c_put_adapter(client->adapter);
	kfree(client);

	return 0;
}

static int faraday_cpufreq_target_index(struct cpufreq_policy *policy, unsigned int index)
{
	struct cpufreq_frequency_table *freq_table = info->freq_table;

	spin_lock(&cpufreq_lock);

	freqs.old = policy->cur;
	freqs.new = freq_table[index].frequency;

	cpufreq_freq_transition_begin(policy, &freqs);

	if (freqs.new != freqs.old) {
		faraday_voltage_target(scm801_soc_volt[index], pmic_i2c_num);
		info->set_freq(info, freqs.new);
	}

	cpufreq_freq_transition_end(policy, &freqs, 0);

	spin_unlock(&cpufreq_lock);

	return 0;
}

#ifdef CONFIG_PM
static int faraday_cpufreq_suspend(struct cpufreq_policy *policy)
{
	return 0;
}

static int faraday_cpufreq_resume(struct cpufreq_policy *policy)
{
	return 0;
}
#endif

static int faraday_cpufreq_init(struct cpufreq_policy *policy)
{
	policy->clk = info->cpu_clk;
	policy->transition_delay_us = 1000000;
	return cpufreq_generic_init(policy, info->freq_table, 0);
}

static struct cpufreq_driver faraday_driver = {
	.name         = "faraday_cpufreq",
	.flags        = CPUFREQ_STICKY | CPUFREQ_ASYNC_NOTIFICATION |
	                CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.init         = faraday_cpufreq_init,
	.verify       = cpufreq_generic_frequency_table_verify,
	.target_index = faraday_cpufreq_target_index,
	.get          = cpufreq_generic_get,
#ifdef CONFIG_PM
	.suspend      = faraday_cpufreq_suspend,
	.resume       = faraday_cpufreq_resume,
#endif
};

static irqreturn_t sysc_isr(int irq, void *dev_id)
{
	unsigned int status; 

	status = readl(info->sysc_base + 0x24);
	writel(status, info->sysc_base + 0x24);    /* clear interrupt */

	return IRQ_HANDLED;
}

static int faraday_cpufreq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct clk *cpu_clk;
	int num, ret = -EINVAL;
	u32 nr, i, j;
	const struct property *prop;
	const __be32 *val;
	struct device_node *np;
	unsigned long freq;
	unsigned long volt;
	info = kzalloc(sizeof(struct faraday_cpu_dvfs_info), GFP_KERNEL);
	if (!info) {
		ret = -ENOMEM;
		dev_err(dev, "failed to alloc memory: %d\n", ret);
		goto err_alloc_info;
	}

	info->dev = &pdev->dev;
	platform_set_drvdata(pdev, info);

	/* find the resources */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sysc");
	info->sysc_base = ioremap(res->start, resource_size(res));
	if (IS_ERR(info->sysc_base)) {
		ret = PTR_ERR(info->sysc_base);
		dev_err(dev, "failed to map sysc: %d\n", ret);
		goto err_free_put_info;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ddrc");
	info->ddrc_base = ioremap(res->start, resource_size(res));
	if (IS_ERR(info->ddrc_base)) {
		ret = PTR_ERR(info->ddrc_base);
		dev_err(dev, "failed to map ddrc: %d\n", ret);
		goto err_free_put_info;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "uart");
	info->uart_base = ioremap(res->start, resource_size(res));
	if (IS_ERR(info->uart_base)) {
		ret = PTR_ERR(info->uart_base);
		dev_err(dev, "failed to map uart: %d\n", ret);
		goto err_free_put_info;
	}

	/* find the irq */
	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(dev, "failed to get irq: %d\n", ret);
		goto err_free_put_info;
	}

	ret = devm_request_irq(dev, ret, sysc_isr, 0, pdev->name, info);
	if (ret) {
		dev_err(dev, "failed to request irq: %d\n", ret);
		goto err_free_put_info;
	}

	cpu_clk = clk_get(dev, NULL);
	if (IS_ERR(cpu_clk)) {
		dev_err(dev, "failed to get cpu clk: %d\n", ret);
		goto err_free_put_info;
	}
	info->cpu_clk = cpu_clk;

	ret = plat_cpufreq_init(info);
	if (ret) {
		dev_err(dev, "failed to init platform: %d\n", ret);
		goto err_free_put_info;
	}

	if (info->set_freq == NULL) {
		pr_err("%s: No set_freq function (ERR)\n", __func__);
		goto err_free_put_info;
	}

	ret = dev_pm_opp_of_add_table(info->dev);
	if (ret) {
		dev_err(info->dev, "failed to init OPP table: %d\n", ret);
		goto err_free_put_info;
	}

	num = dev_pm_opp_get_opp_count(info->dev);
	if (num < 0) {
		ret = num;
		dev_err(info->dev, "no OPP table is found: %d\n", ret);
		goto err_free_opp;
	}

	ret = dev_pm_opp_init_cpufreq_table(info->dev,
	                                    &info->freq_table);
	if (ret) {
		dev_err(info->dev, "failed to init cpufreq table: %d\n", ret);
		goto err_free_opp;
	}

	np = info->dev->of_node;
	if (0 != of_property_read_u32(np, "pmic-i2cnum", &pmic_i2c_num)) {
            printk("pmic-i2cnum error!\n");
            goto err_free_table;
    }

	/* Make scm801_soc_volt array's size same as arm opp number */
	scm801_soc_volt = devm_kzalloc(info->dev, sizeof(*scm801_soc_volt) * num, GFP_KERNEL);
	if (scm801_soc_volt == NULL) {
		ret = -ENOMEM;
		goto err_free_table;
	}

	prop = of_find_property(np, "scm801,operating-points", NULL);
	if (!prop || !prop->value)
		goto err_free_opp;

	/*
	 * Each OPP is a set of tuples consisting of frequency and
	 * voltage like <freq-kHz vol-uV>.
	 */
	nr = prop->length / sizeof(u32);
	if (nr % 2 || (nr / 2) < num)
		goto err_free_opp;

	for (j = 0; j < num; j++) {
		val = prop->value;
		for (i = 0; i < nr / 2; i++) {
			freq = be32_to_cpup(val++);
			volt = be32_to_cpup(val++);
			if (info->freq_table[j].frequency == freq) {
				scm801_soc_volt[soc_opp_count++] = volt;
				break;
			}
		}
	}

	/* use fixed soc opp volt if no valid soc opp info found in dtb */
	if (soc_opp_count != num) {
		dev_warn(info->dev, "can NOT find valid scm701d,soc-operating-points property in dtb, use default value!\n");
		for (j = 0; j < num; j++)
			scm801_soc_volt[j] = PU_SOC_VOLTAGE_NORMAL;
	}

	ret = cpufreq_register_driver(&faraday_driver);
	if (ret) {
		dev_err(info->dev, "failed to register cpufreq driver: %d\n", ret);
		goto err_free_table;
	}

	dev_info(info->dev, "initialization sucessfully\n");

	return 0;

err_free_table:
	dev_pm_opp_free_cpufreq_table(info->dev, &info->freq_table);
err_free_opp:
	dev_pm_opp_of_remove_table(info->dev);
err_free_put_info:
	kfree(info);
err_alloc_info:
	dev_err(info->dev, "failed initialization\n");
	return ret;
}

static int faraday_cpufreq_remove(struct platform_device *pdev)
{
	struct faraday_cpu_dvfs_info *info = platform_get_drvdata(pdev);

	cpufreq_unregister_driver(&faraday_driver);
	dev_pm_opp_free_cpufreq_table(info->dev, &info->freq_table);
	dev_pm_opp_of_remove_table(info->dev);

	return 0;
}

static const struct of_device_id faraday_cpufreq_dt_match[] = {
	{ .compatible = "faraday,faraday-cpufreq" },
	{ }
};
MODULE_DEVICE_TABLE(of, faraday_cpufreq_dt_match);

static struct platform_driver faraday_cpufreq_driver = {
	.probe  = faraday_cpufreq_probe,
	.remove = faraday_cpufreq_remove,
	.driver = {
		.name           = "faraday-cpufreq",
		.owner          = THIS_MODULE,
		.of_match_table = of_match_ptr(faraday_cpufreq_dt_match),
	},
};
module_platform_driver(faraday_cpufreq_driver);

MODULE_DESCRIPTION("Faraday CPUFreq driver");
MODULE_AUTHOR("Bo-Cun Chen <bcchen@faraday-tech.com>");
MODULE_LICENSE("GPL v2+");
