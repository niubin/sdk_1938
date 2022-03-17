/*
 * An I2C driver for the Epson RX8025T RTC
 *
 * Author: xixiangji xixiangji@sgitg.com
 * Copyright 2020 SGITG, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Based on: rtc-pcf8563.c (An I2C driver for the Philips PCF8563 RTC)
 * Copyright 2005-06 Tower Technologies
 */
 
//#define  rx8025_debug      1

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/log2.h>
#include <linux/gpio.h>
#include <linux/interrupt.h> //中断注册注销头文件
#include <linux/irq.h>
#include <linux/delay.h> 
#include <linux/of_gpio.h>


#define DRV_VERSION "0.2"


//#define RX8025_SEC_INT          1

#define RX8025_REG_SC		0x00 /* Second in BCD */
#define RX8025_REG_MN		0x01 /* Minute in BCD */
#define RX8025_REG_HR		0x02 /* Hour in BCD */
#define RX8025_REG_DW		0x03 /* Day of Week */
#define RX8025_REG_DM		0x04 /* Day of Month in BCD */
#define RX8025_REG_MO		0x05 /* Month in BCD */
#define RX8025_REG_YR		0x06 /* Year in BCD */
#define RX8025_REG_RAM		0x07 /* RAM */
#define RX8025_REG_AMN		0x08 /* Alarm Min in BCD*/
#define RX8025_REG_AHR		0x09 /* Alarm Hour in BCD */
#define RX8025_REG_ADM		0x0A
#define RX8025_REG_ADW		0x0A
#define RX8025_REG_TMR0		0x0B
#define RX8025_REG_TMR1		0x0C
#define RX8025_REG_EXT		0x0D /* Extension Register */
#define RX8025_REG_FLAG		0x0E /* Flag Register */
#define RX8025_REG_CTRL		0x0F /* Control Register */


/* Flag Register bit definitions */
#define RX8025_FLAG_UF		0x20 /* Update */
#define RX8025_FLAG_TF		0x10 /* Timer */
#define RX8025_FLAG_AF		0x08 /* Alarm */
#define RX8025_FLAG_VLF		0x02 /* Voltage Low */

/* Control Register bit definitions */
#define RX8025_CTRL_UIE		0x20 /* Update Interrupt Enable */
#define RX8025_CTRL_TIE		0x10 /* Timer Interrupt Enable */
#define RX8025_CTRL_AIE		0x08 /* Alarm Interrupt Enable */
#define RX8025_CTRL_STOP	0x02 /* STOP bit */
#define RX8025_CTRL_RESET	0x01 /* RESET bit */


#define RX8025_EXT_TESL0        0x1
#define RX8025_EXT_TESL1        0x2
#define RX8025_EXT_USEL         0x20
#define RX8025_EXT_TE           0x10
#define RX8025_EXT_TEST         0x80

static struct i2c_driver rx8025_driver;

#ifdef RX8025_SEC_INT
static irqreturn_t rx8025_irq(int irq, void *dev_id)
{
	struct i2c_client * client =(struct i2c_client *) dev_id;
	struct rtc_device *rtc = i2c_get_clientdata(client);
#ifdef rx8025_debug
	struct timeval tv_tick;
	do_gettimeofday(&tv_tick);
	printk("Tick  time   second is %ld, us is %ld\n", tv_tick.tv_sec, tv_tick.tv_usec);
#endif
	rtc_update_irq(rtc, 1, RTC_PF | RTC_IRQF);
	return IRQ_HANDLED;
}
#endif

/*
 * In the routines that deal directly with the rx8025 hardware, we use
 * rtc_time -- month 0-11, hour 0-23, yr = calendar year-epoch.
 */
static int rx8025_get_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	unsigned char date[7];
	int data, err;

#ifdef rx8025_debug
	struct timeval tv_start, tv_finish;
	do_gettimeofday(&tv_start);
	printk("Read time start  second is %ld, us is %ld\n", tv_start.tv_sec, tv_start.tv_usec);
#endif
	err = i2c_smbus_read_i2c_block_data(client, RX8025_REG_SC, 7, date);
	if (err < 0) {
		dev_err(&client->dev, "Unable to read date\n");
		return -EIO;
	}

		/* Check flag register */
	data = i2c_smbus_read_byte_data(client, RX8025_REG_FLAG);
	if (data < 0) {
		dev_err(&client->dev, "Unable to read device flags\n");
		return -EIO;
	}
#ifdef rx8025_debug 
	do_gettimeofday(&tv_finish);
	printk("Read time finish  second is %ld, us is %ld\n", tv_finish.tv_sec, tv_finish.tv_usec);
#endif 


	dev_dbg(&client->dev,
		"%s: raw data is sec=%02x, min=%02x, hr=%02x, "
		"wday=%02x, mday=%02x, mon=%02x, year=%02x\n",
		__func__,
		date[0], date[1], date[2], date[3], date[4], date[5], date[6]);

	tm->tm_sec = bcd2bin(date[RX8025_REG_SC] & 0x7F);
	tm->tm_min = bcd2bin(date[RX8025_REG_MN] & 0x7F);
	tm->tm_hour = bcd2bin(date[RX8025_REG_HR] & 0x3F); /* rtc hr 0-23 */
	tm->tm_wday = ilog2(date[RX8025_REG_DW] & 0x7F);
	tm->tm_mday = bcd2bin(date[RX8025_REG_DM] & 0x3F);
	tm->tm_mon = bcd2bin(date[RX8025_REG_MO] & 0x1F) - 1; /* rtc mn 1-12 */
	tm->tm_year = bcd2bin(date[RX8025_REG_YR]);
	if (tm->tm_year < 70)
		tm->tm_year += 100;	/* assume we are in 1970...2069 */


	dev_dbg(&client->dev, "%s: tm is secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);
	
	err = rtc_valid_tm(tm);
	if (err < 0)
		return err;
	if (data & RX8025_FLAG_VLF)
		return -ERANGE;
	return 0;
}

static int rx8025_set_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	int data, err;
	unsigned char buf[7];
#ifdef rx8025_debug
	struct timeval tv_start, tv_finish;
	do_gettimeofday(&tv_start);
	printk("write time start  second is %ld, us is %ld\n", tv_start.tv_sec, tv_start.tv_usec);
#endif
	
	dev_dbg(&client->dev, "%s: secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	/* hours, minutes and seconds */
	buf[RX8025_REG_SC] = bin2bcd(tm->tm_sec);
	buf[RX8025_REG_MN] = bin2bcd(tm->tm_min);
	buf[RX8025_REG_HR] = bin2bcd(tm->tm_hour);

	buf[RX8025_REG_DM] = bin2bcd(tm->tm_mday);

	/* month, 1 - 12 */
	buf[RX8025_REG_MO] = bin2bcd(tm->tm_mon + 1);

	/* year and century */
	buf[RX8025_REG_YR] = bin2bcd(tm->tm_year % 100);
	buf[RX8025_REG_DW] = (0x1 << tm->tm_wday);
	
	/* write register's data */
	err = i2c_smbus_write_i2c_block_data(client, RX8025_REG_SC, 7, buf);
	if (err < 0) {
		dev_err(&client->dev, "Unable to write to date registers\n");
		return -EIO;
	}
#ifdef rx8025_debug 
	do_gettimeofday(&tv_finish);
	printk("write time finish  second is %ld, us is %ld\n", tv_finish.tv_sec, tv_finish.tv_usec);
#endif 
		/* get VLF and clear it */
	data = i2c_smbus_read_byte_data(client, RX8025_REG_FLAG);
	if (data < 0) {
		dev_err(&client->dev, "Unable to read flag register\n");
		return -EIO;
	}

	err = i2c_smbus_write_byte_data(client, RX8025_REG_FLAG,
		(data & ~(RX8025_FLAG_VLF)));
	if (err != 0) {
		dev_err(&client->dev, "Unable to write flag register\n");
		return -EIO;
	}
	return 0;
}
static int rx8025_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	return rx8025_get_datetime(to_i2c_client(dev), tm);
}

static int rx8025_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	return rx8025_set_datetime(to_i2c_client(dev), tm);
}

static int rx8025_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *tm)
{
	return 0;
}


static const struct rtc_class_ops rx8025_rtc_ops = {
	.read_time	= rx8025_rtc_read_time,
	.set_time	= rx8025_rtc_set_time,
	.set_alarm  = rx8025_rtc_set_alarm,
};

static int rx8025_init_client(struct i2c_client *client)
{
	unsigned char  ext, ctrl;
	int ret = 0;

	ext = i2c_smbus_read_byte_data(client, RX8025_REG_EXT);
#ifdef rx8025_debug 
	printk("Reg ext origin  value is 0x%02x\n", ext);
#endif
	ext &= ~(RX8025_EXT_TEST | RX8025_EXT_USEL);//TEST位必须清零，芯片手册的要求。秒中断位也清零（厂家建议）

#ifdef rx8025_debug 
	printk("Reg ext write after set counter  value is 0x%02x\n", ext);
#endif
	ret = i2c_smbus_write_byte_data(client, RX8025_REG_EXT, ext);
	if (ret != 0) {
		dev_err(&client->dev, "Unable to write ext register\n");
		return -EIO;
	}
	
	ctrl = i2c_smbus_read_byte_data(client, RX8025_REG_CTRL);
#ifdef rx8025_debug 
	printk("Reg ctrl origin  value is 0x%02x\n", ctrl);
#endif
	ctrl &= ~(RX8025_CTRL_TIE | RX8025_CTRL_AIE);//关闭定时中断和闹钟终端，避免对对时和秒脉冲时产生影响
	ctrl |= RX8025_CTRL_UIE;
#ifdef rx8025_debug 
	printk("Reg ctrl  value is 0x%02x\n", ctrl);
#endif
	ret = i2c_smbus_write_byte_data(client, RX8025_REG_CTRL, ctrl);
	if (ret != 0) {
		dev_err(&client->dev, "Unable to write ext register\n");
		return -EIO;
	}
	return ret;
}

static int rx8025_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct rtc_device *rtc;
	int ret = 0;

#ifdef RX8025_SEC_INT
	int gpio;
	struct device *dev = &client->dev;
#endif

	dev_dbg(&client->dev, "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	dev_info(&client->dev, "chip found, driver version " DRV_VERSION "\n");
#ifdef RX8025_SEC_INT
        gpio = of_get_named_gpio(dev->of_node, "second_interrput", 0);
        if(!gpio_is_valid(gpio)) {
                printk("powers gpio is not valid!\n");
                return -ENODEV;
        }
        ret = devm_gpio_request(dev, gpio, "second_interrupt_gpio");
        if(ret < 0) {
                printk("Second interrupt gpio resuest failed\n");
                return ret;
        }
        gpio_direction_input(gpio);

    	client->irq = gpio_to_irq(gpio);
#endif	
	ret = rx8025_init_client(client);
	if (ret) 
		goto out;

	
	rtc = devm_rtc_device_register(&client->dev, rx8025_driver.driver.name, &rx8025_rtc_ops, THIS_MODULE);
	
	rtc->uie_unsupported = 1;
	
	if (IS_ERR(rtc))
		return PTR_ERR(rtc);
#ifdef RX8025_SEC_INT
	if (client->irq > 0) {
		dev_info(&client->dev, "IRQ %d supplied\n", client->irq);
		ret = devm_request_irq(&client->dev, client->irq, rx8025_irq, IRQF_TRIGGER_FALLING, "rx8025", client);
		if (ret) {
			dev_err(&client->dev, "unable to request IRQ\n");
			return ret;
		}
	}
#endif
	i2c_set_clientdata(client, rtc);
out:
	return ret;
}

static int rx8025_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id rx8025_id[] = {
	{ "rx8025", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rx8025_id);

static struct i2c_driver rx8025_driver = {
	.driver		= {
		.name	= "rtc-rx8025",
		.owner	= THIS_MODULE,
	},
	.probe		= rx8025_probe,
	.remove		= rx8025_remove,
	.id_table	= rx8025_id,
};

module_i2c_driver(rx8025_driver);

MODULE_AUTHOR("xixiangji <xixiangji@sgitg.com>");
MODULE_DESCRIPTION("Epson RX-8025 RTC driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
