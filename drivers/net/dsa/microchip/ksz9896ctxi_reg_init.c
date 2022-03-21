#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#define MAX_PORT 6 
static u16 reg_9896c[37][3] = {
	{0x01,	0x6F, 0xDD0B},
	{0x01,	0x75, 0x0060},
	{0x01,	0x79, 0x010A},
	{0x01, 	0x7A, 0x00ED},
	{0x01, 	0x7B, 0x00D3},
	{0x01, 	0x7C, 0x00BC},
	{0x01, 	0x7D, 0x00A8},
	{0x01,  0x7E, 0x0096},
	{0x01, 	0x7F, 0x0085},
	{0x01,	0x80, 0x0077},
	{0x01, 	0x81, 0x006A},
	{0x01, 	0x82, 0x005E},
	{0x01, 	0x83, 0x0054},
	{0x01, 	0x84, 0x004B},
	{0x01,  0x85, 0x0043},
	{0x01, 	0x86, 0x003C},
	{0x01,	0x87, 0x0035},
	{0x01, 	0x88, 0x002F},
	{0x01, 	0x89, 0x002A},
	{0x01, 	0x8A, 0x0026},
	{0x01, 	0x8F, 0x6032},
	{0x01,  0x9D, 0x248C},
	{0x01, 	0xC8, 0x0010},
	{0x01, 	0xC9, 0x0280},
	{0x01, 	0xCA, 0x0141},
	{0x01, 	0xCB, 0x0FCF},
	{0x01, 	0xCC, 0x0FF0},
	{0x01,	0xCE, 0x0100},
	{0x01, 	0xD3, 0x7777},
	{0x01,  0xD9, 0x0100},
	{0x02,	0x00, 0x0010},//MMD LED MODE REGISTER
	{0x07, 	0x3C, 0x0000},//disable EEE
	{0x1C, 	0x00, 0x9400},
	{0x1C, 	0x04, 0x00E2},
	{0x1C, 	0x06, 0x3100},
	{0x1C, 	0x08, 0x2001},
	{0x1C,  0x09, 0xE01C},
};

static s32  ksz_wirte_reg_u16(struct i2c_client *client,  u16 addr, u16 value)//MDIO Manageable Device (MMD) Registers
{
	s32 ret;
	u8 buf[4] = { 0 };
	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 4,
			.buf	= buf,
		},
	};

	buf[0] = (addr & 0xff00) >> 8;
	buf[1] = addr & 0xff;	
	buf[2] = (value & 0xff00) >> 8;
	buf[3] = value & 0xff;

	ret = i2c_transfer(client->adapter, msg, 1);
	return ret;
}

static s32 ksz_wirte_reg_u8(struct i2c_client *client, u16 addr, u8 value)//Switch Register
{
	int ret;
	u8 buf[4] = { 0 };
	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 3,
			.buf	= buf,
		},
	};

	buf[0] = (addr & 0xff00) >> 8;
	buf[1] = (addr & 0xff) >> 0;	
	buf[2] = value;	

	ret = i2c_transfer(client->adapter, msg, 1);
	return ret;
}

static s32  ksz_wirte_reg_u32(struct i2c_client * client, u16 addr, u32 value)
{
	int ret;
	u8 buf[8] = { 0 };
	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 6,
			.buf	= buf,
		},
	};

	buf[0] = (addr & 0xff00) >> 8;
	buf[1] = (addr & 0xff) >> 0;	
	buf[2] = (value & 0xff000000) >> 24;	
	buf[3] = (value & 0xff0000) >> 16;	
	buf[4] = (value & 0xff00) >> 8;	
	buf[5] = (value & 0xff) >> 0;	

	ret = i2c_transfer(client->adapter, msg, 1);
	return ret;
}

static s32 ksz_read_reg_u16(struct i2c_client *client, u16 addr, u16* value)
{
	s32 ret;
	u8 buf[4] = { 0 };

	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= &buf[0],
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 2,
			.buf	= &buf[2],
		},
	};

	buf[0] = (addr & 0xff00)>>8;
	buf[1] = (addr & 0xff) >> 0;
	buf[2] =0xee;
	buf[3] =0xee;
	ret = i2c_transfer(client->adapter, msgs, 2);
	*value =  buf[3] * 256 + buf[2]; //  BUG here lxh
	return ret;
}

static s32 ksz_read_reg_u32(struct i2c_client *client, u16 addr, u32 *value)
{
	s32 ret  = 0;
	u8 buf[8] = { 0 };

	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= &buf[0],
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 4,
			.buf	= &buf[2],
		},
	};

	buf[0] = (addr & 0xff00) >>8;
	buf[1] = (addr & 0xff) >> 0;
	buf[2] = 0xee;
	buf[3] = 0xee;
	buf[4] = 0xee;
	buf[5] = 0xee;
	
	ret = i2c_transfer(client->adapter, msgs, 2);
	*value =  ((u32)buf[2] << 24) + ((u32)buf[3] << 16) + ((u32)buf[4] << 8) + buf[5];
	return ret;
}

static int ksz_reg_init(struct i2c_client *client)
{
	s32 ret;
	s32 port_num, reg_num;
	u32 reg_u32_value;
	u16 reg_u16_value;
	u32 reg_addr;
	reg_addr = 0x03C0;
	ret = ksz_wirte_reg_u16(client, reg_addr, 0x4090);
	if(ret < 0) {
		printk("Write reg_addr %4x reg_u16_value failed!\n", reg_addr);
	}
	reg_addr = 0x03C2;
	ret = ksz_wirte_reg_u16(client, reg_addr, 0x0040);
	if(ret < 0) {
		printk("Write reg_addr %4x reg_u16_value failed!\n", reg_addr);
	}
	reg_addr = 0x03C4;
	ret = ksz_wirte_reg_u16(client, reg_addr, 0x2000);
	if(ret < 0) {
		printk("Write reg_addr %4x reg_u16_value failed!\n", reg_addr);
	}
	reg_addr = 0x3331;
	ret = ksz_wirte_reg_u8(client, reg_addr, 0xD0);
	if(ret < 0) {
		printk("Write reg_addr %4x reg_u16_value failed!\n", reg_addr);
	}
	
	for(port_num = 1; port_num < MAX_PORT; port_num++)
	{
		reg_addr = 0x0100 | (port_num << 12);
		ret = ksz_wirte_reg_u16(client, reg_addr, 0x0140);//disable auto-negotiation
		if(ret < 0) {
			printk("Write reg_addr %4x reg_u16_value failed!\n", reg_addr);
		}
	
		for(reg_num = 0; reg_num < 37; reg_num++) {
			reg_addr = 0x011a | (port_num << 12);
	   		ret =  ksz_wirte_reg_u16(client, reg_addr, reg_9896c[reg_num][0]);
			if(ret < 0) {
				printk("Write reg_addr %4x reg_u16_value failed!\n", reg_addr);
			}
			reg_addr = 0x011c|(port_num << 12);
	   		ret = ksz_wirte_reg_u16(client, reg_addr, reg_9896c[reg_num][1]);
			if(ret < 0) {
				printk("Write reg_addr %4x reg_u16_value failed!\n", reg_addr);
			}
			reg_addr = 0x011a | (port_num << 12);
		   	ret = ksz_wirte_reg_u16(client, reg_addr, reg_9896c[reg_num][0] | 0x4000);
			if(ret < 0) {
				printk("Write reg_addr %4x reg_u16_value failed!\n", reg_addr);
			}
			reg_addr = 0x011c | (port_num << 12);
	   		ret = ksz_wirte_reg_u16(client, reg_addr, reg_9896c[reg_num][2]);
			if(ret < 0) {
				printk("Write reg_addr %4x reg_u16_value failed!\n", reg_addr);
			}
	   	 }	
		 
		reg_addr = 0x013c | (port_num << 12);
		ret  = ksz_read_reg_u32(client, reg_addr , &reg_u32_value);
		if(ret < 0) {
			printk("Read reg_addr %4x reg_u32_value failed!\n", reg_addr);
		}
		reg_u32_value = (reg_u32_value & 0xFFFF) | (0xFA00 << 16);
	
		ret = ksz_wirte_reg_u32(client, reg_addr, reg_u32_value);
		if(ret < 0) {
			printk("Write reg_addr %4x reg_u32_value failed!\n", reg_addr);
		}
	
		reg_addr = 0x0100 | (port_num << 12);
		ret = ksz_wirte_reg_u16(client, reg_addr, 0x1140);//enable auto-negotiation	
		if(ret < 0) {
			printk("Write reg_addr %4x reg_u16_value failed!\n", reg_addr);
		}
	}
		
	
	reg_addr = 0x0330;
	ret  = ksz_read_reg_u16(client, reg_addr, &reg_u16_value);
	if(ret < 0) {
		printk("Read reg_addr %4x reg_u16_value failed!\n", reg_addr);
	}
	ret = ksz_wirte_reg_u8(client, reg_addr, ((u8)reg_u16_value | 0x04) & 0xF7);//Do not check the packet length
	if(ret < 0) {
		printk("Write reg_addr %4x reg_u8_value failed!\n", reg_addr);
	}
	
	reg_addr = 0x6301;
	ret = ksz_read_reg_u16(client, reg_addr, &reg_u16_value);
	if(ret < 0) {
		printk("Read reg_addr %4x reg_u16_value failed!\n", reg_addr);
	}
	ret = ksz_wirte_reg_u8(client, reg_addr, (u8)reg_u16_value | 0x10);//1.5ns delay added
	if(ret < 0) {
		printk("Write reg_addr %4x reg_u8_value failed!\n", reg_addr);
	}
	return ret;
}
static int ksz_reset(struct i2c_client *client)
{
	unsigned int phy_reset;
	int err;
	struct device *dev = &client->dev;
	if (0 != of_property_read_u32(dev->of_node, "phy-reset-gpios", &phy_reset)) {
		printk("Get phy_reset failed!\n");
		return -ENODEV;
	}
	if(!gpio_is_valid(phy_reset)) {
                printk("phy_reset gpio is not valid!\n");
                return -ENODEV;
        }
	err = devm_gpio_request(&client->dev, phy_reset, "phy-reset");
	if (err) {
		dev_err(&client->dev, "failed to get phy-reset-gpios: %d\n", err);
                return -ENODEV;
	}
	gpio_direction_output(phy_reset, 0);
	msleep(1000);
	gpio_set_value(phy_reset, 1);
	devm_gpio_free(&client->dev, phy_reset);
	return 0;
}

static int ksz9896_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret  = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		return  -ENODEV;
	}
	printk("KSZ9896 chip found!\n");
	ret = ksz_reset(client);
	if (ret < 0)	
		return ret;
		
	msleep(80);
	ret = ksz_reg_init(client);
	if (ret < 0)	
		return ret;
	return 0;
}

static int  ksz9896_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ksz9896_id[] = {
	{ "ksz9896", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, ksz9896_id);

static const struct of_device_id of_ksz9896_match[] = {
        { .compatible = "microchip,ksz9896"},
        {}
};
MODULE_DEVICE_TABLE(of, of_ksz9896_match);


static struct i2c_driver ksz9896_driver = {
	.driver	= {
		.name	= "ksz9896",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(of_ksz9896_match),
	},
	.probe		= ksz9896_probe,
	.remove		= ksz9896_remove,
	.id_table	= ksz9896_id,
};
module_i2c_driver(ksz9896_driver);

MODULE_AUTHOR("xixiangji <xixiangji@sgitg.com>");
MODULE_DESCRIPTION("ksz9896ctxi_reg_init driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("v1.0");

