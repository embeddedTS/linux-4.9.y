/*
 * Digital I/O driver for Technologic Systems TS-7120, TS-7100, et al.
 *
 * Copyright (C) 2019 Technologic Systems
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether expressed or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License version 2 for more details.
 */

#include <linux/gpio/driver.h>
#include <linux/of_device.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/delay.h>

#define TS7120_NR_DIO	 	32
#define TS7120_DIO_BASE  	160

/* Register offsets from the 'reg' value in device-tree */
#define BANK_0_OUTPUT_SET_REG 0x10
#define BANK_0_OUTPUT_GET_REG 0x10
#define BANK_0_OUTPUT_ENABLE_SET_REG 0x12
#define BANK_0_OUTPUT_CLR_REG 0x14
#define BANK_0_OUTPUT_ENABLE_CLR_REG 0x16

#define BANK_1_OUTPUT_SET_REG 0x40
#define BANK_1_OUTPUT_GET_REG 0x40
#define BANK_1_OUTPUT_ENABLE_SET_REG 0x42
#define BANK_1_OUTPUT_CLR_REG 0x44
#define BANK_1_OUTPUT_ENABLE_CLR_REG 0x46


struct TS7120_gpio_priv {
	void __iomem  *syscon;
	struct gpio_chip gpio_chip;
	spinlock_t lock;
	unsigned int direction[4];      /* enough for all 118 DIOs, 1=in, 0=out */
	unsigned int ovalue[4];
};


/*
	DIO controlled by the FPGA on the TS-7120:

	Bank 0:
		Header HD20...
			Pin 2 DIO_2							GPIO BASE + 0
			Pin 4 DIO_4							GPIO BASE + 1
			Pin 5 DIO_5							GPIO BASE + 2
			Pin 6 DIO_6							GPIO BASE + 3
			Pin 7 DIO_7							GPIO BASE + 4
			Pin 8 DIO_8							GPIO BASE + 5
			Pin 9 DIO_9							GPIO BASE + 6
			Pin 10 DIO_10						GPIO BASE + 7
			Pin 11 DIO_11						GPIO BASE + 8
			Pin 12 DIO_12						GPIO BASE + 9
			Pin 13 DIO_13						GPIO BASE + 10
			Pin 14 DIO_14						GPIO BASE + 11
		Silab Programming Clock				GPIO BASE + 12
		Silab Programming Data				GPIO BASE + 13
		Enable Blue LED						GPIO BASE + 14
		Syetem Reset#							GPIO BASE + 15

	Bank 1:
		Mikro Reset#							GPIO BASE + 16
		Enable Digital IN 0 & 1				GPIO BASE + 17
		Enable AN_5V							GPIO BASE + 18
		Enable PoE								GPIO BASE + 19
		Enable Nimbel 3.3V					GPIO BASE + 20
		Enable CAN Transceiver				GPIO BASE + 21
		Enable Digital IN 2 & 3				GPIO BASE + 22
		Enable HS SW							GPIO BASE + 23
		Enable XBEE USB						GPIO BASE + 24
		Enable LS OUT 0						GPIO BASE + 25
		Enable WiFi Power						GPIO BASE + 26
		Enable Nimbel 4V						GPIO BASE + 27
		Enable GPS 3.3V						GPIO BASE + 28
		Enable eMMC 3.3V#						GPIO BASE + 29
		Enable LS OUT 1						GPIO BASE + 30
		Enable WiFi Reset#					GPIO BASE + 31

	DIO is controlled by eight 16-bit registers in the FPGA Syscon:

	Bank 0:
		Offset 0x10:  Data Set (write) or Pin State (read)
		Offset 0x12:  Output Enable Set
		Offset 0x14:  Data Clear
		Offset 0x16:  Output Enable Clear

	Bank 1:
		Offset 0x40:  Data Set (write) or Pin State (read)
		Offset 0x42:  Output Enable Set
		Offset 0x44:  Data Clear
		Offset 0x46:  Output Enable Clear

*/

#define IS_VALID_OFFSET(x) ((x) < TS7120_NR_DIO)

static inline struct TS7120_gpio_priv *to_gpio_TS7120(struct gpio_chip *chip)
{
	return container_of(chip, struct TS7120_gpio_priv, gpio_chip);
}

static int TS7120_gpio_get_direction(struct gpio_chip *chip,
					  unsigned int offset)
{
	struct TS7120_gpio_priv *priv = to_gpio_TS7120(chip);

	if (priv == NULL) {
		printk("%s %d, priv is NULL!\n", __func__, __LINE__);
		return -1;
	}

	if (priv->syscon == NULL) {
		  printk("%s %d, priv->syscon is NULL!\n", __func__, __LINE__);
		  return -1;
	}

	if (!IS_VALID_OFFSET(offset))
		return -EINVAL;

	return !!(priv->direction[offset / 32] & (1 << offset % 32));

}

static int TS7120_gpio_direction_input(struct gpio_chip *chip,
						 unsigned int offset)
{
	struct TS7120_gpio_priv *priv = to_gpio_TS7120(chip);
	unsigned long flags;

	if (priv == NULL) {
		printk("%s %d, priv is NULL!\n", __func__, __LINE__);
		return -1;
	}

	if (priv->syscon == NULL) {
		  printk("%s %d, priv->syscon is NULL!\n", __func__, __LINE__);
		  return -1;
	}

	if (!IS_VALID_OFFSET(offset))
		return -EINVAL;

	spin_lock_irqsave(&priv->lock, flags);

	priv->direction[offset / 32] |= (1 << offset % 32);

	if (offset < 16)
		writew((1 << offset), priv->syscon + BANK_0_OUTPUT_ENABLE_CLR_REG);
	else
		writew((1 << (offset-16)), priv->syscon + BANK_1_OUTPUT_ENABLE_CLR_REG);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int TS7120_gpio_direction_output(struct gpio_chip *chip,
					unsigned int offset, int value)
{
	struct TS7120_gpio_priv *priv = to_gpio_TS7120(chip);
	unsigned long flags;
	int ret =0;

	if (!IS_VALID_OFFSET(offset)) {
		printk("offset %d is invalid\n", offset);
			 return -EINVAL;
	}

	if (priv == NULL) {
		printk("%s %d, priv is NULL!\n", __func__, __LINE__);
		return -1;
	}

	if (priv->syscon == NULL) {
		  printk("%s %d, priv->syscon is NULL!\n", __func__, __LINE__);
		  return -1;
	}

	spin_lock_irqsave(&priv->lock, flags);

	if (offset < 16) {
		writew((1 << offset), priv->syscon + BANK_0_OUTPUT_ENABLE_SET_REG);

		if (value)
			writew((1 << offset), priv->syscon + BANK_0_OUTPUT_SET_REG);
		else
			writew((1 << offset), priv->syscon + BANK_0_OUTPUT_CLR_REG);
	} else {

		writew((1 << (offset-16)), priv->syscon + BANK_1_OUTPUT_ENABLE_SET_REG);

		if (value)
			writew((1 << (offset-16)), priv->syscon + BANK_1_OUTPUT_SET_REG);
		else
			writew((1 << (offset-16)), priv->syscon + BANK_1_OUTPUT_CLR_REG);
	}

	priv->direction[offset / 32] &= ~(1 << offset % 32);
	spin_unlock_irqrestore(&priv->lock, flags);

	return ret;
}

static int TS7120_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct TS7120_gpio_priv *priv = to_gpio_TS7120(chip);
	uint16_t reg;

	if (!IS_VALID_OFFSET(offset))
		return -EINVAL;

	if (priv == NULL) {
		printk("%s %d, priv is NULL!\n", __func__, __LINE__);
		return -1;
	}

	if (priv->syscon == NULL) {
		  printk("%s %d, priv->syscon is NULL!\n", __func__, __LINE__);
		  return -1;
	}

	if (offset < 16) {
		reg = readw(priv->syscon + BANK_0_OUTPUT_GET_REG);
		return !!(reg & (1 << offset));
	} else {
		reg = readw(priv->syscon + BANK_1_OUTPUT_GET_REG);
		return !!(reg & (1 << (offset-16)));
	}

}

static void TS7120_gpio_set(struct gpio_chip *chip, unsigned int offset,
				 int value)
{
	struct TS7120_gpio_priv *priv = to_gpio_TS7120(chip);
	unsigned long flags;

	if (priv == NULL) {
		printk("%s %d, priv is NULL!\n", __func__, __LINE__);
		return;
	}
	if (priv->syscon == NULL) {
		  printk("%s %d, priv->syscon is NULL!\n", __func__, __LINE__);
		  return;
	}

	if (!IS_VALID_OFFSET(offset))
		return;

	if ((priv->direction[offset / 32] & (1 << offset % 32))) {
		printk("DIO #%d is not an output\n", priv->gpio_chip.base + offset);
		return;
	}

	spin_lock_irqsave(&priv->lock, flags);

	if (offset < 16) {
		if (value)
			writew((1 << offset), priv->syscon + BANK_0_OUTPUT_SET_REG);
		else
			writew((1 << offset), priv->syscon + BANK_0_OUTPUT_CLR_REG);
	} else {
		if (value)
			writew((1 << (offset-16)), priv->syscon + BANK_1_OUTPUT_SET_REG);
		else
			writew((1 << (offset-16)), priv->syscon + BANK_1_OUTPUT_CLR_REG);
	}

	spin_unlock_irqrestore(&priv->lock, flags);

}


static const struct gpio_chip template_chip = {
	.label			= "TS7120-gpio",
	.owner			= THIS_MODULE,
	.get_direction		= TS7120_gpio_get_direction,
	.direction_input	= TS7120_gpio_direction_input,
	.direction_output	= TS7120_gpio_direction_output,
	.get			= TS7120_gpio_get,
	.set			= TS7120_gpio_set,
	.base			= -1,
	.can_sleep		= false,
};

static const struct of_device_id TS7120_gpio_of_match_table[] = {
	{
		.compatible = "technologic,TS7120-gpio",
		.compatible = "technologic,ts71xxweim-gpio",
	},

	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, TS7120_gpio_of_match_table);

static int TS7120_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct TS7120_gpio_priv *priv;
	u32 ngpio;
	int base;
	int ret;
	void __iomem  *membase;
	struct resource *res = 0;

	match = of_match_device(TS7120_gpio_of_match_table, dev);
	if (!match)
		return -EINVAL;

	if (of_property_read_u32(dev->of_node, "ngpios", &ngpio))
		ngpio = TS7120_NR_DIO;

	if (of_property_read_u32(dev->of_node, "base", &base))
		base = TS7120_DIO_BASE;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (res == NULL) {
		pr_err("Can't get device address\n");
		return -EFAULT;
	}

	membase =  devm_ioremap_nocache(&pdev->dev, res->start,
					  resource_size(res));

	if (IS_ERR(membase)) {
		pr_err("Could not map resource\n");
		return -ENOMEM;;
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->syscon = membase;

	pr_info("FPGA syscon mapped to 0x%08X, %d bytes\n", (unsigned int)priv->syscon, resource_size(res));

	memset(priv->direction, 0xFF, sizeof(priv->direction));
	memset(priv->ovalue, 0, sizeof(priv->ovalue));
	/* Set all the DIO to inputs */

	writew(0xffff, priv->syscon + BANK_0_OUTPUT_ENABLE_CLR_REG);
	writew(0xffff, priv->syscon + BANK_1_OUTPUT_ENABLE_CLR_REG);

	spin_lock_init(&priv->lock);
	priv->gpio_chip = template_chip;
	priv->gpio_chip.label = "TS7120-gpio";
	priv->gpio_chip.ngpio = ngpio;
	priv->gpio_chip.base = base;
	pdev->dev.platform_data = &priv;

#ifdef CONFIG_OF_GPIO
	priv->gpio_chip.of_node = pdev->dev.of_node;
#endif

	ret = gpiochip_add(&priv->gpio_chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to register gpiochip\n");
		return ret;
	}

	platform_set_drvdata(pdev, priv);

	return 0;
}

static int TS7120_gpio_remove(struct platform_device *pdev)
{
	struct TS7120_gpio_priv *priv = platform_get_drvdata(pdev);
	if (priv)
		gpiochip_remove(&priv->gpio_chip);
	return 0;
}

static struct platform_driver TS7120_gpio_driver = {
	.driver = {
		.name = "TS7120-gpio",
		.of_match_table = of_match_ptr(TS7120_gpio_of_match_table),
	},
	.probe = TS7120_gpio_probe,
	.remove = TS7120_gpio_remove,
};
module_platform_driver(TS7120_gpio_driver);

MODULE_AUTHOR("Technologic Systems");
MODULE_DESCRIPTION("GPIO interface for Technologic Systems TS-7120 DIO");
MODULE_LICENSE("GPL");
