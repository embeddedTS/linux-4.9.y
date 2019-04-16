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

#define TSWEIM_NR_DIO	 	32

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


struct tsweim_gpio_priv {
	void __iomem  *syscon;
	struct gpio_chip gpio_chip;
	spinlock_t lock;
	unsigned int direction[4];   /* enough for all 118 DIOs, 1=in, 0=out */
	unsigned int ovalue[4];
};


/*
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

static inline struct tsweim_gpio_priv *to_gpio_tsweim(struct gpio_chip *chip)
{
	return container_of(chip, struct tsweim_gpio_priv, gpio_chip);
}

static int tsweim_gpio_get_direction(struct gpio_chip *chip,
					  unsigned int offset)
{
	struct tsweim_gpio_priv *priv = to_gpio_tsweim(chip);

	if (priv == NULL) {
		printk("%s %d, priv is NULL!\n", __func__, __LINE__);
		return -1;
	}

	if (priv->syscon == NULL) {
		  printk("%s %d, priv->syscon is NULL!\n", __func__, __LINE__);
		  return -1;
	}

	if (!(offset < priv->gpio_chip.ngpio)) {
		return -EINVAL;
	}

	return !!(priv->direction[offset / 32] & (1 << offset % 32));

}

static int tsweim_gpio_direction_input(struct gpio_chip *chip,
						 unsigned int offset)
{
	struct tsweim_gpio_priv *priv = to_gpio_tsweim(chip);
	unsigned long flags;

	if (priv == NULL) {
		printk("%s %d, priv is NULL!\n", __func__, __LINE__);
		return -1;
	}

	if (priv->syscon == NULL) {
		  printk("%s %d, priv->syscon is NULL!\n", __func__, __LINE__);
		  return -1;
	}

	if (!(offset < priv->gpio_chip.ngpio)) {
		return -EINVAL;
	}

	spin_lock_irqsave(&priv->lock, flags);

	priv->direction[offset / 32] |= (1 << offset % 32);

	if (offset < 16) {
		writew((1 << offset),
		  priv->syscon + BANK_0_OUTPUT_ENABLE_CLR_REG);
	} else {
		writew((1 << (offset-16)),
		  priv->syscon + BANK_1_OUTPUT_ENABLE_CLR_REG);
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int tsweim_gpio_direction_output(struct gpio_chip *chip,
					unsigned int offset, int value)
{
	struct tsweim_gpio_priv *priv = to_gpio_tsweim(chip);
	unsigned long flags;
	int ret =0;

	if (!(offset < priv->gpio_chip.ngpio)) {
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
		writew((1 << offset),
		  priv->syscon + BANK_0_OUTPUT_ENABLE_SET_REG);

		if (value) {
			writew((1 << offset),
			  priv->syscon + BANK_0_OUTPUT_SET_REG);
		} else {
			writew((1 << offset),
			  priv->syscon + BANK_0_OUTPUT_CLR_REG);
		}
	} else {

		writew((1 << (offset-16)),
		  priv->syscon + BANK_1_OUTPUT_ENABLE_SET_REG);

		if (value) {
			writew((1 << (offset-16)),
			  priv->syscon + BANK_1_OUTPUT_SET_REG);
		} else {
			writew((1 << (offset-16)),
			  priv->syscon + BANK_1_OUTPUT_CLR_REG);
		}
	}

	priv->direction[offset / 32] &= ~(1 << offset % 32);
	spin_unlock_irqrestore(&priv->lock, flags);

	return ret;
}

static int tsweim_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct tsweim_gpio_priv *priv = to_gpio_tsweim(chip);
	uint16_t reg;

	if (!(offset < priv->gpio_chip.ngpio)) {
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

	if (offset < 16) {
		reg = readw(priv->syscon + BANK_0_OUTPUT_GET_REG);
		return !!(reg & (1 << offset));
	} else {
		reg = readw(priv->syscon + BANK_1_OUTPUT_GET_REG);
		return !!(reg & (1 << (offset-16)));
	}

}

static void tsweim_gpio_set(struct gpio_chip *chip, unsigned int offset,
				 int value)
{
	struct tsweim_gpio_priv *priv = to_gpio_tsweim(chip);
	unsigned long flags;

	if (priv == NULL) {
		printk("%s %d, priv is NULL!\n", __func__, __LINE__);
		return;
	}
	if (priv->syscon == NULL) {
		  printk("%s %d, priv->syscon is NULL!\n", __func__, __LINE__);
		  return;
	}

	if (!(offset < priv->gpio_chip.ngpio)) {
		return;
	}

	if ((priv->direction[offset / 32] & (1 << offset % 32))) {
		printk("DIO #%d is not an output\n",
		  priv->gpio_chip.base + offset);
		return;
	}

	spin_lock_irqsave(&priv->lock, flags);

	if (offset < 16) {
		if (value) {
			writew((1 << offset),
			  priv->syscon + BANK_0_OUTPUT_SET_REG);
		} else {
			writew((1 << offset),
			  priv->syscon + BANK_0_OUTPUT_CLR_REG);
		}
	} else {
		if (value) {
			writew((1 << (offset-16)),
			  priv->syscon + BANK_1_OUTPUT_SET_REG);
		} else {
			writew((1 << (offset-16)),
			  priv->syscon + BANK_1_OUTPUT_CLR_REG);
		}
	}

	spin_unlock_irqrestore(&priv->lock, flags);

}


static const struct gpio_chip template_chip = {
	.label			= "tsweim-gpio",
	.owner			= THIS_MODULE,
	.get_direction		= tsweim_gpio_get_direction,
	.direction_input	= tsweim_gpio_direction_input,
	.direction_output	= tsweim_gpio_direction_output,
	.get			= tsweim_gpio_get,
	.set			= tsweim_gpio_set,
	.base			= -1,
	.can_sleep		= false,
};

static const struct of_device_id tsweim_gpio_of_match_table[] = {
	{
		.compatible = "technologic,tsweim-gpio",
		.compatible = "technologic,ts71xxweim-gpio",
	},

	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, tsweim_gpio_of_match_table);

static int tsweim_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct tsweim_gpio_priv *priv;
	u32 ngpio;
	int base;
	int ret;
	void __iomem  *membase;
	struct resource *res = 0;

	match = of_match_device(tsweim_gpio_of_match_table, dev);
	if (!match)
		return -EINVAL;

	if (of_property_read_u32(dev->of_node, "ngpios", &ngpio)) {
		ngpio = TSWEIM_NR_DIO;
	}

	if (of_property_read_u32(dev->of_node, "base", &base)) {
		base = -1;
	}

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
	if (!priv) {
		return -ENOMEM;
	}

	priv->syscon = membase;

	pr_info("FPGA syscon mapped to 0x%08X, %d bytes\n",
	  (unsigned int)priv->syscon, resource_size(res));

	memset(priv->direction, 0xFF, sizeof(priv->direction));
	memset(priv->ovalue, 0, sizeof(priv->ovalue));
	/* Set all the DIO to inputs */

	writew(0xffff, priv->syscon + BANK_0_OUTPUT_ENABLE_CLR_REG);
	writew(0xffff, priv->syscon + BANK_1_OUTPUT_ENABLE_CLR_REG);

	spin_lock_init(&priv->lock);
	priv->gpio_chip = template_chip;
	priv->gpio_chip.label = "tsweim-gpio";
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

static int tsweim_gpio_remove(struct platform_device *pdev)
{
	struct tsweim_gpio_priv *priv = platform_get_drvdata(pdev);

	if (priv) {
		gpiochip_remove(&priv->gpio_chip);
	}

	return 0;
}

static struct platform_driver tsweim_gpio_driver = {
	.driver = {
		.name = "tsweim-gpio",
		.of_match_table = of_match_ptr(tsweim_gpio_of_match_table),
	},
	.probe = tsweim_gpio_probe,
	.remove = tsweim_gpio_remove,
};
module_platform_driver(tsweim_gpio_driver);

MODULE_AUTHOR("Technologic Systems");
MODULE_DESCRIPTION("GPIO interface for Technologic Systems WEIM FPGA");
MODULE_LICENSE("GPL");
