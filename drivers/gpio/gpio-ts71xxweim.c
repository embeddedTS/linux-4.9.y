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

/* Most that this driver can currently support in a single bank is 16. This is
 * due simply to how the FPGA used for these devices is structured. */
#define TSWEIM_NR_DIO	 		16

/* Register offsets from the 'reg' value passed in device tree source */
#define OUTPUT_SET_REG		0x00
#define OUTPUT_GET_REG		0x00
#define OUTPUT_EN_SET_REG	0x02
#define OUTPUT_CLR_REG		0x04
#define OUTPUT_EN_CLR_REG	0x06

struct tsweim_gpio_priv {
	void __iomem  *syscon;
	struct gpio_chip gpio_chip;
	spinlock_t lock;
};

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

	/* XXX: HACK! */
	return 0;

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

	writew((1 << offset), priv->syscon + OUTPUT_EN_CLR_REG);

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

	writew((1 << offset), priv->syscon + OUTPUT_EN_SET_REG);
	if (value) writew((1 << offset), priv->syscon + OUTPUT_SET_REG);
	else writew((1 << offset), priv->syscon + OUTPUT_CLR_REG);

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

	reg = readw(priv->syscon + OUTPUT_GET_REG);
	return !!(reg & (1 << offset));

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

	spin_lock_irqsave(&priv->lock, flags);

	if (value) writew((1 << offset), priv->syscon + OUTPUT_SET_REG);
	else writew((1 << offset), priv->syscon + OUTPUT_CLR_REG);

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
