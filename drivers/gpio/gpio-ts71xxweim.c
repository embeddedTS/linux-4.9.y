/*
 * Digital I/O driver for embeddedTS TS-7120, TS-7100, et al.
 *
 * Copyright (C) 2019-2022 Technologic Systems, Inc. dba embeddedTS
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
#include <linux/module.h>
#include <linux/delay.h>

/* Most that this driver can currently support in a single bank is 16. This is
 * due simply to how the FPGA used for these devices is structured.
 */
#define TSWEIM_NR_DIO		16

/* Register offsets from the 'reg' value passed in device tree source */
#define TSWEIM_SET_REG		0x00
#define TSWEIM_GET_REG		0x00
#define TSWEIM_EN_SET_REG	0x02
#define TSWEIM_CLR_REG		0x04
#define TSWEIM_EN_CLR_REG	0x06

struct tsweim_gpio_priv {
	void __iomem  *syscon;
	struct gpio_chip gpio_chip;
};

static inline struct tsweim_gpio_priv *to_gpio_tsweim(struct gpio_chip *chip)
{
	return container_of(chip, struct tsweim_gpio_priv, gpio_chip);
}

static int tsweim_gpio_direction_input(struct gpio_chip *chip,
						 unsigned int offset)
{
	struct tsweim_gpio_priv *priv = to_gpio_tsweim(chip);

	if (!(offset < priv->gpio_chip.ngpio))
		return -EINVAL;

	writew((1 << offset), priv->syscon + TSWEIM_EN_CLR_REG);

	return 0;
}

static int tsweim_gpio_direction_output(struct gpio_chip *chip,
					unsigned int offset, int value)
{
	struct tsweim_gpio_priv *priv = to_gpio_tsweim(chip);

	if (!(offset < priv->gpio_chip.ngpio))
		return -EINVAL;

	if (value)
		writew((1 << offset), priv->syscon + TSWEIM_SET_REG);
	else
		writew((1 << offset), priv->syscon + TSWEIM_CLR_REG);

	writew((1 << offset), priv->syscon + TSWEIM_EN_SET_REG);

	return 0;
}

static int tsweim_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct tsweim_gpio_priv *priv = to_gpio_tsweim(chip);
	uint16_t reg;

	if (!(offset < priv->gpio_chip.ngpio))
		return -EINVAL;

	reg = readw(priv->syscon + TSWEIM_GET_REG);
	return !!(reg & (1 << offset));
}

static void tsweim_gpio_set(struct gpio_chip *chip, unsigned int offset,
				 int value)
{
	struct tsweim_gpio_priv *priv = to_gpio_tsweim(chip);

	if (!(offset < priv->gpio_chip.ngpio))
		return;

	if (value)
		writew((1 << offset), priv->syscon + TSWEIM_SET_REG);
	else
		writew((1 << offset), priv->syscon + TSWEIM_CLR_REG);
}

static const struct gpio_chip template_chip = {
	.label			= "tsweim-gpio",
	.owner			= THIS_MODULE,
	.direction_input	= tsweim_gpio_direction_input,
	.direction_output	= tsweim_gpio_direction_output,
	.get			= tsweim_gpio_get,
	.set			= tsweim_gpio_set,
	.base			= -1,
	.can_sleep		= false,
};

static const struct of_device_id tsweim_gpio_of_match_table[] = {
	{ .compatible = "embeddedts,ts71xxweim-gpio", },
	{},
};
MODULE_DEVICE_TABLE(of, tsweim_gpio_of_match_table);

static int tsweim_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct tsweim_gpio_priv *priv;
	u32 ngpio;
	int base;
	void __iomem  *membase;
	struct resource *res;

	match = of_match_device(tsweim_gpio_of_match_table, dev);
	if (!match)
		return -EINVAL;

	if (of_property_read_u32(dev->of_node, "ngpios", &ngpio))
		ngpio = TSWEIM_NR_DIO;

	if (of_property_read_u32(dev->of_node, "base", &base))
		base = -1;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		pr_err("Can't get device address\n");
		return -EFAULT;
	}

	membase =  devm_ioremap_nocache(&pdev->dev, res->start,
					  resource_size(res));
	if (IS_ERR(membase)) {
		pr_err("Could not map resource\n");
		return -ENOMEM;
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->syscon = membase;

	pr_info("FPGA syscon mapped to 0x%08X, %d bytes\n",
	  (unsigned int)priv->syscon, resource_size(res));

	priv->gpio_chip = template_chip;
	priv->gpio_chip.label = "tsweim-gpio";
	priv->gpio_chip.ngpio = ngpio;
	priv->gpio_chip.base = base;
	pdev->dev.platform_data = &priv;
	priv->gpio_chip.of_node = pdev->dev.of_node;

	return devm_gpiochip_add_data(&pdev->dev, &priv->gpio_chip, &priv);
}

static struct platform_driver tsweim_gpio_driver = {
	.driver = {
		.name = "tsweim-gpio",
		.of_match_table = of_match_ptr(tsweim_gpio_of_match_table),
	},
	.probe = tsweim_gpio_probe,
};
module_platform_driver(tsweim_gpio_driver);

MODULE_AUTHOR("embeddedTS");
MODULE_DESCRIPTION("GPIO interface for embeddedTS WEIM FPGA");
MODULE_LICENSE("GPL");
