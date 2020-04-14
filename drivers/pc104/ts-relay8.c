#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/tspc104_bus.h>

#define MAGIC_ID_REG	0x0
#define PLD_REV_REG	0x1
#define RELAY_REG	0x2

#define MAGIC_ID	0x9b

struct tsrelay8_priv {
	struct gpio_chip gpio_chip;
	unsigned int cache;
	struct tspc104_bus *bus;
	struct device *dev;
	unsigned int base;
};

static void tsrelay8_gpio_set(struct gpio_chip *chip, unsigned int offset,
			    int value)
{
	struct tsrelay8_priv *priv = gpiochip_get_data(chip);

	if(value)
		priv->cache |= (1 << offset);
	else
		priv->cache &= ~(1 << offset);

	tspc104_io_write8(priv->bus, priv->base + RELAY_REG, &priv->cache);
}

static int tsrelay8_gpio_set_output(struct gpio_chip *chip,
					unsigned int offset, int value)
{
	/* Always is an output */
	tsrelay8_gpio_set(chip, offset, value);
	return 0;
}

static struct gpio_chip tsrelay8_chip = {
	.label			= "tsrelay8-gpio",
	.owner			= THIS_MODULE,
	.direction_output	= tsrelay8_gpio_set_output,
	.set			= tsrelay8_gpio_set,
	.can_sleep		= true,
	.ngpio			= 8,
	.base			= -1,
};

static int tsrelay8_is_sane(struct tsrelay8_priv *priv)
{
	unsigned int val;
	int ret;

	ret = tspc104_io_read8(priv->bus, priv->base + MAGIC_ID_REG,  &val);
	if(ret)
		return 0;

	if(val != MAGIC_ID){
		dev_err(priv->dev, "Did not detect TS-RELAY8");
		return 0;
	} 

	ret = tspc104_io_read8(priv->bus, priv->base + PLD_REV_REG,  &val);
	if(ret)
		return 0;

	dev_info(priv->dev, "TS-RELAY8 rev %d detected", val);

	return 1;
}

static int technologic_relay8_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tsrelay8_priv *priv;
	const __be32 *addr_be;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		return -ENOMEM;
	}

	addr_be = of_get_property(dev->of_node, "reg", NULL);
	if (!addr_be) 
		return -ENODEV;

	priv->base = be32_to_cpup(addr_be);

	priv->bus = platform_get_drvdata(to_platform_device(pdev->dev.parent));

	priv->dev = &pdev->dev;
	priv->gpio_chip = tsrelay8_chip;
	pdev->dev.platform_data = &priv;

	/* Defaults low */
	priv->cache = 0x0;

	ret = tsrelay8_is_sane(priv);
	if(!ret)
		return -ENODEV;

	ret = devm_gpiochip_add_data(dev, &tsrelay8_chip, priv);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to register gpiochip\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id tsrelay8_of_match[] = {
	{ .compatible = "technologic,tsrelay8", },
	{},
};

static struct platform_driver tsrelay8_driver = {
	.probe = technologic_relay8_probe,
	.driver = {
		.name = "tsrelay8",
		.of_match_table = tsrelay8_of_match,
	},
};
module_platform_driver(tsrelay8_driver);

MODULE_ALIAS("platform:ts_relay8");
MODULE_AUTHOR("Mark Featherston <mark@embeddedarm.com>");
MODULE_DESCRIPTION("Technologic TS-RELAY8 driver");
MODULE_LICENSE("GPL v2");
