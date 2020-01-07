/*
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 * PC104 bus driver for TS systems.  This uses a memory window to provide
 * io / mem cycles at 8/16 bit sizes.
 */

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/tspc104_bus.h>

/* 
 * 0x4050: 32-bit register for kernel space
 *   bit 31: busy, means do not write. (RO) - On write 0 = address, 1 = start cycle
 *   bit 30: IO (1) memory (0)
 *   bit 29: 8-bit (1) 16-bit (0)
 *   bit 28: read (1), write (0)
 *   bit 27: funky-TS mode enable
 *   bit 26-0: address/data
 * 
 */

struct tspc104_bus {
	struct gpio_desc *gpio_reset;
	/* We support the standard pinout, or by default we use
	 * a pinout that allows 16-bit modes without the 40 pin header just
	 * using 64 pins. */
	int use_ts_mode;
	void __iomem *reg;
	spinlock_t lock;
};

static int tspc104_block_while_busy(struct tspc104_bus *bus)
{
	while(readl(bus->reg) & TSISA_GOBSY);

	return 0;
}

int tspc104_reg_write(struct tspc104_bus *bus, unsigned int reg, 
			      unsigned int *val, uint32_t busflags)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&bus->lock, flags);
	writel(reg & 0x7FFFFFF, bus->reg);
	writel((*val & 0xffff) | busflags | TSISA_GOBSY, bus->reg);
	ret = tspc104_block_while_busy(bus);
	spin_unlock_irqrestore(&bus->lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(tspc104_reg_write);

int tspc104_reg_read(struct tspc104_bus *bus, unsigned int reg, 
			      unsigned int *val, uint32_t busflags)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&bus->lock, flags);
	writel(reg & 0x7FFFFFF, bus->reg);
	writel(busflags | TSISA_RDEN | TSISA_GOBSY, bus->reg);
	ret = tspc104_block_while_busy(bus);
	*val = readl(bus->reg);
	spin_unlock_irqrestore(&bus->lock, flags);

	if(busflags & TSISA_8BIT)
		*val &= 0xff;
	else
		*val &= 0xffff;

	return ret;
}
EXPORT_SYMBOL_GPL(tspc104_reg_read);

static int ts_pc104bus_init_pdata(struct platform_device *pdev,
				  struct tspc104_bus *bus)
{
	if(of_property_read_bool(pdev->dev.of_node, "ts-mode"))
		bus->use_ts_mode = 1;
	else
		bus->use_ts_mode = 0;

	bus->gpio_reset = devm_gpiod_get(&pdev->dev, "reset",
					 GPIOD_OUT_HIGH);
	if (IS_ERR(bus->gpio_reset)) {
		dev_err(&pdev->dev, "Failed to get ISA_RESET\n");
		return PTR_ERR(bus->gpio_reset);
	}

	return 0;
}

static int technologic_isa_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *mem;
	struct tspc104_bus *bus;
	int ret;

	bus = devm_kzalloc(dev, sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return -ENOMEM;

	spin_lock_init(&bus->lock);

	ret = ts_pc104bus_init_pdata(pdev, bus);
	if (ret < 0)
		return ret;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(dev, "missing IOMEM\n");
		return -EINVAL;
	}

	bus->reg = devm_ioremap_nocache(dev, mem->start, resource_size(mem));
	if (!bus->reg) {
		dev_err(dev, "failed to remap I/O memory\n");
		return -ENXIO;
	}

	gpiod_direction_output(bus->gpio_reset, 1);
	msleep(10);
	gpiod_set_value(bus->gpio_reset, 0);

	/*
	 * let the child nodes retrieve this instance of the ts-nbus.
	 */
	dev_set_drvdata(dev, bus);
	ret = of_platform_populate(dev->of_node, NULL, NULL, dev);
	if (ret < 0)
		return ret;

	dev_info(dev, "ready\n");

	return 0;
}

static const struct of_device_id tsisa_of_match[] = {
	{ .compatible = "technologic,pc104-bus", },
	{},
};

static struct platform_driver tsisa_driver = {
	.probe = technologic_isa_probe,
	.driver = {
		.name = "ts-pc104-bus",
		.of_match_table = tsisa_of_match,
	},
};
module_platform_driver(tsisa_driver);

MODULE_ALIAS("platform:ts_pc104");
MODULE_AUTHOR("Mark Featherston <mark@embeddedarm.com>");
MODULE_DESCRIPTION("Technologic ISA driver");
MODULE_LICENSE("GPL v2");
