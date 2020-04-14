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
#include <linux/sysfs.h>
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

#define PC104_ADDR_SPACE	((1<<19) - 1)

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
	bus->gpio_reset = devm_gpiod_get(&pdev->dev, "reset",
					 GPIOD_OUT_HIGH);
	if (IS_ERR(bus->gpio_reset)) {
		dev_err(&pdev->dev, "Failed to get ISA_RESET\n");
		return PTR_ERR(bus->gpio_reset);
	}

	return 0;
}

ssize_t isa_io8_read(struct file *filp, struct kobject *kobj,
		     struct bin_attribute *bin_attr,
		     char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tspc104_bus *bus = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < count; i++){
		unsigned int val;
		tspc104_io_read8(bus, off + i, &val);
		buf[i] = (char)val;
	}

	return i;
}

ssize_t isa_io8_write(struct file *filp, struct kobject *kobj,
		      struct bin_attribute *bin_attr,
		      char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tspc104_bus *bus = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < count; i++){
		unsigned int val = (unsigned int)buf[i];
		tspc104_io_write8(bus, off + i, &val);
	}

	return i;
}

ssize_t isa_mem8_read(struct file *filp, struct kobject *kobj,
		     struct bin_attribute *bin_attr,
		     char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tspc104_bus *bus = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < count; i++){
		unsigned int val;
		tspc104_mem_read8(bus, off + i, &val);
		buf[i] = (char)val;
	}

	return i;
}

ssize_t isa_mem8_write(struct file *filp, struct kobject *kobj,
		      struct bin_attribute *bin_attr,
		      char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tspc104_bus *bus = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < count; i++){
		unsigned int val = (unsigned int)buf[i];
		tspc104_mem_write8(bus, off + i, &val);
	}

	return i;
}

ssize_t isa_io16_read(struct file *filp, struct kobject *kobj,
		     struct bin_attribute *bin_attr,
		     char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tspc104_bus *bus = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < count; i += 2){
		unsigned int val;
		tspc104_io_read16(bus, off + i, &val);
		buf[i] = (char)val;
		if(i <= count)
			buf[i+1] = (char)(val >> 8);
	}

	return i;
}

ssize_t isa_io16_write(struct file *filp, struct kobject *kobj,
		      struct bin_attribute *bin_attr,
		      char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tspc104_bus *bus = dev_get_drvdata(dev);
	int i;

	BUG_ON(count % 2 != 0);

	for (i = 0; i < count; i += 2){
		unsigned int val = (unsigned int)(buf[i]);;
		val |= (unsigned int)(buf[i+1] << 8);
		tspc104_io_write16(bus, off + i, &val);
	}

	return i;
}

ssize_t isa_mem16_read(struct file *filp, struct kobject *kobj,
		     struct bin_attribute *bin_attr,
		     char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tspc104_bus *bus = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < count; i += 2){
		unsigned int val;
		tspc104_mem_read16(bus, off + i, &val);
		buf[i] = (char)val;
		if(i <= count)
			buf[i+1] = (char)(val >> 8);
	}

	return i;
}

ssize_t isa_mem16_write(struct file *filp, struct kobject *kobj,
		      struct bin_attribute *bin_attr,
		      char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tspc104_bus *bus = dev_get_drvdata(dev);
	int i;

	BUG_ON(count % 2 != 0);

	for (i = 0; i < count; i += 2){
		unsigned int val = (unsigned int)(buf[i]);;
		val |= (unsigned int)(buf[i+1] << 8);
		tspc104_mem_write16(bus, off + i, &val);
	}

	return i;
}

ssize_t isa_io16alt_read(struct file *filp, struct kobject *kobj,
		     struct bin_attribute *bin_attr,
		     char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tspc104_bus *bus = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < count; i += 2){
		unsigned int val;
		tspc104_io_read16_altpinout(bus, off + i, &val);
		buf[i] = (char)val;
		if(i <= count)
			buf[i+1] = (char)(val >> 8);
	}

	return i;
}

ssize_t isa_io16alt_write(struct file *filp, struct kobject *kobj,
		      struct bin_attribute *bin_attr,
		      char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tspc104_bus *bus = dev_get_drvdata(dev);
	int i;

	BUG_ON(count % 2 != 0);

	for (i = 0; i < count; i += 2){
		unsigned int val = (unsigned int)(buf[i]);;
		val |= (unsigned int)(buf[i+1] << 8);
		tspc104_io_write16_altpinout(bus, off + i, &val);
	}

	return i;
}

ssize_t isa_mem16alt_read(struct file *filp, struct kobject *kobj,
		     struct bin_attribute *bin_attr,
		     char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tspc104_bus *bus = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < count; i += 2){
		unsigned int val;
		tspc104_mem_read16_altpinout(bus, off + i, &val);
		buf[i] = (char)val;
		if(i <= count)
			buf[i+1] = (char)(val >> 8);
	}

	return i;
}

ssize_t isa_mem16alt_write(struct file *filp, struct kobject *kobj,
		      struct bin_attribute *bin_attr,
		      char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tspc104_bus *bus = dev_get_drvdata(dev);
	int i;

	BUG_ON(count % 2 != 0);

	for (i = 0; i < count; i += 2){
		unsigned int val = (unsigned int)(buf[i]);;
		val |= (unsigned int)(buf[i+1] << 8);
		tspc104_mem_write16_altpinout(bus, off + i, &val);
	}

	return i;
}

struct bin_attribute isa_io8_attr = {
	.attr = {
		.name = "io8",
		.mode = S_IRUGO | S_IWUSR,
	},
	.size = PC104_ADDR_SPACE,
	.read = isa_io8_read,
	.write = isa_io8_write,
};

struct bin_attribute isa_mem8_attr = {
	.attr = {
		.name = "mem8",
		.mode = S_IRUGO | S_IWUSR,
	},
	.size = PC104_ADDR_SPACE,
	.read = isa_mem8_read,
	.write = isa_mem8_write,
};

struct bin_attribute isa_io16_attr = {
	.attr = {
		.name = "io16",
		.mode = S_IRUGO | S_IWUSR,
	},
	.size = PC104_ADDR_SPACE,
	.read = isa_io16_read,
	.write = isa_io16_write,
};

struct bin_attribute isa_mem16_attr = {
	.attr = {
		.name = "mem16",
		.mode = S_IRUGO | S_IWUSR,
	},
	.size = PC104_ADDR_SPACE,
	.read = isa_mem16_read,
	.write = isa_mem16_write,
};

struct bin_attribute isa_io16alt_attr = {
	.attr = {
		.name = "ioalt16",
		.mode = S_IRUGO | S_IWUSR,
	},
	.size = PC104_ADDR_SPACE,
	.read = isa_io16alt_read,
	.write = isa_io16alt_write,
};

struct bin_attribute isa_mem16alt_attr = {
	.attr = {
		.name = "memalt16",
		.mode = S_IRUGO | S_IWUSR,
	},
	.size = PC104_ADDR_SPACE,
	.read = isa_mem16alt_read,
	.write = isa_mem16alt_write,
};

static struct bin_attribute *tsisa_sysfs_bin_attrs[] = {
	&isa_io8_attr,
	&isa_mem8_attr,
	&isa_io16_attr,
	&isa_mem16_attr,
	&isa_io16alt_attr,
	&isa_mem16alt_attr,
	NULL,
};

static const struct attribute_group tsisa_sysfs_group = {
	.bin_attrs = tsisa_sysfs_bin_attrs,
};

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

	dev_set_drvdata(dev, bus);
	ret = of_platform_populate(dev->of_node, NULL, NULL, dev);
	if (ret < 0)
		return ret;

	ret = sysfs_create_group(&dev->kobj, &tsisa_sysfs_group);
	if(ret < 0)
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
