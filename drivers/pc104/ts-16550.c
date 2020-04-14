#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/tspc104_bus.h>
#include <linux/ts16550.h>

static int technologic_ts16550_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ts16550_priv *priv;
	struct uart_port *port;
	const __be32 *addr_be;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		return -ENOMEM;
	}
	pdev->dev.platform_data = &priv;
	port = &priv->uart.port;

	addr_be = of_get_property(dev->of_node, "reg", NULL);
	if (!addr_be) 
		return -ENODEV;

	priv->uart.port.irq = platform_get_irq(pdev, 0);
	if(priv->uart.port.irq < 0){
		return priv->uart.port.irq;
	}

	priv->base = be32_to_cpup(addr_be);
	priv->bus = platform_get_drvdata(to_platform_device(pdev->dev.parent));
	priv->dev = &pdev->dev;
	port->flags = UPF_SHARE_IRQ | UPF_BOOT_AUTOCONF |
				UPF_FIXED_PORT | UPF_FIXED_TYPE;
	port->iotype = UPIO_TSISABUS;
	port->irqflags = IRQF_TRIGGER_HIGH;
	port->private_data = priv;
	port->uartclk = 1843200;
	port->fifosize = 16;
	port->type = PORT_16550A;

	ret = serial8250_register_8250_port(&priv->uart);

	if(ret < 0)
		return ret;

	dev_info(dev, "Adding 16550 UART ttyS%d\n", ret);

	return 0;
}

static const struct of_device_id ts16550_of_match[] = {
	{ .compatible = "technologic,ts16550", },
	{},
};

static struct platform_driver ts16550_driver = {
	.probe = technologic_ts16550_probe,
	.driver = {
		.name = "ts16550",
		.of_match_table = ts16550_of_match,
	},
};
module_platform_driver(ts16550_driver);

MODULE_ALIAS("platform:ts16550");
MODULE_AUTHOR("Mark Featherston <mark@embeddedarm.com>");
MODULE_DESCRIPTION("Technologic 16550 PC104 driver");
MODULE_LICENSE("GPL v2");
