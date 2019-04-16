/*
 *  Driver for TS-7120 FPGA interrupt-controller
 *
 *  Copyright (C) 2019 Technologic Systems Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/types.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_device.h>


/**
	This driver is essentially an irq_chip, chained to the interrupt
	referenced in the device-tree node (gpio5 #1 on the TS-7120).
	When loaded, a new irq domain named 'fpga_int' is created, with
	several new interrupt numbers.  These interrupts are then available
	for the FPGA's various peripherals (such as uarts etc).
*/


#define DRIVER_NAME "ts7120-intc"

#define TS7120_IRQ_STATUS		0x24
#define TS7120_IRQ_MASK			0x48
#define TS7120_NUM_FPGA_IRQ	17

static struct TS7120_intc_priv {
	void __iomem  *syscon;
	struct irq_domain *irqdomain;
	int irq;
	u32 mask;
} priv;

static const struct of_device_id TS7120_intc_of_match_table[] = {
	{
		.compatible = "technologic,TS7120-intc",
	},

	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, TS7120_intc_of_match_table);

static void ts7120_intc_mask(struct irq_data *d)
{
	if (priv.syscon) {
		unsigned int mask = readl(priv.syscon + TS7120_IRQ_MASK) & ~BIT(d->hwirq) ;
		priv.mask = mask;
		writel(mask, priv.syscon + TS7120_IRQ_MASK);
	}
}

static void ts7120_intc_unmask(struct irq_data *d)
{
	if (priv.syscon) {
		unsigned int mask = readl(priv.syscon + TS7120_IRQ_MASK) | BIT(d->hwirq) ;
		priv.mask = mask;
		writel(mask, priv.syscon + TS7120_IRQ_MASK);
	}
}

static void  ts7120_irq_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int irq;

	chained_irq_enter(chip, desc);

	if (priv.syscon) {
		unsigned int status;

		while ((status = (priv.mask & readl(priv.syscon + TS7120_IRQ_STATUS)))) {
			irq = 0;
			do {
				if (status & 1)
					generic_handle_irq(irq_linear_revmap(priv.irqdomain, irq));
				status >>= 1;
				irq++;
			} while (status);
		}
	}

	chained_irq_exit(chip, desc);
}

static struct irq_chip ts7120_irq_chip = {
	.name		= "ts7120_intc",
	.irq_mask	= ts7120_intc_mask,
	.irq_unmask	= ts7120_intc_unmask,
};


static int ts7120_intc_irqdomain_map(struct irq_domain *d,
		unsigned int irq, irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &ts7120_irq_chip,
				 handle_level_irq);

	irq_clear_status_flags(irq, IRQ_NOREQUEST | IRQ_NOPROBE);

	return 0;
}

static struct irq_domain_ops ts7120_intc_irqdomain_ops = {
	.map = ts7120_intc_irqdomain_map,
};


static int TS7120_intc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct device_node *np =  pdev->dev.of_node;
	void __iomem  *membase;
	struct resource *res = 0;

	match = of_match_device(TS7120_intc_of_match_table, dev);
	if (!match)
		return -EINVAL;

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

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		pr_err("Can't get interrupt\n");
		return -EFAULT;
	}

	priv.irq = res->start;
	priv.syscon = membase;

	priv.irqdomain = irq_domain_add_linear(np, TS7120_NUM_FPGA_IRQ,
						 &ts7120_intc_irqdomain_ops, &priv);

	if (!priv.irqdomain) {
		pr_err("%s: unable to add irq domain\n", np->name);
		return -ENOMEM;
	}

	priv.mask = 0;
	writel(0, priv.syscon + TS7120_IRQ_MASK);

	irq_set_handler_data(priv.irq, &priv);
	irq_set_chained_handler(priv.irq, ts7120_irq_handler);

	platform_set_drvdata(pdev, &priv);

	pr_info("ts7120-intc IRQ domain created\n");

	return 0;
}

static int TS7120_intc_remove(struct platform_device *pdev)
{
	if (priv.irqdomain) {
		int i, irq;
		for(i=0; i < TS7120_NUM_FPGA_IRQ; i++) {
			if ((irq = irq_find_mapping(	priv.irqdomain, i)) > 0)
				irq_dispose_mapping(irq);
		}
		irq_domain_remove(priv.irqdomain);
		priv.irqdomain = NULL;
	}

	return 0;
}


static struct platform_driver TS7120_intc_driver = {
	.driver = {
		.name = "TS7120-intc",
		.of_match_table = of_match_ptr(TS7120_intc_of_match_table),
	},
	.probe = TS7120_intc_probe,
	.remove = TS7120_intc_remove,
};
module_platform_driver(TS7120_intc_driver);

MODULE_AUTHOR("Technologic Systems");
MODULE_DESCRIPTION("Interrupt Controller for Technologic Systems TS-7120 FPGA");
MODULE_LICENSE("GPL");
