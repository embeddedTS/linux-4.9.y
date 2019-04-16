/*
 * linux/drivers/spi/spioc.c
 *
 * Copyright (C) 2007-2008 Avionic Design Development GmbH
 * Copyright (C) 2008-2009 Avionic Design GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Written by Thierry Reding <thierry.reding@avionic-design.de>
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

/* register definitions */
#define SPIOC_RX(i)	(i * 4)
#define SPIOC_TX(i)	(i * 4)
#define SPIOC_CTRL	0x10
#define SPIOC_DIV	0x14
#define SPIOC_SS	0x18

/* SPIOC_CTRL register */
#define CTRL_LEN(x)	((x < 128) ? x : 0)
#define CTRL_BUSY	(1 <<  8)
#define CTRL_RXNEG	(1 <<  9)
#define CTRL_TXNEG	(1 << 10)
#define CTRL_LSB	(1 << 11)
#define CTRL_IE		(1 << 12)
#define CTRL_ASS	(1 << 13)


/**
 * struct spioc - driver-specific context information
 * @master:	SPI master device
 * @info:	pointer to platform data
 * @clk:	SPI master clock
 * @irq:	SPI controller interrupt
 * @mmio:	physical I/O memory resource
 * @base:	base of memory-mapped I/O
 * @message:	current SPI message
 * @transfer:	current transfer of current SPI message
 * @nx:		number of bytes sent/received for current transfer
 * @queue:	SPI message queue
 */
struct spioc {
	struct spi_master *master;
	struct clk *clk;
	int irq;
	u32 idx;
	struct platform_device *pdev;
	s16 bus_num;
	u16 num_chipselect;
	struct resource *mmio;
	void __iomem *base;
	struct spi_message *message;
	struct spi_transfer *transfer;
	unsigned long nx;
	struct list_head queue;
	struct workqueue_struct *workqueue;
	struct work_struct process_messages;
	struct tasklet_struct process_transfers;

	spinlock_t lock;
};

static inline u32 spioc_read(struct spioc *spioc, unsigned long offset)
{
	return readl(spioc->base + offset);
}

static inline void spioc_write(struct spioc *spioc, unsigned offset,
		u32 value)
{
	writel(value, spioc->base + offset);
}

static void spioc_chipselect(struct spioc *master, struct spi_device *spi)
{
	if (spi)
		spioc_write(master, SPIOC_SS, 1 << spi->chip_select);
	else
		spioc_write(master, SPIOC_SS, 0);
}

/* count is assumed to be less than or equal to the maximum number of bytes
 * that can be transferred in one go */
static void spioc_copy_tx(struct spioc *spioc, const void *src, size_t count)
{
	u32 val = 0;
	int i;

	for (i = 0; i < count; i++) {
		int rem = count - i;
		int reg = (rem - 1) / 4;
		int ofs = (rem - 1) % 4;

		val |= (((u8 *)src)[i] & 0xff) << (ofs * 8);
		if (!ofs) {
			spioc_write(spioc, SPIOC_TX(reg), val);
			val = 0;
		}
	}
}

static void spioc_copy_rx(struct spioc *spioc, void *dest, size_t count)
{
	u32 val = 0;
	int i;

	for (i = 0; i < count; i++) {
		int rem = count - i;
		int reg = (rem - 1) / 4;
		int ofs = (rem - 1) % 4;

		if ((i == 0) || (rem % 4 == 0))
			val = spioc_read(spioc, SPIOC_RX(reg));

		((u8 *)dest)[i] = (val >> (ofs * 8)) & 0xff;
	}
}

static void process_messages(struct work_struct *work)
{
	struct spioc *spioc =
		container_of(work, struct spioc, process_messages);
	unsigned long flags;

	spin_lock_irqsave(&spioc->lock, flags);

	/* obtain next message */
	if (list_empty(&spioc->queue)) {
		spin_unlock_irqrestore(&spioc->lock, flags);
		return;
	}

	spioc->message = list_entry(spioc->queue.next, struct spi_message,
			queue);
	list_del_init(&spioc->message->queue);

	/* process transfers */
	tasklet_schedule(&spioc->process_transfers);
	spin_unlock_irqrestore(&spioc->lock, flags);
}

static void process_transfers(unsigned long data)
{
	struct spioc *spioc = (struct spioc *)data;
	struct spi_transfer *transfer = spioc->transfer;
	size_t rem;
	u32 ctrl;

	/* if this is the start of a message, get a pointer to the first
	 * transfer */
	if (!transfer || (spioc->nx >= transfer->len)) {
		if (!transfer) {
			transfer = list_entry(spioc->message->transfers.next,
					struct spi_transfer, transfer_list);
			spioc_chipselect(spioc, spioc->message->spi);
		} else {
			struct list_head *next = transfer->transfer_list.next;

			if (next != &spioc->message->transfers) {
				transfer = list_entry(next,
						struct spi_transfer,
						transfer_list);
			} else {
				spioc->message->actual_length = spioc->nx;
				complete(spioc->message->context);
				spioc->transfer = NULL;
				spioc->message->status = 0;
				spioc_chipselect(spioc, NULL);
				return;
			}
		}

		spioc->transfer = transfer;
		spioc->nx = 0;
	}

	/* write data to registers */
	rem = min_t(size_t, transfer->len - spioc->nx, 16);
	if (transfer->tx_buf)
		spioc_copy_tx(spioc, transfer->tx_buf + spioc->nx, rem);

	/* read control register */
	ctrl  = spioc_read(spioc, SPIOC_CTRL);
	ctrl &= ~CTRL_LEN(127);    /* clear length bits */
	ctrl |= CTRL_IE            /* assert interrupt on completion */
		  |  CTRL_LEN(rem * 8); /* set word length */
	spioc_write(spioc, SPIOC_CTRL, ctrl);

	/* start transfer */
	ctrl |= CTRL_BUSY;
	spioc_write(spioc, SPIOC_CTRL, ctrl);
}

static int spioc_setup(struct spi_device *spi)
{
	struct spioc *spioc = spi_master_get_devdata(spi->master);
	unsigned long clkdiv = 0x0000ffff;
	u32 ctrl = spioc_read(spioc, SPIOC_CTRL);

	/* make sure we're not busy */
	ctrl &= ~CTRL_BUSY;

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	if (spi->mode & SPI_LSB_FIRST)
		ctrl |=  CTRL_LSB;
	else
		ctrl &= ~CTRL_LSB;

	/* adapt to clock polarity and phase */
	if (spi->mode & SPI_CPOL) {
		if (spi->mode & SPI_CPHA) {
			ctrl |=  CTRL_TXNEG;
			ctrl &= ~CTRL_RXNEG;
		} else {
			ctrl &= ~CTRL_TXNEG;
			ctrl |=  CTRL_RXNEG;
		}
	} else {
		if (spi->mode & SPI_CPHA) {
			ctrl &= ~CTRL_TXNEG;
			ctrl |=  CTRL_RXNEG;
		} else {
			ctrl |=  CTRL_TXNEG;
			ctrl &= ~CTRL_RXNEG;
		}
	}

	/* set the clock divider */
	if (spi->max_speed_hz)
		clkdiv = DIV_ROUND_UP(clk_get_rate(spioc->clk) ,
				2 * spi->max_speed_hz) - 1;

	if (clkdiv > 0x0000ffff)
		clkdiv = 0x0000ffff;

	spioc_write(spioc, SPIOC_DIV, clkdiv);
	spioc_write(spioc, SPIOC_CTRL, ctrl);

	/* deassert chip-select */
	spioc_chipselect(spioc, NULL);

	return 0;
}

static int spioc_transfer(struct spi_device *spi, struct spi_message *message)
{
	struct spi_master *master = spi->master;
	struct spioc *spioc = spi_master_get_devdata(master);
	unsigned long flags;

	spin_lock_irqsave(&spioc->lock, flags);

	message->actual_length = 0;
	message->status = -EINPROGRESS;

	list_add_tail(&message->queue, &spioc->queue);
	queue_work(spioc->workqueue, &spioc->process_messages);

	spin_unlock_irqrestore(&spioc->lock, flags);
	return 0;
}

static void spioc_cleanup(struct spi_device *spi)
{
}

static irqreturn_t spioc_interrupt(int irq, void *dev_id)
{
	struct spioc *spioc = (struct spioc *)dev_id;

	if (!spioc)
		return IRQ_NONE;

	else {
		struct spi_transfer *transfer = spioc->transfer;
		size_t rem;
		spioc_read(spioc, SPIOC_CTRL);

		/* read data from registers */
		rem = min_t(size_t, transfer->len - spioc->nx, 16);
		if (transfer->rx_buf)
			spioc_copy_rx(spioc, transfer->rx_buf + spioc->nx, rem);
		spioc->nx += rem;

		tasklet_schedule(&spioc->process_transfers);
	}

	return IRQ_HANDLED;
}

static int init_queue(struct spi_master *master, const char *buf)
{
	struct spioc *spioc = spi_master_get_devdata(master);

	if (spioc == NULL) {
		pr_err("%s %d error\n", __func__, __LINE__);
		return -EBUSY;
	}

	/* initialize message workqueue */
	INIT_LIST_HEAD(&spioc->queue);
	spin_lock_init(&spioc->lock);
	INIT_WORK(&spioc->process_messages, process_messages);

	/* initialize transfer processing tasklet */
	tasklet_init(&spioc->process_transfers, process_transfers,
			(unsigned long)spioc);

	spioc->workqueue = create_singlethread_workqueue(
			dev_name(master->dev.parent));

	if (!spioc->workqueue)
		return -EBUSY;

	return 0;
}

static int start_queue(struct spi_master *master)
{
	struct spioc *spioc = spi_master_get_devdata(master);

	WARN_ON(spioc->message  != NULL);
	WARN_ON(spioc->transfer != NULL);

	spioc->message  = NULL;
	spioc->transfer = NULL;

	queue_work(spioc->workqueue, &spioc->process_messages);
	return 0;
}

static int stop_queue(struct spi_master *master)
{
	return 0;
}

static int destroy_queue(struct spi_master *master)
{
	struct spioc *spioc = spi_master_get_devdata(master);
	int retval = 0;

	retval = stop_queue(master);
	if (retval)
		return retval;

	destroy_workqueue(spioc->workqueue);
	return 0;
}


static const struct of_device_id opencores_spi_match[] = {
	{ .compatible = "opencores,spi-oc" },
	{},
};
MODULE_DEVICE_TABLE(of, opencores_spi_match);


static int  spioc_probe(struct platform_device *pdev)
{
	struct resource *res = NULL;
	void __iomem *mmio = NULL;
	int retval = 0, irq;
	struct spi_master *master = NULL;
	struct spioc *spioc = NULL;
	struct device_node *node = pdev->dev.of_node;
	char buf[16];
	u32 idx, num_chipselect;

	if (of_property_read_u32(node, "opencores-spi,idx", &idx) < 0) {
		dev_warn(&pdev->dev, "Node idx not defined, assuming 0\n");
		idx = 0;
	}

	if (of_property_read_u32(node, "opencores-spi,num-chipselects",
			&num_chipselect) < 0) {
		dev_warn(&pdev->dev, "Node num_chipselect not defined, assuming 1\n");
		num_chipselect = 1;
	}

	master = spi_alloc_master(&pdev->dev, sizeof(struct spioc));
	if (master == NULL) {
		dev_err(&pdev->dev, "unable to allocate SPI master\n");
		return -ENOMEM;
	}
	spioc = spi_master_get_devdata(master);
	platform_set_drvdata(pdev, master);

	snprintf(buf, sizeof(buf), "spi_oc_%d", idx);

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "IRQ not defined\n");
		return -ENXIO;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "MMIO resource not defined\n");
		return -ENXIO;
	}

	mmio = devm_ioremap_nocache(&pdev->dev, res->start,
					  resource_size(res));
	if (IS_ERR(mmio)) {
		dev_err(&pdev->dev, "can't remap I/O region\n");
		retval =  PTR_ERR(mmio);
		goto err1;
	}

	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST;
	master->setup = spioc_setup;
	master->transfer = spioc_transfer;
	master->cleanup = spioc_cleanup;
	master->dev.of_node = pdev->dev.of_node;
	master->bus_num = pdev->id;
	master->num_chipselect = num_chipselect;

	spioc->master = master;
	spioc->pdev = pdev;

	spioc->idx = idx;
	spioc->irq = irq;
	spioc->mmio = res;
	spioc->base = mmio;
	spioc->message = NULL;
	spioc->transfer = NULL;
	spioc->bus_num = pdev->id;
	spioc->num_chipselect = num_chipselect;
	spioc->nx = 0;

	spioc->clk = devm_clk_get(&pdev->dev, "spi-oc-clk");
	if (IS_ERR(spioc->clk)) {
		dev_err(&pdev->dev, "unable to get SPI master clock\n");
		retval = PTR_ERR(spioc->clk);
		spioc->clk = NULL;
		goto err1;
	}

	retval = init_queue(master, buf);
	if (retval) {
		dev_err(&pdev->dev, "unable to initialize workqueue\n");
		goto free;
	}

	retval = start_queue(master);
	if (retval) {
		dev_err(&pdev->dev, "unable to start workqueue\n");
		goto free;
	}

	retval = devm_request_irq(&pdev->dev, irq, spioc_interrupt, 0,
		"spioc",	spioc);
	if (retval) {
		dev_err(&pdev->dev, "unable to install handler for IRQ #%d\n", irq);
		retval = -EPROBE_DEFER;
		goto free;
	}

	dev_info(&pdev->dev, "IRQ: %d, CLK: %ldHz\n",
		irq, clk_get_rate(spioc->clk));

	retval = devm_spi_register_master(&pdev->dev, master);
	if (retval) {
		dev_err(&pdev->dev, "unable to register SPI master\n");
		retval = -ENOMEM;
		goto free;
	}

	dev_info(&pdev->dev, "SPI master %d registered\n", idx);

out:
	return retval;

free:
	destroy_queue(master);

err1:
	spi_master_put(master);
	goto out;
}

static int  spioc_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	if (master) {
		spi_master_get(master);
		platform_set_drvdata(pdev, NULL);
		destroy_queue(master);
		spi_master_put(master);
	}

	return 0;
}

#ifdef CONFIG_PM
static int spioc_suspend(struct platform_device *pdev, pm_message_t state)
{
	return -ENOSYS;
}

static int spioc_resume(struct platform_device *pdev)
{
	return -ENOSYS;
}
#else
#define spioc_suspend NULL
#define spioc_resume  NULL
#endif /* CONFIG_PM */

static struct platform_driver spioc_driver = {
	.probe = spioc_probe,
	.remove = spioc_remove,
	.suspend = spioc_suspend,
	.resume = spioc_resume,
	.driver = {
		.name  = "spioc",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(opencores_spi_match),
	},
};

static int __init spioc_init(void)
{
	return platform_driver_register(&spioc_driver);
}

static void __exit spioc_exit(void)
{
	platform_driver_unregister(&spioc_driver);
}

module_init(spioc_init);
module_exit(spioc_exit);

MODULE_AUTHOR("Thierry Reding <thierry.reding@avionic-design.de>");
MODULE_DESCRIPTION("OpenCores SPI controller driver");
MODULE_LICENSE("GPL v2");

