/*
 * ADC driver for simple 8-bit ADC on TS-7250-V3
 *
 * Copyright (C) 2021-2022 Technologic Systems, Inc. dba embeddedTS
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

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/completion.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>

/*
 * Core is 4 bytes wide in total.
 * Base + 0x00: ADCFLAGS
 *   bits 15-3: Reserved
 *   bit 2: sample_missed (RW)
 * 	Data in 0x02 was overwritten without being read
 * 	(cleared on write or sample read)
 *   bit 1: sample_ready (RO)
 * 	Data in 0x02 is new and has not been read
 * 	(cleared on write or sample read)
 *   bit 0: standby (RW, default 1) - Stop sampling ADC
 * Base + 0x02: 8-bit sample (RO) (upper 8 bits are 0)
 */

#define TS_ADC_FLAGS			0x0
#define TS_ADC_FLAGS_SAMPLE_MISSED	(1<<2)
#define TS_ADC_FLAGS_SAMPLE_READY	(1<<1)
#define TS_ADC_FLAGS_STANDBY		(1<<0)
#define TS_ADC_RESULT			0x2
#define ADC_TIMEOUT_US			50000

struct ts_simple_adc {
	struct resource *mem;
	void __iomem *base;
	u16 value;
	u8 buffer[16] ____cacheline_aligned;
	struct completion completion;
};

static const struct iio_chan_spec ts_simple_adc_iio_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = 
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 8,
			.storagebits = 16,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

void ts_simple_adc_set_standby(struct ts_simple_adc *adc, int en_standby)
{
	if(en_standby) writew(TS_ADC_FLAGS_STANDBY, adc->base + TS_ADC_FLAGS);
	else writew(0x0, adc->base + TS_ADC_FLAGS);
}

static int ts_simple_adc_iio_read_raw(struct iio_dev *iio_dev,
			struct iio_chan_spec const *chan, int *val,
			int *val2, long mask)
{
	struct ts_simple_adc *adc = iio_priv(iio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (iio_buffer_enabled(iio_dev))
			return -EBUSY;
		ts_simple_adc_set_standby(adc, 0);
		reinit_completion(&adc->completion);
		ret = wait_for_completion_interruptible_timeout(&adc->completion,
								ADC_TIMEOUT_US);
		if (ret == 0)
			return -ETIMEDOUT;

		*val = adc->value;
		ts_simple_adc_set_standby(adc, 1);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 6600;
		*val2 = 8;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		break;
	}
	return -EINVAL;
}

static irqreturn_t ts_simple_adc_isr(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = (struct iio_dev *)dev_id;
	struct ts_simple_adc *adc = iio_priv(indio_dev);

	adc->value = readw(adc->base + TS_ADC_RESULT) & 0xFF;

	if (iio_buffer_enabled(indio_dev)) {
		iio_push_to_buffers_with_timestamp(indio_dev,
					&adc->value,
					iio_get_time_ns(indio_dev));
	} else {
		complete(&adc->completion);
	}

	return IRQ_HANDLED;
}

static const struct iio_info ts_simple_adc_info = {
	.read_raw = &ts_simple_adc_iio_read_raw,
	.driver_module = THIS_MODULE,
};

static int ts_simple_adc_buffer_enable(struct iio_dev *indio_dev)
{
	struct ts_simple_adc *adc = iio_priv(indio_dev);

	ts_simple_adc_set_standby(adc, 0);
	return 0;
}

static int ts_simple_adc_buffer_disable(struct iio_dev *indio_dev)
{
	struct ts_simple_adc *adc = iio_priv(indio_dev);

	ts_simple_adc_set_standby(adc, 1);
	return 0;
}

static const struct iio_buffer_setup_ops ts_simple_setup_ops = {
	.postenable = &ts_simple_adc_buffer_enable,
	.predisable = &ts_simple_adc_buffer_disable,
};

static int ts_simple_adc_probe(struct platform_device *pdev)
{
	struct ts_simple_adc *adc_dev;
	struct iio_dev *indio_dev;
	struct resource *mem;
	struct iio_buffer *buffer;
	int irq;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev,
					  sizeof(struct ts_simple_adc));
	if (!indio_dev)
		return -ENOMEM;

	adc_dev = iio_priv(indio_dev);
	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->info = &ts_simple_adc_info;
	indio_dev->modes = INDIO_BUFFER_SOFTWARE;
	indio_dev->channels = ts_simple_adc_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(ts_simple_adc_iio_channels);
	indio_dev->setup_ops = &ts_simple_setup_ops;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return irq;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	init_completion(&adc_dev->completion);
	adc_dev->base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(adc_dev->base)) {
		dev_err(&pdev->dev, "failed to get ts adc base address\n");
		return PTR_ERR(adc_dev->base);
	}

	buffer = devm_iio_kfifo_allocate(&pdev->dev);
	if (!buffer)
		return -ENOMEM;

	iio_device_attach_buffer(indio_dev, buffer);

	ret = devm_request_irq(&pdev->dev, irq,
			       ts_simple_adc_isr, 0,
			       dev_name(&pdev->dev), indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "failed requesting irq, irq = %d\n", irq);
		return ret;
	}

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id tsadc_of_match[] = {
	{ .compatible = "embeddedts,ts-simple-adc", },
	{ }
};
MODULE_DEVICE_TABLE(of, tsadc_of_match);

static struct platform_driver tsadc_driver = {
	.driver = {
		.name   = "ts_adc",
		.of_match_table = tsadc_of_match,
	},
	.probe	= ts_simple_adc_probe,
};
module_platform_driver(tsadc_driver);

MODULE_AUTHOR("Mark Featherston <mark@embeddedTS.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IIO ADC driver embeddedTS Simple FPGA ADC");
