/*
 * i2c watchdog
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/reboot.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/watchdog.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/system_misc.h>

/* NOTE:
 * We have observed that some large drives can have very large startup times.
 * A 5 minute default timeout means that, even with a very large and slow drive,
 * the system should eventually boot and userspace can actually start feeding.
 */
#define TS_DEFAULT_TIMEOUT (60 * 5)

static struct i2c_client *client;

/* This driver supports the embeddedTS 2nd generation WDT in
 * microcontroller.
 *
 * The 2nd gen WDT is still in I2C microcontroller, but follows more "standard"
 * WDT layouts. There are separate registers for the timeout, and a single
 * register bit for a feed. Additionally, this 2nd gen microcontroller setup
 * uses a register mapped address space, similar to a generic I2C EEPROM.
 */

/* The relevant WDT registers are:
 *
 * Timeout in centiseconds:
 *   0x400: LSB
 *   0x401:
 *   0x402:
 *   0x403: MSB
 *
 * Control:
 *   0x404:
 *     7   - Set if last reboot was caused by WDT (RO)
 *     6:2 - Reserved
 *     1:0 -
 *         Set to 0x1 to cause a feed of <Timeout>
 *         Set to 0x2 to sleep for <Timeout>
 *
 *
 * Writing a value of 0 to the Timeout registers and issuing a feed will disable
 * the WDT completely.
 */

#define	WDT_TIMEOUT	0x400
#define	WDT_CTRL	0x404
#define	WDT_FEED_CMD	0x01
#define	WDT_SLEEP_CMD	0x02

static int ts7100_wdt_write(u16 reg, u8 *data, int len)
{
	u8 *data_buf;
	int ret;
	struct i2c_msg msg;

	data_buf = kzalloc(len+2, GFP_KERNEL);
	if (!data_buf) {
		return -ENOMEM;
	}

	/* MSB of reg is written on the bus first */
	data_buf[0] = ((reg >> 8) & 0xFF);
	data_buf[1] = (reg & 0xFF);
	memcpy(&data_buf[2], data, len);

	/* Write 16-bit register address */
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len+2;
	msg.buf = data_buf;

	dev_dbg(&client->dev, "Writing %d byte(s) to 0x%02X%02X\n",
	  len, data_buf[0], data_buf[1]);

	ret = i2c_transfer(client->adapter, &msg, 1);

	kfree(data_buf);

	if (ret != 1) {
		dev_err(&client->dev, "%s: write error, ret=%d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

/* This sets the timeout to <timeout> seconds. Does not issue a feed after. */
static int ts7100_wdt_set_timeout(struct watchdog_device *wdt,
				   unsigned int timeout)
{
	u8 tmp[4];

	dev_dbg(&client->dev, "%s\n", __func__);

	if (watchdog_timeout_invalid(wdt, timeout)) {
		return -EINVAL;
	}

	wdt->timeout = timeout;
	/* WDT has centisecond granularity, kernel is easier with whole
	 * second increments */
	timeout *= 100;
	tmp[0] = timeout & 0xFF;
	tmp[1] = (timeout >> 8) & 0xFF;
	tmp[2] = (timeout >> 16) & 0xFF;
	tmp[3] = (timeout >> 24) & 0xFF;

	return ts7100_wdt_write(WDT_TIMEOUT, tmp, 4);
}

static int ts7100_wdt_start(struct watchdog_device *wdt)
{
	u8 buf = WDT_FEED_CMD;

	dev_dbg(&client->dev, "%s\n", __func__);
	dev_dbg(&client->dev, "Feeding WDT with %d s\n", wdt->timeout);

	return ts7100_wdt_write(WDT_CTRL, &buf, 1);
}

static int ts_wdt_stop(struct watchdog_device *wdt)
{
	u8 buf = WDT_FEED_CMD;

	dev_dbg(&client->dev, "%s\n", __func__);

	/* Feeding with a timeout of 0 will disable WDT */
	ts7100_wdt_set_timeout(wdt, 0);
	return ts7100_wdt_write(WDT_CTRL, &buf, 1);
}

static int ts7100_wdt_restart(struct watchdog_device *wdt,
  unsigned long a, void *b)
{
	u8 buf = WDT_FEED_CMD;

	dev_dbg(&client->dev, "%s\n", __func__);

	/* Set WDT for 1 s timeout and stall CPU */
	ts7100_wdt_set_timeout(wdt, 1);
	ts7100_wdt_write(WDT_CTRL, &buf, 1);

	while(1);

	return 0;
}

static struct watchdog_info ts7100_wdt_ident = {
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING |
				WDIOF_MAGICCLOSE,
	.identity	= "TS-7100 uC Watchdog",
};

/* NOTE:
 * If .ping is not provided, .start is used. Both would end up doing the same
 * operation anyway.
 * WDIOC_KEEPALIVE ioctl() still functions when .ping is not defined (.start is
 * called instead), so long as WDIOF_KEEPALIVEPING is set as an option
 */
static struct watchdog_ops ts7100_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= ts7100_wdt_start,
	.stop		= ts_wdt_stop,
	.set_timeout	= ts7100_wdt_set_timeout,
	.restart	= ts7100_wdt_restart,
	/* .ioctl unneeded, watchdog core handles the common/necessary ioctls */

};

static struct watchdog_device ts_wdt_wdd = {
	.info			= &ts7100_wdt_ident,
	.ops			= &ts7100_wdt_ops,
	/* A timeout of 0 means disable. Due to how the validity of the timeout
	 * is checked, we need to spec a minimum timeout of 0. This allows the
	 * FDT to spec a timeout of 0, disabling the WDT on startup. This may
	 * cause issues in specific scenarios.
	 */
	.min_timeout		= 0,
	.max_timeout		= 4294967,
	.status			= WATCHDOG_NOWAYOUT_INIT_STATUS,
};

static int ts7100_wdt_probe(struct i2c_client *c,
			  const struct i2c_device_id *id)
{
	int err;
	u32 t;
	client = c;

	dev_dbg(&client->dev, "%s\n", __func__);

	/* TODO: Support power off events through the microcontroller
	pm_power_off = do_ts_halt;
	*/

	watchdog_set_restart_priority(&ts_wdt_wdd, 255);

	/* Attempt to read default value from FDT. If value is set, AND
	 * it is the valid range, that value will be used. Otherwise, use
	 * default kernel defined value.
	 */
	if (!of_property_read_u32(client->dev.of_node, "timeout-sec", &t) &&
	  !watchdog_timeout_invalid(&ts_wdt_wdd, t)) {
		ts_wdt_wdd.timeout = t;
	} else {
		dev_info(&client->dev,
			"Unable to select timeout value, using default\n");
		ts_wdt_wdd.timeout = TS_DEFAULT_TIMEOUT;
	}

	/* Set the timeout (most likely from DTS) and do a single feed until
	 * userspace takes over finally.
	 */
	ts7100_wdt_set_timeout(&ts_wdt_wdd, ts_wdt_wdd.timeout);
	ts7100_wdt_start(&ts_wdt_wdd);

	err = watchdog_register_device(&ts_wdt_wdd);
	if (err) {
		return err;
	}

	return 0;
}

static const struct i2c_device_id ts7100_wdt_id[] = {
	{ "ts7100-wdt", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ts7100_wdt_id);

static const struct of_device_id ts7100_wdt_of_match[] = {
	{ .compatible = "technologic,ts7100-wdt", },
	{ },
};
MODULE_DEVICE_TABLE(of, ts7100_wdt_of_match);

MODULE_ALIAS("platform:ts7100_wdt");

static struct i2c_driver ts7100_wdt_driver = {
	.driver = {
		.name	= "ts7100_wdt",
		.of_match_table = ts7100_wdt_of_match,
		.owner	= THIS_MODULE,
	},
	.probe		= ts7100_wdt_probe,
	.id_table	= ts7100_wdt_id,
};

static int __init ts_reboot_init(void)
{
	return i2c_add_driver(&ts7100_wdt_driver);
}
subsys_initcall(ts_reboot_init);

static void __exit ts_reboot_exit(void)
{
	i2c_del_driver(&ts7100_wdt_driver);
}
module_exit(ts_reboot_exit);

MODULE_AUTHOR("Kris Bahnsen <kris@embeddedTS.com>");
MODULE_DESCRIPTION("embeddedTS' TS-7100 (and compat.) WDT driver");
MODULE_LICENSE("GPL");
