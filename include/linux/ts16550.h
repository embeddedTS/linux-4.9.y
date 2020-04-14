#ifndef __TS16550_H__
#define __TS16550_H__

#include <linux/serial_8250.h>
#include <linux/tspc104_bus.h>
#include <linux/ts16550.h>

struct ts16550_priv {
	struct tspc104_bus *bus;
	struct uart_8250_port uart;
	struct device *dev;
	unsigned int base;
};

#endif //__TS16550_H__