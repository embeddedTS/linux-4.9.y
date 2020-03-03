/*
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _TS_PC104_H
#define _TS_PC104_H

#define TSISA_GOBSY	(1 << 31) /* Addr if 0, go if 1 */
#define TSISA_IO	(1 << 30) /* io=1, mem=0 */
#define TSISA_8BIT	(1 << 29) /* 8bit=1, 16-bit=0 */
#define TSISA_RDEN	(1 << 28) /* read=1, write=0 */
#define TSISA_TS	(1 << 27) /* 1=TS PC104 pinout, 0=standard pinout */

struct tspc104_bus;

extern int tspc104_reg_read(struct tspc104_bus *bus, unsigned int reg, 
			      unsigned int *val, uint32_t flags);
extern int tspc104_reg_write(struct tspc104_bus *bus, unsigned int reg, 
			      unsigned int *val, uint32_t flags);

static inline int tspc104_io_read8(struct tspc104_bus *bus, unsigned int reg,
			       unsigned int *val)
{
	return tspc104_reg_read(bus, reg, val, TSISA_8BIT | TSISA_IO);
}

static inline int tspc104_io_read16(struct tspc104_bus *bus, unsigned int reg,
			       unsigned int *val)
{
	return tspc104_reg_read(bus, reg, val, TSISA_IO);
}

static inline int tspc104_mem_read8(struct tspc104_bus *bus, unsigned int reg,
			       unsigned int *val)
{
	return tspc104_reg_read(bus, reg, val, TSISA_8BIT);
}

static inline int tspc104_mem_read16(struct tspc104_bus *bus, unsigned int reg,
			       unsigned int *val)
{
	return tspc104_reg_read(bus, reg, val, 0);
}

static inline int tspc104_io_write8(struct tspc104_bus *bus, unsigned int reg,
			       unsigned int *val)
{
	return tspc104_reg_write(bus, reg, val, TSISA_8BIT | TSISA_IO);
}

static inline int tspc104_io_write16(struct tspc104_bus *bus, unsigned int reg,
			       unsigned int *val)
{
	return tspc104_reg_write(bus, reg, val, TSISA_IO);
}

static inline int tspc104_mem_write8(struct tspc104_bus *bus, unsigned int reg,
			       unsigned int *val)
{
	return tspc104_reg_write(bus, reg, val, TSISA_8BIT);
}

static inline int tspc104_mem_write16(struct tspc104_bus *bus, unsigned int reg,
			       unsigned int *val)
{
	return tspc104_reg_write(bus, reg, val, 0);
}

/* TS special pinout modes */
static inline int tspc104_mem_write16_altpinout(struct tspc104_bus *bus, unsigned int reg,
			       unsigned int *val)
{
	return tspc104_reg_write(bus, reg, val, TSISA_TS);
}

static inline int tspc104_mem_read16_altpinout(struct tspc104_bus *bus, unsigned int reg,
			       unsigned int *val)
{
	return tspc104_reg_read(bus, reg, val, TSISA_TS);
}

static inline int tspc104_io_write16_altpinout(struct tspc104_bus *bus, unsigned int reg,
			       unsigned int *val)
{
	return tspc104_reg_write(bus, reg, val, TSISA_IO | TSISA_TS);
}

static inline int tspc104_io_read16_altpinout(struct tspc104_bus *bus, unsigned int reg,
			       unsigned int *val)
{
	return tspc104_reg_read(bus, reg, val, TSISA_IO | TSISA_TS);
}

#endif /* _TS_PC104_H */
