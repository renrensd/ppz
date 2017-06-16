/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file peripherals/adxl350.h
 *
 * Analog Devices ADXL350 accelerometer driver common interface (I2C and SPI).
 */

#ifndef ADXL350_H
#define ADXL350_H

#include "std.h"

/* Include address and register definition */
#include "peripherals/adxl350_regs.h"

enum Adxl350ConfStatus
{
	ADXL_CONF_UNINIT = 0,
	ADXL_CONF_RATE   = 1,
	ADXL_CONF_INT    = 2,
	ADXL_CONF_FORMAT = 3,
	ADXL_CONF_ENABLE = 4,
	ADXL_CONF_DONE   = 5
};

struct Adxl350Config
{
	bool_t drdy_int_enable;   ///< Enable Data Ready Interrupt
	bool_t int_invert;        ///< Invert Interrupt FALSE: active high, TRUE: active low
	bool_t full_res;          ///< Full Resolution: FALSE: 10bit, TRUE: full with 4mg/LSB
	bool_t justify_msb;       ///< justify: FALSE: right with sign-extension, TRUE: MSB (left)
	bool_t self_test;         ///< Enable self-test-force causing shift in output data.
	bool_t spi_3_wire;        ///< Set 3-wire SPI mode, if FALSE: 4-wire SPI mode
	enum Adxl350Ranges range; ///< g Range
	enum Adxl350Rates rate;   ///< Data Output Rate
};

static inline void adxl350_set_default_config(struct Adxl350Config *c)
{
	c->drdy_int_enable = FALSE;
	c->int_invert = TRUE;
	c->full_res = TRUE;
	c->justify_msb = FALSE;
	c->self_test = FALSE;
	c->spi_3_wire = FALSE;

	c->rate = ADXL350_RATE_400HZ;
	c->range = ADXL350_RANGE_4G;
}

static inline uint8_t adxl350_data_format(struct Adxl350Config *c)
{
	return ((c->self_test << 7) | (c->spi_3_wire << 6) | (c->int_invert << 5) |
					(c->full_res << 3) | (c->justify_msb << 2) | (c->range));
}

#endif // ADXL350_H
