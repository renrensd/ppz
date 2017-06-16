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
 * @file peripherals/adxrs290_spi.h
 *
 * Driver for ADXRS290 3-axis gyroscope from ST using SPI.
 */

#ifndef ADXRS290_SPI_H
#define ADXRS290_SPI_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/spi.h"

/* Include common ADXRS290 options and definitions */
#include "peripherals/adxrs290.h"

struct Adxrs290_Spi
{
	struct spi_periph *spi_p;
	struct spi_transaction spi_trans;
	volatile uint8_t tx_buf[2];
	volatile uint8_t rx_buf[8];
	enum Adxrs290ConfStatus init_status; ///< init status
	bool_t initialized;                  ///< config done flag
	volatile bool_t data_available;      ///< data ready flag
	uint16_t temp;
	union
	{
		struct Int16Rates rates;           ///< data vector in accel coordinate system
		int16_t value[3];                 ///< data values accessible by channel index
	} data;
	struct Adxrs290Config config;
};

// Functions
extern void adxrs290_spi_init(struct Adxrs290_Spi *adxrs290, struct spi_periph *spi_p, uint8_t addr);
extern void adxrs290_spi_start_configure(struct Adxrs290_Spi *adxrs290);
extern void adxrs290_spi_read(struct Adxrs290_Spi *adxrs290, uint8_t axis);
extern void adxrs290_spi_event(struct Adxrs290_Spi *adxrs290);

/// convenience function: read or start configuration if not already initialized
static inline void adxrs290_spi_periodic(struct Adxrs290_Spi *adxrs290, uint8_t axis)
{
	if( adxrs290->initialized)
	{
		adxrs290_spi_read(adxrs290,axis);
	}
	else
	{
		adxrs290_spi_start_configure(adxrs290);
	}
}

#endif // ADXRS290_SPI_H
