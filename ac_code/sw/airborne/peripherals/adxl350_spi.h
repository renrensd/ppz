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
 * @file peripherals/adxl350_spi.h
 *
 * Driver for the accelerometer ADXL350 from Analog Devices using SPI.
 */

#ifndef ADXL350_SPI_H
#define ADXL350_SPI_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/spi.h"

/* Include common ADXL350 options and definitions */
#include "peripherals/adxl350.h"


struct Adxl350_Spi {
  struct spi_periph *spi_p;
  struct spi_transaction spi_trans;
  volatile uint8_t tx_buf[7];
  volatile uint8_t rx_buf[7];
  enum Adxl350ConfStatus init_status; ///< init status
  bool_t initialized;                 ///< config done flag
  volatile bool_t data_available;     ///< data ready flag
  union {
    struct Int16Vect3 vect;           ///< data vector in accel coordinate system
    int16_t value[3];                 ///< data values accessible by channel index
  } data;
  struct Adxl350Config config;
};

// Functions
extern void adxl350_spi_init(struct Adxl350_Spi *adxl, struct spi_periph *spi_p, uint8_t addr);
extern void adxl350_spi_start_configure(struct Adxl350_Spi *adxl);
extern void adxl350_spi_read(struct Adxl350_Spi *adxl);
extern void adxl350_spi_event(struct Adxl350_Spi *adxl);

/// convenience function: read or start configuration if not already initialized
static inline void adxl350_spi_periodic(struct Adxl350_Spi *adxl)
{
  if (adxl->initialized) {
    adxl350_spi_read(adxl);
  } else {
    adxl350_spi_start_configure(adxl);
  }
}

#endif // ADXL350_SPI_H
