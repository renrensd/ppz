/*
 * Copyright (C) 2011 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *               2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file peripherals/qmc5883.c
 *
 * Driver for QMC5883 magnetometers.
 */

#ifndef QMC5883_H
#define QMC5883_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "math/pprz_algebra_int.h"

/* Address and register definitions */
#include "peripherals/qmc5883_regs.h"

struct Qmc5883Config {
  uint8_t rate;  ///< Data Output Rate Bits,ODR)
  uint8_t rng;   ///< magnetic field measurement range or sensitivity of the sensors
  uint8_t osr;   ///< over sampling rate
  uint8_t mode;	 //
  uint8_t period;	//set/reset period register
};

/** config status states */
enum Qmc5883ConfStatus {
  QMC_CONF_UNINIT,
  QMC_CONF_FBR,
  QMC_CONF_CTL,
  QMC_CONF_DONE
};

/** read status states */
enum Qmc5883ReadStatus
{
  QMC_READ_STATUS,
  QMC_READ_DATA
};

struct Qmc5883 {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  bool_t initialized;                 ///< config done flag
  enum Qmc5883ConfStatus init_status; ///< init status
  enum Qmc5883ReadStatus read_status;
  volatile bool_t data_available;     ///< data ready flag
  union {
    struct Int16Vect3 vect;           ///< data vector in mag coordinate system
    int16_t value[3];                 ///< data values accessible by channel index
  } data;
  struct Qmc5883Config config;
  uint16_t adc_overflow_cnt;          ///< counts number of ADC measurement under/overflows
};


// TODO IRQ handling

// Functions
extern void qmc5883_init(struct Qmc5883 *hmc, struct i2c_periph *i2c_p, uint8_t addr);
extern void qmc5883_start_configure(struct Qmc5883 *hmc);
extern void qmc5883_read_data(struct Qmc5883 *hmc);
extern void qmc5883_read_status(struct Qmc5883 *hmc);
extern void qmc5883_event(struct Qmc5883 *hmc);
extern void qmc5883_periodic(struct Qmc5883 *hmc);


#endif /* QMC5883_H */
