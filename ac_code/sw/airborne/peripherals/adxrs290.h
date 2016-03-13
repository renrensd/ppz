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
 * @file peripherals/adxrs290.h
 *
 * ST ADXRS290 3-axis gyroscope driver common interface (I2C and SPI).
 */

#ifndef ADXRS290_H
#define ADXRS290_H

/* Include address and register definition */
#include "peripherals/adxrs290_regs.h"



enum Adxrs290ConfStatus {
  ADXRS290_CONF_UNINIT = 0,
  ADXRS290_CONF_DEV_ID,
  ADXRS290_CONF_DEV_ID_OK,
  ADXRS290_CONF_DF,
  ADXRS290_CONF_DADY,
  ADXRS290_CONF_PWR,
  ADXRS290_CONF_DONE,
};

struct Adxrs290Config {
  uint8_t pwr;//1->measurement mode; 0->standby mode
  uint8_t df; //Digital Filter
  uint8_t dady;//data ready
};

static inline void adxrs290_set_default_config(struct Adxrs290Config *c)
{
  c->pwr = ADXRS290_TSM_DISABLE | (ADXRS290_PWR_MEAS << 1);
  c->df = ADXRS290_DLPF_480HZ | (ADXRS290_DHPF_0 << 4);
  c->dady = ADXRS290_DATA_READY_VAL;
}

#endif /* ADXRS290_H */
