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
 * @todo DRDY/IRQ handling
 */

#include "peripherals/qmc5883.h"
#include "mcu_periph/sys_time.h"
#include "std.h"


/** QMC5883 startup delay
 *
 *  On startup, the qmc is making a first conversion in single mode.
 *  Trying to configure the mode register before the end of this conversion
 *  seems to void the configuration.
 *  Default conversion rate is 15 Hz (66ms) and worst case is O.75Hz (1.3s).
 *  Let set the default delay to 1.5s afer boot time.
 */
#ifndef QMC5883_STARTUP_DELAY
#define QMC5883_STARTUP_DELAY 1.5
#endif

static void qmc5883_set_default_config(struct Qmc5883Config *c)
{
  c->rate = QMC5883_DEFAULT_ODR;
  c->rng = QMC5883_DEFAULT_RNG;
  c->osr = QMC5883_DEFAULT_OSR;
  c->mode = QMC5883_DEFAULT_MODE;
}

/**
 * Initialize Qmc5883 struct and set default config options.
 * @param qmc   Qmc5883 struct
 * @param i2c_p I2C periperal to use
 * @param addr  I2C address of HMC58xx
 */
void qmc5883_init(struct Qmc5883 *qmc, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  qmc->i2c_p = i2c_p;
  /* set i2c address */
  qmc->i2c_trans.slave_addr = addr;
  qmc->i2c_trans.status = I2CTransDone;
  /* set default config options */
  qmc5883_set_default_config(&(qmc->config));
  qmc->initialized = FALSE;
  qmc->init_status = HMC_CONF_UNINIT;
  qmc->adc_overflow_cnt = 0;
}

static void qmc5883_i2c_tx_reg(struct Qmc5883 *qmc, uint8_t reg, uint8_t val)
{
  qmc->i2c_trans.type = I2CTransTx;
  qmc->i2c_trans.buf[0] = reg;
  qmc->i2c_trans.buf[1] = val;
  qmc->i2c_trans.len_r = 0;
  qmc->i2c_trans.len_w = 2;
  i2c_submit(qmc->i2c_p, &(qmc->i2c_trans));
}

/// Configuration function called once before normal use
static void qmc5883_send_config(struct Qmc5883 *qmc)
{
  switch (qmc->init_status) {
    case HMC_CONF_FBR:
      qmc5883_i2c_tx_reg(qmc, QMC5883_REG_FBR,0x01);
      qmc->init_status++;
      break;
    case HMC_CONF_CTL:
      qmc5883_i2c_tx_reg(qmc, QMC5883_REG_CTL1,qmc->config.rate | qmc->config.rng | qmc->config.osr | qmc->config.mode );
      qmc->init_status++;
      break;
    case HMC_CONF_DONE:
      qmc->initialized = TRUE;
      qmc->i2c_trans.status = I2CTransDone;
      break;
    default:
      break;
  }
}

// Configure
void qmc5883_start_configure(struct Qmc5883 *qmc)
{
  // wait before starting the configuration
  // doing to early may void the mode configuration
  if (qmc->init_status == HMC_CONF_UNINIT && get_sys_time_float() > QMC5883_STARTUP_DELAY) {
    qmc->init_status++;
    if (qmc->i2c_trans.status == I2CTransSuccess || qmc->i2c_trans.status == I2CTransDone) {
      qmc5883_send_config(qmc);
    }
  }
}

// Normal reading
void qmc5883_read(struct Qmc5883 *qmc)
{
  if (qmc->initialized && qmc->i2c_trans.status == I2CTransDone) {
    qmc->i2c_trans.buf[0] = QMC5883_REG_DATXL;
    qmc->i2c_trans.type = I2CTransTxRx;
    qmc->i2c_trans.len_r = 6;
    qmc->i2c_trans.len_w = 1;
    i2c_submit(qmc->i2c_p, &(qmc->i2c_trans));
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx+1]<<8) | _buf[_idx]))

void qmc5883_event(struct Qmc5883 *qmc)
{
  if (qmc->initialized) {
    if (qmc->i2c_trans.status == I2CTransFailed) {
      qmc->i2c_trans.status = I2CTransDone;
    } else if (qmc->i2c_trans.status == I2CTransSuccess) 
    {

        qmc->data.vect.x = Int16FromBuf(qmc->i2c_trans.buf, 0);
        qmc->data.vect.y = Int16FromBuf(qmc->i2c_trans.buf, 2);
        qmc->data.vect.z = Int16FromBuf(qmc->i2c_trans.buf, 4);

		qmc->data_available = TRUE;
		#if 0
		/* only set available if measurements valid: -4096 if ADC under/overflow in sensor */
		if (qmc->data.vect.x != -4096 && qmc->data.vect.y != -4096 &&
		    qmc->data.vect.z != -4096) 
		{
		  qmc->data_available = TRUE;
		}
		else 
		{
		  qmc->adc_overflow_cnt++;
		}
		#endif
      	qmc->i2c_trans.status = I2CTransDone;
    }
  } else if (qmc->init_status != HMC_CONF_UNINIT) { // Configuring but not yet initialized
    if (qmc->i2c_trans.status == I2CTransSuccess || qmc->i2c_trans.status == I2CTransDone) {
      qmc->i2c_trans.status = I2CTransDone;
      qmc5883_send_config(qmc);
    }
    if (qmc->i2c_trans.status == I2CTransFailed) {
      qmc->init_status--;
      qmc->i2c_trans.status = I2CTransDone;
      qmc5883_send_config(qmc); // Retry config (TODO max retry)
    }
  }
}

/// convenience function: read or start configuration if not already initialized
void qmc5883_periodic(struct Qmc5883 *hmc)
{
  if (hmc->initialized) {
    qmc5883_read(hmc);
  } else {
    qmc5883_start_configure(hmc);
  }
}

