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
 * @file peripherals/adxrs290_spi.c
 *
 * Driver for ADXRS290 3-axis gyroscope from ST using SPI.
 */

#include "peripherals/adxrs290_spi.h"
#include "../arch/stm32/subsystems/imu/imu_adiv1_arch.h"
#include "../subsystems/imu/imu_adiv1.h"
#include <libopencm3/stm32/gpio.h>

void adxrs290_spi_init(struct Adxrs290_Spi *adxrs, struct spi_periph *spi_p, uint8_t slave_idx)
{
	/* set spi_peripheral */
	adxrs->spi_p = spi_p;

	/* configure spi transaction */
	adxrs->spi_trans.cpol = SPICpolIdleHigh;
	adxrs->spi_trans.cpha = SPICphaEdge2;
	adxrs->spi_trans.dss = SPIDss8bit;
	adxrs->spi_trans.bitorder = SPIMSBFirst;
	adxrs->spi_trans.cdiv = SPIDiv64;

	adxrs->spi_trans.select = SPISelectUnselect;
	adxrs->spi_trans.slave_idx = slave_idx;
	adxrs->spi_trans.output_length = 2;
	adxrs->spi_trans.input_length = 8;
	// callback currently unused
	adxrs->spi_trans.before_cb = NULL;
	adxrs->spi_trans.after_cb = NULL;
	adxrs->spi_trans.input_buf = &(adxrs->rx_buf[0]);
	adxrs->spi_trans.output_buf = &(adxrs->tx_buf[0]);

	/* set inital status: Success or Done */
	adxrs->spi_trans.status = SPITransDone;

	/* set default ADXRS290 config options */
	adxrs290_set_default_config(&(adxrs->config));

	adxrs->initialized = FALSE;
	adxrs->data_available = FALSE;
	adxrs->init_status = ADXRS290_CONF_UNINIT;
}


static void adxrs290_spi_write_to_reg(struct Adxrs290_Spi *adxrs, uint8_t _reg, uint8_t _val)
{
	adxrs->spi_trans.output_length = 2;
	adxrs->spi_trans.input_length = 0;
	adxrs->tx_buf[0] = _reg;
	adxrs->tx_buf[1] = _val;
	spi_submit(adxrs->spi_p, &(adxrs->spi_trans));
}

// Configuration function called once before normal use
static void adxrs290_spi_send_config(struct Adxrs290_Spi *adxrs)
{
	uint8_t reg_val = 0;

	switch (adxrs->init_status)
	{
	case ADXRS290_CONF_DEV_ID:
		/* query device id */
		adxrs->spi_trans.output_length = 1;
		adxrs->spi_trans.input_length = 2;
		/* set read bit then reg address */
		adxrs->tx_buf[0] = (1 << 7 | ADXRS290_REG_DEV_ID);
		if (spi_submit(adxrs->spi_p, &(adxrs->spi_trans)))
		{
			adxrs->init_status++;
		}
		break;
	case ADXRS290_CONF_DF:
		reg_val = adxrs->config.df;
		adxrs290_spi_write_to_reg(adxrs, ADXRS290_REG_FILTER, reg_val);
		adxrs->init_status++;
		break;
	case ADXRS290_CONF_DADY:
		reg_val = adxrs->config.dady;
		adxrs290_spi_write_to_reg(adxrs, ADXRS290_REG_DATA_READY, reg_val);
		adxrs->init_status++;
		break;
	case ADXRS290_CONF_PWR:
		reg_val = adxrs->config.pwr;
		adxrs290_spi_write_to_reg(adxrs, ADXRS290_REG_POWER_CTL, reg_val);
		adxrs->init_status++;
		break;
	case ADXRS290_CONF_DONE:
		adxrs->initialized = TRUE;
		adxrs->spi_trans.status = SPITransDone;
		break;
	default:
		break;
	}
}

void adxrs290_spi_start_configure(struct Adxrs290_Spi *adxrs)
{
	if (adxrs->init_status == ADXRS290_CONF_UNINIT)
	{
		adxrs->init_status++;
		if (adxrs->spi_trans.status == SPITransSuccess || adxrs->spi_trans.status == SPITransDone)
		{
			adxrs290_spi_send_config(adxrs);
		}
	}
}

void adxrs290_spi_read(struct Adxrs290_Spi *adxrs, uint8_t axis)
{
	if(adxrs->initialized && (adxrs->spi_trans.status == SPITransDone) )
	{
#if 0
		adxrs->spi_trans.output_length = 1;
		adxrs->spi_trans.input_length = 5;
		/* set read bit and multiple byte bit, then address */
		adxrs->tx_buf[0] = (1 << 7 | ADXRS290_REG_OUT_X_L);
		spi_submit(adxrs->spi_p, &(adxrs->spi_trans));
#endif

#if 1
		if(axis == 1)
		{
			if(gpio_get(ADXRS_XY_EOC_PIN_PORT,ADXRS_XY_EOC_PIN) == ADXRS_XY_EOC_PIN)
			{
				adxrs->spi_trans.output_length = 1;
				adxrs->spi_trans.input_length = 5;
				/* set read bit and multiple byte bit, then address */
				adxrs->tx_buf[0] = (1 << 7 | ADXRS290_REG_OUT_X_L);
				spi_submit(adxrs->spi_p, &(adxrs->spi_trans));
				imu_adiv1.adxrs290_xy_eoc = 0;
			}
		}
		else if(axis == 2)
		{
			if(gpio_get(ADXRS_Z_EOC_PIN_PORT,ADXRS_Z_EOC_PIN) == ADXRS_Z_EOC_PIN)
			{
				adxrs->spi_trans.output_length = 1;
				adxrs->spi_trans.input_length = 5;
				/* set read bit and multiple byte bit, then address */
				adxrs->tx_buf[0] = (1 << 7 | ADXRS290_REG_OUT_X_L);
				spi_submit(adxrs->spi_p, &(adxrs->spi_trans));
				imu_adiv1.adxrs290_z_eoc = 0;
			}
		}
#endif
	}
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx+1]<<8) | _buf[_idx]))

uint8_t gyro_test = 0;
void adxrs290_spi_event(struct Adxrs290_Spi *adxrs)
{
	if (adxrs->initialized)
	{
		if (adxrs->spi_trans.status == SPITransFailed)
		{
			adxrs->spi_trans.status = SPITransDone;
		}
		else if (adxrs->spi_trans.status == SPITransSuccess)
		{
			// Successfull reading
			//if( (adxrs->rx_buf[1]!=0) && (adxrs->rx_buf[1]!=255))
			{
				// new xyz data available
				adxrs->data.rates.p = Int16FromBuf(adxrs->rx_buf, 1);//x
				adxrs->data.rates.q = Int16FromBuf(adxrs->rx_buf, 3);//y
				//adxrs->temp = Int16FromBuf(adxrs->rx_buf, 5);
				adxrs->data_available = TRUE;
			}
			adxrs->spi_trans.status = SPITransDone;
		}
	}
	else if (adxrs->init_status != ADXRS290_CONF_UNINIT)     // Configuring but not yet initialized
	{
		switch (adxrs->spi_trans.status)
		{
		case SPITransFailed:
			adxrs->init_status--; // Retry config (TODO max retry)
		case SPITransSuccess:
			if (adxrs->init_status == ADXRS290_CONF_DEV_ID_OK)
			{
				if(adxrs->rx_buf[1] == ADXRS290_DEV_ID_VAL)
				{
					adxrs->init_status++;
					gyro_test = 20;
				}
				else
				{
					adxrs->init_status = ADXRS290_CONF_DEV_ID;
				}
			}
		case SPITransDone:
			adxrs->spi_trans.status = SPITransDone;
			adxrs290_spi_send_config(adxrs);
			break;
		default:
			break;
		}
	}
}
