/*
 * Copyright (C) 2013 Gautier Hattenberger
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
 * @file peripherals/mpu9250_spi.c
 *
 * Driver for the MPU-9250 using SPI.
 *
 */

#include "peripherals/mpu9250_spi.h"

void mpu9250_spi_init(struct Mpu9250_Spi *mpu, struct spi_periph *spi_p, uint8_t slave_idx)
{
	/* set spi_peripheral */
	mpu->spi_p = spi_p;

	/* configure spi transaction */
	mpu->spi_trans.cpol = SPICpolIdleHigh;
	mpu->spi_trans.cpha = SPICphaEdge2;
	mpu->spi_trans.dss = SPIDss8bit;
	mpu->spi_trans.bitorder = SPIMSBFirst;
	mpu->spi_trans.cdiv = SPIDiv64;

	mpu->spi_trans.select = SPISelectUnselect;
	mpu->spi_trans.slave_idx = slave_idx;
	mpu->spi_trans.output_length = 2;
	mpu->spi_trans.input_length = MPU9250_BUFFER_LEN;
	mpu->spi_trans.before_cb = NULL;
	mpu->spi_trans.after_cb = NULL;
	mpu->spi_trans.input_buf = &(mpu->rx_buf[0]);
	mpu->spi_trans.output_buf = &(mpu->tx_buf[0]);

	/* set inital status: Success or Done */
	mpu->spi_trans.status = SPITransDone;

	/* set default MPU9250 config options */
	mpu9250_set_default_config(&(mpu->config));

	mpu->data_available = FALSE;
	mpu->config.initialized = FALSE;
	mpu->config.init_status = MPU9250_CONF_UNINIT;

	mpu->slave_init_status = MPU9250_SPI_CONF_UNINIT;
}

static void mpu9250_spi_write_to_reg(void *mpu, uint8_t _reg, uint8_t _val)
{
	struct Mpu9250_Spi *mpu_spi = (struct Mpu9250_Spi *) (mpu);
	mpu_spi->spi_trans.output_length = 2;
	mpu_spi->spi_trans.input_length = 0;
	mpu_spi->tx_buf[0] = _reg;
	mpu_spi->tx_buf[1] = _val;
	spi_submit(mpu_spi->spi_p, &(mpu_spi->spi_trans));
}

// Configuration function called once before normal use
void mpu9250_spi_start_configure(struct Mpu9250_Spi *mpu)
{
	if (mpu->config.init_status == MPU9250_CONF_UNINIT)
	{
		mpu->config.init_status++;
		if (mpu->spi_trans.status == SPITransSuccess || mpu->spi_trans.status == SPITransDone)
		{
			mpu9250_send_config(mpu9250_spi_write_to_reg, (void *) mpu, &(mpu->config));
		}
	}
}

// Configuration function called once before normal use
bool_t mpu9250_spi_start_self_test(struct Mpu9250_Spi *mpu)
{
	return mpu9250_start_self_test(mpu9250_spi_write_to_reg, (void *) mpu, &(mpu->config));
}
bool_t mpu9250_spi_end_self_test(struct Mpu9250_Spi *mpu)
{
	return mpu9250_end_self_test(mpu9250_spi_write_to_reg, (void *) mpu, &(mpu->config));
}

void mpu9250_spi_read(struct Mpu9250_Spi *mpu)
{
	if (mpu->config.initialized && mpu->spi_trans.status == SPITransDone)
	{
		mpu->spi_trans.output_length = 1;
		mpu->spi_trans.input_length = 1 + mpu->config.nb_bytes;
		/* set read bit and multiple byte bit, then address */
		mpu->tx_buf[0] = MPU9250_REG_INT_STATUS | MPU9250_SPI_READ;
		spi_submit(mpu->spi_p, &(mpu->spi_trans));
	}
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))
static inline bool_t DataValidCheck(uint8_t* buf)
{
	uint8_t temp = 0xFF;
	for (uint8_t i = 0; i < 14; i++)
	{
		temp &= buf[i];
	}
	return (temp != 0xFF);
}

void mpu9250_spi_event(struct Mpu9250_Spi *mpu)
{
	if (mpu->config.initialized)
	{
		if (mpu->spi_trans.status == SPITransFailed)
		{
			mpu->spi_trans.status = SPITransDone;
		}
		else if (mpu->spi_trans.status == SPITransSuccess)
		{
			// Successfull reading
			if (bit_is_set(mpu->rx_buf[1], 0) && DataValidCheck(&(mpu->rx_buf[2])))
			{
				// new data
				mpu->data_accel.vect.x = Int16FromBuf(mpu->rx_buf, 2);
				mpu->data_accel.vect.y = Int16FromBuf(mpu->rx_buf, 4);
				mpu->data_accel.vect.z = Int16FromBuf(mpu->rx_buf, 6);
				mpu->data_rates.rates.p = Int16FromBuf(mpu->rx_buf, 10);
				mpu->data_rates.rates.q = Int16FromBuf(mpu->rx_buf, 12);
				mpu->data_rates.rates.r = Int16FromBuf(mpu->rx_buf, 14);

				// if we are reading slaves, copy the ext_sens_data
				if (mpu->config.nb_slaves > 0)
				{
					/* the buffer is volatile, since filled from ISR
					 * but we know it's ok to use it here so we silence the warning
					 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
					memcpy(mpu->data_ext, (uint8_t *) &(mpu->rx_buf[16]), mpu->config.nb_bytes - 15);
#pragma GCC diagnostic pop
				}

				mpu->data_available = TRUE;
			}
			mpu->spi_trans.status = SPITransDone;
		}
	}
	else if (mpu->config.init_status != MPU9250_CONF_UNINIT)
	{
		// Configuring but not yet initialized
		switch (mpu->spi_trans.status)
		{
		case SPITransFailed:
			mpu->config.init_status--; // Retry config (TODO max retry)
		case SPITransSuccess:
		case SPITransDone:
			mpu9250_send_config(mpu9250_spi_write_to_reg, (void *) mpu, &(mpu->config));
			if (mpu->config.initialized)
			{
				mpu->spi_trans.status = SPITransDone;
			}
			break;
		default:
			break;
		}
	}
}

/** @todo: only one slave so far. */
bool_t mpu9250_configure_i2c_slaves(Mpu9250ConfigSet mpu_set, void *mpu)
{
	struct Mpu9250_Spi *mpu_spi = (struct Mpu9250_Spi *) (mpu);

	if (mpu_spi->slave_init_status == MPU9250_SPI_CONF_UNINIT)
	{
		mpu_spi->slave_init_status++;
	}

	switch (mpu_spi->slave_init_status)
	{
	case MPU9250_SPI_CONF_I2C_MST_CLK:
		/* configure MPU I2C master clock and stop/start between slave reads */
		mpu_set(mpu, MPU9250_REG_I2C_MST_CTRL, ((1 << 4) | mpu_spi->config.i2c_mst_clk));
		mpu_spi->slave_init_status++;
		break;
	case MPU9250_SPI_CONF_I2C_MST_DELAY:
		/* Set I2C slaves delayed sample rate */
		mpu_set(mpu, MPU9250_REG_I2C_MST_DELAY, mpu_spi->config.i2c_mst_delay);
		mpu_spi->slave_init_status++;
		break;
	case MPU9250_SPI_CONF_I2C_MST_EN:
		/* enable internal I2C master and disable primary I2C interface */
		mpu_set(mpu, MPU9250_REG_USER_CTRL, ((1 << MPU9250_I2C_IF_DIS) | (1 << MPU9250_I2C_MST_EN)));
		mpu_spi->slave_init_status++;
		break;
	case MPU9250_SPI_CONF_SLAVES_CONFIGURE:
		/* configure first slave, only one slave supported so far */
		if (mpu_spi->config.slaves[0].configure(mpu_set, mpu))
		{
			mpu_spi->slave_init_status++;
		}
		break;
	case MPU9250_SPI_CONF_DONE:
		return TRUE;
	default:
		break;
	}
	return FALSE;
}
