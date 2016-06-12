/*
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
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
 * @file subsystems/imu/imu_adiv1.c
 * Driver for the Adiv1 v1.x IMU using SPI for the accelerometer.
 */

#include "subsystems/imu.h"
#include "subsystems/abi.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/spi.h"

#include "../../arch/stm32/subsystems/imu/imu_adiv1_arch.h"


/* defaults suitable for Lisa */
PRINT_CONFIG_VAR(ADIV1_ACCEL_SPI_SLAVE_IDX)

PRINT_CONFIG_VAR(ADIV1_GYRO_XY_SPI_SLAVE_IDX)

PRINT_CONFIG_VAR(ADIV1_GYRO_Z_SPI_SLAVE_IDX)

#ifndef ADIV1_SPI_DEV
#define ADIV1_SPI_DEV spi1
#endif
PRINT_CONFIG_VAR(ADIV1_SPI_DEV)

#ifndef ADIV1_I2C_DEV
#define ADIV1_I2C_DEV i2c1
#endif
PRINT_CONFIG_VAR(ADIV1_I2C_DEV)

#ifndef ADIV1_ACCEL_RATE
#define ADIV1_ACCEL_RATE ADXL345_RATE_400HZ
#endif
PRINT_CONFIG_VAR(ADIV1_ACCEL_RATE)


struct ImuAdiv1 imu_adiv1;

void imu_impl_init(void)
{
  imu_adiv1.accel_valid = FALSE;
  imu_adiv1.gyro_xy_valid = FALSE;
  imu_adiv1.gyro_z_valid = FALSE;
  imu_adiv1.mag_valid = FALSE;
  imu_adiv1.adxrs290_xy_eoc = TRUE;
  imu_adiv1.adxrs290_z_eoc = TRUE;
  
  imu_adiv1_arch_init();
  /* Set accel configuration */
  adxl345_spi_init(&imu_adiv1.acc_adxl, &(ADIV1_SPI_DEV), ADIV1_ACCEL_SPI_SLAVE_IDX);
  // set the data rate
  imu_adiv1.acc_adxl.config.rate = ADIV1_ACCEL_RATE;
  /// @todo drdy int handling for adxl345
  //imu_adiv1.acc_adxl.config.drdy_int_enable = TRUE;

  /* Gyro xy configuration and initalization */
  adxrs290_spi_init(&imu_adiv1.gyro_xy, &(ADIV1_SPI_DEV), ADIV1_GYRO_XY_SPI_SLAVE_IDX);

  /* Gyro z configuration and initalization */
  adxrs290_spi_init(&imu_adiv1.gyro_z, &(ADIV1_SPI_DEV), ADIV1_GYRO_Z_SPI_SLAVE_IDX);


   /* initialize mag and set default options */
  hmc58xx_init(&imu_adiv1.mag_hmc, &(ADIV1_I2C_DEV), HMC58XX_ADDR);
  imu_adiv1.mag_hmc.type = HMC_TYPE_5883;
}


void imu_periodic(void)
{
  adxl345_spi_periodic(&imu_adiv1.acc_adxl);

  // Start reading the latest gyroscope data
  adxrs290_spi_periodic(&imu_adiv1.gyro_xy, ADXRS_AXIS_XY);
  adxrs290_spi_periodic(&imu_adiv1.gyro_z, ADXRS_AXIS_Z);

  // Read HMC58XX at 50Hz (main loop for rotorcraft: 512Hz)
  RunOnceEvery(2, hmc58xx_periodic(&imu_adiv1.mag_hmc));
}

void imu_adiv1_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  adxl345_spi_event(&imu_adiv1.acc_adxl);
  if (imu_adiv1.acc_adxl.data_available) {
    VECT3_COPY(imu.accel_unscaled, imu_adiv1.acc_adxl.data.vect);
    imu_adiv1.acc_adxl.data_available = FALSE;
    imu_adiv1.accel_valid = TRUE;
  }

  /* If the adxrs290 spi transaction has succeeded: convert the data */
  adxrs290_spi_event(&imu_adiv1.gyro_xy);
  if (imu_adiv1.gyro_xy.data_available) {
	imu.gyro_unscaled.p = -imu_adiv1.gyro_xy.data.rates.p;
	imu.gyro_unscaled.q = imu_adiv1.gyro_xy.data.rates.q;
	//imu.gyro_unscaled.r = imu_adiv1.gyro_xy.temp;
    imu_adiv1.gyro_xy.data_available = FALSE;
    imu_adiv1.gyro_xy_valid = TRUE;
  }

  adxrs290_spi_event(&imu_adiv1.gyro_z);
  if (imu_adiv1.gyro_z.data_available) {
	imu.gyro_unscaled.r = imu_adiv1.gyro_z.data.rates.q;
    imu_adiv1.gyro_z.data_available = FALSE;
    imu_adiv1.gyro_z_valid = TRUE;
  }

  /* HMC58XX event task */
  hmc58xx_event(&imu_adiv1.mag_hmc);
  if (imu_adiv1.mag_hmc.data_available) {
    // VECT3_COPY(imu.mag_unscaled, imu_adiv1.mag_hmc.data.vect);
    imu.mag_unscaled.x =  imu_adiv1.mag_hmc.data.vect.x;
    imu.mag_unscaled.y = -imu_adiv1.mag_hmc.data.vect.y;
    imu.mag_unscaled.z = -imu_adiv1.mag_hmc.data.vect.z;
    imu_adiv1.mag_hmc.data_available = FALSE;
    imu_adiv1.mag_valid = TRUE;
  }

  if (imu_adiv1.gyro_xy_valid & imu_adiv1.gyro_z_valid) {
    imu_adiv1.gyro_xy_valid = FALSE;
	imu_adiv1.gyro_z_valid = FALSE;
    imu_scale_gyro(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_ADISENS_ID, now_ts, &imu.gyro);
  }
  if (imu_adiv1.accel_valid) {
    imu_adiv1.accel_valid = FALSE;
    imu_scale_accel(&imu);
    AbiSendMsgIMU_ACCEL_INT32(IMU_ADISENS_ID, now_ts, &imu.accel);
  }
  if (imu_adiv1.mag_valid) {
    imu_adiv1.mag_valid = FALSE;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_ADISENS_ID, now_ts, &imu.mag);
  }
}
