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
 * @file subsystems/imu/imu_aspirin.h
 * Interface for the Adiv1 v1.x IMU using SPI for the accelerometer.
 */


#ifndef IMU_ADIV1_H
#define IMU_ADIV1_H

#include "generated/airframe.h"
#include "subsystems/imu.h"


#include "peripherals/adxrs453_spi.h"
#include "peripherals/hmc58xx.h"
#include "peripherals/adxl350_spi.h"

enum ADXRS_AXIS_TYPE
{
	ADXRS_AXIS_X = 1,
	ADXRS_AXIS_Y = 2,
	ADXRS_AXIS_Z = 3,
};

/* include default adi v1 sensitivity definitions */
//#include "subsystems/imu/imu_adiv1_defaults.h"

// Default configuration
#if !defined IMU_GYRO_P_SIGN & !defined IMU_GYRO_Q_SIGN & !defined IMU_GYRO_R_SIGN
#define IMU_GYRO_P_SIGN   1
#define IMU_GYRO_Q_SIGN   1
#define IMU_GYRO_R_SIGN   1
#endif
#if !defined IMU_ACCEL_X_SIGN & !defined IMU_ACCEL_Y_SIGN & !defined IMU_ACCEL_Z_SIGN
#define IMU_ACCEL_X_SIGN  1
#define IMU_ACCEL_Y_SIGN  1
#define IMU_ACCEL_Z_SIGN  1
#endif
#if !defined IMU_MAG_X_SIGN & !defined IMU_MAG_Y_SIGN & !defined IMU_MAG_Z_SIGN
#define IMU_MAG_X_SIGN    1
#define IMU_MAG_Y_SIGN    1
#define IMU_MAG_Z_SIGN    1
#endif

/** default adxrs453 gyro sensitivy and neutral from the datasheet
 * MPU with 300 deg/s has 80 LSB/(deg/s)
 * sens = 1/80 * pi/180 * 2^INT32_RATE_FRAC
 * sens = 1/80 * pi/180 * 4096 = 0.8936085770210967
 I*/
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
// FIXME, TO CONFIRM
#define IMU_GYRO_P_SENS 0.8936085770210967
#define IMU_GYRO_P_SENS_NUM 2251
#define IMU_GYRO_P_SENS_DEN 2519
#define IMU_GYRO_Q_SENS 0.8936085770210967
#define IMU_GYRO_Q_SENS_NUM 2251
#define IMU_GYRO_Q_SENS_DEN 2519
#define IMU_GYRO_R_SENS 0.8936085770210967
#define IMU_GYRO_R_SENS_NUM 2251
#define IMU_GYRO_R_SENS_DEN 2519

#endif
#if !defined IMU_GYRO_P_NEUTRAL & !defined IMU_GYRO_Q_NEUTRAL & !defined IMU_GYRO_R_NEUTRAL
#define IMU_GYRO_P_NEUTRAL 0
#define IMU_GYRO_Q_NEUTRAL 0
#define IMU_GYRO_R_NEUTRAL 0
#endif


/** default accel sensitivy using 16 bit AD7689 adc
 * adxl350 with 4g has 128 LSB/g
 * sens = 9.81 [m/s^2] / 512 [LSB/g] * 2^INT32_ACCEL_FRAC = 19.62
 
 */
#if 0
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
// FIXME
#define IMU_ACCEL_X_SENS 156.96
#define IMU_ACCEL_X_SENS_NUM 15696
#define IMU_ACCEL_X_SENS_DEN 100
#define IMU_ACCEL_Y_SENS 156.96
#define IMU_ACCEL_Y_SENS_NUM 15696
#define IMU_ACCEL_Y_SENS_DEN 100
#define IMU_ACCEL_Z_SENS 156.96
#define IMU_ACCEL_Z_SENS_NUM 15696
#define IMU_ACCEL_Z_SENS_DEN 100
#endif
#if !defined IMU_ACCEL_X_NEUTRAL & !defined IMU_ACCEL_Y_NEUTRAL & !defined IMU_ACCEL_Z_NEUTRAL
#define IMU_ACCEL_X_NEUTRAL 0
#define IMU_ACCEL_Y_NEUTRAL 0
#define IMU_ACCEL_Z_NEUTRAL 0
#endif
#endif

struct ImuAdiv1 {
  volatile uint8_t accel_valid;
  volatile uint8_t gyro_x_valid;
  volatile uint8_t gyro_y_valid;
  volatile uint8_t gyro_z_valid;
  volatile uint8_t mag_valid;
  struct Adxl350_Spi acc_adxl;
  struct Adxrs453_Spi gyro_x;
  struct Adxrs453_Spi gyro_y;
  struct Adxrs453_Spi gyro_z;
  struct Hmc58xx mag_hmc;
};

extern struct ImuAdiv1 imu_adiv1;

extern void imu_adiv1_event(void);

#define ImuEvent imu_adiv1_event

#endif /* IMU_ADIV1_H */
