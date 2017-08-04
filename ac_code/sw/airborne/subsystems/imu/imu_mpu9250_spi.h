/*
 * Copyright (C) 2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file subsystems/imu/imu_mpu9250_spi.h
 *
 * IMU driver for the MPU9250 using SPI
 *
 */

#ifndef IMU_MPU9250_SPI_H
#define IMU_MPU9250_SPI_H

#include "std.h"
#include "generated/airframe.h"
#include "subsystems/imu.h"

#include "peripherals/mpu9250_spi.h"
#ifdef HMC5983_OPTION
#include "peripherals/hmc58xx.h"
#endif	/* HMC5983_OPTION */
#ifdef QMC5883_OPTION
#include "peripherals/qmc5883.h"
#endif	/* QMC5883_OPTION */


/** default gyro sensitivy and neutral from the datasheet
 * MPU with 1000 deg/s has 32.8 LSB/(deg/s)
 * sens = 1/32.8 * pi/180 * 2^INT32_RATE_FRAC
 * sens = 1/32.8 * pi/180 * 4096 = 2.17953
 I*/
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
// FIXME
#define IMU_GYRO_P_SENS 2.17953
#define IMU_GYRO_P_SENS_NUM 18271
#define IMU_GYRO_P_SENS_DEN 8383
#define IMU_GYRO_Q_SENS 2.17953
#define IMU_GYRO_Q_SENS_NUM 18271
#define IMU_GYRO_Q_SENS_DEN 8383
#define IMU_GYRO_R_SENS 2.17953
#define IMU_GYRO_R_SENS_NUM 18271
#define IMU_GYRO_R_SENS_DEN 8383
#endif

enum selftest_state
{
	UNTESTED,
	GET_STATIC_DATA,
	CONFIG_TEST,
	CONFIG_RECOVER,
	RECOVER_CHECK,
	FINISHED
};

struct Imu_Selftest
{
	bool_t result;
	enum selftest_state state;
	uint8_t static_counter;
	uint8_t test_counter;
	//uint8_t recover_counter;
	struct Int32Vect3 static_accel;
	struct Int32Vect3 test_accel;
	struct Int32Vect3 response_accel;
	//struct Int32Vect3 recover_accel;
	struct Int32Rates static_gyro;
	struct Int32Rates test_gyro;
	struct Int32Rates response_gyro;
	//struct Int32Rates recover_gyro;
};
/** default accel sensitivy from the datasheet
 * MPU with 8g has 4096 LSB/g
 * sens = 9.81 [m/s^2] / 4096 [LSB/g] * 2^INT32_ACCEL_FRAC = 2.4525
 */


struct ImuMpu9250
{
	struct Mpu9250_Spi mpu;
	struct Imu_Selftest selftest;

	struct spi_transaction wait_slave4_trans;
	volatile uint8_t wait_slave4_tx_buf[1];
	volatile uint8_t wait_slave4_rx_buf[2];
	volatile bool_t slave4_ready;
#ifdef HMC5983_OPTION
	struct Hmc58xx mag_hmc;
	volatile uint8_t mag_valid;
#endif
#ifdef QMC5883_OPTION
	struct Qmc5883 mag_qmc;
	volatile uint8_t mag_valid;
#endif
};

extern struct ImuMpu9250 imu_mpu9250;

extern void imu_mpu9250_event(void);

#define ImuEvent imu_mpu9250_event

#endif /* IMU_MPU9250_SPI_H */
