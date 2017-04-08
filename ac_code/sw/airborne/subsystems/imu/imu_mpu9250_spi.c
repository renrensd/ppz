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
 * @file subsystems/imu/imu_mpu9250_spi.c
 *
 * IMU driver for the MPU9250 using SPI
 *
 */

#include "subsystems/imu.h"
#include "subsystems/abi.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/spi.h"
#include "peripherals/ak8963_regs.h"
#ifdef CALIBRATION_OPTION
#include "calibration.h"
#endif

/* SPI defaults set in subsystem makefile, can be configured from airframe file */
PRINT_CONFIG_VAR(IMU_MPU9250_SPI_SLAVE_IDX)
PRINT_CONFIG_VAR(IMU_MPU9250_SPI_DEV)


#if !defined IMU_MPU9250_GYRO_LOWPASS_FILTER && !defined IMU_MPU9250_ACCEL_LOWPASS_FILTER && !defined  IMU_MPU9250_SMPLRT_DIV
#if (PERIODIC_FREQUENCY == 60) || (PERIODIC_FREQUENCY == 120)
/* Accelerometer: Bandwidth 41Hz, Delay 5.9ms
 * Gyroscope: Bandwidth 41Hz, Delay 5.9ms sampling 1kHz
 * Output rate: 100Hz
 */
#define IMU_MPU9250_GYRO_LOWPASS_FILTER MPU9250_DLPF_GYRO_41HZ
#define IMU_MPU9250_ACCEL_LOWPASS_FILTER MPU9250_DLPF_ACCEL_41HZ
#define IMU_MPU9250_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#elif PERIODIC_FREQUENCY == 512
/* Accelerometer: Bandwidth 184Hz, Delay 5.8ms
 * Gyroscope: Bandwidth 250Hz, Delay 0.97ms sampling 8kHz
 * Output rate: 2kHz
 */
#define IMU_MPU9250_GYRO_LOWPASS_FILTER MPU9250_DLPF_GYRO_41HZ
#define IMU_MPU9250_ACCEL_LOWPASS_FILTER MPU9250_DLPF_ACCEL_41HZ
#define IMU_MPU9250_SMPLRT_DIV 0
PRINT_CONFIG_MSG("Gyro/Accel output rate is 2kHz at 8kHz internal sampling")
#else
/* By default, don't go too fast */
#define IMU_MPU9250_SMPLRT_DIV 9
#define IMU_MPU9250_GYRO_LOWPASS_FILTER MPU9250_DLPF_GYRO_41HZ
#define IMU_MPU9250_ACCEL_LOWPASS_FILTER MPU9250_DLPF_ACCEL_41HZ
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#endif
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_SMPLRT_DIV)
PRINT_CONFIG_VAR(IMU_MPU9250_GYRO_LOWPASS_FILTER)
PRINT_CONFIG_VAR(IMU_MPU9250_ACCEL_LOWPASS_FILTER)

#ifndef IMU_MPU9250_GYRO_RANGE
#define IMU_MPU9250_GYRO_RANGE MPU9250_GYRO_RANGE_1000
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_GYRO_RANGE)

#ifndef IMU_MPU9250_ACCEL_RANGE
#define IMU_MPU9250_ACCEL_RANGE MPU9250_ACCEL_RANGE_16G
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_ACCEL_RANGE)

// Default channels order
#ifndef IMU_MPU9250_CHAN_X
#define IMU_MPU9250_CHAN_X 0
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_CHAN_X)
#ifndef IMU_MPU9250_CHAN_Y
#define IMU_MPU9250_CHAN_Y 1
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_CHAN_Y)
#ifndef IMU_MPU9250_CHAN_Z
#define IMU_MPU9250_CHAN_Z 2
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_CHAN_Z)

#ifndef IMU_MPU9250_X_SIGN
#define IMU_MPU9250_X_SIGN 1
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_X_SIGN)
#ifndef IMU_MPU9250_Y_SIGN
#define IMU_MPU9250_Y_SIGN -1
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_Y_SIGN)
#ifndef IMU_MPU9250_Z_SIGN
#define IMU_MPU9250_Z_SIGN -1
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_Z_SIGN)

#ifndef IMU_MPU9250_MAG_STARTUP_DELAY
#define IMU_MPU9250_MAG_STARTUP_DELAY 1
#endif

#ifdef HMC5983_OPTION
#ifndef HMC5983_I2C_DEV
#define HMC5983_I2C_DEV i2c1
#endif
#define IMU_MPU9250_READ_MAG FALSE
#endif	/* HMC5983_OPTION */

#ifdef QMC5883_OPTION
#define QMC5883_I2C_DEV i2c1
#define IMU_MPU9250_READ_MAG FALSE
#endif	/* QMC5883_OPTION */

struct ImuMpu9250 imu_mpu9250;

void mpu_wait_slave4_ready(void);
void mpu_wait_slave4_ready_cb(struct spi_transaction *t);
bool_t imu_mpu9250_configure_mag_slave(Mpu9250ConfigSet mpu_set, void *mpu);

void imu_selftest_init(void);
bool_t imu_selftest_handle(struct Int32Vect3 accel, struct Int32Rates rates);


void imu_impl_init(void)
{
  /* MPU9250 */
  mpu9250_spi_init(&imu_mpu9250.mpu, &(IMU_MPU9250_SPI_DEV), IMU_MPU9250_SPI_SLAVE_IDX);
  // change the default configuration
  imu_mpu9250.mpu.config.smplrt_div = IMU_MPU9250_SMPLRT_DIV;
  imu_mpu9250.mpu.config.dlpf_gyro_cfg = IMU_MPU9250_GYRO_LOWPASS_FILTER;
  imu_mpu9250.mpu.config.dlpf_accel_cfg = IMU_MPU9250_ACCEL_LOWPASS_FILTER;
  imu_mpu9250.mpu.config.gyro_range = IMU_MPU9250_GYRO_RANGE;
  imu_mpu9250.mpu.config.accel_range = IMU_MPU9250_ACCEL_RANGE;

  //intial selftest
  imu_selftest_init();

  /* "internal" ak8963 magnetometer as I2C slave */
#if IMU_MPU9250_READ_MAG
  /* read 15 bytes for status, accel, gyro + 7 bytes for mag slave */
  imu_mpu9250.mpu.config.nb_bytes = 22;
  imu_mpu9250.mpu.config.nb_slaves = 1;
#endif
  /* set callback function to configure mag */
  imu_mpu9250.mpu.config.slaves[0].configure = &imu_mpu9250_configure_mag_slave;

  /* Set MPU I2C master clock */
  imu_mpu9250.mpu.config.i2c_mst_clk = MPU9250_MST_CLK_400KHZ;
  /* Enable I2C slave0 delayed sample rate */
  imu_mpu9250.mpu.config.i2c_mst_delay = 1;


  /* configure spi transaction for wait_slave4 */
  imu_mpu9250.wait_slave4_trans.cpol = SPICpolIdleHigh;
  imu_mpu9250.wait_slave4_trans.cpha = SPICphaEdge2;
  imu_mpu9250.wait_slave4_trans.dss = SPIDss8bit;
  imu_mpu9250.wait_slave4_trans.bitorder = SPIMSBFirst;
  imu_mpu9250.wait_slave4_trans.cdiv = SPIDiv64;

  imu_mpu9250.wait_slave4_trans.select = SPISelectUnselect;
  imu_mpu9250.wait_slave4_trans.slave_idx = IMU_MPU9250_SPI_SLAVE_IDX;
  imu_mpu9250.wait_slave4_trans.output_length = 1;
  imu_mpu9250.wait_slave4_trans.input_length = 2;
  imu_mpu9250.wait_slave4_trans.before_cb = NULL;
  imu_mpu9250.wait_slave4_trans.after_cb = mpu_wait_slave4_ready_cb;
  imu_mpu9250.wait_slave4_trans.input_buf = &(imu_mpu9250.wait_slave4_rx_buf[0]);
  imu_mpu9250.wait_slave4_trans.output_buf = &(imu_mpu9250.wait_slave4_tx_buf[0]);

  imu_mpu9250.wait_slave4_trans.status = SPITransDone;
  imu_mpu9250.slave4_ready = FALSE;

#ifdef HMC5983_OPTION
  /* initialize mag and set default options */
  hmc58xx_init(&imu_mpu9250.mag_hmc, &(HMC5983_I2C_DEV), HMC58XX_ADDR);
  imu_mpu9250.mag_hmc.type = HMC_TYPE_5883;
#endif /* HMC5983_OPTION */

	#ifdef QMC5883_OPTION
  	/* initialize mag and set default options */
  	qmc5883_init(&imu_mpu9250.mag_qmc, &(QMC5883_I2C_DEV), QMC5883_ADDR);
	#endif /* HMC5983_OPTION */
}

void imu_selftest_init(void)
{
	imu_mpu9250.selftest.result = FALSE;
	imu_mpu9250.selftest.state = UNTESTED;
	imu_mpu9250.selftest.static_counter = 0;
	imu_mpu9250.selftest.test_counter = 0;
	//imu_mpu9250.selftest.recover_counter = 0;
}

/*default 16G,2048->1g*/
#define RECOVER_ACCEL_DIFF 200
#define RECOVER_GYRO_DIFF 400
#define ACCEL_REPONSE_MIN 500
#define ACCEL_RESPONSE_MAX 1500
#define GYRO_RESPONSE_MIN 2000
#define GYRO_RESPONSE_MAX 9000
#define INTERVAL_ABS_VALID(_a, _min, _max)    ( abs(_a) > (_min) && abs(_a) < (_max) )

bool_t imu_selftest_handle(struct Int32Vect3 accel, struct Int32Rates rates)
{
//	if(get_sys_time_msec()<6000) return FALSE;
	if(imu_mpu9250.selftest.state != FINISHED)  //no send data from abi
	{
		if(imu_mpu9250.selftest.state == UNTESTED)
		{
			if( ((accel.x||accel.y||accel.z) && (rates.p||rates.q||rates.r)) )  //request data is valid
			{
				VECT3_ADD(imu_mpu9250.selftest.static_accel, accel);
				RATES_ADD(imu_mpu9250.selftest.static_gyro, rates);
				imu_mpu9250.selftest.static_counter++;
				if(imu_mpu9250.selftest.static_counter == 10)
				{
					VECT3_SDIV(imu_mpu9250.selftest.static_accel, 
								imu_mpu9250.selftest.static_accel, 
								imu_mpu9250.selftest.static_counter);
					RATES_SDIV(imu_mpu9250.selftest.static_gyro, 
								imu_mpu9250.selftest.static_gyro, 
								imu_mpu9250.selftest.static_counter);
					imu_mpu9250.selftest.state++;
				}
			}
		}
		//CONFIG_TEST, do in periodic
		else if(imu_mpu9250.selftest.state == CONFIG_TEST)
		{
			if( abs(accel.x)>ACCEL_REPONSE_MIN   //request data is valid
			    &&abs(accel.y)>ACCEL_REPONSE_MIN
			    && abs(rates.p)>GYRO_RESPONSE_MIN
			    && abs(rates.r)>GYRO_RESPONSE_MIN) 
			{
				VECT3_ADD(imu_mpu9250.selftest.test_accel, accel);
				RATES_ADD(imu_mpu9250.selftest.test_gyro, rates);
				imu_mpu9250.selftest.test_counter++;
				if(imu_mpu9250.selftest.test_counter == 10)
				{
					VECT3_SDIV(imu_mpu9250.selftest.test_accel, 
								imu_mpu9250.selftest.test_accel, 
								imu_mpu9250.selftest.test_counter);
					RATES_SDIV(imu_mpu9250.selftest.test_gyro, 
								imu_mpu9250.selftest.test_gyro, 
								imu_mpu9250.selftest.test_counter);
					/*caculate response data*/
					VECT3_DIFF(imu_mpu9250.selftest.response_accel,
						       imu_mpu9250.selftest.test_accel,
						       imu_mpu9250.selftest.static_accel);
					RATES_DIFF(imu_mpu9250.selftest.response_gyro,
						       imu_mpu9250.selftest.test_gyro,
						       imu_mpu9250.selftest.static_gyro);
					if( INTERVAL_ABS_VALID(imu_mpu9250.selftest.response_accel.x, ACCEL_REPONSE_MIN, ACCEL_RESPONSE_MAX)
						&& INTERVAL_ABS_VALID(imu_mpu9250.selftest.response_accel.y, ACCEL_REPONSE_MIN, ACCEL_RESPONSE_MAX)
						&& INTERVAL_ABS_VALID(imu_mpu9250.selftest.response_accel.z, ACCEL_REPONSE_MIN, ACCEL_RESPONSE_MAX)
						&& INTERVAL_ABS_VALID(imu_mpu9250.selftest.response_gyro.p, GYRO_RESPONSE_MIN, GYRO_RESPONSE_MAX)
						&& INTERVAL_ABS_VALID(imu_mpu9250.selftest.response_gyro.q, GYRO_RESPONSE_MIN, GYRO_RESPONSE_MAX)
						&& INTERVAL_ABS_VALID(imu_mpu9250.selftest.response_gyro.r, GYRO_RESPONSE_MIN, GYRO_RESPONSE_MAX))
					{
						imu_mpu9250.selftest.result = TRUE;
					}
					imu_mpu9250.selftest.state++;
				}
			}
		}
		//CONFIG_RECOVER, do in periodic
		else if(imu_mpu9250.selftest.state == RECOVER_CHECK)
		{
			if( abs(accel.x)<(ACCEL_REPONSE_MIN/5)   //request data is valid
			    &&abs(accel.y)<(ACCEL_REPONSE_MIN/5)
			    && abs(rates.p)<(GYRO_RESPONSE_MIN/10)
			    && abs(rates.r)<(GYRO_RESPONSE_MIN/10) ) 
			{
				imu_mpu9250.selftest.state++;
				/*
				VECT3_ADD(imu_mpu9250.selftest.recover_accel, accel);
				RATES_ADD(imu_mpu9250.selftest.recover_gyro, rates);
				imu_mpu9250.selftest.recover_counter++;
				if(imu_mpu9250.selftest.recover_counter == 10)
				{
					VECT3_SDIV(imu_mpu9250.selftest.recover_accel, 
								imu_mpu9250.selftest.recover_accel, 
								imu_mpu9250.selftest.recover_counter);
					RATES_SDIV(imu_mpu9250.selftest.recover_gyro, 
								imu_mpu9250.selftest.recover_gyro, 
								imu_mpu9250.selftest.recover_counter);

					VECT3_DIFF(imu_mpu9250.selftest.recover_accel,
						       imu_mpu9250.selftest.recover_accel,
						       imu_mpu9250.selftest.static_accel);
					RATES_DIFF(imu_mpu9250.selftest.recover_gyro,
						       imu_mpu9250.selftest.recover_gyro,
						       imu_mpu9250.selftest.static_gyro);
					if( (abs(imu_mpu9250.selftest.recover_accel.x) < RECOVER_ACCEL_DIFF)
						&& (abs(imu_mpu9250.selftest.recover_accel.y) < RECOVER_ACCEL_DIFF)
						&& (abs(imu_mpu9250.selftest.recover_accel.z) < RECOVER_ACCEL_DIFF)
						&& (abs(imu_mpu9250.selftest.recover_gyro.p) < RECOVER_GYRO_DIFF)
						&& (abs(imu_mpu9250.selftest.recover_gyro.q) < RECOVER_GYRO_DIFF)
						&& (abs(imu_mpu9250.selftest.recover_gyro.r) < RECOVER_GYRO_DIFF) )
					{
						imu_mpu9250.selftest.state++;
					}
					else
					{
						//imu_mpu9250.selftest.state--;   //try to reset
					}					
				}
				*/
			}
		}
		return FALSE;
	}  /*end of unfinished*/
	else
	{
		return TRUE;
	}

}


void imu_periodic(void)
{
	switch(imu_mpu9250.selftest.state)
	{
		case GET_STATIC_DATA:
			if(mpu9250_spi_start_self_test(&imu_mpu9250.mpu))
			{
				imu_mpu9250.selftest.state++;
			}
			break;
		case CONFIG_RECOVER:
			if(mpu9250_spi_end_self_test(&imu_mpu9250.mpu))
			{
				imu_mpu9250.selftest.state++;
			}
			break;
		default:
			mpu9250_spi_periodic(&imu_mpu9250.mpu);
			break;
	}

#ifdef HMC5983_OPTION
   // Read HMC58XX at 50Hz (main loop for rotorcraft: 512Hz)
  RunOnceEvery(2, hmc58xx_periodic(&imu_mpu9250.mag_hmc));
#endif /* HMC5983_OPTION */

#ifdef QMC5883_OPTION
   // Read HMC58XX at 50Hz (main loop for rotorcraft: 512Hz)
  RunOnceEvery(2, qmc5883_periodic(&imu_mpu9250.mag_qmc));
#endif /* HMC5983_OPTION */
}

#define Int16FromBuf(_buf,_idx) ((int16_t)(_buf[_idx] | (_buf[_idx+1] << 8)))
void imu_mpu9250_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  // If the MPU9250 SPI transaction has succeeded: convert the data
  mpu9250_spi_event(&imu_mpu9250.mpu);

  if (imu_mpu9250.mpu.data_available) {
    // set channel order
    //no6/7/8
/*
    struct Int32Vect3 accel = {
      + (int32_t)(imu_mpu9250.mpu.data_accel.value[IMU_MPU9250_CHAN_Y]),
      + (int32_t)(imu_mpu9250.mpu.data_accel.value[IMU_MPU9250_CHAN_X]),
      - (int32_t)(imu_mpu9250.mpu.data_accel.value[IMU_MPU9250_CHAN_Z])
    };
    struct Int32Rates rates = {
      + (int32_t)(imu_mpu9250.mpu.data_rates.value[IMU_MPU9250_CHAN_Y]),
      + (int32_t)(imu_mpu9250.mpu.data_rates.value[IMU_MPU9250_CHAN_X]),
      - (int32_t)(imu_mpu9250.mpu.data_rates.value[IMU_MPU9250_CHAN_Z])
    };
*/
    //after no8, imu install reverse direction
    struct Int32Vect3 accel = {
      - (int32_t)(imu_mpu9250.mpu.data_accel.value[IMU_MPU9250_CHAN_Y]),
      - (int32_t)(imu_mpu9250.mpu.data_accel.value[IMU_MPU9250_CHAN_X]),
      - (int32_t)(imu_mpu9250.mpu.data_accel.value[IMU_MPU9250_CHAN_Z])
    };
    struct Int32Rates rates = {
      - (int32_t)(imu_mpu9250.mpu.data_rates.value[IMU_MPU9250_CHAN_Y]),
      - (int32_t)(imu_mpu9250.mpu.data_rates.value[IMU_MPU9250_CHAN_X]),
      - (int32_t)(imu_mpu9250.mpu.data_rates.value[IMU_MPU9250_CHAN_Z])
    };
	
	
    if(1)//imu_selftest_handle(accel, rates))
    {
	    // unscaled vector
	    VECT3_COPY(imu.accel_unscaled, accel);
	    RATES_COPY(imu.gyro_unscaled, rates);

	    imu_mpu9250.mpu.data_available = FALSE;
	    imu_scale_gyro(&imu);
	    imu_scale_accel(&imu);
		
		AbiSendMsgIMU_GYRO_INT32(IMU_MPU9250_ID, now_ts, &imu.gyro);
		AbiSendMsgIMU_ACCEL_INT32(IMU_MPU9250_ID, now_ts, &imu.accel);
		AbiSendMsgIMU_GYRO_MONI(IMU_MPU9250_ID, now_ts, &imu.gyro_scaled);
		AbiSendMsgIMU_ACCEL_MONI(IMU_MPU9250_ID, now_ts, &imu.accel_scaled);
    }
  }
  
#if IMU_MPU9250_READ_MAG
    if (!bit_is_set(imu_mpu9250.mpu.data_ext[6], 3)) { //mag valid just HOFL == 0
      /** FIXME: assumes that we get new mag data each time instead of reading drdy bit */
      struct Int32Vect3 mag;
      mag.x =  (IMU_MPU9250_X_SIGN) * Int16FromBuf(imu_mpu9250.mpu.data_ext, 2 * IMU_MPU9250_CHAN_Y);
      mag.y =  (IMU_MPU9250_Y_SIGN) * Int16FromBuf(imu_mpu9250.mpu.data_ext, 2 * IMU_MPU9250_CHAN_X);
      mag.z = -(IMU_MPU9250_Z_SIGN) * Int16FromBuf(imu_mpu9250.mpu.data_ext, 2 * IMU_MPU9250_CHAN_Z);
      VECT3_COPY(imu.mag_unscaled, mag);
      imu_scale_mag(&imu);
      AbiSendMsgIMU_MAG_INT32(IMU_MPU9250_ID, now_ts, &imu.mag);
	  AbiSendMsgIMU_MAG_MONI(IMU_MPU9250_ID, now_ts, &imu.mag_scaled);
    }
#endif

#ifdef HMC5983_OPTION
	/* HMC58XX event task */
	hmc58xx_event(&imu_mpu9250.mag_hmc);
	if (imu_mpu9250.mag_hmc.data_available) 
	{
	    // VECT3_COPY(imu.mag_unscaled, imu_mpu9250.mag_hmc.data.vect);
	    imu.mag_unscaled.x = +imu_mpu9250.mag_hmc.data.vect.y;
	    imu.mag_unscaled.y = +imu_mpu9250.mag_hmc.data.vect.x;
	    imu.mag_unscaled.z = +imu_mpu9250.mag_hmc.data.vect.z;
	    imu_mpu9250.mag_hmc.data_available = FALSE;
	    imu_mpu9250.mag_valid = TRUE;
		#ifdef CALIBRATION_OPTION
		cali_magraw_to_txt(imu.mag_unscaled.x, imu.mag_unscaled.y, imu.mag_unscaled.z);
		#endif	/* CALIBRATION_OPTION */
	}

	if (imu_mpu9250.mag_valid) 
	{
	    imu_mpu9250.mag_valid = FALSE;
	    imu_scale_mag(&imu);
	    AbiSendMsgIMU_MAG_INT32(IMU_ADISENS_ID, now_ts, &imu.mag);
		AbiSendMsgIMU_MAG_MONI(IMU_MPU9250_ID, now_ts, &imu.mag_scaled);
  	}
#endif /* HMC5983_OPTION */

#ifdef QMC5883_OPTION
	qmc5883_event(&imu_mpu9250.mag_qmc);
	if (imu_mpu9250.mag_qmc.data_available) 
	{
	  /*no 4/5*/
/*
	  imu.mag_unscaled.x = +imu_mpu9250.mag_qmc.data.vect.y;
	  imu.mag_unscaled.y = +imu_mpu9250.mag_qmc.data.vect.x;
	  imu.mag_unscaled.z = -imu_mpu9250.mag_qmc.data.vect.z;
*/
      /*after no 6++*/
	  imu.mag_unscaled.x = +imu_mpu9250.mag_qmc.data.vect.x;
	  imu.mag_unscaled.y = -imu_mpu9250.mag_qmc.data.vect.y;
	  imu.mag_unscaled.z = -imu_mpu9250.mag_qmc.data.vect.z;

	    imu_mpu9250.mag_qmc.data_available = FALSE;
	    imu_mpu9250.mag_valid = TRUE;
		#ifdef CALIBRATION_OPTION
		cali_magraw_to_txt(imu.mag_unscaled.x, imu.mag_unscaled.y, imu.mag_unscaled.z);
		#endif	/* CALIBRATION_OPTION */
	}

	if (imu_mpu9250.mag_valid) 
	{
	    imu_mpu9250.mag_valid = FALSE;
	    imu_scale_mag(&imu);
	    AbiSendMsgIMU_MAG_INT32(IMU_ADISENS_ID, now_ts, &imu.mag);
		AbiSendMsgIMU_MAG_MONI(IMU_MPU9250_ID, now_ts, &imu.mag_scaled);
  	}
#endif /* HMC5983_OPTION */

}

// hack with waiting to avoid creating another event loop to check the mag config status
static inline void mpu_set_and_wait(Mpu9250ConfigSet mpu_set, void *mpu, uint8_t _reg, uint8_t _val)
{
  mpu_set(mpu, _reg, _val);
  while (imu_mpu9250.mpu.spi_trans.status != SPITransSuccess);
}

/** function to configure akm8963 mag
 * @return TRUE if mag configuration finished
 */
bool_t imu_mpu9250_configure_mag_slave(Mpu9250ConfigSet mpu_set, void *mpu)
{
  // wait before starting the configuration of the mag
  // doing to early may void the mode configuration
  if (get_sys_time_float() < IMU_MPU9250_MAG_STARTUP_DELAY) {
    return FALSE;
  }

  //config AK8963 soft reset
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV4_ADDR, (MPU9250_MAG_ADDR >> 1));
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV4_REG, AK8963_REG_CNTL2);
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV4_DO, 1);
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV4_CTRL, (1 << 7)); // Slave 4 enable

  mpu_wait_slave4_ready();

  // Set it to continious measuring mode 2
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV4_ADDR, (MPU9250_MAG_ADDR >> 1));
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV4_REG, AK8963_REG_CNTL1);
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV4_DO, AK8963_CNTL1_CM_2);
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV4_CTRL, (1 << 7)); // Slave 4 enable

  mpu_wait_slave4_ready();

  //Config SLV0 for continus Read
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV0_ADDR, (MPU9250_MAG_ADDR >> 1) | MPU9250_SPI_READ);
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV0_REG, AK8963_REG_HXL);
  // Put the enable command as last.
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV0_CTRL,
                   (1 << 7) |    // Slave 0 enable
                   (7 << 0));    // Read 7 bytes (mag x,y,z + status)

  return TRUE;
}

void mpu_wait_slave4_ready(void)
{
  while (!imu_mpu9250.slave4_ready) {
    if (imu_mpu9250.wait_slave4_trans.status == SPITransDone) {
      imu_mpu9250.wait_slave4_tx_buf[0] = MPU9250_REG_I2C_MST_STATUS | MPU9250_SPI_READ;
      spi_submit(imu_mpu9250.mpu.spi_p, &(imu_mpu9250.wait_slave4_trans));
    }
  }
}

void mpu_wait_slave4_ready_cb(struct spi_transaction *t)
{
  if (bit_is_set(t->input_buf[1], MPU9250_I2C_SLV4_DONE)) {
    imu_mpu9250.slave4_ready = TRUE;
  } else {
    imu_mpu9250.slave4_ready = FALSE;
  }
  t->status = SPITransDone;
}
