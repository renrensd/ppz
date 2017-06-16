/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : RN R&D SW1      									   *
*   AUTHOR	   :            										   *
************************************************************************
* Object        :
* Module        :
* Instance      :
* Description   :
*-----------------------------------------------------------------------
* Version:
* Date:
* Author:
***********************************************************************/
/*-History--------------------------------------------------------------
* Version       Date    Name    Changes and comments
*
*=====================================================================*/
#ifndef _MONITORING_IMU_H_
#define _MONITORING_IMU_H_

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "subsystems/imu.h"

/**** Definition of constants ****/


/**** Definition of types ****/
struct Imu_Monitor
{
	uint32_t accel_ground_counter;   //inspect counter
	uint32_t gyro_ground_counter;
	uint32_t mag_ground_counter;
	bool_t accel_ground_check;  //set TRUE to stop check
	bool_t gyro_ground_check;
	bool_t mag_ground_check;
	//bool_t accel_flight_check;  //set TRUE to stop check
	//bool_t gyro_flight_check;
	//bool_t mag_flight_check;
	struct Int32Vect3 accel_aver;    //inspect average
	struct Int32Rates gyro_aver;
	struct Int32Vect3 mag_aver;
	struct Uint16Vect3 accel_interval_counter;  //interval out counter
	struct Uint16Vect3 gyro_interval_counter;
	struct Uint16Vect3 mag_interval_counter;
	struct Uint8Vect3 accel_fix_counter;       //fix data counter
	struct Uint8Vect3 gyro_fix_counter;
	struct Uint8Vect3 mag_fix_counter;
	uint8_t  mag_len2_counter;
	uint32_t accel_update_counter;   //data update counter
	uint32_t gyro_update_counter;
	uint32_t mag_update_counter;

	//0:gyro;  1:accel;  2:mag
	//##code=0:normal  bit1:fix_data  bit2:frequence  bit3:noise  bit4:range  bit5:interference(mag)  bit6:no data update
	//error will update(change according to current state)
	uint8_t imu_error[3];
	bool_t imu_status;   //0:fail  1:ok  ##once set fail,keep fail until restart
};

/**** Declaration of variables ****/
extern struct Imu_Monitor imu_moni;

/**** Declaration of functions ****/
extern void imu_moni_init(void);
extern uint8_t imu_ground_check(void);
extern void imu_ground_reset(void);
extern void imu_flight_check(void);
extern void imu_frequence_check(void);
extern bool_t gyro_offset_caculate(struct Imu *_imu);




#endif /*_MONITORING_IMU_H_ */

/****************************** END OF FILE ***************************/

