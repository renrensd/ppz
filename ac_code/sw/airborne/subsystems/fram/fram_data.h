/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				       *
*				  All Rights Reserved.          					   *
*   Department : R&D SW      									       *
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
#ifndef _FRAM_DATA_H_
#define _FRAM_DATA_H_
#include "std.h"
#include "modules/mag_cali/mag_cali.h"

/**** Definition of constants ****/
#define SW_VERSION_LONGTH     16
#define MAG_CALI_FRAM_DATA_LENGTH	(MAG_CALI_PERS_DATA_STRUCT_LENGTH)
#define ACC_CALI_FRAM_DATA_LENGTH   (ACC_CALI_PERS_DATA_STRUCT_LENGTH)

/**** Definition of types ****/
struct AccCali_PersData
{
	float gain[3];
	float offset[3];
	uint32_t crc16;
} __attribute((aligned (4)));

#define ACC_CALI_PERS_DATA_STRUCT_LENGTH	(sizeof(struct AccCali_PersData))

struct MagCali_PersData
{
	struct EcefCoor_i cali_ecef_pos_i;
	float gain[2];
	float offset[2];
	uint32_t cali_ok;
	uint32_t crc16;
} __attribute((aligned (4)));

#define MAG_CALI_PERS_DATA_STRUCT_LENGTH	(sizeof(struct MagCali_PersData))

/**** Definition of macros ****/


/**** Declaration of constants ****/


/**** Declaration of variables ****/

extern const uint8_t cl_software_version_array[];
extern const uint8_t* const cl_data_array[];
extern const uint8_t cl_ac_serial_number_array[];
extern const uint8_t cl_ac_hw_version_array[];
extern const uint8_t cl_gcs_mac_address[];
extern const uint8_t cl_rc_mac_address[];
extern const uint16_t gcs_msg_version;
/**** Declaration of functions ****/

#endif //_FRAM_DATA_H_
