/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				       *
*				  All Rights Reserved.          					   *
*   Department 	: R&D SW      									       *
*   AUTHOR	   	:            										   *
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
#ifndef _ADXRS453_H_
#define _ADXRS453_H_

/* Include address and register definition */
#include "peripherals/adxrs453_reg.h"

/**** Definition of types ****/
enum Adxrs453ConfStatus
{
	ADXRS453_CONF_UNINIT = 0,
	ADXRS453_CONF_STARTUP_STEP1,
	ADXRS453_CONF_STARTUP_STEP1_WAIT,
	ADXRS453_CONF_STARTUP_STEP2,
	ADXRS453_CONF_STARTUP_STEP2_WAIT,
	ADXRS453_CONF_STARTUP_STEP3,
	ADXRS453_CONF_STARTUP_STEP3_WAIT,
	ADXRS453_CONF_STARTUP_STEP4,
	ADXRS453_CONF_STARTUP_STEP4_WAIT,
	ADXRS453_READ_PID1,
	ADXRS453_READ_PID1_OK,
	ADXRS453_CONF_DONE,
};

struct Adxrs453Config
{
	uint8_t pwr;  // 1->measurement mode; 0->standby mode
	uint8_t df;  //Digital Filter
	uint8_t dady;//data ready
};

/**** Definition of macros ****/
#define ADXRS453_STARTUP_DELAY	50 /*50 ms */
#define ADXRS453_STARTUP_DELAY2	100 /*100 ms */

/**** Declaration of constants ****/


/**** Declaration of variables ****/


/**** Declaration of functions ****/


#endif /* _ADXRS453_H_ */

/****************************** END OF FILE ***************************/

