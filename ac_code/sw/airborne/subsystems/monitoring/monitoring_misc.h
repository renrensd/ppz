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
#ifndef _MONITORING_MISC_H_
#define _MONITORING_MISC_H_ 

#include "std.h"
#include "math/pprz_algebra_int.h"

/**** Definition of constants ****/


/**** Definition of types ****/ 



/**** Definition of macros ****/

/**** Declaration of constants ****/

/**** Declaration of variables ****/


/**** Declaration of functions ****/
extern void misc_moni_init(void);
extern uint8_t board_ground_check(void);
extern uint8_t battery_ground_check(void);
extern void battery_flight_check(void);
extern void gps_flight_check(void);
extern bool_t ops_ground_check(void);
extern void ops_flight_check(void);
extern void rc_communication_flight_check(void);
extern void gcs_communication_flight_check(void);
extern void lift_flight_check(void);
extern void task_running_check(void);
extern void mode_convert_check(void);
extern uint8_t autopilot_ground_check(void);


#endif /*_MONITORING_MISC_H_ */

/****************************** END OF FILE ***************************/




