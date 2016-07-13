/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : RN R&D SW2      									   *
*   AUTHOR	   :             										   *
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
#ifndef _OPS_APP_IF_H_
#define _OPS_APP_IF_H_ 
#include "../../modules/system/types.h"
/**** Definition of constants ****/

#define PARAM_FLOW_SPEED  1
#define PARAM_FLOW_DENSITY  2
#define PARAM_DROP_CAPCITY  3
#define PARAM_SPRAY_ATOM  4
#define PARAM_SPRAY_CHANNEL  5

/**** Definition of types ****/ 

/**** Definition of macros ****/
struct OPS_INFO
{
	uint8_t spraying_flag;
	uint16_t res_cap;  //Percentage of the liquid residual capacity.
	uint8_t work_state;		//module work state.
	uint8_t spray_state;
	uint16_t vel;		//aircraft velocity, mm/s.
	uint8_t init_status;
	uint8_t con_flag;	//the flag that aircraft connected to ops.
	uint8_t ops_debug;
};

struct OPS_CONFIG_PARAM
{
	uint16_t flow_min;	// mL/min
	uint16_t flow_m2;	// mL/m2
	uint16_t drop_cm2;	//droplets/cm2
	uint16_t atom;		//um
	uint8_t spray_chal;	
};
/**** Declaration of constants ****/


/**** Declaration of variables ****/
extern struct OPS_INFO ops_info;
extern struct OPS_CONFIG_PARAM ops_param;


/**** Declaration of functions ****/
extern void ops_task(void);
extern void ops_init(void);
extern void ops_heart_beat_handler(uint8_t *param);
extern void ops_update_aircraft_vel(void);
extern void ops_heart_beat_lose_handler(void);
extern void ops_update_config_param(uint16_t param, uint8_t param_type);
extern void rc_update_ops_config_param(uint8_t grade);

#endif /*_OPS_APP_IF_H_*/
/****************************** END OF FILE ***************************/

