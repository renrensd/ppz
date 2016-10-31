/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : R&D SW      									   *
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

/**** System include files ****/  


#include "subsystems/rc_nav/rc_nav_drift.h"
#include "subsystems/rc_nav/rc_nav_xbee.h"


#include "modules/system/timer_if.h"
#include "modules/system/timer_class.h"
#include "modules/system/timer_def.h"


//TIMER(TIMER_RC_SPEED_RL_DRIFT_STOP,             rc_speed_rl_drift_stop,                 TIMER_TASK_RC)
//TIMER(TIMER_RC_RATE_ROTATION_DRIFT_STOP,        rc_rate_rotation_drift_stop,            TIMER_TASK_RC)
/***********************************************************************
* FUNCTION    : rc_speed_rl_drift
* DESCRIPTION : 
* INPUTS      : sign:orientation  1:right -1:left
* RETURN      : none
***********************************************************************/
void rc_speed_rl_drift(int8_t sign)
{
	tm_kill_timer(TIMER_RC_SPEED_RL_DRIFT_STOP);
	
	rc_motion_info.speed_rl =(float)sign * RL_DRIFT_SPEED;
	rc_setpoint_speed_parse(rc_motion_info.speed_fb,rc_motion_info.speed_rl);
	
	tm_create_timer(TIMER_RC_SPEED_RL_DRIFT_STOP, (2000 MSECONDS), TIMER_ONE_SHOT,0);
}

/***********************************************************************
* FUNCTION    : rc_rate_rotation_drift
* DESCRIPTION : 
* INPUTS      : sign:orientation   1:CW   -1:CCW
* RETURN      : none
***********************************************************************/
void rc_rate_rotation_drift(int8_t sign)
{
	tm_kill_timer(TIMER_RC_RATE_ROTATION_DRIFT_STOP);
	
	rc_motion_info.rotation_rate =(float)sign *ROTATION_DRIFT_RATE;
	rc_turn_rate =RATE_BFP_OF_REAL( rc_motion_info.rotation_rate );
	
	tm_create_timer(TIMER_RC_RATE_ROTATION_DRIFT_STOP, (1000 MSECONDS), TIMER_ONE_SHOT,0);
}

/***********************************************************************
* FUNCTION    : rc_speed_rl_drift_stop
* DESCRIPTION : use timer,interrupt set speed 0
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void rc_speed_rl_drift_stop(void)
{
	rc_motion_info.speed_rl =0.0;
	rc_setpoint_speed_parse(rc_motion_info.speed_fb,rc_motion_info.speed_rl);
}

/***********************************************************************
* FUNCTION    : rc_rate_rotation_drift_stop
* DESCRIPTION : use timer,interrupt set rate 0
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void rc_rate_rotation_drift_stop(void)
{
	rc_motion_info.rotation_rate =0.0;
	rc_turn_rate =0;
}


