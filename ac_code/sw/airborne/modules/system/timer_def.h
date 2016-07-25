/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : R&D SW      									   *
*   AUTHOR	   : 										   *
************************************************************************
* Object        : 
* Module        : 
* Instance      : 
* Description   : 
*-----------------------------------------------------------------------
* Version:  V0.01
* Date:     2016/3/2
* Author:   
***********************************************************************/
/*-History--------------------------------------------------------------
* Version       Date       Name               Changes and comments
* V0.01         
*=====================================================================*/

BEGIN_TIMERS

TIMER(TIMER_OPS_HB_POLL,                 		   ops_heart_beat_lose_handler,           TIMER_TASK_OPS)
TIMER(TIMER_OPS_MSG_CONF_PARAM,                 ops_msg_config_param,                  TIMER_TASK_OPS)
TIMER(TIMER_OPS_MSG_HEART_BEAT,                 ops_msg_heart_beat,                    TIMER_TASK_OPS)
TIMER(TIMER_OPS_MSG_START_SPRAY,                ops_msg_start_spraying,                TIMER_TASK_OPS)
TIMER(TIMER_OPS_MSG_STOP_SPRAY,                 ops_msg_stop_spraying,                 TIMER_TASK_OPS)
TIMER(TIMER_RC_SPEED_RL_DRIFT_STOP,             rc_speed_rl_drift_stop,                 TIMER_TASK_RC)
TIMER(TIMER_RC_RATE_ROTATION_DRIFT_STOP,        rc_rate_rotation_drift_stop,            TIMER_TASK_RC)
#ifdef GCS_V1_OPTION
TIMER(TIMER_XBEE_HEARTBEAT_MSG,                 xbee_msg_aircraft_ready_broadcast,      TIMER_TASK_TELEMETRY)
TIMER(TIMER_GCS_SPRAY_BREAK_CONTINUAL_MSG,      spray_break_continual_msg,              TIMER_TASK_GCS)
#endif
END_TIMERS


/****************************** END OF FILE ***************************/

