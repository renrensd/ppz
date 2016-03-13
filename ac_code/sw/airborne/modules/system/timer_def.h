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

TIMER(TIMER_OPS_HB_POLL,                 		ops_heart_beat_lose_handler,          TIMER_TASK_OPS)
TIMER(TIMER_OPS_MSG_CONF_PARAM,                 ops_msg_config_param,                 TIMER_TASK_OPS)
TIMER(TIMER_OPS_MSG_HEART_BEAT,                 ops_msg_heart_beat,                 TIMER_TASK_OPS)
TIMER(TIMER_OPS_MSG_START_SPRAY,                ops_msg_start_spraying,                 TIMER_TASK_OPS)
TIMER(TIMER_OPS_MSG_STOP_SPRAY,                 ops_msg_stop_spraying,                 TIMER_TASK_OPS)

END_TIMERS


/****************************** END OF FILE ***************************/

