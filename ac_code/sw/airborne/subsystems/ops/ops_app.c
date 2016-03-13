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

/*---Public include files---------------------------------------------*/
#include "modules/system/types.h"
#include "uart_ops_if.h"
#include "../../../include/std.h"
#include "modules/system/timer_if.h"
#include "modules/system/timer_class.h"
#include "modules/system/timer_def.h"


/*---Private include files--------------------------------------------*/
#include "ops_app.h"   
#include "ops_app_if.h" 
#include "ops_comm_if.h"
#include "ops_msg_uart_def.h"   
#include "ops_msg_if.h"   


/*===VARIABLES========================================================*/
struct OPS_INFO ops_info;
struct OPS_CONFIG_PARAM ops_param;

uint16_t ops_test;
/*---Global-----------------------------------------------------------*/
void ops_task(void)
{ 
	ops_comm_read_polling();
	ops_comm_send_polling();

	ops_spray_msg_handler();
	tm_stimulate(TIMER_TASK_OPS);

#if 0
	//ops_test++;
	if(ops_test == 1000)
	{
		ops_test = 0;
		ops_msg_stop_spraying();
	}
	else if(ops_test == 2000)
	{
		ops_test = 0;
		ops_msg_start_spraying();
	}
	else if(ops_test == 3000)
	{
		ops_test = 0;
		ops_msg_config_param();
	}
#endif
}

/***********************************************************************
* FUNCTION    : ops_spray_msg_handler
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void ops_spray_msg_handler(void)
{
	if(ops_info.spraying_flag == OPS_SET_SPRAYING_START)
	{
		ops_info.spraying_flag = OPS_SET_SPRAYING_NONE;
		if(ops_info.spray_state == OPS_SPRAY_IS_OFF)
		{
			ops_msg_start_spraying();
			tm_create_timer(TIMER_OPS_MSG_START_SPRAY, (500 MSECONDS), 10,0);
		}
	}
	else if(ops_info.spraying_flag == OPS_SET_SPRAYING_STOP)
	{
		ops_info.spraying_flag = OPS_SET_SPRAYING_NONE;
		if(ops_info.spray_state == OPS_SPRAY_IS_ON)
		{
			ops_msg_stop_spraying();
			tm_create_timer(TIMER_OPS_MSG_STOP_SPRAY, (500 MSECONDS), 10,0);
		}
	}
}

/***********************************************************************
* FUNCTION    : ops_msg_start_spraying
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void ops_start_spraying(void)
{
	if(ops_info.con_flag == OPS_CONNECTED)
	{
		ops_info.spraying_flag = OPS_SET_SPRAYING_START;
	}
}

/***********************************************************************
* FUNCTION    : ops_msg_start_spraying
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void ops_stop_spraying(void)
{
	if(ops_info.con_flag == OPS_CONNECTED)
	{
		ops_info.spraying_flag = OPS_SET_SPRAYING_STOP;
	}
}

/***********************************************************************
* FUNCTION    : ops_init
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void ops_init(void)
{
	ops_comm_create();
	ops_param.flow_min = OPS_DEFAULT_FLOW_MIN;
	ops_param.flow_m2 = OPS_DEFAULT_FLOW_M2;
	ops_param.drop_cm2 = OPS_DEFAULT_DROP_CM2;
	ops_param.atom = OPS_DEFAULT_ATOM;
	ops_param.spray_chal = OPS_DEFAULT_SPRAY_CHAL;
	ops_info.init_status = OPS_CONF_NOT_CONNECT;
	ops_info.con_flag = OPS_NOT_CONNECT;

	ops_info.spray_state = OPS_SPRAY_IS_OFF;
}

/***********************************************************************
* FUNCTION    : ops_heart_beat_handler
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void ops_heart_beat_handler(uint8_t *param)
{
	tm_kill_timer(TIMER_OPS_HB_POLL);
	ops_msg_heart_beat();
	ops_info.res_cap = (*param << 8) | (*(param+1));
	ops_info.work_state = *(param+2);
	ops_info.spray_state = *(param+3);
	ops_info.con_flag = OPS_CONNECTED;

	if(ops_info.init_status == OPS_CONF_NOT_CONNECT)
	{
		ops_info.init_status = OPS_CONF_PARAM;
		tm_create_timer(TIMER_OPS_MSG_CONF_PARAM, (500 MSECONDS), 10,0);
	}

	tm_create_timer(TIMER_OPS_HB_POLL, (3000 MSECONDS), TIMER_ONE_SHOT,0);
}

/***********************************************************************
* FUNCTION    : ops_update_aircraft_vel
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void ops_update_aircraft_vel(uint8_t *param)
{
	ops_info.vel = *param;
}

/***********************************************************************
* FUNCTION    : ops_heart_beat_lose_handler
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void ops_heart_beat_lose_handler(void)
{
	ops_info.con_flag = OPS_NOT_CONNECT;
	ops_info.init_status = OPS_CONF_NOT_CONNECT;
}

/**************** END OF FILE *****************************************/

