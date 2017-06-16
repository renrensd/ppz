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
#ifndef _OPS_APP_H_
#define _OPS_APP_H

#include BOARD_CONFIG


/**** Definition of constants ****/
#define OPS_DEFAULT_FLOW_MIN 	250
#define OPS_DEFAULT_FLOW_M2 	400
#define OPS_DEFAULT_DROP_CM2 	50
#define OPS_DEFAULT_ATOM 		2
#define OPS_DEFAULT_SPRAY_CHAL 	0x0f   // 4 channel
#define OPS_DEFAULT_SPRAY_WIDE  300

#define OPS_CONFIG_PARAM_TIMEOUT 	(OPS_PERIODIC_FREQUENCY*0.5)	//500ms
#define OPS_MSG_SPRAY_TIMEOUT 		(OPS_PERIODIC_FREQUENCY*0.5)	//500ms

#define OPS_CNT_MAX 65535


/**** Definition of types ****/
enum OPS_SET_SPRAYING_PARAM
{
	OPS_SET_SPRAYING_NONE = 0x00,
	OPS_SET_SPRAYING_STOP,
	OPS_SET_SPRAYING_START
};

enum OPS_WORK_STATE_PARAM
{
	OPS_STATE_NOZZLE_BIT = 0x01,
	OPS_STATE_PUMP_BIT = 0x02,
	OPS_STATE_OVERALL_BIT = 0x04,
};

enum OPS_CONIG_STAUS
{
	OPS_CONF_NOT_CONNECT,
	OPS_CONF_PARAM,
	OPS_CONF_START_SPRAY,
	OPS_CONF_STOP_SPRAY,

	OPS_CONF_IDLE,
};

enum OPS_CONNECT_PARAM
{
	OPS_NOT_CONNECT = 0x00,
	OPS_CONNECTED,
};
/**** Declaration of constants ****/


/**** Declaration of variables ****/


/**** Declaration of functions ****/
void ops_spray_msg_handler(void);


#endif /*_OPS_APP_H_*/

/****************************** END OF FILE ***************************/

