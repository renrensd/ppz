/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : R&D SW      									   *
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
#ifndef _OPS_MSG_UART_DEF_H_
#define _OPS_MSG_UART_DEF_H_
/**** Definition of constants ****/

/**** Definition of macros ****/
enum OPS_COMPONENT_ID
{
	OPS_UART_DEV_MANAGE_SERVEICE = 0x01,
	OPS_AIRCRAFT_SERVICE = 0x02,
	OPS_UPGRADE_SERVICE = 0X07,
};

enum OPS_MSG_TYPE
{
	OPS_REQ_ACK_NEEDED,
	OPS_REQ_ACK_NOT_NEEDED,
	OPS_RES_ACK_NEEDED,
	OPS_RES_ACK_NOT_NEEDED,

	OPS_FRAME_TYPE_MAX,
	OPS_ACK_FRAME = 0xFF,
};

enum OPS_MSG_PRIORITY
{
	OPS_PRIO_SAFETY = 0x01,
	OPS_PRIO_DIAGNOSE,
	OPS_PRIO_GENERAL,
	OPS_PRIO_DEBUG,
};

enum OPS_ERROR_REASON
{
	OPS_ERROR_NO_ERROR,
	OPS_ERROR_PRECONDITION_NOT_FULFILLED,
	OPS_ERROR_BUSY_RESOURCE,
	OPS_ERROR_HW_FAILURE,
	OPS_ERROR_OTHER_REASONS,
};

enum OPS_RESPONSE_RESULT_PARAM
{
	OPS_POSITIVE_RESULT = 0x00,
	OPS_BAD_RESULT = 0x10,
	OPS_NEGATIVE_RESULT = 0x20,
};

enum OPS_RESPONSE_POSITIVE_RESULT_PARAM
{
	OPS_WORKING_NORMAL = 0x01,
	OPS_NETWORK_SIGNAL_OK = 0x02,
	OPS_GPS_DEVICE_OK = 0x04,
	OPS_3G_DEVICE_OK = 0x08,
};

enum OPS_STATE_RESULT_PARAM
{
	OPS_STATE_FAIL,
	OPS_STATE_OK,
};

/////////////////// Device Management service //////////////////////////////////////
enum OPS_DEV_MANGEMENT_MSG
{
	OPS_HEART_BEAT = 0x0101,

};

enum OPS_CONNECT_REQ_PARAM
{
	OPS_REQ_CONNECT,
	OPS_REQ_DISCONNECT,
};


/////////////////// Aircraft service //////////////////////////////////////
enum OPS_AIRCRAFT_SERVICE_MSG
{
	OPS_SPRAYING_CONTROL = 0x0201,
	OPS_MSGID_CONFIG_PARAM,
	OPS_REQUEST_SOFTWARE_VERSION,
};

enum OPS_SPRAYING_CONTROL_PARAM
{
	OPS_SPRAYING_STOP = 0x00,
	OPS_FORWARD_SPRAYING_START = 0xF1,
	OPS_BACKWARD_SPRAYING_START = 0xF9,
	OPS_SELF_CLEAN = 0xF3,
};

enum OPS_SPRAY_RES_STATE_PARAM
{
	OPS_SPRAY_IS_OFF,
	OPS_SPRAY_IS_ON,
};

enum OPS_UPDATE_MSG_ID
{
	OPS_UPGRADE_ID=0x0701,
};

enum OPS_UPDATE_PROCESS_PARAM
{
	OPS_UPDATE_REQ_DATA=0XA0,
	OPS_UPDATE_REQ_RES_DATA,   //A1
	OPS_UPDATE_ASK_READY_DATA,//A2
	OPS_UPDATE_ASK_READY_RES_DATA,//A3
	OPS_UPDATE_DATA,//A4
	OPS_UPDATE_RES_DATA,//A5
	OPS_UPDATE_OVER_DATA,//A6
	OPS_UPDATE_OVER_RES_DATA, //A7
};

#endif /*_OPS_MSG_UART_DEF_H_*/
/****************************** END OF FILE ***************************/

