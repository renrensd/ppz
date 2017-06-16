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
#ifndef _BBOX_MSG_UART_DEF_H_
#define _BBOX_MSG_UART_DEF_H_
/**** Definition of constants ****/

/**** Definition of macros ****/
enum BBOX_COMPONENT_ID
{
	BBOX_MANAGE_SERVICE = 0x00,
	BBOX_LOG_DATA_SERVICE = 0x01,
	BBOX_FAULT_DATA_SERVICE = 0x02,
	BBOX_CALIBRATION_DATA_SERVICE = 0x03,
	BBOX_CONTROL_COMMAND_SERVICE = 0x04,
	BBOX_UPGRADE_SERVIC=0X05,
};

enum BBOX_UPGRADE_PROCESS_PARAM	//add by lg
{
	BBOX_UPDATE_REQ_DATA=0X01,
	BBOX_UPDATE_REQ_RES_DATA,    //0X02
	BBOX_UPDATE_ASK_READY_DATA,	//0X03
	BBOX_UPDATE_ASK_READY_RES_DATA,	//0X04
	BBOX_UPDATE_DATA,				//0X05
	BBOX_UPDATE_RES_DATA,		//0X06
	BBOX_UPDATE_OVER_DATA,		//0X07
	BBOX_UPDATE_OVER_RES_DATA, 	//0X08
};


enum BBOX_ERROR_REASON
{
	BBOX_ERROR_NO_ERROR,
	BBOX_ERROR_PRECONDITION_NOT_FULFILLED,
	BBOX_ERROR_BUSY_RESOURCE,
	BBOX_ERROR_HW_FAILURE,
	BBOX_ERROR_OTHER_REASONS,
};

enum BBOX_RESPONSE_RESULT_PARAM
{
	BBOX_POSITIVE_RESULT = 0x00,
	BBOX_BAD_RESULT = 0x10,
	BBOX_NEGATIVE_RESULT = 0x20,
};

enum BBOX_RESPONSE_POSITIVE_RESULT_PARAM
{
	BBOX_WORKING_NORMAL = 0x01,

};

enum BBOX_STATE_RESULT_PARAM
{
	BBOX_STATE_FAIL,
	BBOX_STATE_OK,
};

/////////////////// Management service //////////////////////////////////////
enum BBOX_MANGEMENT_MSG
{
	BBOX_HEART_BEAT = 0x0001,

};

enum BBOX_CONNECT_PARAM
{
	BBOX_IS_NORMAL,
	BBOX_IS_ERROR,
};


/////////////////// Log data //////////////////////////////////////
enum BBOX_LOG_DATA_MSG
{
	BBOX_LOG_DATA = 0x0001,

};


#endif /*_BBOX_MSG_UART_DEF_H_*/
/****************************** END OF FILE ***************************/

