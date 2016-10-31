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
	STATE_FAIL,
	STATE_OK,
};

/////////////////// Management service //////////////////////////////////////
enum BBOX_MANGEMENT_MSG
{
	BBOX_HEART_BEAT = 0x0001,

};

enum BBOX_CONNECT_PARAM
{
	BBOX_IS_CONNECT,
	BBOX_IS_DISCONNECT,
};


/////////////////// Log data //////////////////////////////////////
enum BBOX_LOG_DATA_MSG
{
	BBOX_LOG_DATA = 0x0001,

};


#endif /*_BBOX_MSG_UART_DEF_H_*/
/****************************** END OF FILE ***************************/

