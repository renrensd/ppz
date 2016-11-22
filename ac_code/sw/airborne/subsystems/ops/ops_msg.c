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
#include "../../modules/system/types.h"
#include "../../../include/std.h"
/*---Public include files---------------------------------------------*/
#include "modules/system/timer_if.h"
#include "modules/system/timer_class.h"
#include "modules/system/timer_def.h"
/*---Private include files--------------------------------------------*/
#include "ops_msg.h"   
#include "ops_msg_if.h"   
#include "ops_app.h"   
#include "ops_app_if.h"   
#include "uart_ops_if.h"
#include "ops_msg_uart_def.h"
#include "ops_comm_if.h"


/*===VARIABLES========================================================*/
/*---Global-----------------------------------------------------------*/

/*******************************************************************************
**  FUNCTION      : ops_msg_heart_beat                                         
**  DESCRIPTION   : void 
**  PARAMETERS    : void
**  RETURN        : void                                                          
*******************************************************************************/
void ops_msg_heart_beat(void)
{
	uint8_t arg[4];
   	arg[0] = OPS_PRIO_GENERAL;
	arg[1] = OPS_POSITIVE_RESULT;
	arg[2] = (ops_info.vel >> 8) & 0xff;
	arg[3] = ops_info.vel & 0xff;
	
   	ops_comm_send_frame(OPS_RES_ACK_NOT_NEEDED,OPS_HEART_BEAT,4, &arg[0]);
}

/*******************************************************************************
**  FUNCTION      : ops_msg_self_clean                                         
**  DESCRIPTION   : 
**  PARAMETERS    : 
**  RETURN        : void                                                          
*******************************************************************************/
void ops_msg_self_clean(void)
{
	uint8_t arg[2];
   	arg[0] = OPS_PRIO_GENERAL;
	arg[1] = OPS_SELF_CLEAN;

   	//ops_comm_send_frame(OPS_REQ_ACK_NEEDED,OPS_SPRAYING_CONTROL,2, &arg[0]);
	ops_comm_send_frame(OPS_REQ_ACK_NOT_NEEDED,OPS_SPRAYING_CONTROL,2, &arg[0]);
}

/*******************************************************************************
**  FUNCTION      : ops_msg_start_spraying                                         
**  DESCRIPTION   : void 
**  PARAMETERS    : flow - spraying flow(mL/min). 
				    atom_val - Set liquid particles atomization value (um)
**  RETURN        : void                                                          
*******************************************************************************/
void ops_msg_start_spraying(void)
{
	uint8_t arg[2];
   	arg[0] = OPS_PRIO_GENERAL;
	arg[1] = OPS_SPRAYING_START;

   	//ops_comm_send_frame(OPS_REQ_ACK_NEEDED,OPS_SPRAYING_CONTROL,2, &arg[0]);
	ops_comm_send_frame(OPS_REQ_ACK_NOT_NEEDED,OPS_SPRAYING_CONTROL,2, &arg[0]);
}

/*******************************************************************************
**  FUNCTION      : ops_msg_stop_spraying                                         
**  DESCRIPTION   : void 
**  PARAMETERS    : void
**  RETURN        : void                                                          
*******************************************************************************/
void ops_msg_stop_spraying(void)
{
	uint8_t arg[2];
   	arg[0] = OPS_PRIO_GENERAL;
	arg[1] = OPS_SPRAYING_STOP;

   	//ops_comm_send_frame(OPS_REQ_ACK_NEEDED,OPS_SPRAYING_CONTROL,2, &arg[0]);
   	ops_comm_send_frame(OPS_REQ_ACK_NOT_NEEDED,OPS_SPRAYING_CONTROL,2, &arg[0]);
}

/*******************************************************************************
**  FUNCTION      : ops_msg_request_sv(void)                                         
**  DESCRIPTION   : void 
**  PARAMETERS    : void
**  RETURN        : void                                                          
*******************************************************************************/
void ops_msg_request_sv(void)
{
	uint8_t arg[6];
   	arg[0] = OPS_PRIO_DIAGNOSE;

   	ops_comm_send_frame(OPS_REQ_ACK_NOT_NEEDED,OPS_REQUEST_SOFTWARE_VERSION,6, &arg[0]);
}

/*******************************************************************************
**  FUNCTION      : ops_msg_config_param                                         
**  DESCRIPTION   : void 
**  PARAMETERS    : void
**  RETURN        : void                                                          
*******************************************************************************/
void ops_msg_config_param(void)
{
	uint8_t arg[10];
   	arg[0] = OPS_PRIO_GENERAL;
	arg[1] = (ops_param.flow_min >> 8) & 0xff;
	arg[2] = ops_param.flow_min & 0xff;
	arg[3] = (ops_param.flow_m2 >> 8) & 0xff;
	arg[4] = ops_param.flow_m2 & 0xff;
	arg[5] = (ops_param.drop_cm2 >> 8) & 0xff;
	arg[6] = ops_param.drop_cm2 & 0xff;
	arg[7] = (ops_param.atom >> 8) & 0xff;
	arg[8] = ops_param.atom & 0xff;
	arg[9] = ops_param.spray_chal;

   	//ops_comm_send_frame(OPS_REQ_ACK_NEEDED,OPS_MSGID_CONFIG_PARAM,0x0a, &arg[0]);
   	ops_comm_send_frame(OPS_REQ_ACK_NOT_NEEDED,OPS_MSGID_CONFIG_PARAM,0x0a, &arg[0]);
}

/*---Private----------------------------------------------------------*/
/***********************************************************************
*  Name         : ops_msg_create
*  Description : creat mp5 msg navi  
*  Parameter  : None
*  Returns      : None
***********************************************************************/
void ops_msg_create(void)
{
    ops_msg_init_var();
}

/***********************************************************************
*  Name         : ops_uart_msg_send
*  Description : send nav frame to output fifo      
*  Parameter  : 
*  Returns      : 
***********************************************************************/
BOOL ops_uart_msg_send(U8 cmd_type, U16 service, U8 nArgs,U8 const *pArg)
{
	U8 ret_val = FALSE; 
	U8 frame[UART_OPS_FRAME_SIZE];
	ops_uart_msg_frame_mk(frame,cmd_type, service, nArgs, pArg);
	ret_val = uart_ops_send_frame(frame,OPS_UART_BASE_LEN+nArgs);
	return ret_val;
}

/***********************************************************************
*  Name        : ops_uart_msg_handle
*  Description : it is a callback function, only called by "uart_ops_handle_frame" .     
*  Parameter   :  
*  Returns     : 
*  Note		   :ops_uart_frame->param[0] indicate priority,so actual data is ops_uart_frame->param[1].
***********************************************************************/
void ops_uart_msg_handle(U8 const *frame)
{
	OPS_UART_FRAME  ops_uart_frame;
	
	ops_uart_msg_frame_decode((const U8*)frame,&ops_uart_frame);
	switch (ops_uart_frame.msg_id.msgid.compid)
	{
		case OPS_UART_DEV_MANAGE_SERVEICE:
			ops_msg_device_manage_handler(&ops_uart_frame);
			break;
		case OPS_AIRCRAFT_SERVICE:
            ops_msg_device_manage_handler(&ops_uart_frame);
			break;
		default:
			break;
	}
	
}


/****************************************************************************************************
OPS UART FRAME:

frame type   |         command    id            |   cmd size   | param1...n
1byte        |  compid(H:8bit)  msgseq(L:8bit)  |   1 byte     | n byte

*****************************************************************************************************/
void ops_uart_msg_frame_mk(U8 *frame,U8 cmd_type, U16 service, U8 nArgs,U8 const *pArg )
{
	U8 idx;
	frame[OPS_CMD_TYPE] = cmd_type;
	frame[OPS_CMD_ID_H] = (U8)(service>>8);
	frame[OPS_CMD_ID_L] = (U8)service;
	frame[OPS_CMD_SIZE] = nArgs;
	for(idx=0;idx<nArgs;idx++)
	{
		frame[OPS_CMD_PARAM_START+idx] = pArg[idx];
	}
}

/****************************************************************************************************
OPS UART FRAME:

frame type   |         command    id            |   cmd size   | param1...n
1byte        |  compid(H:8bit)  msgseq(L:8bit)  |   1 byte     | n byte

*****************************************************************************************************/
void ops_uart_msg_frame_decode(U8 const *frame, OPS_UART_FRAME  *ops_uart_frame)
{
	(*ops_uart_frame).type = *(frame+OPS_CMD_TYPE);
	(*ops_uart_frame).msg_id.msg_id = (((U16)*(frame+OPS_CMD_ID_H))<<8) + (U16)*(frame+OPS_CMD_ID_L);
	(*ops_uart_frame).size =*(frame+OPS_CMD_SIZE);
	(*ops_uart_frame).param = (U8*)(frame+OPS_CMD_PARAM_START);
}

/***********************************************************************
*  Name         : ops_msg_init_var
*  Description :   
*  Parameter  : 
*  Returns      : 
***********************************************************************/
static void ops_msg_init_var (void)
{
    
}

/*******************************************************************************
**  FUNCTION      : ops_msg_device_manage_handler                                         
**  DESCRIPTION   :  
**  PARAMETERS    : NAVI_FRAME *navi_msg
**  RETURN        : void                                                          
*******************************************************************************/
void ops_msg_device_manage_handler(OPS_UART_FRAME *ops_msg)
{
	uint8_t param[50];
	
    if(NULL != ops_msg->param + 1)
    {
        param[1] = *(ops_msg->param + 1);
    }
	if(NULL != ops_msg->param + 2)
    {
        param[2] = *(ops_msg->param + 2);
    }
	if(NULL != ops_msg->param + 3)
    {
        param[3] = *(ops_msg->param + 3);
    }

	switch(ops_msg->msg_id.msg_id)
	{
		case OPS_HEART_BEAT:
			ops_heart_beat_handler(ops_msg->param + 1);	
			break;
		case OPS_SPRAYING_CONTROL:
			if(param[1] == OPS_POSITIVE_RESULT)
			{
				tm_kill_timer(TIMER_OPS_MSG_START_SPRAY);
				tm_kill_timer(TIMER_OPS_MSG_STOP_SPRAY);
			}	
			break;
		case OPS_MSGID_CONFIG_PARAM:
			if(param[1] == OPS_POSITIVE_RESULT)
			{
				tm_kill_timer(TIMER_OPS_MSG_CONF_PARAM);
			}
			else
			{
				//TODOM:error logo.
			}
			break;
		case OPS_REQUEST_SOFTWARE_VERSION:
			if(param[1] == OPS_POSITIVE_RESULT)
			{
				ops_software_version_handler(ops_msg->param + 1, ops_msg->size);
			}
			break;
		
		default:
			break;
		
	}
}


/**************** END OF FILE *****************************************/

