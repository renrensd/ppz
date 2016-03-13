/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : RN R&D SW2      									   *
*   AUTHOR	   : 										   *
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
#ifndef _OPS_UART_MSG_H_
#define _OPS_UART_MSG_H_ 
#include "../../modules/system/types.h"
#include "ops_msg_if.h"   
/**** Definition of constants ****/


/**** Definition of macros ****/

/**** Declaration of constants ****/

/**** Declaration of variables ****/


/**** Declaration of functions ****/
static void ops_msg_init_var (void);
static void ops_uart_msg_frame_mk(U8 *frame,U8 cmd_type, U16 service, U8 nArgs,U8 const *pArg );
static void ops_uart_msg_frame_decode(U8 const *frame, OPS_UART_FRAME  *ops_uart_frame);
void ops_uart_msg_device_manage_handler(OPS_UART_FRAME *ops_msg);

#endif /*_OPS_UART_MSG_H_*/

/****************************** END OF FILE ***************************/


