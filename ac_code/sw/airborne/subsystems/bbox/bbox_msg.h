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
#ifndef _BBOX_UART_MSG_H_
#define _BBOX_UART_MSG_H_ 
#include "subsystems/datalink/can_transport.h"
/**** Definition of constants ****/


/**** Definition of macros ****/

/**** Declaration of constants ****/

/**** Declaration of variables ****/


/**** Declaration of functions ****/
static void bbox_can_msg_send(uint8_t nArgs, uint8_t const *pArg);
static void bbox_can_send_frame(struct can_transport *p, uint8_t nArgs, uint8_t const *pArg);



#endif /*_BBOX_UART_MSG_H_*/

/****************************** END OF FILE ***************************/


