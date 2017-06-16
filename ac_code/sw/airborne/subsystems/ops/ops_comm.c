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

/**** System include files ****/
#include "../../../include/std.h"
#include "../../modules/system/types.h"
/*---Public include files---------------------------------------------*/

#include "ops_app_if.h"

/*---Private include files--------------------------------------------*/
#include "ops_comm.h"
#include "ops_comm_if.h"
#include "uart_ops_if.h"
#include "ops_msg_if.h"


/*===VARIABLES========================================================*/

/*---Global-----------------------------------------------------------*/

/*---Private----------------------------------------------------------*/



/*===FUNCTIONS========================================================*/

/*---Global-----------------------------------------------------------*/
/***********************************************************************
*  Name        : ops_comm_create
*  Description : power up to create ops uart comm
*  Parameter   : void
*  Returns     : void
***********************************************************************/
void ops_comm_create(void)
{
	uart_ops_create();
}

/***********************************************************************
*  Name        : ops_comm_send_frame
*  Description : Send one frame with command & response_state & data to fifo
*  Parameter   : message structure
*  Returns     : TRUE/FALSE
***********************************************************************/
BOOL ops_comm_send_frame(U8 type, U16 id, U8 nArgs, U8 const *pArg)
{
	U8 ret_val = FALSE;

	//os_mut_wait(mutex_ops_comm_fifo, 0xFFFF);

	/** if fifo  full, the ret_val=FALSE **/
	ret_val = ops_uart_msg_send(type, id, nArgs, pArg);

	//os_mut_release(mutex_ops_comm_fifo);
	return ret_val;
}

/***********************************************************************
*  Name        : ops_comm_send_polling
*  Description : check if any frame in fifo to send
*  Parameter   : void
*  Returns     : void
***********************************************************************/
void ops_comm_send_polling(void)
{
	uart_ops_send_polling();
}
/***********************************************************************
*  Name        : ops_comm_read_polling
*  Description : check if any frame in fifo to read
*  Parameter   : void
*  Returns     : void
***********************************************************************/
void ops_comm_read_polling(void)
{
	uart_ops_read_polling();
}

/***********************************************************************
*  Name        : ops_comm_down
*  Description : shut down comm before sleep
*  Parameter   : void
*  Returns     : void
***********************************************************************/
void ops_comm_down(void)
{
	uart_ops_down();
}

/**************** END OF FILE *****************************************/
