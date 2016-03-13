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
#ifndef _UART_OPS_IF_H_
#define _UART_OPS_IF_H_ 
#include "../../modules/system/types.h"
/**** Definition of constants ****/

#define UART_OPS_FRAME_SIZE 150


/**** Definition of types ****/ 

typedef struct 
{
    BOOL bValid; /** if valid data? **/
    U8 length;
    U8  buff[UART_OPS_FRAME_SIZE];   
} UART_OPS_FRAME_TYPE;

/**** Definition of macros ****/

/**** Declaration of constants ****/


/**** Declaration of variables ****/


/**** Declaration of functions ****/
extern void uart_ops_create(void);
extern void uart_ops_polling(void);
extern void uart_ops_read_polling(void);
extern void uart_ops_send_polling(void);
extern BOOL uart_ops_send_frame(U8 const *frame,U8 length);
extern void uart_ops_rx_proc(void);
extern void uart_ops_tx_proc(void);
extern void uart_ops_down(void);
#endif /*_UART_OPS_IF_H_*/
/****************************** END OF FILE ***************************/

