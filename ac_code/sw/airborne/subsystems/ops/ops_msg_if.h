/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				       *
*				  All Rights Reserved.          					   *
*   Department : R&D SW      									       *
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
#ifndef _OPS_MSG_IF_H_
#define _OPS_MSG_IF_H_ 

#include "../../modules/system/types.h"

/**** Definition of types ****/ 
#define OPS_CMD_TYPE        0
#define OPS_CMD_ID_H        1
#define OPS_CMD_ID_L        2
#define OPS_CMD_SIZE        3
#define OPS_CMD_PRIO		 4
#define OPS_CMD_PARAM_START 4

#define OPS_UART_BASE_LEN 4

typedef union
{
    struct
    {
        U8 msgseq;
        U8 compid;
    }msgid;
    U16 msg_id;
} OPS_MSG_ID;

typedef struct 
{
    U8 type;
    OPS_MSG_ID msg_id;
    U8 size;
    U8 *param;
} OPS_UART_FRAME;

/**** Definition of macros ****/


/**** Declaration of constants ****/


/**** Declaration of variables ****/


/**** Declaration of functions ****/
extern void ops_uart_msg_handle(U8 const *frame);
BOOL ops_uart_msg_send(U8 cmd_type, U16 service, U8 nArgs,U8 const *pArg);
extern void ops_msg_heart_beat(void);
extern void ops_msg_device_manage_handler(OPS_UART_FRAME *ops_msg);
extern void ops_msg_stop_spraying(void);
extern void ops_msg_start_spraying(void);
extern void ops_msg_start_selfclean(void);
extern void ops_msg_stop_selfclean(void);
extern void ops_start_spraying(void);
extern void ops_stop_spraying(void);
extern uint8_t get_spray_switch_state(void);
extern void ops_msg_config_param(void);


#endif /*_OPS_MSG_IF_H_*/
/****************************** END OF FILE ***************************/
