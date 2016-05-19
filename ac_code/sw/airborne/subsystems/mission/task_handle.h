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
#ifndef _TASK_HANDLE_H_
#define _TASK_HANDLE_H_

#include "subsystems/mission/task_manage.h"

extern uint8_t get_max_pending_id(void);
extern int8_t get_task_wp_offset(uint8_t id);
extern uint8_t check_task_wp_id(uint8_t id);
extern uint8_t task_lla_to_enu_convert(struct EnuCoor_i *enu, struct LlaCoor_i *lla);

#endif /*_TASK_HANDLE_H_*/

/****************************** END OF FILE ***************************/