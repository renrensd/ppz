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
#ifndef _TASK_PROCESS_H_
#define _TASK_PROCESS_H_

#include "subsystems/mission/task_manage.h"

typedef enum Gcs_Run_State
{
	GCS_RUN_NONE = 0,
	GCS_RUN_NORMAL,
	GCS_RUN_PAUSE,
	GCS_RUN_HOME,
	GCS_RUN_RELAND,
	GCS_RUN_LANDING,
	GCS_RUN_ERROR
} Gcs_State;


extern enum Gcs_Task_Cmd gcs_task_cmd;
extern enum Gcs_Task_Cmd last_task_cmd;
extern bool_t from_wp_useful;

extern void task_init(void);
extern bool_t auto_task_ready_check(void);
extern Gcs_State gcs_task_run(void);
extern void send_task_info_pc(void); 

#endif /*_TASK_MANAGE_H_*/
/****************************** END OF FILE ***************************/