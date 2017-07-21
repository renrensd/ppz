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
#ifndef _TASK_SPARY_MISC_H_
#define _TASK_SPARY_MISC_H_

#include "subsystems/mission/task_manage.h"

#define SPRAY_CONVERT_FAIL 0
#define SPRAY_CONVERT_CONTINUAL 1
#define SPRAY_CONVERT_SUCCESS 2

typedef struct Spray_Convert
{
	struct EnuCoor_i center;
	int32_t radius;
	int32_t heading_sp;
	bool_t  useful;

} Spray_Convert_Info;

typedef struct Spray_Continual
{
	uint8_t flag_record;
	uint8_t flag_ack;
	uint8_t break_spray_work;
	struct LlaCoor_i break_pos_lla;
} Spray_Conti_Info;

extern Spray_Convert_Info spray_convert_info;
extern Spray_Conti_Info spray_continual_info;
extern void task_spray_misc_init(void);
extern uint8_t spray_convert_caculate(void);
extern bool_t spray_break_and_continual(void);
extern void spray_bac_msg_stop(void);
extern void spray_break_continual_msg(void);


#endif /*_TASK_SPARY_MISC_H_*/
/****************************** END OF FILE ***************************/