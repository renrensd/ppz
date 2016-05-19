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
#ifndef _TASK_SPARY_CONVERT_H_
#define _TASK_SPARY_CONVERT_H_

#include "subsystems/mission/task_manage.h"


typedef struct Spray_Convert
{
	struct EnuCoor_i center;
	int32_t radius;
	int32_t heading_sp;
	bool_t  useful;
	
} Spray_Convert_Info;


extern Spray_Convert_Info spray_convert_info;

extern void spray_convert_init(void);
extern bool_t spray_convert_caculate(void);


#endif /*_TASK_SPARY_CONVERT_H_*/
/****************************** END OF FILE ***************************/