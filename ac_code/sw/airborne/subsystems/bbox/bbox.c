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

/*---Public include files---------------------------------------------*/
#include "../../../include/std.h"
#include "modules/system/timer_if.h"
#include "modules/system/timer_class.h"
#include "modules/system/timer_def.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/can_transport.h"

/*---Private include files--------------------------------------------*/
#include "bbox.h"   
#include "bbox_if.h" 
#include "bbox_msg_def.h"   


/*===VARIABLES========================================================*/
struct BBOX_INFO bbox_info;

/*---Global-----------------------------------------------------------*/
/***********************************************************************
* FUNCTION    : bbox_task
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void bbox_task(void)
{ 
	bbox_send_polling();
	bbox_read_polling();
	
	tm_stimulate(TIMER_TASK_EVENT);
}

/***********************************************************************
* FUNCTION    : bbox_init
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void bbox_init(void)
{
	bbox_can_init();
	bbox_info.first_con = FALSE;
	bbox_info.con_flag = TRUE;
	bbox_info.start_log = FALSE;
	//tm_create_timer(TIMER_CAN_TEST_MSG, (40 MSECONDS), TIMER_PERIODIC,0);
}

void can_transport_test_msg(void)
{
	DOWNLINK_SEND_PONG(can_tp, can_tp);
}

/**************** END OF FILE *****************************************/

