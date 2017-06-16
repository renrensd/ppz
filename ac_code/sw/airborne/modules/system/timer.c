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
* Version       Date    Name          Changes and comments
* 0.1           define the basic function.
*=====================================================================*/

/**** System include files ****/
#include "types.h"
#include "timer_if.h"

/*---Public include files---------------------------------------------*/

/*---Private include files--------------------------------------------*/
#include "timer.h"
#include "timer_if.h"
#include "timer_class.h"
#include "timer_def.h"
#include "timer_class.h"
#include "timer_def.h"
#include "timer_class.h"
#include "timer_def.h"

/*===VARIABLES========================================================*/
U16 sys_timer_tick_counter;

/*---Global-----------------------------------------------------------*/

/*---Private----------------------------------------------------------*/
TIMER_CLASS timer_status[TIMER_MAX];
U8 timer_tick_poll_counter;

/*===CONSTS========================================================*/


/*===FUNCTIONS========================================================*/

/*---Global-----------------------------------------------------------*/
/***********************************************************************
*  Name        : tm_reset_create
*  Description : this function init all the timers.
*  Parameter   : void
*  Returns     : void
***********************************************************************/
void tm_reset_create(void)
{
	U8 i;
	for(i = 0x00; i < TIMER_MAX; i++)
	{
		timer_status[i].counter = TIMER_NOT_ACTIVE;
		timer_status[i].backupCounter = TIMER_NOT_ACTIVE;
		timer_status[i].param = TIMER_NO_PARAM;
	}
	timer_tick_poll_counter = 0;

	sys_timer_tick_counter = 0;
}

/***********************************************************************
*  Name        : tm_create_timer
*  Description : this function create timer which is defined by the
                 parameters.
*  Parameter   : timer_id : the No. of the timer defined in enum TIMER_IDS
                 period : time, the unit is 1 mseconds.
                 type : Once or periodic timer.
                 param : timeout param.
*  Returns     : void
***********************************************************************/
void tm_create_timer(U8 timer_id, U16 period, U8 type, U16 param)
{
	if(timer_id < TIMER_MAX)
	{
		if(period == TIMER_NOT_ACTIVE)
		{
			period = TIMER_NOT_ACTIVE - 1;
		}
		timer_status[timer_id].counter = period;
		timer_status[timer_id].type = type;
		timer_status[timer_id].rep_cnt = type;
		if(type == TIMER_ONE_SHOT)
		{
			timer_status[timer_id].backupCounter = TIMER_NO_BACKUP_COUNTER;
		}
		else
		{
			timer_status[timer_id].backupCounter = period;
		}

		timer_status[timer_id].param = param;
	}
}

/***********************************************************************
*  Name        : tm_kill_timer
*  Description : this function kill timer which is defined by the
                 parameter timer_id.
*  Parameter   : timer_id : the No. of the timer defined in enum TIMER_IDS
*  Returns     : void
***********************************************************************/
void tm_kill_timer(U8 timer_id)
{
	if(timer_id < TIMER_MAX)
	{
		timer_status[timer_id].counter = TIMER_NOT_ACTIVE;
		timer_status[timer_id].backupCounter = TIMER_NO_BACKUP_COUNTER;
		timer_status[timer_id].param = TIMER_NO_PARAM;
	}
}

/***********************************************************************
*  Name        : tm_stimulate
*  Description :
*  Parameter   : void
*  Returns     : void
*  Remark      :
***********************************************************************/
void tm_stimulate(U8 task_id)
{
	U8 i;
	for(i = 0x00; i < TIMER_MAX; i++)
	{
		if((timer_array[i].task_id == task_id)
				&& (timer_status[i].counter == TIMER_TIMEOUT))
		{
			if(timer_status[i].backupCounter != TIMER_NO_BACKUP_COUNTER)
			{
				if(timer_status[i].type == TIMER_PERIODIC)
				{
					timer_status[i].counter = timer_status[i].backupCounter;
				}
				else
				{
					if(--timer_status[i].rep_cnt)
					{
						timer_status[i].counter = timer_status[i].backupCounter;
					}
					else
					{
						timer_status[i].counter = TIMER_NOT_ACTIVE;
					}
				}
			}
			else	//TIMER_ONE_SHOT,end timer
			{
				timer_status[i].counter = TIMER_NOT_ACTIVE;
			}

			timer_array[i].action_func(timer_status[i].param);
		}
	}
}

/***********************************************************************
*  Name        : tm_task_tick_polling
*  Description :
*  Parameter   : void
*  Returns     : void
***********************************************************************/
void tm_task_tick_polling(void)
{
	U8 i;

	sys_timer_tick_counter += HW_TIMER_TICK;

	timer_tick_poll_counter++;

	if(timer_tick_poll_counter >= SYS_TIMER_COUNTER)
	{
		timer_tick_poll_counter = 0;
		for(i = 0x00; i < TIMER_MAX; i++)
		{
			if(timer_status[i].counter != TIMER_NOT_ACTIVE
					&& timer_status[i].counter != TIMER_TIMEOUT)
			{
				timer_status[i].counter--;
			}
		}
	}
}

/***********************************************************************
*  Name        : tm_get_system_timer_tick
*  Description : return current system tick, in millisecond.
*  Parameter   : void
*  Returns     : void
***********************************************************************/
U16 tm_get_system_timer_tick(void)
{
	return( sys_timer_tick_counter );
}

/***********************************************************************
*  Name        : tm_is_time_passed
*  Description : check if specific duration passed.
*  Parameter   : tick : last tick returned by "tm_get_system_timer_tick"
*                duration: duration of time, in millisecond
*  Returns     : void
***********************************************************************/
BOOL tm_is_time_passed(U16 tick, U16 duration)
{
	U16 temp = tm_get_system_timer_tick();

	if( tick > temp )
	{
		return( ((TIMER_MAX_SYS_TIMER_TICK - tick) + temp) > duration );
	}

	return( (temp - tick) > duration );
}

/***********************************************************************
*  Name        : tm_is_timer_active
*  Description : check if specific timer is active or not
*  Parameter   :
*  Returns     : void
***********************************************************************/
BOOL tm_is_timer_active(U8 timer_id)
{
	return (BOOL)(timer_status[timer_id].counter != TIMER_NOT_ACTIVE);
}
/*---Private----------------------------------------------------------*/



/**************** END OF FILE *****************************************/
