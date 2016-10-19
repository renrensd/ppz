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
#ifndef _TIMER_IF_H_
#define _TIMER_IF_H_ 
#include "types.h" 
/**** Definition of constants ****/


/**** Definition of types ****/ 
typedef enum
{
	TIMER_TASK_OPS,
	TIMER_TASK_RC,
	TIMER_TASK_GCS,
	TIMER_TASK_MODULE,
	TIMER_TASK_TELEMETRY,
	TIMER_TASK_IMU,
	TIMER_TASK_MAX    
}TIMER_TASK_IDS;

typedef enum
{
    TIMER_PERIODIC,    
    TIMER_ONE_SHOT,
}TIMER_TYPES;

typedef struct
{
    U16 counter;
    U16 backupCounter;
    U16 param;
	U8 type;
	U8 rep_cnt;
}TIMER_CLASS;


typedef struct
{
    TIMER_FUNC action_func;
    U8 task_id; 
}TIMER_DESC_TYPE;

/**** Definition of macros ****/
#define TIMER_TIMEOUT               0x0000
#define TIMER_NOT_ACTIVE            0xFFFF
#define TIMER_NO_PARAM              0x00
#define TIMER_NO_BACKUP_COUNTER     0xFFFF
#define TIMER_MAX_SYS_TIMER_TICK    0xFFFF

/**** Declaration of constants ****/
#define HW_TIMER_TICK                   1   /* 1 ms */
#define SYS_TIMER_COUNTER               10
#define SYS_TIMER_TICK                  (HW_TIMER_TICK*SYS_TIMER_COUNTER) /*1ms*10 = 10ms*/

#define MSECOND     / SYS_TIMER_TICK
#define SECOND      *1000L/ SYS_TIMER_TICK
#define MSECONDS    / SYS_TIMER_TICK
#define SECONDS     *1000L/ SYS_TIMER_TICK
#define MINUTE      *60000L/ SYS_TIMER_TICK
#define MINUTES     *60000L/ SYS_TIMER_TICK


/**** Declaration of variables ****/


/**** Declaration of functions ****/
extern void tm_reset_create(void);
extern void tm_create_timer(U8 timer_id, U16 period, U8 type, U16 param);
extern void tm_kill_timer(U8 timer_id);
extern void tm_msg_handler(U32 *mptr);
extern void tm_kill_msg_handler(U32 *pTimer_id);
extern void tm_stimulate(U8 task_id);
extern void tm_task_tick_polling(void);
extern U16 tm_get_system_timer_tick(void);
extern BOOL tm_is_time_passed(U16 tick, U16 duration);
extern BOOL tm_is_timer_active(U8 timer_id);


#endif /*_TIMER_IF_H_*/

/****************************** END OF FILE ***************************/

