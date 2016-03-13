/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : R&D SW      									   *
*   AUTHOR	   : 										   *
************************************************************************
* Object        : 
* Module        : 
* Instance      : 
* Description   : 
*-----------------------------------------------------------------------
* Version:  V0.01
* Date:     2012/1/9
* Author:   
***********************************************************************/
/*-History--------------------------------------------------------------
* Version       Date       Name               Changes and comments
* V0.01         2012/2/4        initial version
*=====================================================================*/

#ifndef _TIMER_CLASS_H_
#define _TIMER_CLASS_H_
#include "types.h" 
/**** Definition of constants ****/


/**** Definition of types ****/ 



/**** Declaration of constants ****/


/**** Declaration of variables ****/


/**** Declaration of functions ****/


/**** Definition of macros ****/

#define TIMER_PASS_1

typedef void (*timer)(void);

#endif //_TIMER_CLASS_H_

#ifdef TIMER_PASS_3
    #undef  TIMER_PASS_3
    #undef  BEGIN_TIMERS
    #undef  END_TIMERS
    #undef  TIMER
    #define BEGIN_TIMERS        const TIMER_DESC_TYPE timer_array [TIMER_MAX] = {
    #define END_TIMERS          };
    #define TIMER(x,y,z)          {y,z},
#endif

#ifdef TIMER_PASS_2
    #undef  TIMER_PASS_2
    #define TIMER_PASS_3
    #undef  BEGIN_TIMERS
    #undef  END_TIMERS
    #undef  TIMER
    #define BEGIN_TIMERS
    #define END_TIMERS
    #define TIMER(x,y,z)         extern void y(U16 param);
#endif

#ifdef TIMER_PASS_1
    #undef TIMER_PASS_1
    #define TIMER_PASS_2
    #define BEGIN_TIMERS        enum TIMERS_ENUM {
    #define END_TIMERS          TIMER_MAX };
    #define TIMER(x,y,z)          x,
#endif



/****************************** END OF FILE ***************************/
