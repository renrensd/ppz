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
#ifndef _ATOMIC_TUNER_IF_H_
#define _ATOMIC_TUNER_IF_H_ 

/**** Definition of constants ****/


/**** Definition of types ****/ 


/**** Definition of macros ****/


/**** Declaration of constants ****/


/**** Declaration of variables ****/


/**** Declaration of functions ****/
extern BOOL at_tuner_polling(void);
extern void at_tuner_op_cancel(void);
extern U8 at_tuner_get_state(void);
extern void at_tuner_create(void);
extern U8 at_tuner_power_off(void);
extern U8 at_tuner_power_on(void);

#endif /*_ATOMIC_TUNER_IF_H_*/

/****************************** END OF FILE ***************************/

