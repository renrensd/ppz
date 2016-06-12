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
#ifndef _WDG_H_
#define _WDG_H 
#include "stm32f4xx_wwdg.h"


/**** Definition of constants ****/
#define WDG_FEED_TIME	14 //14*2 = 28ms.


/**** Definition of types ****/ 

/**** Declaration of constants ****/


/**** Declaration of variables ****/


/**** Declaration of functions ****/ 
void wdg_nvic_config(void);


extern void wdg_init(void);
extern void wdg_feed(void);
extern void wdg_enable(void);
extern void wdg_feed_handle(void);


#endif /*_WDG_H_*/

/****************************** END OF FILE ***************************/

