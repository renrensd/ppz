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



/*---Private include files--------------------------------------------*/
#include <stm32f4xx.h>
#include "stm32f4xx_conf.h"
#include "wdg.h"


/*===VARIABLES========================================================*/
volatile uint8_t wdg_time_cnt;

/*---Global-----------------------------------------------------------*/
/***********************************************************************
* FUNCTION    : wdg_nvic_config
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void wdg_nvic_config(void) 
{
 	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	
}

/***********************************************************************
* FUNCTION    : wdg_init
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void wdg_init(void)
{
  /* Enable WWDG clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
  /* WWDG clock counter = (PCLK1 (42MHz)/4096)/8 = 1281 Hz (~780 us)  */
  WWDG_SetPrescaler(WWDG_Prescaler_8);
  /* Set Window value to 120; WWDG counter should be refreshed only when the counter
    is below 80 (and greater than 64) otherwise a reset will be generated */
  WWDG_SetWindowValue(120);
  /* Enable WWDG and set counter value to 127, WWDG timeout = ~780 us * 64 = 49.92 ms 
    In this case the refresh window is: ~780 * (127-120) = 5.46ms < refresh window < ~780 * 64 = 49.9ms
  */
  wdg_nvic_config();
  WWDG_ClearFlag();
  WWDG_EnableIT();  
  wdg_time_cnt = 0;

  DBGMCU_APB1PeriphConfig(DBGMCU_WWDG_STOP,ENABLE);
}

/***********************************************************************
* FUNCTION    : wdg_feed
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void wdg_feed(void)
{
    WWDG_SetCounter(127);
}

/***********************************************************************
* FUNCTION    : wdg_enable
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void wdg_enable(void)
{
     WWDG_Enable(127);
}

/***********************************************************************
* FUNCTION    : wdg_feed_handle
* DESCRIPTION : feed time is:5ms ~ 49ms.
				main thread frequency 500hz - 2ms.
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void wdg_feed_handle(void)
{
    wdg_time_cnt++;
	if(wdg_time_cnt > WDG_FEED_TIME)	//28ms.
	{
		wdg_time_cnt = 0;
		wdg_feed();
	}
}
/**************** END OF FILE *****************************************/

