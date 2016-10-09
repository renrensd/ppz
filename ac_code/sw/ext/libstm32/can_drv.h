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
#ifndef _CAN_DRV_H_
#define _CAN_DRV_H_ 
#include "stm32f4xx_can.h"


/**** Definition of constants ****/

/**** Definition of types ****/ 
typedef void(* can_rx_callback_tt)(uint32_t id, uint8_t *buf, int len);

/**** Declaration of constants ****/


/**** Declaration of variables ****/


/**** Declaration of functions ****/ 


void can_drv_nvic_config(void);


extern void can_drv_init(void);
extern int can_drv_transmit(uint32_t id, const uint8_t *buf, uint8_t len);




#endif /*_CAN_DRV_H_*/

/****************************** END OF FILE ***************************/

