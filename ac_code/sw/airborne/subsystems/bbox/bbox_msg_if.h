/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				       *
*				  All Rights Reserved.          					   *
*   Department : R&D SW      									       *
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
#ifndef _BBOX_MSG_IF_H_
#define _BBOX_MSG_IF_H_ 

/**** Definition of types ****/ 

/**** Definition of macros ****/


/**** Declaration of constants ****/


/**** Declaration of variables ****/


/**** Declaration of functions ****/
extern void bbox_msg_heart_beat(void);
extern void bbox_msg_handle(uint16_t can_id, uint8_t *frame, uint8_t len);

#endif /*_BBOX_MSG_IF_H_*/
/****************************** END OF FILE ***************************/
