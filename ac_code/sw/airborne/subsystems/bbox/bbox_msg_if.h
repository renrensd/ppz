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

extern void bbox_msg_request_upgrade(void);
extern void bbox_msg_ready_status(void);
extern void bbox_msg_send_frame(uint8_t *pt_value,uint8_t length);
void bbox_msg_update_over(uint8_t num,uint8_t *arg);
/**** Declaration of functions ****/
extern void bbox_msg_heart_beat(void);
extern void bbox_msg_log_start(void);
extern void bbox_msg_log_end(void);
extern void bbox_write_file_fault(char *buf, uint8_t len);
extern void bbox_request_software_version(void);

extern void bbox_msg_handle(uint16_t can_id, uint8_t *frame, uint8_t len);

#endif /*_BBOX_MSG_IF_H_*/
/****************************** END OF FILE ***************************/
