/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : RN R&D SW2      									   *
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
#ifndef _GCS_NAV_XBEE_H_
#define _GCS_NAV_XBEE_H_

#include <inttypes.h>
#include "std.h"

extern uint8_t gcs_count;  
extern bool_t  gcs_lost;


extern void gcs_lost_check(void);
extern void gcs_vrc_set_connect(void);
extern void gcs_set_rc_type(void);
extern void gcs_vrc_ack_timer(void);


#endif /*_GCS_NAV_XBEE_H_*/
/****************************** END OF FILE ***************************/
