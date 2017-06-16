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

#ifndef RC_NAV_DRIFT_H
#define RC_NAV_DRIFT_H

#include "std.h"

extern void rc_speed_rl_drift(int8_t sign);
extern void rc_rate_rotation_drift(int8_t sign);
extern void rc_speed_rl_drift_stop(void);
extern void rc_rate_rotation_drift_stop(void);

#endif /* END OF RC_NAV_DRIFT_H */
