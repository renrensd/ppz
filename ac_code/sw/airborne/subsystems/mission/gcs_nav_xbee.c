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
#include "subsystems/mission/gcs_nav_xbee.h"
#include "firmwares/rotorcraft/nav_flight.h"
#include "subsystems/datalink/datalink.h"
#if DATALINK == XBEE
#include "subsystems/datalink/xbee.h"
#endif

#if DATALINK == TRANSPTA
#include "subsystems/datalink/transpta.h"
#endif	/* TRANSPTA */
/*for virtual RC*/
#include "subsystems/rc_nav/rc_nav_xbee.h"

#include "modules/system/timer_if.h"
#include "modules/system/timer_class.h"
#include "modules/system/timer_def.h"


#define GCS_MAX_COUNT  20   //lost_time_out =20/(2hz)=10s

uint8_t gcs_count;
bool_t  gcs_lost;

static inline void gcs_set_connect(void);

void gcs_lost_check(void)
{
#ifdef GCS_V1_OPTION
	if(!gcs_lost)
	{
		gcs_count++;
		if (gcs_count>GCS_MAX_COUNT)
		{
			gcs_lost = TRUE;
			gcs_count = 0;
			XbeeSetGcsConFalse();   //close gcs communication,wait for restart connect
		}
	}
	else
	{
		if(xbee_con_info.gcs_con_available)
		{
			XbeeSetGcsConFalse();
		}
	}
#endif	/* GCS_V1_OPTION */
}

static inline void gcs_set_connect(void)
{
	gcs_count = 0;    //reset gcs_count,use for check gcs_lost
	gcs_lost = FALSE;
}

void gcs_vrc_set_connect(void)
{
	gcs_set_connect();
	if(rc_type == VIRTUAL_RC)
	{
		rc_set_connect();
	}
}

void gcs_set_rc_type(void)
{
	if(flight_mode == nav_rc_mode)
	{
		rc_type = VIRTUAL_RC;
	}
	else
	{
		rc_type = REAL_RC;
	}
}

//TIMER(TIMER_GCS_VRC_ACK_MSG,                    send_heart_beat_A2VR_msg,               TIMER_TASK_GCS)
void gcs_vrc_ack_timer(void)
{
	tm_kill_timer(TIMER_GCS_VRC_ACK_MSG);
	tm_create_timer(TIMER_GCS_VRC_ACK_MSG, (500 MSECONDS), TIMER_THREE_SHOT,0);
}
/****************************** END OF FILE ***************************/
