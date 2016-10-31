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
#include "subsystems/datalink/datalink.h"
#if DATALINK == XBEE
#include "subsystems/datalink/xbee.h"
#endif

#if DATALINK == TRANSPTA
#include "subsystems/datalink/transpta.h"
#endif	/* TRANSPTA */

#define GCS_MAX_COUNT  20   //lost_time_out =12/(2hz)=6s

uint8_t gcs_count;  
bool_t  gcs_lost;


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

void gcs_set_connect(void)
{
	gcs_count = 0;    //reset gcs_count,use for check gcs_lost
	gcs_lost = FALSE;
}


/****************************** END OF FILE ***************************/
