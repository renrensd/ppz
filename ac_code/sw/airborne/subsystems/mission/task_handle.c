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

#include "subsystems/mission/task_handle.h"
#include "subsystems/mission/task_manage.h"

#include "firmwares/rotorcraft/nav_flight.h"


/***********************************************************************
* FUNCTION    : get_max_pending_id
* DESCRIPTION :
* INPUTS      : none
* RETURN      : max wp_id in pending waypoints
***********************************************************************/
uint8_t get_max_pending_id(void)
{
	if(0 == nb_pending_wp)
	{
		return next_wp.wp_id;
	}
	else
	{
		return task_wp[nb_pending_wp-1].wp_id;
	}
}

/***********************************************************************
* FUNCTION    : get_task_wp_offset
* DESCRIPTION :
* INPUTS      : wp_id
* RETURN      : if exist, offset in pending,
***********************************************************************/
int8_t get_task_wp_offset(uint8_t id)
{
	for(uint8_t i=0; i<nb_pending_wp; i++)
	{
		if( task_wp[i].wp_id == id )
		{
			return i;
		}
	}
	return -1;
}

/***********************************************************************
* FUNCTION    : check_task_wp_id
* DESCRIPTION :
* INPUTS      : wp_id
* RETURN      : TRUE or FALSE
***********************************************************************/
uint8_t check_task_wp_id(uint8_t id)
{
	for(uint8_t i=0; i<nb_pending_wp; i++)
	{
		if( task_wp[i].wp_id == id )
		{
			return TRUE;
		}
	}
	return FALSE;
}

/***********************************************************************
* FUNCTION    : task_lla_to_enu_convert
* DESCRIPTION : convert lla pos to enu pos
* INPUTS      : pointer of lla/enu data
* RETURN      : TRUE or FALSE
***********************************************************************/
uint8_t task_lla_to_enu_convert(struct EnuCoor_i *enu, struct LlaCoor_i *lla)
{
	/* return fail if there is no valid local coordinate system*/
#if 1 //only for debug not open  
	if (!state.ned_initialized_i)
	{
		return FALSE;
	}
#endif

	/* lla conver to ecef,using double precision */
	struct LlaCoor_d in_d;
	in_d.lon = (double)(lla->lon)/100000000.0;
	in_d.lat = (double)(lla->lat)/100000000.0;
	in_d.alt = (double)(state.ned_origin_f.lla.alt) + (double)(ac_config_info.max_flight_height);
	/* calls the floating point transformation */
	struct EcefCoor_d out_d;
	ecef_of_lla_d(&out_d, &in_d);
	/* convert the output to fixed point       */
	struct EcefCoor_i in_ecef;
	in_ecef.x = (int32_t)CM_OF_M(out_d.x);
	in_ecef.y = (int32_t)CM_OF_M(out_d.y);
	in_ecef.z = (int32_t)CM_OF_M(out_d.z);

	/* ecef convert to enu,  enu  ENU point in cm, ecef ECEF point in cm */
	enu_of_ecef_point_i(enu, &state.ned_origin_i, &in_ecef);

	/* check MAX_DISTANCE from lef */
	if( abs(enu->x) > POINT_MAX_DISTANCE || abs(enu->y) > POINT_MAX_DISTANCE  )
	{
		return FALSE;
	}
	else
	{
		enu->x = POS_BFP_OF_REAL(enu->x)/100;
		enu->y = POS_BFP_OF_REAL(enu->y)/100;
		return TRUE;
	}
}

uint8_t task_lla_d_to_enu_i_convert(struct EnuCoor_i *enu, struct LlaCoor_d *lla)
{
	/* return fail if there is no valid local coordinate system*/
#if 1 //only for debug not open  
	if (!state.ned_initialized_i)
	{
		return FALSE;
	}
#endif

	/* lla conver to ecef,using double precision */
	struct LlaCoor_d in_d;
	in_d.lon = (lla->lon);
	in_d.lat = (lla->lat);
	in_d.alt = (double)(state.ned_origin_f.lla.alt) + (double)(ac_config_info.max_flight_height);
	/* calls the floating point transformation */
	struct EcefCoor_d out_d;
	ecef_of_lla_d(&out_d, &in_d);
	/* convert the output to fixed point       */
	struct EcefCoor_i in_ecef;
	in_ecef.x = (int32_t)CM_OF_M(out_d.x);
	in_ecef.y = (int32_t)CM_OF_M(out_d.y);
	in_ecef.z = (int32_t)CM_OF_M(out_d.z);

	/* ecef convert to enu,  enu  ENU point in cm, ecef ECEF point in cm */
	enu_of_ecef_point_i(enu, &state.ned_origin_i, &in_ecef);

	/* check MAX_DISTANCE from lef */
	if( abs(enu->x) > POINT_MAX_DISTANCE || abs(enu->y) > POINT_MAX_DISTANCE  )
	{
		return FALSE;
	}
	else
	{
		enu->x = POS_BFP_OF_REAL(enu->x)/100;
		enu->y = POS_BFP_OF_REAL(enu->y)/100;
		return TRUE;
	}
}

/****************************** END OF FILE ***************************/
