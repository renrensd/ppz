/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : RN R&D SW2      									   *
*   AUTHOR	   :             										 	   *
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
#include "subsystems/mission/task_spray_misc.h"
#include "subsystems/datalink/downlink.h"
#include "uplink_ac.h"
#include "subsystems/ops/ops_app_if.h"
#include "modules/system/timer_if.h"
#include "modules/system/timer_class.h"
#include "modules/system/timer_def.h"


Spray_Convert_Info spray_convert_info;
Spray_Conti_Info spray_continual_info;

static void spray_convert_init(void);
static void spray_continual_init(void);

void task_spray_misc_init(void)
{
	spray_convert_init();
	spray_continual_init();
}

static void spray_convert_init(void)
{
	spray_convert_info.useful = FALSE;
}

static void spray_continual_init(void)
{
	spray_continual_info.flag_record = FALSE;
	spray_continual_info.flag_ack = FALSE;
}


/******here is diagrammatic sketch for convert caculate****/
/* (from wp)       D(shadow point)
   A *************************** B(next wp)
       *           *
          *        *
             *     *
                *  *
E ****************** C(next line wp)
*/
/**********************************************************/
uint8_t spray_convert_caculate(void)
{
	if( SPRAY_LINE==from_wp.action
			&& SPRAY_CONVERT==next_wp.action
			&& SPRAY_LINE==task_wp[0].action
			&& 1<nb_pending_wp)
	{
		struct Int32Vect2 wp_a, wp_b, wp_c, wp_d, wp_e; /*a:from b:next c:convert*/

		VECT2_COPY(wp_a, from_wp.wp_en);
		VECT2_COPY(wp_b, next_wp.wp_en);
		VECT2_COPY(wp_c, task_wp[0].wp_en);
		VECT2_COPY(wp_e, task_wp[1].wp_en);

		struct Int32Vect2 vect_ab, vect_ac, vect_ad, vect_r;
		VECT2_DIFF(vect_ab, wp_b, wp_a);
		VECT2_DIFF(vect_ac, wp_c, wp_a);
		double s = (double)(VECT2_DOT_PRODUCT(vect_ab, vect_ac)) / (double)(VECT2_NORM2(vect_ab));
		VECT2_SMUL( vect_ad, vect_ab, BFP_OF_REAL(s, 8) ); /*convert wp shadow vect*/
		VECT2_SDIV( vect_ad, vect_ad, (1<<8) );
		VECT2_SUM(wp_d, wp_a, vect_ad);

		VECT2_SDIV(wp_c, wp_c, 2);
		VECT2_SDIV(wp_d, wp_d, 2);
		VECT2_DIFF(vect_r, wp_c, wp_d);

		/*get radius for next spray line*/
		spray_convert_info.radius = VECT2_NORM2(vect_r);
		spray_convert_info.radius = (int32_t)sqrt((double)spray_convert_info.radius);

		/*get circle center for next spray line*/
		//VECT2_SUM(spray_convert_info.center, wp_c, wp_d);
		if( s>=1 )
		{
			VECT2_SUM(spray_convert_info.center, wp_c, wp_d);
		}
		else
		{
			VECT2_SUM(spray_convert_info.center, wp_b, vect_r);
		}

		int32_t line_orientation = int32_atan2(vect_ab.y, vect_ab.x);  /*toward east*/
		int32_t next_orientation = int32_atan2(vect_ac.y, vect_ac.x);
		INT32_ANGLE_NORMALIZE(line_orientation);
		INT32_ANGLE_NORMALIZE(next_orientation);
		int32_t deta_orientation = next_orientation - line_orientation;
		INT32_ANGLE_NORMALIZE(deta_orientation);
		if( deta_orientation > 0 )
		{
			spray_convert_info.radius = -spray_convert_info.radius;
		}
		else if( deta_orientation == 0 )
		{
			return SPRAY_CONVERT_FAIL;  /*line reclosing*/
		}

		/*caculate orientation of next spray line*/
		spray_convert_info.heading_sp = int32_atan2( (wp_e.y-2*wp_c.y), (wp_e.x-2*wp_c.x) );
		spray_convert_info.heading_sp = INT32_ANGLE_PI_2 - spray_convert_info.heading_sp;
		return SPRAY_CONVERT_SUCCESS;

	}
	return SPRAY_CONVERT_CONTINUAL;
}

/*resrve*/
bool_t spray_break_and_continual(void)
{
	if(!spray_continual_info.flag_record)
	{
		spray_continual_info.break_pos_lla = *stateGetPositionLla_i();
		spray_continual_info.break_spray_work = ops_info.spray_state;
		spray_continual_info.flag_record = TRUE;
#ifdef GCS_V1_OPTION
		tm_kill_timer(TIMER_GCS_SPRAY_BREAK_CONTINUAL_MSG);
		tm_create_timer(TIMER_GCS_SPRAY_BREAK_CONTINUAL_MSG, (2000 MSECONDS), 20,0);
#endif
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/*resrve*/
void spray_break_continual_msg(void)
{
	uint8_t wp_type = 1;  //default lla coordinate
	double pos_break_lon = (double)( (int64_t)(spray_continual_info.break_pos_lla.lon) /10000000.0 );
	double pos_break_lat = (double)( (int64_t)(spray_continual_info.break_pos_lla.lat) /10000000.0 );
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_GCS);
	DOWNLINK_SEND_EMERGENCY_RECORD_STATE(SecondChannel, SecondDevice,
																			 &wp_type,
																			 &spray_continual_info.break_spray_work,
																			 &pos_break_lon,
																			 &pos_break_lat);
}

/*resrve*/
void spray_bac_msg_stop(void)
{
	spray_continual_info.flag_ack = TRUE;
#ifdef GCS_V1_OPTION
	tm_kill_timer(TIMER_GCS_SPRAY_BREAK_CONTINUAL_MSG);
#endif
}

/****************************** END OF FILE ***************************/
