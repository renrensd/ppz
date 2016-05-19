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

#include "subsystems/mission/task_spray_convert.h"


Spray_Convert_Info spray_convert_info;

void spray_convert_init(void)
{
	spray_convert_info.useful = FALSE;
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

bool_t spray_convert_caculate(void)
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
		/*get circle center for next spray line*/
		VECT2_SUM(spray_convert_info.center, wp_c, wp_d);
		VECT2_DIFF(vect_r, wp_c, wp_d);
		spray_convert_info.radius = VECT2_NORM2(vect_r);
		/*get radius for next spray line*/
		spray_convert_info.radius = (int32_t)sqrt((double)spray_convert_info.radius);
		
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
			return FALSE;  /*line reclosing*/
		}

		/*caculate orientation of next spray line*/
		spray_convert_info.heading_sp = int32_atan2( (wp_e.y-2*wp_c.y), (wp_e.x-2*wp_c.x) );
		spray_convert_info.heading_sp = INT32_ANGLE_PI_2 - spray_convert_info.heading_sp;
		return TRUE;
		
	}
	return FALSE;
}

/****************************** END OF FILE ***************************/