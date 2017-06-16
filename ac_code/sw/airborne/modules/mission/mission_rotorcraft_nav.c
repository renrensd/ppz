/*
 * Copyright (C) 2014 Kadir Cimenci
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/mission/mission_rotorcraft_nav.c
 *  @brief mission navigation for rotorcrafts
 *
 *  Implement specific navigation routines for the mission control
 *  of a rotorcraft
 */

#include <stdio.h>
#include "modules/mission/mission_common.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"


//Buffer zone in [m] before MAX_DIST_FROM_HOME
#define BUFFER_ZONE_DIST 3

/// Utility function: converts lla (float) to local point (float)
bool_t mission_point_of_lla(struct EnuCoor_f *point, struct LlaCoor_f *lla)
{
	// return FALSE if there is no valid local coordinate system
	if (!state.ned_initialized_f)
	{
		return FALSE;
	}
	// change geoid alt to ellipsoid alt
	//lla->alt = lla->alt - state.ned_origin_f.hmsl + state.ned_origin_f.lla.alt;

	//Compute ENU components from LLA with respect to ltp origin
	//struct EnuCoor_f tmp_enu_point_f;
	enu_of_lla_point_f(point, &state.ned_origin_f, lla);
	//VECT2_COPY(*point, tmp_enu_point_f);
	return TRUE;
}

//Function that converts target wp from float point versions to int
bool_t mission_element_convert(struct _mission_element *el)
{
	struct _mission_element tmp_element = *el;
	uint8_t i = 0;

	switch (tmp_element.type)
	{
	case MissionWP:
		ENU_BFP_OF_REAL(el->element.mission_wp.wp.wp_i, tmp_element.element.mission_wp.wp.wp_f);
		break;
	case MissionCircle:
		ENU_BFP_OF_REAL(el->element.mission_circle.center.center_i, tmp_element.element.mission_circle.center.center_f);
		break;
	case MissionSegment:
		ENU_BFP_OF_REAL(el->element.mission_segment.from.from_i, tmp_element.element.mission_segment.from.from_f);
		ENU_BFP_OF_REAL(el->element.mission_segment.to.to_i, tmp_element.element.mission_segment.to.to_f);
		break;
	case MissionPath:
		for (i = 0; i < 5; i++)
		{
			ENU_BFP_OF_REAL(el->element.mission_path.path.path_i[i], tmp_element.element.mission_path.path.path_f[i]);
		}
		break;
	default:
		// invalid element type
		return FALSE;
		break;
	}

	return TRUE;
}

// navigation time step
const float dt_navigation = 1.0 / ((float)NAV_FREQ);

//  last_mission_wp, last target wp from mission elements, not used actively and kept for future implementations
struct EnuCoor_i last_mission_wp = { 0., 0., 0. };

/** Navigation function to a single waypoint
*/
static inline bool_t mission_nav_wp(struct _mission_element *el)
{
	struct EnuCoor_i *target_wp = &(el->element.mission_wp.wp.wp_i);

	//Check proximity and wait for 'duration' seconds in proximity circle if desired
	if (nav_approaching_from(target_wp, NULL, CARROT))
	{
		last_mission_wp = *target_wp;

		if (el->duration > 0.)
		{
			if (nav_check_wp_time(target_wp, el->duration))
			{
				return FALSE;
			}
		}
		else
		{
			return FALSE;
		}

	}
	//Go to Mission Waypoint
	horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
	VECT3_COPY(navigation_target, *target_wp);
	NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
	NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(target_wp->z), 0.);

	return TRUE;
}

/** Navigation function on a circle
*/
static inline bool_t mission_nav_circle(struct _mission_element *el)
{
	struct EnuCoor_i *center_wp = &(el->element.mission_circle.center.center_i);
	int32_t radius = POS_BFP_OF_REAL(el->element.mission_circle.radius);

	//Draw the desired circle for a 'duration' time
	horizontal_mode = HORIZONTAL_MODE_CIRCLE;
	nav_circle(center_wp, radius);
	NavVerticalAutoThrottleMode(RadOfDeg(0.0));
	NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(center_wp->z), 0.);

	if (el->duration > 0. && mission.element_time >= el->duration)
	{
		return FALSE;
	}

	return TRUE;
}

/** Navigation function along a segment
*/
static inline bool_t mission_nav_segment(struct _mission_element *el)
{
	struct EnuCoor_i *from_wp = &(el->element.mission_segment.from.from_i);
	struct EnuCoor_i *to_wp   = &(el->element.mission_segment.to.to_i);

	//Check proximity and wait for 'duration' seconds in proximity circle if desired
	if (nav_approaching_from(to_wp, from_wp, CARROT))
	{
		last_mission_wp = *to_wp;

		if (el->duration > 0.)
		{
			if (nav_check_wp_time(to_wp, el->duration))
			{
				return FALSE;
			}
		}
		else
		{
			return FALSE;
		}
	}

	//Route Between from-to
	horizontal_mode = HORIZONTAL_MODE_ROUTE;
	nav_route(from_wp, to_wp);
	NavVerticalAutoThrottleMode(RadOfDeg(0.0));
	NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(to_wp->z), 0.);

	return TRUE;
}


/** Navigation function along a path
*/
static inline bool_t mission_nav_path(struct _mission_element *el)
{
	if (el->element.mission_path.nb == 0)
	{
		return FALSE; // nothing to do
	}

	if (el->element.mission_path.path_idx == 0)   //first wp of path
	{
		el->element.mission_wp.wp.wp_i = el->element.mission_path.path.path_i[0];
		if (!mission_nav_wp(el))
		{
			el->element.mission_path.path_idx++;
		}
	}

	else if (el->element.mission_path.path_idx < el->element.mission_path.nb)   //standart wp of path
	{

		struct EnuCoor_i *from_wp = &(el->element.mission_path.path.path_i[(el->element.mission_path.path_idx) - 1]);
		struct EnuCoor_i *to_wp   = &(el->element.mission_path.path.path_i[el->element.mission_path.path_idx]);

		//Check proximity and wait for t seconds in proximity circle if desired
		if (nav_approaching_from(to_wp, from_wp, CARROT))
		{
			last_mission_wp = *to_wp;

			if (el->duration > 0.)
			{
				if (nav_check_wp_time(to_wp, el->duration))
				{
					el->element.mission_path.path_idx++;
				}
			}
			else
			{
				el->element.mission_path.path_idx++;
			}
		}
		//Route Between from-to
		horizontal_mode = HORIZONTAL_MODE_ROUTE;
		nav_route(from_wp, to_wp);
		NavVerticalAutoThrottleMode(RadOfDeg(0.0));
		NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(from_wp->z), 0.);
	}
	else
	{
		return FALSE;  //end of path
	}

	return TRUE;
}

/** Navigation function flight nav_path and survey spray
*/
static inline bool_t mission_nav_survey(struct _mission_element *el)
{
	if (el->element.mission_path.nb == 0)
	{
		return FALSE; // nothing to do
	}

	if (el->element.mission_path.path_idx == 0)   //first wp of path
	{
		el->element.mission_wp.wp.wp_i = el->element.mission_path.path.path_i[0];
		if (!mission_nav_wp(el))
		{
			el->element.mission_path.path_idx++;
		}
	}

	else if (el->element.mission_path.path_idx < el->element.mission_path.nb)   //standart wp of path
	{

		struct EnuCoor_i *from_wp = &(el->element.mission_path.path.path_i[(el->element.mission_path.path_idx) - 1]);
		struct EnuCoor_i *to_wp   = &(el->element.mission_path.path.path_i[el->element.mission_path.path_idx]);

		//Check proximity and wait for t seconds in proximity circle if desired
		if (nav_approaching_from(to_wp, from_wp, CARROT))
		{
			last_mission_wp = *to_wp;

			if (el->duration > 0.)
			{
				if (nav_check_wp_time(to_wp, el->duration))
				{
					el->element.mission_path.path_idx++;
				}
			}
			else
			{
				el->element.mission_path.path_idx++;
			}
		}
		//Route Between from-to
		horizontal_mode = HORIZONTAL_MODE_ROUTE;
		nav_route(from_wp, to_wp);
		NavVerticalAutoThrottleMode(RadOfDeg(0.0));
		NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(from_wp->z), 0.);
	}
	else
	{
		return FALSE;  //end of path
	}

	return TRUE;
}


bool_t mission_run()
{


	struct _mission_element *el = NULL;
	if ((el = mission_get()) == NULL)    // mission is run over
	{
		//mission.element_time = 0;
		//mission.current_idx  = 0;
		//call hover to handle error
		return FALSE;
	}
	//run current mission
	bool_t el_running = FALSE;   //TRUE mean current misssion is not finished
	switch (el->type)
	{
	case MissionSurvey:
		//call function
		break;
	case MissionPath:
		el_running = mission_nav_path(el);
		break;
	case MissionHome:
		//call function
		break;
	case MissionReland:
		//call function
		break;
		//reserve
	case MissionCircle:
		el_running = mission_nav_circle(el);
		break;
	case MissionSegment:
		el_running = mission_nav_segment(el);
		break;
	default:
		// invalid type or pattern not yet handled,will goto next mission
		break;
	}

	// increment element_time
	mission.element_time += dt_navigation;

	if (!el_running)  //current mission is finished
	{
		// reset timer
		mission.element_time = 0.;
		// go to next element
		mission.current_idx = (mission.current_idx + 1); //% MISSION_ELEMENT_NB; no need recycle
	}
	return TRUE;
}


