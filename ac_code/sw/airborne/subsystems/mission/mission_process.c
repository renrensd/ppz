
/** @file subsystems/mission/mission_process.c
 *  @brief mission navigation for rotorcrafts
 *
 *  Implement specific navigation routines for the mission control
 *  of a rotorcraft
 */

#include <stdio.h>
#include "subsystems/mission/mission_manage.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/nav/nav_survey_poly_osam.h"

#define ERROR_LAND_DISTANCE 3

// navigation time step
const float dt_navigation = 1.0 / ((float)NAV_FREQ);

//last_mission_wp, last target wp from mission elements, not used actively and kept for future implementations
struct EnuCoor_i last_mission_wp = { 0., 0., 0. };


static inline bool_t mission_nav_wp(struct EnuCoor_i *first_wp);
static inline bool_t mission_nav_circle(struct _mission_element *el);
static inline bool_t mission_nav_path(struct _mission_element *el);
static inline bool_t mission_nav_survey(struct _mission_element *el);
static inline bool_t mission_nav_hover(void);
static inline bool_t mission_nav_home(struct _mission_element *el);



/// Utility function: converts lla (float) to local point (float)
bool_t mission_point_of_lla(struct EnuCoor_f *point, struct LlaCoor_f *lla)
{
	// return FALSE if there is no valid local coordinate system
	if (!state.ned_initialized_f)
	{
		return FALSE;
	}
	//change geoid alt to ellipsoid alt
	lla->alt = lla->alt - state.ned_origin_f.hmsl + state.ned_origin_f.lla.alt;
	//Compute ENU components from LLA with respect to ltp origin
	enu_of_lla_point_f(point, &state.ned_origin_f, lla);
	point->z=1.5;  //fixed height
	return TRUE;
}



/** Navigation function to a single waypoint
*/
static inline bool_t mission_nav_wp(struct EnuCoor_i *first_wp)
{
	struct EnuCoor_i *target_wp = first_wp;

	//Check proximity and wait for 'duration' seconds in proximity circle if desired
	if (nav_approaching_from(target_wp, NULL, CARROT))
	{
		last_mission_wp = *target_wp;
		/*
		 if (el->duration > 0.)
		{
		   if (nav_check_wp_time(target_wp, el->duration)) { return FALSE; }
		 }
		else { return FALSE; }
		 */
		return FALSE;
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

static inline bool_t mission_nav_segment(struct _mission_element *el)
{
  struct EnuCoor_i *from_wp = &(el->element.mission_segment.from.from_i);
  struct EnuCoor_i *to_wp   = &(el->element.mission_segment.to.to_i);

  //Check proximity and wait for 'duration' seconds in proximity circle if desired
  if (nav_approaching_from(to_wp, from_wp, CARROT)) {
    last_mission_wp = *to_wp;

    if (el->duration > 0.) {
      if (nav_check_wp_time(to_wp, el->duration)) { return FALSE; }
    } else { return FALSE; }
  }

  //Route Between from-to
  horizontal_mode = HORIZONTAL_MODE_ROUTE;
  nav_route(from_wp, to_wp);
  NavVerticalAutoThrottleMode(RadOfDeg(0.0));
  NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(to_wp->z), 0.);

  return TRUE;
}
*/

/** Navigation function along a path
*/
static inline bool_t mission_nav_path(struct _mission_element *el)
{
	//enter first waypoint,using HORIZONTAL_MODE_WAYPOINT
	if (el->element.mission_path.path_idx == 0)
	{
		//first wp of path
		struct EnuCoor_i *wp_first;
		wp_first= el->element.mission_path.path_p;
		if (!mission_nav_wp(wp_first))
		{
			el->element.mission_path.path_idx++;
		}
	}

	//flight path along
	else if (el->element.mission_path.path_idx < el->element.mission_path.nb_wp)
	{
		//standart wp of path
		struct EnuCoor_i *from_wp = el->element.mission_path.path_p+el->element.mission_path.path_idx - 1;
		struct EnuCoor_i *to_wp   = el->element.mission_path.path_p+el->element.mission_path.path_idx;

		//Check proximity and wait for t seconds in proximity circle if desired
		if (nav_approaching_from(to_wp, from_wp, 0))
		{
			last_mission_wp = *to_wp;
			/*
			if (el->duration > 0.)
			{
			  if (nav_check_wp_time(to_wp, el->duration))
			{ el->element.mission_path.path_idx++; }
			}
			else
			{ el->element.mission_path.path_idx++; }
			*/
			el->element.mission_path.path_idx++;
		}
		//set heading along the path
		nav_set_heading_forward_line(from_wp,to_wp);
		if( nav_check_heading() )    //check heading error
		{
			//Route Between from-to
			horizontal_mode = HORIZONTAL_MODE_ROUTE;
			nav_route(from_wp, to_wp);
			NavVerticalAutoThrottleMode(RadOfDeg(0.0));
			NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(from_wp->z), 0.);
		}
		//else {   RunOnceEvery(8, mission_status_report());   }  //only use debug

	}

	//path end
	else
	{
		el->element.mission_path.path_idx=el->element.mission_path.nb_wp-1;   //decline last "++"
		el->status=success_done;
		return FALSE;
	}

	el->status=running;
	return TRUE;
}


/** Navigation function survey flight
*/
static inline bool_t mission_nav_survey(struct _mission_element *el)
{
	if (el->element.mission_survey.nb_survey < 3)
	{
		el->status=fail_done;
		return FALSE; // nothing to do if area is not exist
	}
	//call survey functions,need change the true or false result!!!!
	switch( survey_run_ms(el) )   //return 0:running,1:success,2:fail
	{
	case 0:
		break;
	case 1:
		el->status=success_done;
		return FALSE;
	case 2:
		el->status=fail_done;
		return FALSE;
	default:
		el->status=fail_done;
		return FALSE;
	}
	el->status=running;
	return TRUE;
}

/** Navigation function flight home(normal)
*/
static inline bool_t mission_nav_home(struct _mission_element *el)
{
	bool_t home_state;
	home_state = mission_nav_path(el);
	if(!home_state)
	{
		/*
		     float distance_land2 =get_dist2_to_point(el->element.mission_path.path_p + el->element.mission_path.nb_wp);
		if( distance_land2>(ERROR_LAND_DISTANCE*ERROR_LAND_DISTANCE) )  //error distance >3m
		{
		         el->status=fail_done;
			//something need to do			;
		}
		else
		*/
		el->status=success_done;
		return FALSE;
	}
	else
	{
		el->status=running;
		return TRUE;
	}
}

static inline bool_t mission_nav_hover(void)
{
	horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
	struct EnuCoor_i pos_current;
	pos_current=*stateGetPositionEnu_i();
	VECT3_COPY(navigation_target, pos_current);
	NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
	NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(pos_current.z), 0.);
}

/*
 *  ms_code:
 *       =0  running;
 *       =1  finish home,can land(hover)
 *       =2  no mission(hover)
 *       =3  cann't parse mission_status(hover)
 *       =4  spray_insert mission is not finished
 *       =5  finish home,fail_done(hover)
 *       =6  home run over,other status(hover)
 *       =10 cann't handle mission type(hover)
 */
//void debug_mission() {  mission.current_idx = mission.current_idx + 1; }

uint8_t mission_run()
{
	RunOnceEvery(2*NAV_FREQ, send_current_mission());
	uint8_t ms_code=0;  //default value
	//if(mission_priority)  run mission
	struct _mission_element *el = NULL;
	if ( (el = get_mission()) == NULL )
	{
		// mission is run over  trigger exeption mission--->hover--->home
		mission_nav_hover();
		return ms_code=2;  //mission run over,no normal back home
	}
	else
	{
		if(el->status!=standby && el->status!=running)
		{
			mission_nav_hover();
			return ms_code=3;
		}
	}

	//run current mission
	bool_t el_running = FALSE;   //TRUE mean current misssion is not finished
	switch (el->type)
	{
	case ms_path:
		el_running =mission_nav_path(el);
		break;
	case ms_spray_normal:
		el_running =mission_nav_survey(el);
		break;
	case ms_spray_insert:
		mission_nav_hover();
		return ms_code=4;  //no finished
		el_running =mission_nav_survey(el);
		break;
	case ms_home:
		el_running =mission_nav_home(el);
		if(!el_running)
		{
			send_current_mission();
			mission_nav_hover();
			if(el->status==success_done)  return ms_code=1;
			else if(el->status==fail_done)  return ms_code=5;
			else return  ms_code=6;
		}
		break;

	default: //hover
		mission_nav_hover();
		return ms_code=10;  // error, type not yet handled

	}

	//ms_debug_tid = sys_time_register_timer(10, (sys_time_cb)debug_mission);

	// increment element_time
	mission.element_time += dt_navigation;

	if (!el_running)  //current mission is finished
	{
		send_current_mission();
		// reset timer
		mission.element_time = 0.;
		// go to next element
		mission.current_idx = (mission.current_idx + 1); //% MISSION_ELEMENT_NB; no need recycle
	}
	return ms_code=0;
}



/*mission_status_check(el)
{
	switch(el->status)
	{
		case ready:
		case stop:
		case pause:
		case success_done:
		case fail_done:
		default:


	}
}
*/
