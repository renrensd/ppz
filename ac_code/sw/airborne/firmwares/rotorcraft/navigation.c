/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 */

/**
 * @file firmwares/rotorcraft/navigation.c
 *
 * Rotorcraft navigation functions.
 */


#define NAV_C

#include "firmwares/rotorcraft/navigation.h"

#include "pprz_debug.h"
#include "subsystems/gps.h" // needed by auto_nav from the flight plan
#include "subsystems/ins.h"
#include "state.h"

#include "firmwares/rotorcraft/autopilot.h"
#include "generated/modules.h"
#include "generated/flight_plan.h"
//#ifdef USE_MISSION
#include "firmwares/rotorcraft/nav_flight.h"
//#endif

/* for default GUIDANCE_H_USE_REF */
#include "firmwares/rotorcraft/guidance/guidance_h.h"

#include "math/pprz_algebra_int.h"

#include "subsystems/datalink/downlink.h"
#include "messages.h"
#include "mcu_periph/uart.h"

struct EnuCoor_i navigation_target;  //now only use pos x/y,send to navigation_carrot
struct EnuCoor_i navigation_carrot;
struct EnuCoor_i navigation_obstacle;

struct EnuCoor_i nav_last_point;

uint8_t last_wp UNUSED;

/** Maximum distance from HOME waypoint before going into failsafe mode */
#ifndef FAILSAFE_MODE_DISTANCE
#define FAILSAFE_MODE_DISTANCE (1.5*MAX_DIST_FROM_HOME)
#endif

const float max_dist_from_home = MAX_DIST_FROM_HOME;
const float max_dist2_from_home = MAX_DIST_FROM_HOME * MAX_DIST_FROM_HOME;
float failsafe_mode_dist2 = FAILSAFE_MODE_DISTANCE * FAILSAFE_MODE_DISTANCE;
float dist2_to_home;
bool_t too_far_from_home;

bool_t exception_flag[10] = {0}; //exception flags that can be used in the flight plan

float dist2_to_wp;
bool_t route_brake_flag;
float  accel_route_brake;

bool_t stop_brake_flag;
float  accel_stop_brake;

uint8_t set_carrot_angle;

uint8_t horizontal_mode;
struct EnuCoor_i nav_segment_start, nav_segment_end;
struct EnuCoor_i nav_circle_center;
int32_t nav_circle_radius, nav_circle_qdr, nav_circle_radians;

int32_t nav_leg_progress;
int32_t nav_leg_length;

int32_t nav_leg_progress2_from;  // <0:signed AC not reach start waypoint
int32_t nav_leg_progress2_end;   // <0:sign AC out of end waypoint

bool_t nav_survey_active;

int32_t nav_roll, nav_pitch;
int32_t nav_heading;
float nav_radius;
float nav_climb_vspeed, nav_descend_vspeed;
uint8_t CARROT_DIST_MUL=2;
	

/** default nav_circle_radius in meters */
#ifndef DEFAULT_CIRCLE_RADIUS
#define DEFAULT_CIRCLE_RADIUS 5.
#endif

#ifndef NAV_CLIMB_VSPEED
#define NAV_CLIMB_VSPEED 0.5
#endif

#ifndef NAV_DESCEND_VSPEED
#define NAV_DESCEND_VSPEED -0.5
#endif

uint8_t vertical_mode;
uint32_t nav_throttle;
int32_t nav_climb, nav_altitude, nav_flight_altitude;
int32_t flight_limit_height;
float flight_altitude;

static inline void nav_set_altitude(void);

#define CLOSE_TO_WAYPOINT (15 << 8)
#define CARROT_DIST (12 << 8)
/* from fixedwing ,using in survey */
float nav_circle_radians_no_rewind; 
bool_t  nav_in_circle;
bool_t  nav_in_segment;
float   nav_shift;

/** minimum horizontal distance to waypoint to mark as arrived */
#ifndef ARRIVED_AT_WAYPOINT
#define ARRIVED_AT_WAYPOINT 0.5
#endif

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

void set_exception_flag(uint8_t flag_num) {
  exception_flag[flag_num] = 1;
}

static void send_nav_status(struct transport_tx *trans, struct link_device *dev)
{
  float dist_home = sqrtf(dist2_to_home);
  float dist_wp = sqrtf(dist2_to_wp);
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  pprz_msg_send_ROTORCRAFT_NAV_STATUS(trans, dev, AC_ID,
                                      &block_time, &stage_time,
                                      &dist_home, &dist_wp,
                                      &nav_block, &nav_stage,
                                      &horizontal_mode);
  if (horizontal_mode == HORIZONTAL_MODE_ROUTE) {
    float sx = POS_FLOAT_OF_BFP(nav_segment_start.x);
    float sy = POS_FLOAT_OF_BFP(nav_segment_start.y);
    float ex = POS_FLOAT_OF_BFP(nav_segment_end.x);
    float ey = POS_FLOAT_OF_BFP(nav_segment_end.y);
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
    pprz_msg_send_SEGMENT(trans, dev, AC_ID, &sx, &sy, &ex, &ey);
  } else if (horizontal_mode == HORIZONTAL_MODE_CIRCLE) {
    float cx = POS_FLOAT_OF_BFP(nav_circle_center.x);
    float cy = POS_FLOAT_OF_BFP(nav_circle_center.y);
    float r = POS_FLOAT_OF_BFP(nav_circle_radius);
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
    pprz_msg_send_CIRCLE(trans, dev, AC_ID, &cx, &cy, &r);
  }
}

static void send_wp_moved(struct transport_tx *trans, struct link_device *dev)
{
  static uint8_t i;
  i++;
  if (i >= nb_waypoint) { i = 0; }
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  pprz_msg_send_WP_MOVED_ENU(trans, dev, AC_ID,
                             &i,
                             &(waypoints[i].enu_i.x),
                             &(waypoints[i].enu_i.y),
                             &(waypoints[i].enu_i.z));
}
#endif

void nav_init(void)
{ 
  nav_block = 0;
  nav_stage = 0;
  waypoints_init();
  nav_altitude = POS_BFP_OF_REAL(SECURITY_HEIGHT);
  nav_flight_altitude = nav_altitude;
  flight_altitude = SECURITY_ALT;
  VECT3_COPY(navigation_target, waypoints[WP_HOME].enu_i);
  VECT3_COPY(navigation_carrot, waypoints[WP_HOME].enu_i);
  VECT3_COPY(navigation_obstacle, waypoints[WP_S5].enu_i);
 #ifndef NPS_SIMU
  nav_flight_init();   //could run with flight_plan together
 #endif

  horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
  vertical_mode = VERTICAL_MODE_ALT;

  nav_roll = 0;
  nav_pitch = 0;
  nav_heading = 0;
  nav_radius = DEFAULT_CIRCLE_RADIUS;
  nav_climb_vspeed = NAV_CLIMB_VSPEED;
  nav_descend_vspeed = NAV_DESCEND_VSPEED;
  nav_throttle = 0;
  nav_climb = 0;
  nav_leg_progress = 0;
  nav_leg_length = 1;

  too_far_from_home = FALSE;
  dist2_to_home = 0;
  dist2_to_wp = 0;
  route_brake_flag = FALSE;
  accel_route_brake = 0.0;
  set_carrot_angle = 50;
  flight_limit_height =FLIGHT_LIMIT_HEIGHT;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_NAV_STATUS, send_nav_status);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WP_MOVED, send_wp_moved);
#endif
}

static inline void UNUSED nav_advance_carrot(void)
{
  struct EnuCoor_i *pos = stateGetPositionEnu_i();
  /* compute a vector to the waypoint */
  struct Int32Vect2 path_to_waypoint;
  VECT2_DIFF(path_to_waypoint, navigation_target, *pos);

  /* saturate it */
  VECT2_STRIM(path_to_waypoint, -(1 << 15), (1 << 15));

  int32_t dist_to_waypoint = int32_vect2_norm(&path_to_waypoint);

  if (dist_to_waypoint < CLOSE_TO_WAYPOINT) {
    VECT2_COPY(navigation_carrot, navigation_target);
  } else {
    struct Int32Vect2 path_to_carrot;
    VECT2_SMUL(path_to_carrot, path_to_waypoint, CARROT_DIST);
    VECT2_SDIV(path_to_carrot, path_to_carrot, dist_to_waypoint);
    VECT2_SUM(navigation_carrot, path_to_carrot, *pos);
  }
}

void nav_run(void)
{

#if GUIDANCE_H_USE_REF
  // if GUIDANCE_H_USE_REF, CARROT_DIST is not used
  VECT2_COPY(navigation_carrot, navigation_target);
  VECT3_COPY(navigation_obstacle, waypoints[WP_S5].enu_i);
#else
  nav_advance_carrot();
#endif
  if((horizontal_mode == HORIZONTAL_MODE_WAYPOINT) && (guidance_h.mode == GUIDANCE_H_MODE_NAV))
  {
  	struct FloatVect2 target_wp;
  	target_wp.x = POS_FLOAT_OF_BFP(navigation_carrot.y);
  	target_wp.y = POS_FLOAT_OF_BFP(navigation_carrot.x);
  	guidance_h_trajectory_tracking_set_hover(target_wp);
  }

  nav_set_altitude();
}

void nav_set_flight_speed(float speed)
{
	guidance_h_trajectory_tracking_set_ref_speed(speed);
}

void nav_circle(struct EnuCoor_i *wp_center, int32_t radius)
{
  if (radius == 0) {
    VECT2_COPY(navigation_target, *wp_center);
    dist2_to_wp = get_dist2_to_point(wp_center);
	//change psi while cirle run
	struct EnuCoor_f WP_C;
	ENU_FLOAT_OF_BFP(WP_C, *wp_center);
	nav_set_heading_towards(WP_C.x, WP_C.y);
  } 
  else 
  {
    struct Int32Vect2 pos_diff;
    VECT2_DIFF(pos_diff, *stateGetPositionEnu_i(), *wp_center); /*vect toward circle*/
    // go back to half metric precision or values are too large
    //INT32_VECT2_RSHIFT(pos_diff,pos_diff,INT32_POS_FRAC/2);
    // store last qdr
    int32_t last_qdr = nav_circle_qdr;
    // compute qdr(orientation toward center)
    nav_circle_qdr = int32_atan2(pos_diff.y, pos_diff.x);
    // increment circle radians
    if (nav_circle_radians != 0) {
      int32_t angle_diff = nav_circle_qdr - last_qdr;
      INT32_ANGLE_NORMALIZE(angle_diff);
      nav_circle_radians += angle_diff;
    } else {
      // Smallest angle to increment at next step
      nav_circle_radians = 1;
    }

    // direction of rotation
    int8_t sign_radius = radius > 0 ? 1 : -1;
	int32_t trigo_diff = nav_circle_qdr - last_qdr;
    INT32_ANGLE_NORMALIZE(trigo_diff);
    trigo_diff *= - sign_radius;
    if (trigo_diff > 0) { // do not rewind if the change in angle is in the opposite sense than nav_radius
      nav_circle_radians_no_rewind += trigo_diff;
    }
/*
	int32_t diff_qdr = nav_circle_qdr - last_qdr;
	INT32_ANGLE_NORMALIZE(diff_qdr);
	if( diff_qdr * sign_radius > 0 )
	{
	    nav_circle_qdr = last_qdr;
	}	
*/
    // absolute radius
    int32_t abs_radius = abs(radius);
    // carrot_angle2
    int32_t carrot_angle = (((CARROT_DIST/3) << INT32_ANGLE_FRAC) / abs_radius); 
    Bound(carrot_angle, (INT32_ANGLE_PI / 8), INT32_ANGLE_PI/ 5);
    carrot_angle = nav_circle_qdr - sign_radius * carrot_angle;
    int32_t s_carrot, c_carrot;
    PPRZ_ITRIG_SIN(s_carrot, carrot_angle);
    PPRZ_ITRIG_COS(c_carrot, carrot_angle);
    // compute setpoint
    VECT2_ASSIGN(pos_diff, abs_radius * c_carrot, abs_radius * s_carrot);
    INT32_VECT2_RSHIFT(pos_diff, pos_diff, INT32_TRIG_FRAC);
    VECT2_SUM(navigation_target, *wp_center, pos_diff);	
	//change psi while circle run
	if(sign_radius==1) 
	{
		nav_heading=INT32_ANGLE_PI-carrot_angle;
	}
	else 
	{
		nav_heading=-carrot_angle;
	}
  }
  
  nav_circle_center = *wp_center;
  nav_circle_radius = radius;
  horizontal_mode = HORIZONTAL_MODE_CIRCLE;
}

bool_t nav_spray_convert(struct EnuCoor_i wp_center, int32_t radius, int32_t sp_heading)
{
    static bool_t leave_flag = TRUE;
    struct Int32Vect2 pos_diff;
    int8_t sign_radius = radius > 0 ? 1 : -1;
    int32_t abs_radius = abs(radius);
    VECT2_DIFF(pos_diff, *stateGetPositionEnu_i(), wp_center); /*vect radial direction*/

    // compute qdr(orientation toward center)
    int32_t last_nav_circle_qdr = nav_circle_qdr;
    nav_circle_qdr = int32_atan2(pos_diff.y, pos_diff.x);
    if(!leave_flag)
    {
	int32_t diff_qdr = nav_circle_qdr - last_nav_circle_qdr;
	INT32_ANGLE_NORMALIZE(diff_qdr);
	if( diff_qdr * sign_radius > 0 )
	{
	    nav_circle_qdr = last_nav_circle_qdr;
	}
    }

    //int32_t carrot_angle = (((CARROT_DIST/4) << INT32_ANGLE_FRAC) / abs_radius); 
    //Bound(carrot_angle, (INT32_ANGLE_PI / 8), INT32_ANGLE_PI*3/11);
	int32_t carrot_angle = INT32_RAD_OF_DEG_Q12(set_carrot_angle);
	Bound(carrot_angle, (INT32_ANGLE_PI / 8), INT32_ANGLE_PI_2);
	
    int32_t heading_ccw = (1+sign_radius)*INT32_ANGLE_PI_2 - sp_heading;
	
    // carrot_angle to guidance circle forward
    carrot_angle = nav_circle_qdr - sign_radius * carrot_angle; /*CCW angle,0 is east*/  
	
	INT32_COURSE_NORMALIZE(carrot_angle);
	INT32_COURSE_NORMALIZE(heading_ccw);
	int32_t deta_angle = heading_ccw-carrot_angle;
	INT32_ANGLE_NORMALIZE(deta_angle);

	leave_flag = FALSE;  //set default false
	if(sign_radius == 1)
	{
		if( deta_angle > 0 && deta_angle < INT32_ANGLE_PI_2)
		{
			carrot_angle = heading_ccw;
            //if(deta_angle > INT32_ANGLE_PI*0.2)
            //{
      			leave_flag = TRUE;
            //}
		}
		nav_heading = INT32_ANGLE_PI-carrot_angle;   /*set heading follow circle*/
	}
	else
	{
		if( deta_angle< 0 && deta_angle > -INT32_ANGLE_PI_2 )
		{
			carrot_angle = heading_ccw;
            //if(deta_angle < -INT32_ANGLE_PI*0.2)
            //{
       			leave_flag = TRUE;
            //}
		}	
		nav_heading = -carrot_angle;   /*set heading follow circle*/
	}
	
   int32_t s_carrot, c_carrot;
   PPRZ_ITRIG_SIN(s_carrot, carrot_angle);
   PPRZ_ITRIG_COS(c_carrot, carrot_angle);
   // compute setpoint
   VECT2_ASSIGN(pos_diff, abs_radius * c_carrot, abs_radius * s_carrot);
   INT32_VECT2_RSHIFT(pos_diff, pos_diff, INT32_TRIG_FRAC);
   VECT2_SUM(navigation_target, wp_center, pos_diff);	
  
   nav_circle_center = wp_center;
   nav_circle_radius = radius;
   horizontal_mode = HORIZONTAL_MODE_CIRCLE;

   return leave_flag;
}

#if 0
void nav_route(struct EnuCoor_i *wp_star, struct EnuCoor_i *wp_end)
{
  //static uint8_t last_regulate_ratio = 2;
  //uint8_t regulate_ratio = 2;
  struct Int32Vect2 wp_diff, pos_diff, wp_diff_prec ,wp_corre;
  struct EnuCoor_i pos_current;
  
  VECT2_DIFF(wp_diff, *wp_end, *wp_star);                 //wp_diff is deta of route(x,y)
  pos_current = *stateGetPositionEnu_i();
  VECT2_DIFF(pos_diff, pos_current, *wp_star);            //pos_diff is deta from star(x,y)
  // go back to metric precision or values are too large
  VECT2_COPY(wp_diff_prec, wp_diff);                      //pos_diff_prec is deta of route(x,y)

  uint32_t leg_length2 = Max(((int64_t)wp_diff.x*(int64_t)wp_diff.x + (int64_t)wp_diff.y*(int64_t)wp_diff.y)>>16, 1);   //leg_length2 is route length square(real)
  int32_t leg_progress2 = ((int64_t)pos_diff.x*(int64_t)wp_diff.x + (int64_t)pos_diff.y*(int64_t)wp_diff.y)>>16;        //leg_progress2 is route_len * (shadow of cur_star len)
  nav_leg_progress2_from = leg_progress2;   // <0:signed AC not reach start waypoint
  nav_leg_length = int32_sqrt(leg_length2);                                             //nav_leg_length is real length of route
  nav_leg_progress = (float)(leg_progress2 / nav_leg_length);                                    //nav_leg_progress is shadow length of flighted line
 
#if 0  //use cornering calibrate
  /** caculate corrective wp to ajust progress length ,by WHP**/
  //get dot product point   
  int16_t s = (int16_t)( ((double)(leg_progress2))/((double)(leg_length2))*1000 );
  VECT2_SMUL(wp_corre, wp_diff_prec, s);
  VECT2_SDIV(wp_corre, wp_corre, 1000);
  VECT2_SUM(wp_corre, wp_corre, *wp_star);
  
  //caculate the corrective wp
  VECT2_DIFF(wp_corre, wp_corre, pos_current);
  uint32_t corner_len2_int = wp_corre.x*wp_corre.x+wp_corre.y*wp_corre.y;
  float corner_len2 = FLOAT_OF_BFP(corner_len2_int, 16);
  
  if(corner_len2 <0.16)  regulate_ratio = 2;
  else if(corner_len2 <0.26)  regulate_ratio = 3;
  else if(corner_len2 <0.82)  regulate_ratio = 5;
  else if(corner_len2 <2.26)  regulate_ratio = 8;
  else  regulate_ratio = 14;
  
  regulate_ratio = (last_regulate_ratio*3 + regulate_ratio*2)/5;
  last_regulate_ratio = regulate_ratio;
#endif  

  int32_t progress = 8;//Max( ( (CARROT_DIST/regulate_ratio) >>INT32_POS_FRAC), 0 );     //TODOM: int32_t progress = Max((CARROT_DIST >> INT32_POS_FRAC), 0);
  nav_leg_progress += progress;                 //carrot length   
  int32_t prog_2 = nav_leg_length;
  Bound(nav_leg_progress, 0, prog_2);
  struct Int32Vect2 progress_pos;
  VECT2_SMUL(progress_pos, wp_diff_prec, ((float)nav_leg_progress) / nav_leg_length);  //product the ratio
  VECT2_SUM(navigation_target, *wp_star, progress_pos);

  nav_segment_start = *wp_star;
  nav_segment_end = *wp_end;
  horizontal_mode = HORIZONTAL_MODE_ROUTE;

  dist2_to_wp = get_dist2_to_point(wp_end);
  if( dist2_to_wp < (ROUTE_BRAKE_DISTANCE*ROUTE_BRAKE_DISTANCE) )  //default 10m brake distance
  {
  	route_brake_flag = TRUE;
  }
  else
  {
  	route_brake_flag = FALSE;
  }
	#if 0//PERIODIC_TELEMETRY
	RunOnceEvery(32, {
     xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
     DOWNLINK_SEND_ROUTE_LEN(DefaultChannel, DefaultDevice, &nav_leg_progress, &progress_pos.x, &progress_pos.y);}
	 );
	#endif
}
#else
void nav_route(struct EnuCoor_i *wp_star, struct EnuCoor_i *wp_end)
{
	struct FloatVect2 start_wp, end_wp;
	start_wp.x = POS_FLOAT_OF_BFP(wp_star->y);
	start_wp.y = POS_FLOAT_OF_BFP(wp_star->x);

	end_wp.x = POS_FLOAT_OF_BFP(wp_end->y);
	end_wp.y = POS_FLOAT_OF_BFP(wp_end->x);

	guidance_h_trajectory_tracking_set_segment(start_wp, end_wp);

  nav_segment_start = *wp_star;
  nav_segment_end = *wp_end;
}
#endif

//return arrive at wp/true or false,after using approaching_time later
bool_t nav_approaching_from(struct EnuCoor_i *wp, struct EnuCoor_i *from, int16_t approaching_time)  
{
  bool_t arrive_flag = FALSE; 
  int32_t dist_to_point;
  struct Int32Vect2 diff;
  struct EnuCoor_i *pos = stateGetPositionEnu_i();

  /* if an approaching_time is given, estimate diff after approching_time secs */
  if (approaching_time != 0) 
  {
    struct Int32Vect2 estimated_pos;
    struct Int32Vect2 estimated_progress;
    struct EnuCoor_i *speed = stateGetSpeedEnu_i();
    VECT2_SMUL(estimated_progress, *speed, approaching_time);
    INT32_VECT2_RSHIFT(estimated_progress, estimated_progress, (INT32_SPEED_FRAC - INT32_POS_FRAC));
    VECT2_SUM(estimated_pos, *pos, estimated_progress);
    VECT2_DIFF(diff, *wp, estimated_pos);
  }
  /* else use current position */
  else 
  {
    VECT2_DIFF(diff, *wp, *pos);
  }
  /* compute distance of estimated/current pos to target wp
   * distance with half metric precision (6.25 cm)
   */
  INT32_VECT2_RSHIFT(diff, diff, INT32_POS_FRAC / 2);
  dist_to_point = int32_vect2_norm(&diff);

  /* return TRUE if we have arrived */
  if (dist_to_point < BFP_OF_REAL(ARRIVED_AT_WAYPOINT, INT32_POS_FRAC / 2)) {
    arrive_flag = TRUE;
  }

  /* if coming from a valid waypoint */
  if (from != NULL) {
    /* return TRUE if normal line at the end of the segment is crossed */
    struct Int32Vect2 from_diff;
    VECT2_DIFF(from_diff, *wp, *from);
    INT32_VECT2_RSHIFT(from_diff, from_diff, INT32_POS_FRAC / 2);
	nav_leg_progress2_end = diff.x * from_diff.x + diff.y * from_diff.y;  // <0:sign AC out of end waypoint
    if( nav_leg_progress2_end < 0)
    {
		arrive_flag = TRUE;
    }
  }

  return arrive_flag;
}

//return arrive at wp/true or false,after using approaching_time later
bool_t nav_approaching_target(struct EnuCoor_i *wp, struct EnuCoor_i *from, float arrived_distance)  
{
  bool_t arive_flag = FALSE; 
  int32_t dist_to_point;
  struct Int32Vect2 diff;
  struct EnuCoor_i *pos = stateGetPositionEnu_i();

  if (arrived_distance == 0.0) 
  {
    arrived_distance = 0.15f;  //avoid zero distance
  }

  VECT2_DIFF(diff, *wp, *pos);

  INT32_VECT2_RSHIFT(diff, diff, INT32_POS_FRAC / 2);
  dist_to_point = int32_vect2_norm(&diff);

  /* return TRUE if we have arrived */
  if (dist_to_point < BFP_OF_REAL(arrived_distance, INT32_POS_FRAC / 2)) 
  {
    arive_flag = TRUE;
  }

  /* if coming from a valid waypoint */
  if (from != NULL) 
  {
    /* return TRUE if normal line at the end of the segment is crossed */
    struct Int32Vect2 from_diff;
    VECT2_DIFF(from_diff, *wp, *from);
    INT32_VECT2_RSHIFT(from_diff, from_diff, INT32_POS_FRAC / 2);
	 nav_leg_progress2_end = diff.x * from_diff.x + diff.y * from_diff.y;  // <0:sign AC out of end waypoint
    if( nav_leg_progress2_end < 0)
    {
		arive_flag = TRUE;
    }
  }

  return arive_flag;
}

float distance_to_target(void)
{
	struct EnuCoor_i pos_now_enu, diff_wp;
	pos_now_enu = *stateGetPositionEnu_i();
	VECT3_DIFF(diff_wp, navigation_target, pos_now_enu);
	uint32_t distance = int32_sqrt(VECT2_NORM2(diff_wp));
	return POS_FLOAT_OF_BFP(distance);	
}

/** Check the time spent in a radius of 'ARRIVED_AT_WAYPOINT' around a wp  */
bool_t nav_check_wp_time(struct EnuCoor_i *wp, uint16_t stay_time)
{
  uint16_t time_at_wp;
  uint32_t dist_to_point;
  static uint16_t wp_entry_time = 0;
  static bool_t wp_reached = FALSE;
  static struct EnuCoor_i wp_last = { 0, 0, 0 };
  struct Int32Vect2 diff;

  if ((wp_last.x != wp->x) || (wp_last.y != wp->y)) {
    wp_reached = FALSE;
    wp_last = *wp;
  }
  VECT2_DIFF(diff, *wp, *stateGetPositionEnu_i());
  INT32_VECT2_RSHIFT(diff, diff, INT32_POS_FRAC / 2);
  dist_to_point = int32_vect2_norm(&diff);
  if (dist_to_point < BFP_OF_REAL(ARRIVED_AT_WAYPOINT, INT32_POS_FRAC / 2)) {
    if (!wp_reached) {
      wp_reached = TRUE;
      wp_entry_time = autopilot_flight_time;
      time_at_wp = 0;
    } else {
      time_at_wp = autopilot_flight_time - wp_entry_time;
    }
  } else {
    time_at_wp = 0;
    wp_reached = FALSE;
  }
  if (time_at_wp > stay_time) {
    INT_VECT3_ZERO(wp_last);
    return TRUE;
  }
  return FALSE;
}

static inline void nav_set_altitude(void)
{
  static int32_t last_nav_alt = 0;
  if (abs(nav_altitude - last_nav_alt) > (POS_BFP_OF_REAL(0.10)))   //TODOM: 0.2
  {
    nav_flight_altitude = nav_altitude;
    last_nav_alt = nav_altitude;
  }
}


/** Reset the geographic reference to the current GPS fix */
unit_t nav_reset_reference(void)
{
  ins_reset_local_origin();
  /* update local ENU coordinates of global waypoints */
  waypoints_localize_all();
  return 0;
}

unit_t nav_reset_alt(void)
{
  ins_reset_altitude_ref();
  waypoints_localize_all();
  return 0;
}

void nav_init_stage(void)
{
  VECT3_COPY(nav_last_point, *stateGetPositionEnu_i());
  stage_time = 0;
  nav_circle_radians = 0;
  horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
  //add for survey module use for circle  by whp
  nav_circle_radians_no_rewind = 0;
  nav_in_circle=FALSE;
  nav_in_segment=FALSE;
  nav_shift=0.0;
}

#include <stdio.h>
void nav_periodic_task(void)
{
  RunOnceEvery(NAV_FREQ, { stage_time++;  block_time++; });

  nav_survey_active = FALSE;

  dist2_to_wp = 0;

  /* from nav_flight.c or flight_plan.h */
 #ifndef USE_MISSION
   auto_nav(); 
 #else
   nav_flight();
 #endif


  /* run carrot loop */
  nav_run();
}

void auto_nav_fp(void)
{
	auto_nav(); 
}

void nav_move_waypoint_lla(uint8_t wp_id, struct LlaCoor_i* new_lla_pos) {
   if (stateIsLocalCoordinateValid()) {
     struct EnuCoor_i enu;
     enu_of_lla_point_i(&enu, &state.ned_origin_i, new_lla_pos);
     // convert ENU pos from cm to BFP with INT32_POS_FRAC
     enu.x = POS_BFP_OF_REAL(enu.x)/100;
     enu.y = POS_BFP_OF_REAL(enu.y)/100;
     enu.z = POS_BFP_OF_REAL(enu.z)/100;
     nav_move_waypoint(wp_id, &enu);
   }
 }
 
void nav_move_waypoint(uint8_t wp_id, struct EnuCoor_i * new_pos) {
 #if PERIODIC_TELEMETRY
   if (wp_id < nb_waypoint) {
     VECT3_COPY(waypoints[wp_id].enu_i,(*new_pos));
	 xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
     DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id, &(new_pos->x),
                                &(new_pos->y), &(new_pos->z));
   }
 #endif
 }
 
void navigation_update_wp_from_speed(uint8_t wp, struct Int16Vect3 speed_sp, int16_t heading_rate_sp)
{
  //  MY_ASSERT(wp < nb_waypoint); FIXME
  int32_t s_heading, c_heading;
  PPRZ_ITRIG_SIN(s_heading, nav_heading);
  PPRZ_ITRIG_COS(c_heading, nav_heading);
  // FIXME : scale POS to SPEED
  struct Int32Vect3 delta_pos;
  VECT3_SDIV(delta_pos, speed_sp, NAV_FREQ); /* fixme :make sure the division is really a >> */
  INT32_VECT3_RSHIFT(delta_pos, delta_pos, (INT32_SPEED_FRAC - INT32_POS_FRAC));
  waypoints[wp].enu_i.x += (s_heading * delta_pos.x + c_heading * delta_pos.y) >> INT32_TRIG_FRAC;
  waypoints[wp].enu_i.y += (c_heading * delta_pos.x - s_heading * delta_pos.y) >> INT32_TRIG_FRAC;
  waypoints[wp].enu_i.z += delta_pos.z;
  int32_t delta_heading = heading_rate_sp / NAV_FREQ;
  delta_heading = delta_heading >> (INT32_SPEED_FRAC - INT32_POS_FRAC);
  nav_heading += delta_heading;

  INT32_COURSE_NORMALIZE(nav_heading);
  #if PERIODIC_TELEMETRY
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  RunOnceEvery(10, DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp,
                                              &(waypoints[wp].enu_i.x),
                                              &(waypoints[wp].enu_i.y),
                                              &(waypoints[wp].enu_i.z)));
  #endif
}

bool_t nav_detect_ground(void)
{
  if (!autopilot_ground_detected) { return FALSE; }
  autopilot_ground_detected = FALSE;
  return TRUE;
}

bool_t nav_is_in_flight(void)
{
  return autopilot_in_flight;
}

/** Home mode navigation */
void nav_home(void)
{
  horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
  VECT3_COPY(navigation_target, waypoints[WP_HOME].enu_i);

  vertical_mode = VERTICAL_MODE_ALT;
  nav_altitude = waypoints[WP_HOME].enu_i.z;
  nav_flight_altitude = nav_altitude;

  dist2_to_wp = dist2_to_home;

  /* run carrot loop */
  nav_run();
}

/** Returns squared horizontal distance to given point */
float get_dist2_to_point(struct EnuCoor_i *p)
{
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  struct FloatVect2 pos_diff;
  pos_diff.x = POS_FLOAT_OF_BFP(p->x) - pos->x;
  pos_diff.y = POS_FLOAT_OF_BFP(p->y) - pos->y;
  return pos_diff.x * pos_diff.x + pos_diff.y * pos_diff.y;
}

/** Returns squared horizontal distance to given waypoint */
float get_dist2_to_waypoint(uint8_t wp_id)
{
  return get_dist2_to_point(&waypoints[wp_id].enu_i);
}

/** Computes squared distance to the HOME waypoint potentially sets
 * #too_far_from_home
 */
void compute_dist2_to_home(void)
{
  dist2_to_home = get_dist2_to_waypoint(WP_HOME);
  too_far_from_home = dist2_to_home > max_dist2_from_home;
}

/** Set nav_heading in degrees. */
bool_t nav_set_heading_rad(float rad)
{
  nav_heading = ANGLE_BFP_OF_REAL(rad);
  INT32_COURSE_NORMALIZE(nav_heading);
  return FALSE;
}

/** Set nav_heading in degrees. */
bool_t nav_set_heading_deg(float deg)
{
  return nav_set_heading_rad(RadOfDeg(deg));
}

/** Set nav_heading along a given path. */
bool_t nav_set_heading_forward_line(struct EnuCoor_i *wp_from,struct EnuCoor_i *wp_to)
{
  struct Int32Vect2 pos_diff;
  VECT2_DIFF(pos_diff, *wp_to, *wp_from);
  // don't change heading if closer than 0.5m to target
  if( abs(pos_diff.x)>65 || abs(pos_diff.y)>65 )
  {
    nav_heading  = int32_atan2(pos_diff.x, pos_diff.y);
  }
  return FALSE;
}

/** Set nav_heading along a given path. */
bool_t nav_set_heading_parallel_line(struct EnuCoor_i *wp_from,struct EnuCoor_i *wp_to)
{
  struct Int32Vect2 pos_diff;
  VECT2_DIFF(pos_diff, *wp_to, *wp_from);
  // don't change heading if closer than 0.5m to target
  if( abs(pos_diff.x)>65 || abs(pos_diff.y)>65 )
  {
  	int32_t pre_heading = int32_atan2(pos_diff.x, pos_diff.y);
	int32_t sp_heading = nav_heading;
	INT32_ANGLE_NORMALIZE(pre_heading);
	INT32_ANGLE_NORMALIZE(nav_heading);
	int32_t delta_angle = pre_heading - nav_heading;
	INT32_ANGLE_NORMALIZE(delta_angle);
	delta_angle = abs(delta_angle);
	if( delta_angle>ANGLE_BFP_OF_REAL(0.2f) && delta_angle<ANGLE_BFP_OF_REAL(3.14f-0.2f) )  //12deg error allowed
	{
		nav_heading = pre_heading;
	}	
  }
  return FALSE;
}

/** Set heading to point towards x,y position in local coordinates */
bool_t nav_set_heading_towards(float x, float y)
{
  struct FloatVect2 target = {x, y};
  struct FloatVect2 pos_diff;
  VECT2_DIFF(pos_diff, target, *stateGetPositionEnu_f());
  // don't change heading if closer than 0.5m to target
  if (VECT2_NORM2(pos_diff) > 0.25) {
    float heading_f = atan2f(pos_diff.x, pos_diff.y);
    nav_heading = ANGLE_BFP_OF_REAL(heading_f);
  }
  // return false so it can be called from the flightplan
  // meaning it will continue to the next stage
  return FALSE;
}

/** Set heading in the direction of a waypoint */
bool_t nav_set_heading_towards_waypoint(uint8_t wp)
{
  return nav_set_heading_towards(WaypointX(wp), WaypointY(wp));
}

/** Set heading to the current yaw angle */
bool_t nav_set_heading_current(void)
{
  nav_heading = stateGetNedToBodyEulers_i()->psi;
  return FALSE;
}

/** check heading deviation, less than 3deg(0.06rad) return TRUE*/
bool_t nav_check_heading(void)
{
	int32_t psi = stateGetNedToBodyEulers_i()->psi;
	int32_t heading_sp = nav_heading;
	INT32_COURSE_NORMALIZE(psi);
	INT32_COURSE_NORMALIZE(heading_sp);
	int32_t diff_angle = abs(heading_sp - psi);
	if(diff_angle > INT32_ANGLE_PI)
	{
		diff_angle = INT32_ANGLE_2_PI - diff_angle;
	}
	if( diff_angle < ANGLE_BFP_OF_REAL(0.15) ) return TRUE;
	else return FALSE;
}

bool_t nav_check_height(void)
{
	if( abs(stateGetPositionNed_i()->z + nav_flight_altitude) < POS_BFP_OF_REAL(0.15) )
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

void record_current_waypoint(struct EnuCoor_i *wp)
{
	VECT3_COPY(*wp,*stateGetPositionEnu_i());
}

bool_t get_nav_route_mediacy(void)
{
	return ( (nav_leg_progress2_from > 0) && (nav_leg_progress2_end > 0) );
}

float smooth_brake_accel = 1.0;
float pause_brake_accel = 2.0;

void set_stop_brake(uint8_t brake_grade)
{
	stop_brake_flag = TRUE;
	switch ( brake_grade )
	{
		case SMOOTH_BRAKE:
			//accel_stop_brake = SMOOTH_BRAKE_ACCEL;
			accel_stop_brake = smooth_brake_accel;
			break;
		case URGENT_BRAKE:
			//accel_stop_brake = URGENT_BRAKE_ACCEL;
			accel_stop_brake = pause_brake_accel;
			break;
		default:
			accel_stop_brake = SMOOTH_BRAKE_ACCEL;
			break;			
	}
}

void release_stop_brake(void)
{
	stop_brake_flag = FALSE;
}
