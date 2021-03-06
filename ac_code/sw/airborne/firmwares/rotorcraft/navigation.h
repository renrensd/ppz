/*
* Copyright (C) 2008-2011  The Paparazzi Team
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
 * @file firmwares/rotorcraft/navigation.h
 *
 * Rotorcraft navigation functions.
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "std.h"
#include "math/pprz_geodetic_int.h"

//#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/navigation/waypoints.h"
#include "subsystems/navigation/common_flight_plan.h"

#define NAV_FREQ 32	//TODOM: 16

#define FLIGHT_LIMIT_HEIGHT 2560  //default 10m height
#define BARO_LIMIT_HEIGHT  563    //2.2m 2**8
#define ERROR_LIMIT_HEIGHT  256   //1m

#define ROUTE_BRAKE_DISTANCE  10  //unit:m
#define SMOOTH_BRAKE 1
#define PAUSE_BRAKE 2
#define URGENT_BRAKE 3
#define SMOOTH_BRAKE_ACCEL 1.0f
#define PAUSE_BRAKE_ACCEL  1.6f
#define URGENT_BRAKE_ACCEL 2.5

typedef enum Flight_Direct
{
	NAV_FORWARD = 0,
	NAV_BACKWARD =1
} FLIGHT_DIRECT;

extern FLIGHT_DIRECT flight_direct;

extern struct EnuCoor_i navigation_target;
extern struct EnuCoor_i navigation_carrot;

extern void nav_init(void);
extern void nav_run(void);

extern uint8_t last_wp __attribute__((unused));

extern uint8_t horizontal_mode;
extern struct EnuCoor_i nav_segment_start, nav_segment_end;
extern struct EnuCoor_i nav_circle_center;
extern int32_t nav_circle_radius, nav_circle_qdr, nav_circle_radians;
#define HORIZONTAL_MODE_WAYPOINT  0
#define HORIZONTAL_MODE_ROUTE     1
#define HORIZONTAL_MODE_CIRCLE    2
#define HORIZONTAL_MODE_ATTITUDE  3
#define HORIZONTAL_MODE_RC     4
extern int32_t nav_roll, nav_pitch;     ///< with #INT32_ANGLE_FRAC
extern int32_t nav_heading; ///< with #INT32_ANGLE_FRAC
extern float nav_radius;
extern float nav_climb_vspeed, nav_descend_vspeed;

extern int32_t nav_leg_progress;
extern int32_t nav_leg_length;

extern bool_t nav_survey_active;

extern uint8_t vertical_mode;
extern uint32_t nav_throttle;  ///< direct throttle from 0:MAX_PPRZ, used in VERTICAL_MODE_MANUAL
extern int32_t nav_climb, nav_altitude, nav_flight_altitude;
extern int32_t flight_limit_height;
extern float flight_altitude;
#define VERTICAL_MODE_MANUAL      0
#define VERTICAL_MODE_CLIMB       1
#define VERTICAL_MODE_ALT         2

extern float dist2_to_home;      ///< squared distance to home waypoint
extern bool_t too_far_from_home;
extern float failsafe_mode_dist2; ///< maximum squared distance to home wp before going to failsafe mode
extern float dist2_to_wp;       ///< squared distance to next waypoint

extern bool_t route_brake_flag;
extern bool_t stop_brake_flag;
extern float  accel_stop_brake;

extern float smooth_brake_accel;
extern float pause_brake_accel;
extern float urgent_brake_accel;

extern uint8_t set_carrot_angle;

extern float nav_circle_radians_no_rewind; /* Cumulated, add by whp */

extern float get_dist2_p2p(struct EnuCoor_i s, struct EnuCoor_i e);
extern float get_dist2_to_waypoint(uint8_t wp_id);
extern float get_dist2_to_point(struct EnuCoor_i *p);
extern void compute_dist2_to_home(void);
extern void nav_home(void);

unit_t nav_reset_reference(void) __attribute__((unused));
unit_t nav_reset_alt(void) __attribute__((unused));
void nav_periodic_task(void);
bool_t nav_detect_ground(void);
bool_t nav_is_in_flight(void);
extern uint8_t CARROT_DIST_MUL;

extern bool_t exception_flag[10];
extern void set_exception_flag(uint8_t flag_num);

extern bool_t nav_set_heading_rad(float rad);
extern bool_t nav_set_heading_deg(float deg);
extern bool_t nav_set_heading_towards(float x, float y);
extern bool_t nav_set_heading_towards_waypoint(uint8_t wp);
extern bool_t nav_set_heading_current(void);
extern bool_t nav_set_heading_forward_line(struct EnuCoor_i *wp_from,struct EnuCoor_i *wp_to);
extern bool_t nav_set_heading_parallel_line(struct EnuCoor_i *wp_from,struct EnuCoor_i *wp_to);
extern bool_t nav_check_heading(void);
extern bool_t nav_check_height(void);
extern void record_current_waypoint(struct EnuCoor_i *wp);
extern bool_t nav_spray_convert(struct EnuCoor_i wp_center, int32_t radius, int32_t sp_heading);
extern bool_t get_nav_route_mediacy(void);
extern void set_stop_brake(uint8_t brake_grade);
extern void release_stop_brake(void);
extern void nav_set_flight_speed(float speed);

extern void auto_nav_fp(void);


/** default approaching_time for a wp */
#ifndef CARROT
#define CARROT 0
#endif

#define NavKillMode()  {autopilot_set_mode(AP_MODE_KILL);Flag_AC_Flight_Ready=FALSE;}
#define NavKillThrottle() ({ if (autopilot_mode == AP_MODE_NAV) { autopilot_set_motors_on(FALSE); } FALSE; })
#define NavResurrect() ({ if (autopilot_mode == AP_MODE_NAV) { autopilot_set_motors_on(TRUE); } FALSE; })


#define NavSetGroundReferenceHere() ({ nav_reset_reference(); FALSE; })
#define NavSetAltitudeReferenceHere() ({ nav_reset_alt(); FALSE; })

#define NavSetWaypointHere(_wp) ({ waypoint_set_here_2d(_wp); FALSE; })
#define NavCopyWaypoint(_wp1, _wp2) ({ waypoint_copy(_wp1, _wp2); FALSE; })
#define NavCopyWaypointPositionOnly(_wp1, _wp2) ({ waypoint_position_copy(_wp1, _wp2); FALSE; })

#define HOVER_STEADY_SPEED_H  0.20
#define HOVER_STEADY_ACCEL_H  0.3
#define HOVER_STEADY_ATT  5.0

extern bool_t NavGetHoverSteady(void);

/** Normalize a degree angle between 0 and 359 */
#define NormCourse(x) { \
    while (x < 0) x += 360; \
    while (x >= 360) x -= 360; \
  }

/*********** Navigation to  waypoint,once arrived it will stay there********************/
#define NavGotoWaypoint(_wp) { \
    horizontal_mode = HORIZONTAL_MODE_WAYPOINT; \
    VECT3_COPY(navigation_target, waypoints[_wp].enu_i); \
    dist2_to_wp = get_dist2_to_waypoint(_wp); \
  }

/*********** Navigation to  waypoint using waypoint*************************************/
#define NavGotoWaypoint_wp(_wp) { \
    horizontal_mode = HORIZONTAL_MODE_WAYPOINT; \
    VECT3_COPY(navigation_target, _wp); \
    dist2_to_wp = get_dist2_to_point(&_wp); \
  }

/*********** Navigation on a circle **************************************/
extern void nav_circle(struct EnuCoor_i *wp_center, int32_t radius);
#define NavCircleWaypoint(_center, _radius) { \
    horizontal_mode = HORIZONTAL_MODE_CIRCLE; \
    nav_circle(&waypoints[_center].enu_i, POS_BFP_OF_REAL(_radius)); \
  }
/***************from fixedwing start */
/** Normalize a degree angle between 0 and 359 */
#define NavCircleCountNoRewind() (nav_circle_radians_no_rewind / (2*M_PI))

/***************from fixedwing end */

#define NavCircleCount() ((float)abs(nav_circle_radians) / INT32_ANGLE_2_PI)
#define NavCircleQdr() ({ int32_t qdr = INT32_DEG_OF_RAD(INT32_ANGLE_PI_2 - nav_circle_qdr) >> INT32_ANGLE_FRAC; NormCourse(qdr); qdr; })

/** True if x (in degrees) is close to the current QDR (less than 10 degrees)*/
//#define NavQdrCloseTo(x) {} //TODOM:
//#define NavCourseCloseTo(x) {}
#define CloseDegAngles(_c1, _c2) ({ float _diff = _c1 - _c2; NormCourse(_diff); 350 < _diff || _diff < 10; })
#define NavQdrCloseTo(x) CloseDegAngles(x, NavCircleQdr())
#define NavCourseCloseTo(x) CloseDegAngles(x, DegOfRad(*stateGetHorizontalSpeedDir_f()))

/*********** Navigation along a line *************************************/
extern void nav_route(struct EnuCoor_i *wp_star, struct EnuCoor_i *wp_end);
#define NavSegment(_start, _end) { \
    horizontal_mode = HORIZONTAL_MODE_ROUTE; \
    nav_route(&waypoints[_start].enu_i, &waypoints[_end].enu_i); \
  }

/** Nav glide routine */
#define NavGlide(_last_wp, _wp) { \
    int32_t start_alt = waypoints[_last_wp].enu_i.z; \
    int32_t diff_alt = waypoints[_wp].enu_i.z - start_alt; \
    int32_t alt = start_alt + ((diff_alt * nav_leg_progress) / nav_leg_length); \
    NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(alt),0); \
  }

/** Proximity tests on approaching a wp */
bool_t nav_approaching_from(struct EnuCoor_i *wp, struct EnuCoor_i *from, int16_t approaching_time);
#define NavApproaching(wp, time) nav_approaching_from(&waypoints[wp].enu_i, NULL, time)
#define NavApproachingFrom(wp, from, time) nav_approaching_from(&waypoints[wp].enu_i, &waypoints[from].enu_i, time)
extern bool_t nav_approaching_target(struct EnuCoor_i *wp, struct EnuCoor_i *from, float arrived_distance);
extern float distance_to_target(void);
extern void set_current_pos_to_target(void);
/** Check the time spent in a radius of 'ARRIVED_AT_WAYPOINT' around a wp  */
bool_t nav_check_wp_time(struct EnuCoor_i *wp, uint16_t stay_time);
#define NavCheckWaypointTime(wp, time) nav_check_wp_time(&waypoints[wp].enu_i, time)

/** Set the climb control to auto-throttle with the specified pitch
    pre-command */
#define NavVerticalAutoThrottleMode(_pitch) { \
    nav_pitch = ANGLE_BFP_OF_REAL(_pitch); \
  }

/** Set the climb control to auto-pitch with the specified throttle
    pre-command */
#define NavVerticalAutoPitchMode(_throttle) {}

/** Set the vertical mode to altitude control with the specified altitude
 setpoint and climb pre-command. */
#define NavVerticalAltitudeMode(_alt, _pre_climb) { \
    vertical_mode = VERTICAL_MODE_ALT; \
    nav_altitude = POS_BFP_OF_REAL(_alt); \
  }

/** Set the vertical mode to climb control with the specified climb setpoint */
#define NavVerticalClimbMode(_climb) { \
    vertical_mode = VERTICAL_MODE_CLIMB; \
    nav_climb = SPEED_BFP_OF_REAL(_climb); \
  }

/** Set the vertical mode to fixed throttle with the specified setpoint */
#define NavVerticalThrottleMode(_throttle) { \
    vertical_mode = VERTICAL_MODE_MANUAL;      \
    nav_throttle = _throttle;                  \
  }

/** Set the heading of the rotorcraft, nothing else */
#define NavHeading nav_set_heading_rad

#define NavAttitude(_roll) { \
    horizontal_mode = HORIZONTAL_MODE_ATTITUDE; \
    nav_roll = ANGLE_BFP_OF_REAL(_roll); \
  }

#define NavStartDetectGround() ({ autopilot_detect_ground_once = TRUE; FALSE; })
#define NavDetectGround() nav_detect_ground()

#define nav_IncreaseShift(x) {}

#define nav_SetNavRadius(x) {}


#define navigation_SetFlightAltitude(x) { \
    flight_altitude = x; \
    nav_flight_altitude = POS_BFP_OF_REAL(flight_altitude - state.ned_origin_f.hmsl); \
  }


#define GetPosX() (stateGetPositionEnu_f()->x)
#define GetPosY() (stateGetPositionEnu_f()->y)
#define GetPosAlt() (stateGetPositionEnu_f()->z+state.ned_origin_f.hmsl)
#define GetPosZ_u() (stateGetPositionEnu_f()->z)
#define GetAltRef() (state.ned_origin_f.hmsl)

extern void nav_move_waypoint(uint8_t wp_id, struct EnuCoor_i * new_pos);
extern void nav_move_waypoint_lla(uint8_t wp_id, struct LlaCoor_i* new_lla_pos);
extern void navigation_update_wp_from_speed(uint8_t wp, struct Int16Vect3 speed_sp, int16_t heading_rate_sp);

#endif /* NAVIGATION_H */
