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

/** @file firmwares/rotorcraft/guidance/guidance_h.c
 *  Horizontal guidance for rotorcrafts.
 *
 */

#include "generated/airframe.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_flip.h"
#include "firmwares/rotorcraft/guidance/guidance_indi.h"
#include "firmwares/rotorcraft/guidance/guidance_module.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/radio_control.h"

#include "firmwares/rotorcraft/stabilization/stabilization_none.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

/* for guidance_v_thrust_coeff */
#include "firmwares/rotorcraft/guidance/guidance_v.h"

#include "state.h"

#if USE_FLOW
#include "subsystems/ins/flow_hf_float.h"  //add for inital flow filter when entering hover mode
#endif

#ifndef GUIDANCE_H_AGAIN
#define GUIDANCE_H_AGAIN 0
#endif

#ifndef GUIDANCE_H_VGAIN
#define GUIDANCE_H_VGAIN 0
#endif

/* error if some gains are negative */
#if (GUIDANCE_H_PGAIN < 0) ||                   \
  (GUIDANCE_H_DGAIN < 0)   ||                   \
  (GUIDANCE_H_IGAIN < 0)   ||                   \
  (GUIDANCE_H_AGAIN < 0)   ||                   \
  (GUIDANCE_H_VGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif

#ifndef GUIDANCE_H_MAX_BANK
#define GUIDANCE_H_MAX_BANK RadOfDeg(20)
#endif

PRINT_CONFIG_VAR(GUIDANCE_H_USE_REF)
PRINT_CONFIG_VAR(GUIDANCE_H_USE_SPEED_REF)

#ifndef GUIDANCE_H_APPROX_FORCE_BY_THRUST
#define GUIDANCE_H_APPROX_FORCE_BY_THRUST TRUE
#endif

#ifndef GUIDANCE_INDI
#define GUIDANCE_INDI FALSE
#endif

struct HorizontalGuidance guidance_h;

int32_t transition_percentage;
int32_t transition_theta_offset;

//uint8_t guidance_h_mode;
//bool_t guidance_h_use_ref;
//bool_t guidance_h_approx_force_by_thrust;

//struct Int32Vect2 guidance_h_pos_sp;
//struct Int32Vect2 guidance_h_pos_ref;
//struct Int32Vect2 guidance_h_speed_ref;
//struct Int32Vect2 guidance_h_accel_ref;
//#if GUIDANCE_H_USE_SPEED_REF
//struct Int32Vect2 guidance_h_speed_sp;
//#endif

/*
 * internal variables
 */
struct Int32Vect2 guidance_h_pos_err;
struct Int32Vect2 guidance_h_speed_err;
struct Int32Vect2 guidance_h_trim_att_integrator;

struct position_float guidance_hf_pos_err;  //waihuan weizhi  wucha 
struct position_float guidance_hf_speed_err;//nei huan sudu  wu cha 
struct position_float guidance_h_ref_pos_pre;

struct position_float sum_err; //wai huan jifen 
struct position_float sum_in_err;// nie huan ji fen 

struct position_float i_term; //wai huan jifen xiang 
struct position_float ii_term;//nei huan ji fen xiang 

struct position_float d_term;//wai huan weifen 
struct position_float in_d_term;//nei huan wei fen 

struct position_float desire_speed;//nei huan su du gei ding 
struct position_float desire_speed_pre;//nei huan qian yi shike su du gei ding 

#define I_SUM 2.5  //wai huan ji fen xian fu 
#define I_SUM_I 0.26 // nei huan ji fen xian fu 
#define dt_512 512
bool_t onto_position = TRUE;

struct Int32Vect2 h_speed_fl;
/** horizontal guidance command.
 * In north/east with #INT32_ANGLE_FRAC
 * @todo convert to real force command
 */
struct Int32Vect2  guidance_h_cmd_earth;
//struct Int32Eulers guidance_h_rc_sp;
//int32_t guidance_h_heading_sp;
int32_t rc_turn_rate;    //using in HORIZONTAL_MODE_RC,cmd of psi rate

//int32_t guidance_h_pgain;
//int32_t guidance_h_dgain;
//int32_t guidance_h_igain;
//int32_t guidance_h_again;
//int32_t guidance_h_vgain;

//add in bouble control loop
int32_t sum_in_x = 0;
int32_t sum_in_y = 0;
//add
static void guidance_h_update_reference(void);
#if !GUIDANCE_INDI
static void guidance_h_traj_run(bool_t in_flight);
#endif
static void guidance_h_hover_enter(void);
static void guidance_h_nav_enter(void);
static inline void transition_run(void);
static void read_rc_setpoint_speed_i(struct Int32Vect2 *speed_sp, bool_t in_flight);

float fsg_h(float x,float d);
int32_t fhan_control(float pos,float vel,  float repul, float h1);
float fhan_h(float signal,float x1,float x2);
void Tracking_differntiator_hx(float signal);
void Tracking_differntiator_hy(float signal);

/*
******************************************************
-------------------new functions defined here-----------
\/ \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/
*/
#define Sign(_x) ((_x) > 0 ? 1 : (-1))

/*
int inner_PID_x(int error,int sumerror, int d_error, int kp, int ki, int kd){
	int manipulate;
	manipulate = (error*kp) + (d_error*kd) + (sumerror*ki);
	return manipulate;
}

int inner_PID_y(int error, int sumerror, int d_error, int kp, int ki, int kd){
	int manipulate;
	manipulate = (error*kp) + (d_error*kd) + (sumerror*ki);
	return manipulate;
}
*/


int32_t c_x=1000000;
int32_t c_y=1000000;
/*
void test_signal(void)
{
	static uint16_t time=0;
	time++;
	if(time>5000)
	{
		time = 0;
		c_x = -1000000*Sign(c_x);
		c_y = -1000000*Sign(c_y);
	}
}
*/


/*here is define the variables that do the differentiation of my control signal in inner loop*/
//*****************************************************//
float xh1 = 0.0;
float xh2 = 0.0;
float yh1 = 0.0;
float yh2 = 0.0;
float hh = 0.002*512*1024;
float hh0 = 0.2*512*1024;
float r_h = 300.0;
//*****************************************************//
//                define done here                                                    //

/*here is define the functions that do the differentiation of my control signal in inner loop*/
//*****************************************************//
float fsg_h(float x,float d)
{
	float f;
	f = ( Sign(x+d)-Sign(x-d) )/2;
	return f;
}

int32_t fhan_control(float pos, float vel, float repul, float h1)
{
	float d,a0,y,a1,a2,sy,a,sa,fout;
	d = repul * h1 * h1;
	a0 = h1 * vel;
	y = pos + a0;
	a1 = sqrt( (float)(d * (d + 8*fabs(y))) );
	a2 = a0 + Sign(y)*(a1-d)/2;
	sy = (Sign(y+d) - Sign(y-d))/2;
	a = (a0+y-a2)*sy + a2;
	sa = (Sign(a+d) - Sign(a-d))/2;
	fout = -repul * (a/d - Sign(a)) * sa - repul*Sign(a);
	
	return (int32_t)fout;
}


float fhan_h(float signal,float x1,float x2)
{
	float d,a0,y,a1,a2,a,out;
	d = r_h * hh0*hh0;
	a0 = hh0 * x2;
	y = (x1-signal) + a0;
	a1 = sqrt( d*(d + 8*fabs(y)) );
	a2 = a0 + Sign(y)*(a1-d)/2;
	a = (a0+y)*fsg_h(y,d) + a2*(1-fsg_h(y,d));
	out = -r_h*(a/d)*fsg_h(a,d) - r_h*Sign(a)*(1-fsg_h(a,d));
	
	return out;
}

void Tracking_differntiator_hx(float signal)
{
	float fh;
	fh = fhan_h(signal,xh1,xh2);
	
	xh2 = xh2 + hh*fh;
	xh1 = xh1 + hh*xh2;
}

void Tracking_differntiator_hy(float signal)
{
	float fh;
	fh = fhan_h(signal,yh1,yh2);
	
	yh2 = yh2 + hh*fh;
	yh1 = yh1 + hh*yh2;
}

//*****************************************************//
//                define done here                                                    //


#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_gh(struct transport_tx *trans, struct link_device *dev)
{
  struct NedCoor_i *pos = stateGetPositionNed_i();
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  pprz_msg_send_GUIDANCE_H_INT(trans, dev, AC_ID,
                               &guidance_h.sp.pos.x, &guidance_h.sp.pos.y,
                               &guidance_h.ref.pos.x, &guidance_h.ref.pos.y,
                               &(pos->x), &(pos->y));
}


static void send_hover_loop(struct transport_tx *trans, struct link_device *dev)
{
  struct NedCoor_i *pos = stateGetPositionNed_i();
  struct NedCoor_i *speed = stateGetSpeedNed_i();
  struct NedCoor_i *accel = stateGetAccelNed_i();
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  pprz_msg_send_HOVER_LOOP(trans, dev, AC_ID,
                           &guidance_h.sp.pos.x,
                           &guidance_h.sp.pos.y,
                           &(pos->x), &(pos->y),
                           &(speed->x), &(speed->y),
                           &(accel->x), &(accel->y),
                           &guidance_h_pos_err.x,
                           &guidance_h_pos_err.y,
                           &guidance_hf_speed_err.x,
                           &guidance_hf_speed_err.y,
                           //&guidance_h_speed_err.x,
                           //&guidance_h_speed_err.y,
                           //&guidance_h_trim_att_integrator.x,
                           //&guidance_h_trim_att_integrator.y,
                           &i_term.x,
                           &i_term.y,
                           &guidance_h_cmd_earth.x,
                           &guidance_h_cmd_earth.y,
                           &guidance_h.sp.heading,
                           &desire_speed.x,
                           &desire_speed.y,
                           &ii_term.x,
                           &ii_term.y);
}

static void send_href(struct transport_tx *trans, struct link_device *dev)
{
	int32_t temp_c_x = c_x;
	int32_t temp_xh1 = (int32_t)xh1;
	float temp_xh2 = xh2;
	int32_t temp_c_y = c_y;
	int32_t temp_yh1 = (int32_t)yh1;
	float temp_yh2 = yh2;
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  pprz_msg_send_GUIDANCE_H_REF_INT(trans, dev, AC_ID,
                                   &guidance_h.sp.pos.x, &guidance_h.ref.pos.x,
                                   &guidance_h.sp.speed.x, &guidance_h.ref.speed.x,
                                   &guidance_h.ref.accel.x,
                                   &guidance_h.sp.pos.y, &guidance_h.ref.pos.y,
                                   &guidance_h.sp.speed.y, &guidance_h.ref.speed.y,
                                   &guidance_h.ref.accel.y,
                                   &temp_c_x,
                                   &temp_c_y,
                                   &temp_xh1,
                                   &temp_yh1,
                                   &temp_xh2,
                                   &temp_yh2);
}

static void send_tune_hover(struct transport_tx *trans, struct link_device *dev)
{ 
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  pprz_msg_send_ROTORCRAFT_TUNE_HOVER(trans, dev, AC_ID,
                                      &radio_control.values[RADIO_ROLL],
                                      &radio_control.values[RADIO_PITCH],
                                      &radio_control.values[RADIO_YAW],
                                      &stabilization_cmd[COMMAND_ROLL],
                                      &stabilization_cmd[COMMAND_PITCH],
                                      &stabilization_cmd[COMMAND_YAW],
                                      &stabilization_cmd[COMMAND_THRUST],
                                      &(stateGetNedToBodyEulers_i()->phi),
                                      &(stateGetNedToBodyEulers_i()->theta),
                                      &(stateGetNedToBodyEulers_i()->psi));
}

#endif

void guidance_h_init(void)
{

  guidance_h.mode = GUIDANCE_H_MODE_KILL;
  guidance_h.use_ref = GUIDANCE_H_USE_REF;
  guidance_h.approx_force_by_thrust = GUIDANCE_H_APPROX_FORCE_BY_THRUST;

  INT_VECT2_ZERO(guidance_h.sp.pos);
  INT_VECT2_ZERO(guidance_h_trim_att_integrator);
  sum_in_x=0;
  sum_in_y=0;
  INT_EULERS_ZERO(guidance_h.rc_sp);
  guidance_h.sp.heading = 0;
  guidance_h.gains.p = GUIDANCE_H_PGAIN;
  guidance_h.gains.i = GUIDANCE_H_IGAIN;
  guidance_h.gains.d = GUIDANCE_H_DGAIN;
  guidance_h.gains.a = GUIDANCE_H_AGAIN;
  guidance_h.gains.v = GUIDANCE_H_VGAIN;
  guidance_h.gains_f.p=GUIDANCE_H_FPGAIN;
  guidance_h.gains_f.i=GUIDANCE_H_FIGAIN;
  guidance_h.gains_f.d=GUIDANCE_H_FDGAIN;
  guidance_h.gains_f.in_p=GUIDANCE_H_FIPGAIN;
  guidance_h.gains_f.in_i=GUIDANCE_H_FIIGAIN;
  guidance_h.gains_f.in_d=GUIDANCE_H_FIDGAIN;
  hh0 = GUIDANCE_H_TD_H0;
  hh = GUIDANCE_H_TD_H;
  r_h = GUIDANCE_H_TD_R;
  
  transition_percentage = 0;
  transition_theta_offset = 0;
  rc_turn_rate = 0;

  gh_ref_init();

#if GUIDANCE_H_MODE_MODULE_SETTING == GUIDANCE_H_MODE_MODULE
  guidance_h_module_init();
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_H_INT, send_gh);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_HOVER_LOOP, send_hover_loop);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_H_REF_INT, send_href);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_TUNE_HOVER, send_tune_hover);
#endif

#if GUIDANCE_INDI
  guidance_indi_enter();
#endif
}


static inline void reset_guidance_reference_from_current_position(void)
{
  VECT2_COPY(guidance_h.ref.pos, *stateGetPositionNed_i());
  VECT2_COPY(guidance_h.ref.speed, *stateGetSpeedNed_i());
  INT_VECT2_ZERO(guidance_h.ref.accel);
  gh_set_ref(guidance_h.ref.pos, guidance_h.ref.speed, guidance_h.ref.accel);

  INT_VECT2_ZERO(guidance_h_trim_att_integrator);
  sum_in_x = 0;
  sum_in_y = 0;
}

void guidance_h_mode_changed(uint8_t new_mode)
{
  if (new_mode == guidance_h.mode) {
    return;
  }

  if (new_mode != GUIDANCE_H_MODE_FORWARD && new_mode != GUIDANCE_H_MODE_RATE) {
    transition_percentage = 0;
    transition_theta_offset = 0;
  }

  switch (new_mode) {
    case GUIDANCE_H_MODE_RC_DIRECT:
      stabilization_none_enter();
      break;

    case GUIDANCE_H_MODE_RATE:
      stabilization_rate_enter();
      break;

    case GUIDANCE_H_MODE_CARE_FREE:
      stabilization_attitude_reset_care_free_heading();
    case GUIDANCE_H_MODE_FORWARD:
    case GUIDANCE_H_MODE_ATTITUDE:
#if NO_ATTITUDE_RESET_ON_MODE_CHANGE
      /* reset attitude stabilization if previous mode was not using it */
      if (guidance_h.mode == GUIDANCE_H_MODE_KILL ||
          guidance_h.mode == GUIDANCE_H_MODE_RATE ||
          guidance_h.mode == GUIDANCE_H_MODE_RC_DIRECT)
#endif
        stabilization_attitude_enter();
      break;

    case GUIDANCE_H_MODE_HOVER:
#if GUIDANCE_INDI
      guidance_indi_enter();
#endif
      guidance_h_hover_enter();
#if NO_ATTITUDE_RESET_ON_MODE_CHANGE
      /* reset attitude stabilization if previous mode was not using it */
      if (guidance_h.mode == GUIDANCE_H_MODE_KILL ||
          guidance_h.mode == GUIDANCE_H_MODE_RATE ||
          guidance_h.mode == GUIDANCE_H_MODE_RC_DIRECT)
#endif
        stabilization_attitude_enter();
      break;

#if GUIDANCE_H_MODE_MODULE_SETTING == GUIDANCE_H_MODE_MODULE
    case GUIDANCE_H_MODE_MODULE:
      guidance_h_module_enter();
      break;
#endif

    case GUIDANCE_H_MODE_NAV:
      guidance_h_nav_enter();
#if NO_ATTITUDE_RESET_ON_MODE_CHANGE
      /* reset attitude stabilization if previous mode was not using it */
      if (guidance_h.mode == GUIDANCE_H_MODE_KILL ||
          guidance_h.mode == GUIDANCE_H_MODE_RATE ||
          guidance_h.mode == GUIDANCE_H_MODE_RC_DIRECT)
#endif
        stabilization_attitude_enter();
      break;

    case GUIDANCE_H_MODE_FLIP:
      guidance_flip_enter();
      break;

    default:
      break;
  }

  guidance_h.mode = new_mode;

}


void guidance_h_read_rc(bool_t  in_flight)
{

  switch (guidance_h.mode) {

    case GUIDANCE_H_MODE_RC_DIRECT:
      stabilization_none_read_rc();
      break;

    case GUIDANCE_H_MODE_RATE:
#if SWITCH_STICKS_FOR_RATE_CONTROL
      stabilization_rate_read_rc_switched_sticks();
#else
      stabilization_rate_read_rc();
#endif
      break;
    case GUIDANCE_H_MODE_CARE_FREE:
      stabilization_attitude_read_rc(in_flight, TRUE, FALSE);
      break;
    case GUIDANCE_H_MODE_FORWARD:
      stabilization_attitude_read_rc(in_flight, FALSE, TRUE);
      break;
    case GUIDANCE_H_MODE_ATTITUDE:
      stabilization_attitude_read_rc(in_flight, FALSE, FALSE);
      break;
    case GUIDANCE_H_MODE_HOVER:
      stabilization_attitude_read_rc_setpoint_eulers(&guidance_h.rc_sp, in_flight, FALSE, FALSE);
#if GUIDANCE_H_USE_SPEED_REF
      read_rc_setpoint_speed_i(&guidance_h.sp.speed, in_flight);
#endif
      break;

#if GUIDANCE_H_MODE_MODULE_SETTING == GUIDANCE_H_MODE_MODULE
    case GUIDANCE_H_MODE_MODULE:
      guidance_h_module_read_rc();
      break;
#endif

    case GUIDANCE_H_MODE_NAV:
      if (radio_control.status == RC_OK) {
        stabilization_attitude_read_rc_setpoint_eulers(&guidance_h.rc_sp, in_flight, FALSE, FALSE);
      } else {
        INT_EULERS_ZERO(guidance_h.rc_sp);
      }
      break;
    case GUIDANCE_H_MODE_FLIP:
      stabilization_attitude_read_rc(in_flight, FALSE, FALSE);
      break;
    default:
      break;
  }

}

void guidance_h_run(bool_t  in_flight)
{
  switch (guidance_h.mode) {

    case GUIDANCE_H_MODE_RC_DIRECT:
      stabilization_none_run(in_flight);
      break;

    case GUIDANCE_H_MODE_RATE:
      stabilization_rate_run(in_flight);
      break;

    case GUIDANCE_H_MODE_FORWARD:
      if (transition_percentage < (100 << INT32_PERCENTAGE_FRAC)) {
        transition_run();
      }
    case GUIDANCE_H_MODE_CARE_FREE:
    case GUIDANCE_H_MODE_ATTITUDE:
      stabilization_attitude_run(in_flight);
      break;

    case GUIDANCE_H_MODE_HOVER:
      /* set psi command from RC */
      guidance_h.sp.heading = guidance_h.rc_sp.psi;  
      /* fall trough to GUIDED to update ref, run traj and set final attitude setpoint */

    case GUIDANCE_H_MODE_GUIDED:
      /* guidance_h.sp.pos and guidance_h.sp.heading need to be set from external source */
      if (!in_flight) {
        guidance_h_hover_enter();
      }

      guidance_h_update_reference();

#if GUIDANCE_INDI
      guidance_indi_run(in_flight, guidance_h.sp.heading);
#else
      /* compute x,y earth commands */
      guidance_h_traj_run(in_flight);
      /* set final attitude setpoint */
      stabilization_attitude_set_earth_cmd_i(&guidance_h_cmd_earth, guidance_h.sp.heading);
#endif
      stabilization_attitude_run(in_flight);
      break;

    case GUIDANCE_H_MODE_NAV:
      if (!in_flight) {
        guidance_h_nav_enter();
      }

      if (horizontal_mode == HORIZONTAL_MODE_ATTITUDE) 
      {
	        struct Int32Eulers sp_cmd_i;
	        sp_cmd_i.phi = nav_roll;
	        sp_cmd_i.theta = nav_pitch;
	        sp_cmd_i.psi = nav_heading;
	        stabilization_attitude_set_rpy_setpoint_i(&sp_cmd_i);
      } 
	  /* add HORIZONTAL_MODE_RC for RC control functions ,similar to hover mode */
       else if (horizontal_mode == HORIZONTAL_MODE_RC) 
       {
	        guidance_h_update_reference();

	        /* set psi command ,using rc_turn_rate from rc_turn_cmd*/
	        nav_heading += (rc_turn_rate/PERIODIC_FREQUENCY); //add deta heading
	        guidance_h.sp.heading = nav_heading;
			
	        INT32_ANGLE_NORMALIZE(guidance_h.sp.heading);
	        /* compute x,y earth commands */
	        guidance_h_traj_run(in_flight);
	        /* set final attitude setpoint */
	        stabilization_attitude_set_earth_cmd_i(&guidance_h_cmd_earth, guidance_h.sp.heading);
      }
	  /* in waypoint,route,circle mode*/
        else 
        {
	        INT32_VECT2_NED_OF_ENU(guidance_h.sp.pos, navigation_carrot);

	        guidance_h_update_reference();

	        /* set psi command */
	        guidance_h.sp.heading = nav_heading;
	        INT32_ANGLE_NORMALIZE(guidance_h.sp.heading);
            #if GUIDANCE_INDI
	        guidance_indi_run(in_flight, guidance_h.sp.heading);
            #else
	        /* compute x,y earth commands */
	        guidance_h_traj_run(in_flight);
	        /* set final attitude setpoint */
	        stabilization_attitude_set_earth_cmd_i(&guidance_h_cmd_earth,  guidance_h.sp.heading);
            #endif
        }
	 #if 0 //TODOM:use intergrater limit in take off,see above ground signal
	  stabilization_attitude_run(above_ground);	  
	 #else
      stabilization_attitude_run(in_flight);
	 #endif
      break;

#if GUIDANCE_H_MODE_MODULE_SETTING == GUIDANCE_H_MODE_MODULE
    case GUIDANCE_H_MODE_MODULE:
      guidance_h_module_run(in_flight);
      break;
#endif

    case GUIDANCE_H_MODE_FLIP:
      guidance_flip_run();
      break;

    default:
      break;
  }
}


static void guidance_h_update_reference(void)
{
  /* compute reference even if usage temporarily disabled via guidance_h_use_ref */
#if GUIDANCE_H_USE_REF
#if GUIDANCE_H_USE_SPEED_REF
  //add HORIZONTAL_MODE_RC using speed_sp,by whp
  if (guidance_h.mode == GUIDANCE_H_MODE_HOVER || horizontal_mode == HORIZONTAL_MODE_RC) {
    gh_update_ref_from_speed_sp(guidance_h.sp.speed);
  } else
#endif
    gh_update_ref_from_pos_sp(guidance_h.sp.pos);
#endif

  /* either use the reference or simply copy the pos setpoint */
  if (guidance_h.use_ref) {
    /* convert our reference to generic representation */
    INT32_VECT2_RSHIFT(guidance_h.ref.pos,   gh_ref.pos, (GH_POS_REF_FRAC - INT32_POS_FRAC));
    INT32_VECT2_LSHIFT(guidance_h.ref.speed, gh_ref.speed, (INT32_SPEED_FRAC - GH_SPEED_REF_FRAC));
    INT32_VECT2_LSHIFT(guidance_h.ref.accel, gh_ref.accel, (INT32_ACCEL_FRAC - GH_ACCEL_REF_FRAC));
  } else {
    VECT2_COPY(guidance_h.ref.pos, guidance_h.sp.pos);
    INT_VECT2_ZERO(guidance_h.ref.speed);
    INT_VECT2_ZERO(guidance_h.ref.accel);
  }

#if GUIDANCE_H_USE_SPEED_REF
  if (guidance_h.mode == GUIDANCE_H_MODE_HOVER || horizontal_mode == HORIZONTAL_MODE_RC) {
    VECT2_COPY(guidance_h.sp.pos, guidance_h.ref.pos); // for display only
  }
#endif
}


#define MAX_POS_ERR   POS_BFP_OF_REAL(16.)
#define MAX_SPEED_ERR SPEED_BFP_OF_REAL(16.)

#ifndef GUIDANCE_H_THRUST_CMD_FILTER
#define GUIDANCE_H_THRUST_CMD_FILTER 10
#endif

/* with a pgain of 100 and a scale of 2,
 * you get an angle of 5.6 degrees for 1m pos error */
#define GH_GAIN_SCALE 2

#if !GUIDANCE_INDI
static void guidance_h_traj_run(bool_t in_flight)
{
  /* maximum bank angle: default 20 deg, max 40 deg*/
  //static const int32_t traj_max_bank = Min(BFP_OF_REAL(GUIDANCE_H_MAX_BANK, INT32_ANGLE_FRAC),
  //                                     BFP_OF_REAL(RadOfDeg(40), INT32_ANGLE_FRAC));
  static const int32_t total_max_bank = BFP_OF_REAL(RadOfDeg(15), INT32_ANGLE_FRAC);
  static const int32_t total_limit_bank = BFP_OF_REAL(RadOfDeg(3), INT32_ANGLE_FRAC);	//add by whp.

  static int32_t flag_of_int_inner_x = 1;
  static int32_t flag_of_int_inner_y = 1;

  struct position_float d_term_temp; //wia huan wen fen 
  struct position_float in_d_term_temp;// nei huan wei fen 
  struct position_float control_temp;// kongzhi bian liang 
  
  /* compute position error    */
  struct NedCoor_i speed_now = *stateGetSpeedNed_i();
  h_speed_fl.x = h_speed_fl.x + (speed_now.x-h_speed_fl.x) * guid_v.speed_filter_coef;
  h_speed_fl.y = h_speed_fl.y + (speed_now.y-h_speed_fl.y) * guid_v.speed_filter_coef;
  
  VECT2_DIFF(guidance_h_pos_err, guidance_h.ref.pos, *stateGetPositionNed_i());
  /* saturate it               */
  VECT2_STRIM(guidance_h_pos_err, -MAX_POS_ERR, MAX_POS_ERR);

  /* compute speed error    */
  VECT2_DIFF(guidance_h_speed_err, guidance_h.ref.speed, h_speed_fl);
  /* saturate it               */
  VECT2_STRIM(guidance_h_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);
#if 0
//**********pid_double_loop****

//********position
	if (1)
	{
		if (1)//(guidance_h.sp.speed.x == 0) && (guidance_h.sp.speed.y == 0))
		{
			//if (onto_position)
			//{
				//guidance_h_hover_enter();
			//}
			//onto_position = FALSE;
			guidance_hf_pos_err.x = POS_FLOAT_OF_BFP(
					guidance_h.ref.pos.x - stateGetPositionNed_i()->x);
			guidance_hf_pos_err.y = POS_FLOAT_OF_BFP(
					guidance_h.ref.pos.y - stateGetPositionNed_i()->y);

			VECT2_STRIM(guidance_hf_pos_err, -2.0, 2.0);

			sum_err.x += flag_of_int_inner_x * (guidance_hf_pos_err.x)
					* (1.0f / 512.0f);
			sum_err.y += flag_of_int_inner_y * (guidance_hf_pos_err.y)
					* (1.0f / 512.0f);

			i_term.x = (sum_err.x) * (guidance_h.gains_f.i);
			i_term.y = (sum_err.y) * (guidance_h.gains_f.i);

			BoundAbs(i_term.x, I_SUM);
			BoundAbs(i_term.y, I_SUM);

            d_term_temp.x =-(SPEED_FLOAT_OF_BFP(stateGetSpeedNed_i()->x));
			d_term_temp.y =-(SPEED_FLOAT_OF_BFP(stateGetSpeedNed_i()->y));

			//d_term_temp.x = (POS_FLOAT_OF_BFP(guidance_h.ref.pos.x)-guidance_h_ref_pos_pre.x)*512.0f-(SPEED_FLOAT_OF_BFP(stateGetSpeedNed_i()->x));
			//d_term_temp.y = (POS_FLOAT_OF_BFP(guidance_h.ref.pos.y)-guidance_h_ref_pos_pre.y)*512.0f-(SPEED_FLOAT_OF_BFP(stateGetSpeedNed_i()->y));

			//guidance_h_ref_pos_pre.x=POS_FLOAT_OF_BFP(guidance_h.ref.pos.x);
			//guidance_h_ref_pos_pre.y=POS_FLOAT_OF_BFP(guidance_h.ref.pos.y);

			d_term.x = d_term.x
					+ (d_term_temp.x - d_term.x) * guid_v.speed_filter_coef;
			d_term.y = d_term.y
					+ (d_term_temp.y - d_term.y) * guid_v.speed_filter_coef;

			desire_speed.x = (guidance_hf_pos_err.x) * (guidance_h.gains_f.p)
					+ i_term.x + (d_term.x) * (guidance_h.gains_f.d);
			desire_speed.y = (guidance_hf_pos_err.y) * (guidance_h.gains_f.p)
					+ i_term.y + (d_term.y) * (guidance_h.gains_f.d);
		}
		else
		{
			onto_position = TRUE;
			desire_speed.x = SPEED_FLOAT_OF_BFP(guidance_h.sp.speed.x);
			desire_speed.y = SPEED_FLOAT_OF_BFP(guidance_h.sp.speed.y);
		}
	}
	else
	{
		desire_speed.x = SPEED_FLOAT_OF_BFP(guidance_h.sp.speed.x);
		desire_speed.y = SPEED_FLOAT_OF_BFP(guidance_h.sp.speed.y);
	}


  BoundAbs(desire_speed.x,5.0);
  BoundAbs(desire_speed.y,5.0);



//********position****speed
 guidance_hf_speed_err.x=desire_speed.x-(SPEED_FLOAT_OF_BFP(stateGetSpeedNed_i()->x));
 guidance_hf_speed_err.y=desire_speed.y-(SPEED_FLOAT_OF_BFP(stateGetSpeedNed_i()->y));

 VECT2_STRIM(guidance_hf_speed_err,-5.0,5.0);

 sum_in_err.x+=flag_of_int_inner_x*(guidance_hf_speed_err.x)*(1.0f/512.0f);
 sum_in_err.y+=flag_of_int_inner_y*(guidance_hf_speed_err.y)*(1.0f/512.0f);

 ii_term.x=(sum_in_err.x)*(guidance_h.gains_f.in_i);
 ii_term.y=(sum_in_err.y)*(guidance_h.gains_f.in_i);

 BoundAbs(ii_term.x, I_SUM_I*1000);
 BoundAbs(ii_term.y, I_SUM_I*1000);

 in_d_term_temp.x=(desire_speed.x-desire_speed_pre.x)*dt_512-(ACCEL_FLOAT_OF_BFP(stateGetAccelNed_i()->x));
 in_d_term_temp.y=(desire_speed.y-desire_speed_pre.y)*dt_512-(ACCEL_FLOAT_OF_BFP(stateGetAccelNed_i()->y));

 desire_speed_pre.x = desire_speed.x;
 desire_speed_pre.y = desire_speed.y;
 
 in_d_term.x=in_d_term.x+(in_d_term_temp.x-in_d_term.x)* guid_v.accd_filter_coef;
 in_d_term.y=in_d_term.y+(in_d_term_temp.y-in_d_term.y)* guid_v.accd_filter_coef;

 control_temp.x=(guidance_hf_speed_err.x)*(guidance_h.gains_f.in_p)+ii_term.x+(in_d_term.x)*(guidance_h.gains_f.in_d);
 control_temp.y=(guidance_hf_speed_err.y)*(guidance_h.gains_f.in_p)+ii_term.y+(in_d_term.y)*(guidance_h.gains_f.in_d);

 guidance_h_cmd_earth.x=(int32_t)(control_temp.x*4096);
 guidance_h_cmd_earth.y=(int32_t)(control_temp.y*4096);
 guidance_h_cmd_earth.x=(guidance_h_cmd_earth.x/1000);
 guidance_h_cmd_earth.y=(guidance_h_cmd_earth.y/1000);

 VECT2_STRIM(guidance_h_cmd_earth, -total_max_bank, total_max_bank);
#endif
 
#if 1
#if 0
  //old PID loop from pprz
  int32_t pd_x =
    ((guidance_h_pgain * guidance_h_pos_err.x) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_h_dgain * (guidance_h_speed_err.x >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
  int32_t pd_y =
    ((guidance_h_pgain * guidance_h_pos_err.y) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_h_dgain * (guidance_h_speed_err.y >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
  guidance_h_cmd_earth.x =
    pd_x +
    ((guidance_h_vgain * guidance_h_speed_ref.x) >> 17) + 
    ((guidance_h_again * guidance_h_accel_ref.x) >> 8);   
  guidance_h_cmd_earth.y =
    pd_y +
    ((guidance_h_vgain * guidance_h_speed_ref.y) >> 17) + 
    ((guidance_h_again * guidance_h_accel_ref.y) >> 8);   

  /* trim max bank angle from PD */
  //  VECT2_STRIM(guidance_h_cmd_earth, -traj_max_bank, traj_max_bank);

  /* Update pos & speed error integral, zero it if not in_flight.
   * Integrate twice as fast when not only POS but also SPEED are wrong,
   * but do not integrate POS errors when the SPEED is already catching up.
   */

   
  if (in_flight) {
    
    guidance_h_trim_att_integrator.x += (guidance_h_igain * pd_x);
    guidance_h_trim_att_integrator.y += (guidance_h_igain * pd_y);

    VECT2_STRIM(guidance_h_trim_att_integrator, -(traj_max_bank << 16), (traj_max_bank << 16));
   
    guidance_h_cmd_earth.x += (guidance_h_trim_att_integrator.x >> 16);
    guidance_h_cmd_earth.y += (guidance_h_trim_att_integrator.y >> 16);
  } else {
    INT_VECT2_ZERO(guidance_h_trim_att_integrator);
  }
#endif

/*-------------------new functions run below-----------*/

/*outer PID here*/

  sum_in_x += flag_of_int_inner_x * guidance_h.gains.i * guidance_h_pos_err.x;
  sum_in_y += flag_of_int_inner_y * guidance_h.gains.i * guidance_h_pos_err.y;

  if(!in_flight||guidance_h.gains.i==0)
  {
  	  sum_in_x = 0;
	  sum_in_y = 0;
  }

/*
  c_x = guidance_h.ref.speed.x 
  	    + guidance_h_pos_err.x*12*guidance_h.gains.a
  	    + (guidance_h_speed_err.x>>6)*guidance_h.gains.v
  	    + (sum_in_x>>7)*guidance_h.gains.i;

  c_y = guidance_h.ref.speed.y 
  	    + guidance_h_pos_err.y*12*guidance_h.gains.a
  	    + (guidance_h_speed_err.y>>6)*guidance_h.gains.v
  	    + (sum_in_y>>7)*guidance_h.gains.i;
*/
  int32_t px = (guidance_h_pos_err.x<<5)*guidance_h.gains.a;
  int32_t dx = (guidance_h_speed_err.x>>6)*guidance_h.gains.v;
  int32_t ix = sum_in_x>>11;
  int32_t py = (guidance_h_pos_err.y<<5)*guidance_h.gains.a;
  int32_t dy = (guidance_h_speed_err.y>>6)*guidance_h.gains.v;
  int32_t iy = sum_in_y>>11;

  c_x = guidance_h.ref.speed.x + px + dx + ix;
  c_y = guidance_h.ref.speed.y + py + dy + iy;

/*
  c_x = guidance_h.ref.speed.x 
  	    + (guidance_h_pos_err.x<<5)*guidance_h.gains.a
  	    + (guidance_h_speed_err.x>>6)*guidance_h.gains.v
  	    + (sum_in_x>>11);

  c_y = guidance_h.ref.speed.y 
  	    + (guidance_h_pos_err.y<<5)*guidance_h.gains.a
  	    + (guidance_h_speed_err.y>>6)*guidance_h.gains.v
  	    + (sum_in_y>>11);
*/
  int32_t error_x= c_x - h_speed_fl.x;
  int32_t error_y= c_y - h_speed_fl.y;

  Tracking_differntiator_hx( (float)(error_x) );
  Tracking_differntiator_hy( (float)(error_y) );	

	
/*inner PID here*/	

  guidance_h_cmd_earth.x = -fhan_control( (float)error_x, xh2/256.0, 100.0*(float)(guidance_h.gains.p), 0.2*(float)(guidance_h.gains.d) );
  guidance_h_cmd_earth.y = -fhan_control( (float)error_y, yh2/256.0, 100.0*(float)(guidance_h.gains.p), 0.2*(float)(guidance_h.gains.d) );	
#endif

/*-------------------new functions run end-----------*/


  /* compute a better approximation of force commands by taking thrust into account */
  if (guidance_h.approx_force_by_thrust && in_flight)
  {
    static int32_t thrust_cmd_filt;
    int32_t vertical_thrust = (stabilization_cmd[COMMAND_THRUST] * guidance_v_thrust_coeff) >> INT32_TRIG_FRAC;
    thrust_cmd_filt = (thrust_cmd_filt * GUIDANCE_H_THRUST_CMD_FILTER + vertical_thrust) /
                      (GUIDANCE_H_THRUST_CMD_FILTER + 1);
    guidance_h_cmd_earth.x = ANGLE_BFP_OF_REAL(atan2f((guidance_h_cmd_earth.x * MAX_PPRZ / INT32_ANGLE_PI_2),
                             thrust_cmd_filt));
    guidance_h_cmd_earth.y = ANGLE_BFP_OF_REAL(atan2f((guidance_h_cmd_earth.y * MAX_PPRZ / INT32_ANGLE_PI_2),
                             thrust_cmd_filt));
  }
  
  flag_of_int_inner_x = 1;
  flag_of_int_inner_y = 1;
  if(abs(guidance_h_cmd_earth.x) >= (total_max_bank))
  {
	flag_of_int_inner_x = 0;
  }
  if(abs(guidance_h_cmd_earth.y) >= (total_max_bank))
  {
	flag_of_int_inner_y = 0;
  }	

 #ifndef NPS_SIMU   //sonar  don't work in simulator
  //if((agl_dist_value_filtered<DISTANCE_ABOVE_GROUNG) || stateGetHorizontalSpeedNorm_f()>30)
  if(stateGetPositionEnu_f()->z < (DISTANCE_ABOVE_GROUNG+0.1) || stateGetHorizontalSpeedNorm_f()>30.0)
  {    
  	VECT2_STRIM(guidance_h_cmd_earth, -total_limit_bank, total_limit_bank);  
	flag_of_int_inner_x = 0;
	flag_of_int_inner_y = 0;
  }  //angle is 3deg limited
 #endif

  VECT2_STRIM(guidance_h_cmd_earth, -total_max_bank, total_max_bank);  //angle is 30` limited

  
}
#endif


static void guidance_h_hover_enter(void)
{
  /* set horizontal setpoint to current position */
  VECT2_COPY(guidance_h.sp.pos, *stateGetPositionNed_i());

  reset_guidance_reference_from_current_position();
  guidance_h.sp.heading = stateGetNedToBodyEulers_i()->psi;
  guidance_h.rc_sp.psi = stateGetNedToBodyEulers_i()->psi;

  #if USE_FLOW  
  flow_hff_init(0.0, 0.0, 0.0, 0.0);//zero to pos and vel,later flow filter update when hover running
  #endif 
}

static void guidance_h_nav_enter(void)
{
  /* horizontal position setpoint from navigation/flightplan */
  INT32_VECT2_NED_OF_ENU(guidance_h.sp.pos, navigation_carrot);

  reset_guidance_reference_from_current_position();

  nav_heading = stateGetNedToBodyEulers_i()->psi;
}

void guidance_h_nav_rc_enter(void)
{
  /* set horizontal setpoint to current position */
  VECT2_COPY(guidance_h.sp.pos, *stateGetPositionNed_i());

  reset_guidance_reference_from_current_position();

  //nav_heading = stateGetNedToBodyEulers_i()->psi;
  horizontal_mode = HORIZONTAL_MODE_RC;    //request to set mode
}

static inline void transition_run(void)
{
  //Add 0.00625%
  transition_percentage += 1 << (INT32_PERCENTAGE_FRAC - 4);

#ifdef TRANSITION_MAX_OFFSET
  const int32_t max_offset = ANGLE_BFP_OF_REAL(TRANSITION_MAX_OFFSET);
  transition_theta_offset = INT_MULT_RSHIFT((transition_percentage << (INT32_ANGLE_FRAC - INT32_PERCENTAGE_FRAC)) / 100,
                            max_offset, INT32_ANGLE_FRAC);
#endif
}

/// read speed setpoint from RC
static void read_rc_setpoint_speed_i(struct Int32Vect2 *speed_sp, bool_t in_flight)
{
  if (in_flight) {
    // negative pitch is forward
    int64_t rc_x = -radio_control.values[RADIO_PITCH];
    int64_t rc_y = radio_control.values[RADIO_ROLL];
    DeadBand(rc_x, MAX_PPRZ / 10);
    DeadBand(rc_y, MAX_PPRZ / 10);

    // convert input from MAX_PPRZ range to SPEED_BFP
    int32_t max_speed = SPEED_BFP_OF_REAL(GUIDANCE_H_REF_MAX_SPEED);
    /// @todo calc proper scale while making sure a division by zero can't occur
    //int32_t rc_norm = sqrtf(rc_x * rc_x + rc_y * rc_y);
    //int32_t max_pprz = rc_norm * MAX_PPRZ / Max(abs(rc_x), abs(rc_y);
    rc_x = rc_x * max_speed / MAX_PPRZ;
    rc_y = rc_y * max_speed / MAX_PPRZ;

    /* Rotate from body to NED frame by negative psi angle */
    int32_t psi = -stateGetNedToBodyEulers_i()->psi;
    int32_t s_psi, c_psi;
    PPRZ_ITRIG_SIN(s_psi, psi);
    PPRZ_ITRIG_COS(c_psi, psi);
    speed_sp->x = (int32_t)(((int64_t)c_psi * rc_x + (int64_t)s_psi * rc_y) >> INT32_TRIG_FRAC);
    speed_sp->y = (int32_t)((-(int64_t)s_psi * rc_x + (int64_t)c_psi * rc_y) >> INT32_TRIG_FRAC);
  } else {
    speed_sp->x = 0;
    speed_sp->y = 0;
  }
}

void guidance_h_set_igain(uint32_t igain)
{
  guidance_h.gains.i = igain;
  INT_VECT2_ZERO(guidance_h_trim_att_integrator);
  sum_in_x = 0;
  sum_in_y = 0;
}

bool_t guidance_h_set_guided_pos(float x, float y)
{
  if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED) {
    guidance_h.sp.pos.x = POS_BFP_OF_REAL(x);
    guidance_h.sp.pos.y = POS_BFP_OF_REAL(y);
    return TRUE;
  }
  return FALSE;
}

bool_t guidance_h_set_guided_heading(float heading)
{
  if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED) {
    guidance_h.sp.heading = ANGLE_BFP_OF_REAL(heading);
    INT32_ANGLE_NORMALIZE(guidance_h.sp.heading);
    return TRUE;
  }
  return FALSE;
}
