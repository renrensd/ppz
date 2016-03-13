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
#include "math/pprz_algebra_float.h"
#include "modules/laser/laser_r2100.h"
#include "generated/airframe.h"
#include "subsystems/navigation/waypoints.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_module.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/guidance/guidance_h_ref.h"

#include "firmwares/rotorcraft/stabilization/stabilization_none.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

/* for guidance_v_thrust_coeff */
#include "firmwares/rotorcraft/guidance/guidance_v.h"

#include "state.h"
#define Sign(_x) ((_x) > 0 ? 1 : (-1))
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
#define GUIDANCE_H_APPROX_FORCE_BY_THRUST FALSE
#endif


uint8_t guidance_h_mode;
bool_t guidance_h_use_ref;
bool_t guidance_h_approx_force_by_thrust;
int lock_flag = 0;
int lock_flag_last = 0;

struct FloatVect2 a_force;
struct FloatVect2 r_force; 
struct Int32Vect2 cross_vector;
struct Int32Vect2 vector_to_obstacle; 
struct Int32Vect2 guidance_h_pos_sp;
struct Int32Vect2 guidance_h_pos_obstacle;
struct Int32Vect2 guidance_h_pos_ref;
struct Int32Vect2 guidance_h_pos_ref_fake; 
struct Int32Vect2 guidance_h_speed_ref_fake;
struct Int32Vect2 guidance_h_accel_ref_fake;
struct Int32Vect2 guidance_h_pos_sp_fake; 
struct Int32Vect2 guidance_h_pos_err_fake;
struct Int32Vect2 guidance_h_speed_err_fake;
struct NedCoor_i accel_for_identification; 
struct Int32Vect2 vector_to_sp;
extern struct EnuCoor_i navigation_obstacle; 
extern struct LASER_R2100_DATA laser_data;
extern float gh_max_speed=0.8;

struct NedCoor_i vector_of_pos;
struct NedCoor_i vector_of_speed; 
struct Int32Vect2 guidance_h_speed_ref;
struct Int32Vect2 guidance_h_accel_ref;
#if GUIDANCE_H_USE_SPEED_REF
struct Int32Vect2 guidance_h_speed_sp;
#endif
struct Int32Vect2 guidance_h_pos_err;
struct Int32Vect2 guidance_h_speed_err;
struct Int32Vect2 guidance_h_trim_att_integrator;
float temp_d2_o_x ;
float temp_d2_o_y;
struct Int32Vect2 buf_for_sp;
struct Int32Vect2  guidance_h_cmd_earth;
struct Int32Eulers guidance_h_rc_sp;
int32_t guidance_h_heading_sp;
int32_t Angle_of_laser[11] = {-44*71.5,-36*71.5,-28*71.5,-20*71.5,-12*71.5,0,12*71.5,20*71.5,28*71.5,36*71.5,44*71.5}; 
int32_t rc_turn_rate;    //using in HORIZONTAL_MODE_RC,cmd of psi rate

int32_t guidance_h_pgain;
int32_t guidance_h_dgain;
int32_t guidance_h_igain;
int32_t guidance_h_again;
int32_t guidance_h_vgain;
int32_t d_obstacle;
int32_t add_force_x;	 
int32_t add_force_y;	
int32_t d_sp;
int32_t mid1; 
int32_t transition_percentage;
int32_t transition_theta_offset;
int32_t alpha;

int j = 0;
int max_dis = 0;

int32_t speed_to_obstacle_x = 0;
int32_t speed_to_obstacle_y = 0;
int32_t buff_pos_to_obstacle_x = 0;
int32_t buff_pos_to_obstacle_y = 0; 


float ks_x;
float ks_y;

static void guidance_h_update_reference(void);
static void guidance_h_traj_run(bool_t in_flight);
static void guidance_h_hover_enter(void);
static void guidance_h_nav_enter(void);
static inline void transition_run(void);
static void read_rc_setpoint_speed_i(struct Int32Vect2 *speed_sp, bool_t in_flight);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_gh(struct transport_tx *trans, struct link_device *dev)
{
  struct NedCoor_i *pos = stateGetPositionNed_i();
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  pprz_msg_send_GUIDANCE_H_INT(trans, dev, AC_ID,
                               &guidance_h_pos_sp.x, &guidance_h_pos_sp.y,
                               &guidance_h_pos_ref_fake.x, &guidance_h_pos_ref_fake.y,
                               &(pos->x), &(pos->y));
}



static void calculating_position_of_obstacle(void)
{
//getting position of drone.
	vector_of_pos = *stateGetPositionNed_i();
//get heading angle of drone
int32_t psi = stateGetNedToBodyEulers_i()->psi;


int i=0;


	for( i=0;i<11;i++){
		if(laser_data.dis[i]>=5000){
			laser_data.dis[i]=0;
		}
	}
	max_dis=laser_data.dis[0];
	j=0;
	for( i=1;i<11;i++){
		if(max_dis<=laser_data.dis[i]){
			max_dis=max_dis;
			j=j;
		}else{
			max_dis=laser_data.dis[i];
			j=i;
		}
		}
	max_dis = max_dis*0.256;
	alpha = psi + Angle_of_laser[j] ;
	float angle = ANGLE_FLOAT_OF_BFP(alpha);
	guidance_h_pos_obstacle.y = vector_of_pos.y+max_dis*sinf(angle);
	guidance_h_pos_obstacle.x = vector_of_pos.x+max_dis*cosf(angle);
	
}



static void send_hover_loop(struct transport_tx *trans, struct link_device *dev)
{
  struct NedCoor_i *pos = stateGetPositionNed_i();
  struct NedCoor_i *speed = stateGetSpeedNed_i();
  struct NedCoor_i *accel = stateGetAccelNed_i();
  int32_t tell =( vector_to_obstacle.x)*( vector_to_obstacle.x)>>8;
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  pprz_msg_send_HOVER_LOOP(trans, dev, AC_ID,
                           & guidance_h_pos_sp_fake.x ,
                           & lock_flag,
                           &(pos->x), &(pos->y),
                           &(speed->x), &(speed->y),
                           &(accel->x), &(accel->y),
                           &guidance_h_pos_err.x,
                           &guidance_h_pos_err.y,
                           &guidance_h_speed_err.x,
                           &guidance_h_speed_err.y,
                           &guidance_h_trim_att_integrator.x,
                           &guidance_h_trim_att_integrator.y,
                           &guidance_h_cmd_earth.x,
                           &guidance_h_cmd_earth.y,
                           &Angle_of_laser[j] );
}

static void send_href(struct transport_tx *trans, struct link_device *dev)
{
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  pprz_msg_send_GUIDANCE_H_REF_INT(trans, dev, AC_ID,
                                   &guidance_h_pos_sp.x, &guidance_h_pos_ref_fake.x,
                                   &guidance_h_speed_sp.x, &guidance_h_speed_ref_fake.x,
                                   &ks_x,
                                   &guidance_h_pos_sp.y, &guidance_h_pos_ref_fake.y,
                                   &guidance_h_speed_sp.y, &guidance_h_speed_ref_fake.y,
                                   &guidance_h_accel_ref.y);
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

  guidance_h_mode = GUIDANCE_H_MODE_KILL;
  guidance_h_use_ref = GUIDANCE_H_USE_REF;
  guidance_h_approx_force_by_thrust = GUIDANCE_H_APPROX_FORCE_BY_THRUST;

  INT_VECT2_ZERO(guidance_h_pos_sp);
  INT_VECT2_ZERO(guidance_h_trim_att_integrator);
  INT_EULERS_ZERO(guidance_h_rc_sp);
  guidance_h_heading_sp = 0;
  guidance_h_pgain = GUIDANCE_H_PGAIN;
  guidance_h_igain = GUIDANCE_H_IGAIN;
  guidance_h_dgain = GUIDANCE_H_DGAIN;
  guidance_h_again = GUIDANCE_H_AGAIN;
  guidance_h_vgain = GUIDANCE_H_VGAIN;
  transition_percentage = 0;
  transition_theta_offset = 0;
  buf_for_sp.x = guidance_h_pos_sp.x;
  buf_for_sp.y = guidance_h_pos_sp.y;
  guidance_h_pos_sp_fake.x = 0;
  guidance_h_pos_sp_fake.y = 0;
  guidance_h_pos_obstacle.y = -100000;
  guidance_h_pos_obstacle.x = -100000; 

  gh_ref_init();

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "GUIDANCE_H_INT", send_gh);
  register_periodic_telemetry(DefaultPeriodic, "HOVER_LOOP", send_hover_loop);
  register_periodic_telemetry(DefaultPeriodic, "GUIDANCE_H_REF", send_href);
  register_periodic_telemetry(DefaultPeriodic, "ROTORCRAFT_TUNE_HOVER", send_tune_hover);
#endif
}


static inline void reset_guidance_reference_from_current_position(void)
{
  VECT2_COPY(guidance_h_pos_ref, *stateGetPositionNed_i());
  VECT2_COPY(guidance_h_speed_ref, *stateGetSpeedNed_i());
  INT_VECT2_ZERO(guidance_h_accel_ref);
  gh_set_ref(guidance_h_pos_ref, guidance_h_speed_ref, guidance_h_accel_ref);

  INT_VECT2_ZERO(guidance_h_trim_att_integrator);
}

void guidance_h_mode_changed(uint8_t new_mode)
{
  if (new_mode == guidance_h_mode) {
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
      if (guidance_h_mode == GUIDANCE_H_MODE_KILL ||
          guidance_h_mode == GUIDANCE_H_MODE_RATE ||
          guidance_h_mode == GUIDANCE_H_MODE_RC_DIRECT)
#endif
        stabilization_attitude_enter();
      break;

    case GUIDANCE_H_MODE_HOVER:
      guidance_h_hover_enter();
#if NO_ATTITUDE_RESET_ON_MODE_CHANGE
      /* reset attitude stabilization if previous mode was not using it */
      if (guidance_h_mode == GUIDANCE_H_MODE_KILL ||
          guidance_h_mode == GUIDANCE_H_MODE_RATE ||
          guidance_h_mode == GUIDANCE_H_MODE_RC_DIRECT)
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
      if (guidance_h_mode == GUIDANCE_H_MODE_KILL ||
          guidance_h_mode == GUIDANCE_H_MODE_RATE ||
          guidance_h_mode == GUIDANCE_H_MODE_RC_DIRECT)
#endif
        stabilization_attitude_enter();
      break;

    default:
      break;
  }

  guidance_h_mode = new_mode;

}


void guidance_h_read_rc(bool_t  in_flight)
{

  switch (guidance_h_mode) {

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
      stabilization_attitude_read_rc_setpoint_eulers(&guidance_h_rc_sp, in_flight, FALSE, FALSE);
#if GUIDANCE_H_USE_SPEED_REF
      read_rc_setpoint_speed_i(&guidance_h_speed_sp, in_flight);
#endif
      break;

#if GUIDANCE_H_MODE_MODULE_SETTING == GUIDANCE_H_MODE_MODULE
    case GUIDANCE_H_MODE_MODULE:
      guidance_h_module_read_rc();
      break;
#endif

    case GUIDANCE_H_MODE_NAV:
      if (radio_control.status == RC_OK) {
        stabilization_attitude_read_rc_setpoint_eulers(&guidance_h_rc_sp, in_flight, FALSE, FALSE);
      } else {
        INT_EULERS_ZERO(guidance_h_rc_sp);
      }
      break;
    default:
      break;
  }

}








static void Condition_check_avoidance(void)
{

struct Int32Vect2 middle_point; 
int32_t length_middle; 	
	if(2*d_obstacle<d_sp){
		if(d_obstacle<=3000){
			if((abs(guidance_h_speed_ref.x)<=50000)&&(abs(guidance_h_speed_ref.y)<=50000)){
				lock_flag = 1;
			
			}else{
				lock_flag=0;
			}
		}else{
			lock_flag=0;
		}
		}
	if((lock_flag)==1){
		middle_point.x = 1;
		middle_point.y=1;

		
		if(abs(vector_to_obstacle.y)>0){
			
			middle_point.x = 1;
			if(abs(vector_to_obstacle.y)<1){
				middle_point.y = 10;}
			else{
				middle_point.y = 1*(-vector_to_obstacle.x/vector_to_obstacle.y);
				}
			
		}
		if(abs(vector_to_obstacle.x)>0){
			
			middle_point.y= 1;
			
			if(abs(vector_to_obstacle.x)<1){
				middle_point.x = 10;
				}
			else{
				middle_point.x = 1*(-vector_to_obstacle.y/vector_to_obstacle.x);
				}
		
	
		}	

		length_middle = sqrt(middle_point.x*middle_point.x+middle_point.y*middle_point.y);
		
		guidance_h_pos_sp_fake.x +=(100* middle_point.x/length_middle)*0.1; 
		guidance_h_pos_sp_fake.y +=(100* middle_point.y/length_middle) *0.1;
		lock_flag_last = lock_flag;
	}
	/*
	if(d_obstacle<2000){
		middle_point.x = 1;
		middle_point.y=1;

		
		if(abs(vector_to_obstacle.y)>0){
			
			middle_point.x = 1;
			middle_point.y = -vector_to_obstacle.x/vector_to_obstacle.y;
		}
		if(abs(vector_to_obstacle.x)>0){
			
			middle_point.y= 1;
			middle_point.x= -vector_to_obstacle.y/vector_to_obstacle.x;
		}	

		
		
		//guidance_h_pos_sp_fake.x +=1* middle_point.x; 
		//guidance_h_pos_sp_fake.y +=1* middle_point.y ;
		lock_flag_last = lock_flag;
	}*/
			
}




	
	


static void anti_lock_function(void){

switch(lock_flag-lock_flag_last){
case 1: guidance_h_pos_sp.x += 10000; 
		guidance_h_pos_sp.y += 10000;
		break; 
case 0: break; 
case -1: guidance_h_pos_sp.x -= 3000; 
		guidance_h_pos_sp.y -= 3000;
		break; 
default : break;
}



}

static void setpoint_h_avoidence(void)
{
//calculating_position_of_obstacle();
Condition_check_avoidance();
float Ka= 300;
float Kr = 10;
float krd = 0.00;
float r_force_length = 0;
float a_force_length = 0;
static  float  buff_of_force_x = 0; 
static  float  buff_of_force_y = 0; 
static struct FloatVect2 buff_of_force; 
static struct FloatVect2 buff_of_rforce; 
static  float  buff_of_rforce_x = 0; 
static  float  buff_of_rforce_y = 0; 

int32_t mid2 = 100000/10000;

	VECT2_DIFF(vector_to_sp,guidance_h_pos_sp, guidance_h_pos_sp_fake);
			
	d_sp = ((vector_to_sp.x)*(vector_to_sp.x)>>8)+((vector_to_sp.y)*(vector_to_sp.y)>>8);		


			
		if(d_sp<=10)
		{
			d_sp = 10;
		}
		
	a_force.x = Ka* vector_to_sp.x/(d_sp);		
	a_force.y = Ka* vector_to_sp.y/(d_sp);	

	
	a_force_length = sqrt(a_force.x*a_force.x+ a_force.y*a_force.y);
				
		if ((abs(a_force.x)>=gh_ref.max_speed*0.4)||(abs(a_force.y)>=0.4*gh_ref.max_speed))
		{
			a_force.x =0.4*gh_ref.max_speed*a_force.x/a_force_length;
			a_force.y =0.4*gh_ref.max_speed*a_force.y/a_force_length;
					
		}

			


		if(( abs(buff_of_force_x)>10)||(abs(buff_of_force_y)>10))
		{
			buff_of_force.x = buff_of_force_x;
			buff_of_force.y = buff_of_force_y;
			buff_of_force_x=0;
			buff_of_force_y=0;
			VECT2_SUM(guidance_h_pos_sp_fake, guidance_h_pos_sp_fake, buff_of_force);
			buff_of_force.x = 0;
			buff_of_force.y = 0;
			
		}
		else
		{
			buff_of_force_x = buff_of_force_x + a_force.x;
			buff_of_force_y = buff_of_force_y + a_force.y;
		}

			

			
	vector_to_obstacle.y = (- navigation_obstacle.x + guidance_h_pos_sp_fake.y);
	vector_to_obstacle.x = (- navigation_obstacle.y + guidance_h_pos_sp_fake.x);
					
					
	 d_obstacle = ((vector_to_obstacle.x)*(vector_to_obstacle.x)>>8)+((vector_to_obstacle.y)*(vector_to_obstacle.y)>>8);
					


		if(d_obstacle<=0.01)
		{
			d_obstacle = 0.01;
		}


	mid1 = (100000/d_obstacle);


	r_force.x = Kr*((mid1-mid2)*(mid1-mid2))*((vector_to_obstacle.x)*mid1);
	r_force.y = Kr*((mid1-mid2)*(mid1-mid2))*((vector_to_obstacle.y)*mid1);

		if (d_obstacle>=2000)
		{
			r_force.x = 0;
			r_force.y = 0;
			
		}
		
	r_force_length = sqrt(r_force.x*r_force.x+ r_force.y*r_force.y);
				

		if ((abs(r_force.x)>=10)||(abs(r_force.y)>=10))
		{
			r_force.x =10*r_force.x/r_force_length;
			r_force.y =10*r_force.y/r_force_length;
			
		}
		
		if(( abs(buff_of_rforce_x)>15)||(abs(buff_of_rforce_y)>15))
		{
			buff_of_rforce.x = buff_of_rforce_x;
			buff_of_rforce.y = buff_of_rforce_y;
			buff_of_rforce_x=0;
			buff_of_rforce_y=0;
			VECT2_SUM(guidance_h_pos_sp_fake, guidance_h_pos_sp_fake, buff_of_rforce);
			buff_of_rforce.x = 0;
			buff_of_rforce.y = 0;
			
		}
		else
		{
			buff_of_rforce_x = buff_of_rforce_x + r_force.x;
			buff_of_rforce_y = buff_of_rforce_y +r_force.y;
		}
	
	
}

void guidance_h_run(bool_t  in_flight)
{
  switch (guidance_h_mode) {

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
      if (!in_flight) {
        guidance_h_hover_enter();
      }

      guidance_h_update_reference();

      /* set psi command */
      guidance_h_heading_sp = guidance_h_rc_sp.psi;
      /* compute x,y earth commands */
      guidance_h_traj_run(in_flight);
      /* set final attitude setpoint */
      stabilization_attitude_set_earth_cmd_i(&guidance_h_cmd_earth, guidance_h_heading_sp);
      stabilization_attitude_run(in_flight);
      break;

    case GUIDANCE_H_MODE_NAV:
      if (!in_flight) {
        guidance_h_nav_enter();
      }

      if (horizontal_mode == HORIZONTAL_MODE_ATTITUDE) {
        struct Int32Eulers sp_cmd_i;
        sp_cmd_i.phi = nav_roll;
        sp_cmd_i.theta = nav_pitch;
        /** @todo: heading can't be set via attitude block yet.
         * use current euler psi for now, should be real heading
         */
        sp_cmd_i.psi = stateGetNedToBodyEulers_i()->psi;
        stabilization_attitude_set_rpy_setpoint_i(&sp_cmd_i);
      } 
	  /* add HORIZONTAL_MODE_RC for RC control functions ,similar to hover mode */
	  else if (horizontal_mode == HORIZONTAL_MODE_RC) {
        guidance_h_update_reference();

        /* set psi command ,using rc_turn_rate from rc_turn_cmd*/
        nav_heading +=(rc_turn_rate/PERIODIC_FREQUENCY); //add deta heading
		guidance_h_heading_sp =nav_heading;
		
        INT32_ANGLE_NORMALIZE(guidance_h_heading_sp);
        /* compute x,y earth commands */
        guidance_h_traj_run(in_flight);
        /* set final attitude setpoint */
        stabilization_attitude_set_earth_cmd_i(&guidance_h_cmd_earth,
                                               guidance_h_heading_sp);
      }
	  /* in waypoint,route,circle mode*/
	  else {
        INT32_VECT2_NED_OF_ENU(guidance_h_pos_sp, navigation_carrot);

        guidance_h_update_reference();
	    setpoint_h_avoidence();

        /* set psi command */
        guidance_h_heading_sp = nav_heading;
        INT32_ANGLE_NORMALIZE(guidance_h_heading_sp);
        /* compute x,y earth commands */
        guidance_h_traj_run(in_flight);
        /* set final attitude setpoint */
        stabilization_attitude_set_earth_cmd_i(&guidance_h_cmd_earth,
                                               guidance_h_heading_sp);
      }
	 #if 1 //use intergrater limit in take off,see above ground signal
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

    default:
      break;
  }
}


static void guidance_h_update_reference(void)
{
  /* compute reference even if usage temporarily disabled via guidance_h_use_ref */
#if GUIDANCE_H_USE_REF
#if GUIDANCE_H_USE_SPEED_REF
  if (guidance_h_mode == GUIDANCE_H_MODE_HOVER || horizontal_mode == HORIZONTAL_MODE_RC) { //add HORIZONTAL_MODE_RC using speed_sp
    gh_update_ref_from_speed_sp(guidance_h_speed_sp);   //guidance_h_speed_sp is from RC stick(roll/pitch) or RC_cmd
  } else
#endif
    


	

    gh_update_ref_from_pos_sp(guidance_h_pos_sp_fake);
#endif

  /* either use the reference or simply copy the pos setpoint */
  if (guidance_h_use_ref) {
    /* convert our reference to generic representation */
    INT32_VECT2_RSHIFT(guidance_h_pos_ref,   gh_ref.pos, (GH_POS_REF_FRAC - INT32_POS_FRAC));
    INT32_VECT2_LSHIFT(guidance_h_speed_ref, gh_ref.speed, (INT32_SPEED_FRAC - GH_SPEED_REF_FRAC));
    INT32_VECT2_LSHIFT(guidance_h_accel_ref, gh_ref.accel, (INT32_ACCEL_FRAC - GH_ACCEL_REF_FRAC));
  } else {
    VECT2_COPY(guidance_h_pos_ref, guidance_h_pos_sp);
    INT_VECT2_ZERO(guidance_h_speed_ref);
    INT_VECT2_ZERO(guidance_h_accel_ref);
  }

#if GUIDANCE_H_USE_SPEED_REF
  if (guidance_h_mode == GUIDANCE_H_MODE_HOVER || horizontal_mode == HORIZONTAL_MODE_RC) {
    VECT2_COPY(guidance_h_pos_sp, guidance_h_pos_ref); // for carrot pos display only
  }
#endif
}

static void guidance_h_update_reference_fake(void)
{
  /* compute reference even if usage temporarily disabled via guidance_h_use_ref */
#if GUIDANCE_H_USE_REF

    gh_update_ref_from_pos_sp(guidance_h_pos_sp);
#endif

  /* either use the reference or simply copy the pos setpoint */
  if (guidance_h_use_ref) {
    /* convert our reference to generic representation */
    INT32_VECT2_RSHIFT(guidance_h_pos_ref_fake,   gh_ref.pos, (GH_POS_REF_FRAC - INT32_POS_FRAC));
    INT32_VECT2_LSHIFT(guidance_h_speed_ref_fake, gh_ref.speed, (INT32_SPEED_FRAC - GH_SPEED_REF_FRAC));
    INT32_VECT2_LSHIFT(guidance_h_accel_ref_fake, gh_ref.accel, (INT32_ACCEL_FRAC - GH_ACCEL_REF_FRAC));
  } else {
    VECT2_COPY(guidance_h_pos_ref_fake, guidance_h_pos_sp_fake);
    INT_VECT2_ZERO(guidance_h_speed_ref_fake);
    INT_VECT2_ZERO(guidance_h_accel_ref_fake);
  }


}



#define MAX_POS_ERR   POS_BFP_OF_REAL(2.)
#define MAX_SPEED_ERR SPEED_BFP_OF_REAL(1.5)

#ifndef GUIDANCE_H_THRUST_CMD_FILTER
#define GUIDANCE_H_THRUST_CMD_FILTER 10
#endif

/* with a pgain of 100 and a scale of 2,
 * you get an angle of 5.6 degrees for 1m pos error */
#define GH_GAIN_SCALE 2

static void guidance_h_traj_run(bool_t in_flight)
{
  /* maximum bank angle: default 20 deg, max 40 deg*/
  static const int32_t traj_max_bank = Min(BFP_OF_REAL(GUIDANCE_H_MAX_BANK, INT32_ANGLE_FRAC),
                                       BFP_OF_REAL(RadOfDeg(40), INT32_ANGLE_FRAC));
  static const int32_t total_max_bank = BFP_OF_REAL(RadOfDeg(45), INT32_ANGLE_FRAC);

  /* compute position error    */
  VECT2_DIFF(guidance_h_pos_err, guidance_h_pos_ref, *stateGetPositionNed_i());
  /* saturate it               */
  VECT2_STRIM(guidance_h_pos_err, -MAX_POS_ERR, MAX_POS_ERR);

  /* compute speed error    */
  VECT2_DIFF(guidance_h_speed_err, guidance_h_speed_ref, *stateGetSpeedNed_i());
  /* saturate it               */
  VECT2_STRIM(guidance_h_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);

/*this part is for the propergate of adaptive gain*/
 	 ks_x = 1.0;
	 ks_y = 1.0;

	float d_k_x =(guidance_h_pos_err.x  >>INT32_POS_FRAC) ;
		d_k_x = d_k_x*(guidance_h_pos_ref.x  >>INT32_POS_FRAC);
	float d_k_y =(guidance_h_pos_err.y >>INT32_POS_FRAC);
		d_k_y = d_k_y*(guidance_h_pos_ref.y >>INT32_POS_FRAC);

	ks_x = ks_x+d_k_x*0.0;
	ks_y = ks_y+d_k_y*0.0;
	guidance_h_pos_ref_fake.x = ks_x*guidance_h_pos_ref.x;
	guidance_h_pos_ref_fake.y = ks_y*guidance_h_pos_ref.y;
	guidance_h_speed_ref_fake.x = ks_x*guidance_h_speed_ref.x;
	guidance_h_speed_ref_fake.y = ks_y*guidance_h_speed_ref.y;
	guidance_h_accel_ref_fake.x = ks_x*guidance_h_accel_ref.x;
	guidance_h_accel_ref_fake.y = ks_y*guidance_h_accel_ref.y;

		

	
	 VECT2_DIFF(guidance_h_pos_err_fake, guidance_h_pos_ref_fake, *stateGetPositionNed_i());
	  /* saturate it               */
	 VECT2_STRIM(guidance_h_pos_err_fake, -MAX_POS_ERR, MAX_POS_ERR);

	  /* compute speed error    */
	 VECT2_DIFF(guidance_h_speed_err_fake, guidance_h_speed_ref_fake, *stateGetSpeedNed_i());
	  /* saturate it               */
	 VECT2_STRIM(guidance_h_speed_err_fake, -MAX_SPEED_ERR, MAX_SPEED_ERR);
	 float U = stabilization_cmd[COMMAND_THRUST];// need to be altered to Force, using the data calculated. 
	 struct FloatEulers *att_t = stateGetNedToBodyEulers_f();
	 struct FloatEulers att_at = *att_t;
	  
	 accel_for_identification = *stateGetAccelNed_i();
	static float m = 80000;
	static dm = 0;

	  /*the new type of control method was applied here */
	  /*
		 float k8 = guidance_h_igain/100;
		 float k9 = guidance_h_pgain;
		 float  k10 = guidance_h_dgain;*/
	/*	 
		 float k8 = 0.8;
		 float k9 = 10;
		 float  k10 = 100;
	 float k11 = k9;
	 float k12 = k10;
	 float k13 = k8;
	 int integral_flag_x = 1; 
	 int integral_flag_y = 1; 
	 */
	/*integral and anti-windup */
/*
		 guidance_h_trim_att_integrator.x +=  integral_flag_x*0.02*guidance_h_pos_err_fake.x;
	if (guidance_h_trim_att_integrator.x>4096)
	{
		guidance_h_trim_att_integrator.x = 4096;
	}
	if(guidance_h_trim_att_integrator.x<-4096)
	{
		guidance_h_trim_att_integrator.x = -4096;
	}
		 
		 guidance_h_trim_att_integrator.y +=  integral_flag_y*0.02*guidance_h_pos_err_fake.y;

	 if (guidance_h_trim_att_integrator.y>4096)
	{
		guidance_h_trim_att_integrator.y = 4096;
	}
	if(guidance_h_trim_att_integrator.y<-4096)
	{
		guidance_h_trim_att_integrator.y = -4096;
	}

		
	  float acc_refx = (guidance_h_accel_ref_fake.x>> INT32_ACCEL_FRAC);
	  float z8 = guidance_h_trim_att_integrator.x>>INT32_POS_FRAC;
	  float dz8 = guidance_h_pos_err_fake.x>> INT32_POS_FRAC;
	  float ddz8 = guidance_h_speed_err_fake.x >> INT32_ACCEL_FRAC;
	  float z9 = dz8 + k8*z8;
	  float dz9 = ddz8+ k8*dz8;
	  float z10 = k9*z9 + ddz8 + k8*dz8;
	  float ux = (m/((U-1000)*(U-1000)))*(k10*z10 + k9*dz9 + acc_refx + k8*ddz8);
	if ((ux > 0.5*4096)&&(dz8*ux>0))
	{
		integral_flag_x = 0;
	}
	else
	{
		integral_flag_x = 1;
		
	}
		


	  float acc_refy = (guidance_h_accel_ref_fake.y>> INT32_ACCEL_FRAC);
	  float z13 = guidance_h_trim_att_integrator.y>>INT32_POS_FRAC;
	  float dz13 =   guidance_h_pos_err_fake.y>> INT32_POS_FRAC;
	  float ddz13 = guidance_h_speed_err_fake.y >> INT32_ACCEL_FRAC;
	  float z11 = dz13+k13*z13;
	  float dz11 = ddz13 + k13*dz13;
	  float z12 = k11*z11 + ddz13 + k13*dz13;
	  float uy = (m/((U-1000)*(U-1000)))*(k12*z12 + k11*dz11 + acc_refy + k13*ddz13);
	if ((uy > 0.5*4096)&&(dz13*uy>0))
	{
		integral_flag_y = 0;
	}
	else
	{
		integral_flag_y = 1;
		
	}
  */

	  float acc_refx = (guidance_h_accel_ref_fake.x>> INT32_ACCEL_FRAC);
	  float speed_errx = (guidance_h_speed_err_fake.x>> INT32_ACCEL_FRAC);
	  float z9 = guidance_h_pos_err_fake.x>> INT32_POS_FRAC;// it's an int32 value
	  float k9 =100;
	  float x10 = (guidance_h_speed_ref_fake.x >> INT32_ACCEL_FRAC) + k9*z9;
	  float z10 = x10- ((guidance_h_speed_ref_fake.x-guidance_h_speed_err_fake.x)>> INT32_ACCEL_FRAC);
	  float k10 =200;
	
	  float ux = (m/((U-1000)*(U-1000)))*(acc_refx + k9*speed_errx + z9 + k10*z10);
	  
	  float tempa= (ux - sinf(att_at.phi)*sinf(att_at.psi))/(cosf(att_at.phi)*cosf(att_at.psi));
	  //struct FloatEulars *att_at = stateGetNedToBodyEulers_f();
	
	  if (tempa>=1)
	    {
	 	tempa= 1;
	     }
	 if (tempa<=-1)
	    {
	 	tempa= -1;
	     }
	  float ttt =  asin(0);
	  
	  stab_att_sp_euler.theta = -4096* ttt;
	  //stab_att_sp_euler.psi = 0;//4096*z10;
	   //stab_att_sp_euler.phi = 0;//4096*tempa;

	    if (ttt>=0.5)
	    {
	 	stab_att_sp_euler.theta = -4096*0.5;
	     }
	 if (ttt<=-0.5)
	    {
	 	stab_att_sp_euler.theta = 4096*0.5;
	     }
	     
	     

	  float z11 = guidance_h_pos_err_fake.y>> INT32_POS_FRAC;
	  float k11 =k9;	


																																																																																																																																												
	  float x12 = (guidance_h_speed_ref_fake.y>> INT32_ACCEL_FRAC) + k11*z11;
	  float z12 = x12 - ((guidance_h_speed_ref_fake.y-guidance_h_speed_err_fake.y)>> INT32_ACCEL_FRAC);
	  float k12 = k10;
	  float acc_refy = guidance_h_accel_ref_fake.y>>INT32_ACCEL_FRAC;
	  float speed_erry = guidance_h_speed_err_fake.y>> INT32_ACCEL_FRAC;
	  float uy = (m/((U-1000)*(U-1000)))*(acc_refy + k11*speed_erry + z11 + k12*z12);
	  
	  float  temp=ux*sinf(att_at.psi) - uy*cosf(att_at.psi);
	  
	  if (temp>=1)
	    {
	 	temp = 1;
	     }
	 if (temp<=-1)
	    {
	 	temp = -1;
	     }
	  ttt =  asin(0); // don't know is there any arcsin function. 
	  
		stab_att_sp_euler.phi = -4096*ttt;
		
		  if (ttt>=1)
		  {
		  stab_att_sp_euler.phi = -4096*1;
		   }
	   if (ttt<=-1)
		  {
		  stab_att_sp_euler.phi = 4096*1;
		   }
		
	guidance_h_cmd_earth.x = ux;
	guidance_h_cmd_earth.y = uy; 
  /* run PID */
  int32_t pd_x =
    ((guidance_h_pgain * guidance_h_pos_err.x) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_h_dgain * (guidance_h_speed_err.x >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
  int32_t pd_y =
    ((guidance_h_pgain * guidance_h_pos_err.y) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_h_dgain * (guidance_h_speed_err.y >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
  guidance_h_cmd_earth.x =
    pd_x +
    ((guidance_h_vgain * guidance_h_speed_ref.x) >> 17) + /* speed feedforward gain */
    ((guidance_h_again * guidance_h_accel_ref.x) >> 8);   /* acceleration feedforward gain */
  guidance_h_cmd_earth.y =
    pd_y +
    ((guidance_h_vgain * guidance_h_speed_ref.y) >> 17) + /* speed feedforward gain */
    ((guidance_h_again * guidance_h_accel_ref.y) >> 8);   /* acceleration feedforward gain */

  /* trim max bank angle from PD */
  VECT2_STRIM(guidance_h_cmd_earth, -traj_max_bank, traj_max_bank);

  /* Update pos & speed error integral, zero it if not in_flight.
   * Integrate twice as fast when not only POS but also SPEED are wrong,
   * but do not integrate POS errors when the SPEED is already catching up.
   */
  if (in_flight) {
    /* ANGLE_FRAC (12) * GAIN (8) * LOOP_FREQ (9) -> INTEGRATOR HIGH RES ANGLE_FRAX (28) */
    guidance_h_trim_att_integrator.x += (guidance_h_igain * pd_x);
    guidance_h_trim_att_integrator.y += (guidance_h_igain * pd_y);
    /* saturate it  */
    VECT2_STRIM(guidance_h_trim_att_integrator, -(traj_max_bank << 16), (traj_max_bank << 16));
    /* add it to the command */
    guidance_h_cmd_earth.x += (guidance_h_trim_att_integrator.x >> 16);
    guidance_h_cmd_earth.y += (guidance_h_trim_att_integrator.y >> 16);
  } else {
    INT_VECT2_ZERO(guidance_h_trim_att_integrator);
  }

  /* compute a better approximation of force commands by taking thrust into account */
  if (guidance_h_approx_force_by_thrust && in_flight) {
    static int32_t thrust_cmd_filt;
    int32_t vertical_thrust = (stabilization_cmd[COMMAND_THRUST] * guidance_v_thrust_coeff) >> INT32_TRIG_FRAC;
    thrust_cmd_filt = (thrust_cmd_filt * GUIDANCE_H_THRUST_CMD_FILTER + vertical_thrust) /
                      (GUIDANCE_H_THRUST_CMD_FILTER + 1);
    guidance_h_cmd_earth.x = ANGLE_BFP_OF_REAL(atan2f((guidance_h_cmd_earth.x * MAX_PPRZ / INT32_ANGLE_PI_2),
                             thrust_cmd_filt));
    guidance_h_cmd_earth.y = ANGLE_BFP_OF_REAL(atan2f((guidance_h_cmd_earth.y * MAX_PPRZ / INT32_ANGLE_PI_2),
                             thrust_cmd_filt));
  }

  VECT2_STRIM(guidance_h_cmd_earth, -total_max_bank, total_max_bank);  //angle is 45` limited
}

static void guidance_h_hover_enter(void)
{

  /* set horizontal setpoint to current position */
  VECT2_COPY(guidance_h_pos_sp, *stateGetPositionNed_i());
  
  reset_guidance_reference_from_current_position();
  
  guidance_h_rc_sp.psi = stateGetNedToBodyEulers_i()->psi;
#if USE_FLOW  
  flow_hff_init(0.0, 0.0, 0.0, 0.0);//zero to pos and vel,later flow filter update when hover running
#endif 
}

static void guidance_h_nav_enter(void)
{

  /* horizontal position setpoint from navigation/flightplan */
  INT32_VECT2_NED_OF_ENU(guidance_h_pos_sp, navigation_carrot);

  reset_guidance_reference_from_current_position();

  nav_heading = stateGetNedToBodyEulers_i()->psi;
}

void guidance_h_nav_rc_enter(void)
{

  /* set horizontal setpoint to current position */
  VECT2_COPY(guidance_h_pos_sp, *stateGetPositionNed_i());

  reset_guidance_reference_from_current_position();

  nav_heading = stateGetNedToBodyEulers_i()->psi;
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
    DeadBand(rc_x, MAX_PPRZ / 20);
    DeadBand(rc_y, MAX_PPRZ / 20);

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
