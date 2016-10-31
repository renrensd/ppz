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
 * @file stabilization_attitude_euler_float.c
 *
 * Rotorcraft attitude stabilization in euler float version.
 */

#include "generated/airframe.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"

#include "std.h"
#include "paparazzi.h"
#include "math/pprz_algebra_float.h"
#include "state.h"
#include "mcu_periph/sys_time.h"

#ifndef USE_ATT_REF
#define USE_ATT_REF 1
#endif

struct FloatAttitudeGains stabilization_gains;
struct FloatEulers stab_d_rate_sum_err;

struct FloatEulers stab_att_sp_euler;
struct AttRefEulerFloat att_ref_euler_f;

float stabilization_att_fb_cmd[COMMANDS_NB];
struct FloatRates desired_rate;

static bool_t stab_sum_flag;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_att(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatRates *body_rate = stateGetBodyRates_f();
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  //float foo = 0.0;
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  pprz_msg_send_STAB_ATTITUDE_FLOAT(trans, dev, AC_ID,
                                    &(body_rate->p), &(body_rate->q), &(body_rate->r),
                                    &(att->phi), &(att->theta), &(att->psi),
                                    &stab_att_sp_euler.phi,
                                    &stab_att_sp_euler.theta,
                                    &stab_att_sp_euler.psi,
                                    &stab_d_rate_sum_err.phi,
                                    &stab_d_rate_sum_err.theta,
                                    &stab_d_rate_sum_err.psi,
                                    &stabilization_att_fb_cmd[COMMAND_ROLL],
                                    &stabilization_att_fb_cmd[COMMAND_PITCH],
                                    &stabilization_att_fb_cmd[COMMAND_YAW],
                                    &desired_rate.p,
                                    &desired_rate.q,
                                    &desired_rate.r,
                                    &stabilization_cmd[COMMAND_ROLL],
                                    &stabilization_cmd[COMMAND_PITCH],
                                    &stabilization_cmd[COMMAND_YAW]);
}

static void send_att_ref(struct transport_tx *trans, struct link_device *dev)
{
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  pprz_msg_send_STAB_ATTITUDE_REF_FLOAT(trans, dev, AC_ID,
                                        &stab_att_sp_euler.phi,
                                        &stab_att_sp_euler.theta,
                                        &stab_att_sp_euler.psi,
                                        &att_ref_euler_f.euler.phi,
                                        &att_ref_euler_f.euler.theta,
                                        &att_ref_euler_f.euler.psi,
                                        &att_ref_euler_f.rate.p,
                                        &att_ref_euler_f.rate.q,
                                        &att_ref_euler_f.rate.r,
                                        &att_ref_euler_f.accel.p,
                                        &att_ref_euler_f.accel.q,
                                        &att_ref_euler_f.accel.r);
}
#endif

void stabilization_attitude_init(void)
{
  stab_sum_flag = FALSE;
  attitude_ref_euler_float_init(&att_ref_euler_f);

  VECT3_ASSIGN(stabilization_gains.p,
               STABILIZATION_ATTITUDE_PHI_PGAIN,
               STABILIZATION_ATTITUDE_THETA_PGAIN,
               STABILIZATION_ATTITUDE_PSI_PGAIN);
  /*
  VECT3_ASSIGN(stabilization_gains.d,
               STABILIZATION_ATTITUDE_PHI_DGAIN,
               STABILIZATION_ATTITUDE_THETA_DGAIN,
               STABILIZATION_ATTITUDE_PSI_DGAIN);
  VECT3_ASSIGN(stabilization_gains.dd,
               STABILIZATION_ATTITUDE_PHI_DDGAIN,
               STABILIZATION_ATTITUDE_THETA_DDGAIN,
               STABILIZATION_ATTITUDE_PSI_DDGAIN);
  */

  VECT3_ASSIGN(stabilization_gains.p_rate,
               STABILIZATION_ATTITUDE_PHIRATE_PGAIN,
               STABILIZATION_ATTITUDE_THETARATE_PGAIN,
               STABILIZATION_ATTITUDE_PSIRATE_PGAIN);

  VECT3_ASSIGN(stabilization_gains.d_rate,
               STABILIZATION_ATTITUDE_PHIRATE_DGAIN,
               STABILIZATION_ATTITUDE_THETARATE_DGAIN,
               STABILIZATION_ATTITUDE_PSIRATE_DGAIN);

  VECT3_ASSIGN(stabilization_gains.i_rate,
               STABILIZATION_ATTITUDE_PHIRATE_IGAIN,
               STABILIZATION_ATTITUDE_THETARATE_IGAIN,
               STABILIZATION_ATTITUDE_PSIRATE_IGAIN);

  VECT3_ASSIGN(stabilization_gains.ff_rate,
               STABILIZATION_ATTITUDE_PHIRATE_FFGAIN,
               STABILIZATION_ATTITUDE_THETARATE_FFGAIN,
               STABILIZATION_ATTITUDE_PSIRATE_FFGAIN);

  FLOAT_EULERS_ZERO(stab_d_rate_sum_err);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_FLOAT, send_att);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_REF_FLOAT, send_att_ref);
#endif
}

void stabilization_attitude_read_rc(bool_t in_flight, bool_t in_carefree, bool_t coordinated_turn)
{
  stabilization_attitude_read_rc_setpoint_eulers_f(&stab_att_sp_euler, in_flight, in_carefree, coordinated_turn);
}

void stabilization_attitude_enter(void)
{

  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = stabilization_attitude_get_heading_f();

  attitude_ref_euler_float_enter(&att_ref_euler_f, stab_att_sp_euler.psi);

  FLOAT_EULERS_ZERO(stab_d_rate_sum_err);
}

void stabilization_attitude_set_failsafe_setpoint(void)
{
  stab_att_sp_euler.phi = 0.0;
  stab_att_sp_euler.theta = 0.0;
  stab_att_sp_euler.psi = stateGetNedToBodyEulers_f()->psi;
}

void stabilization_attitude_set_rpy_setpoint_i(struct Int32Eulers *rpy)
{
  EULERS_FLOAT_OF_BFP(stab_att_sp_euler, *rpy);
}

void stabilization_attitude_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading)
{
  struct FloatVect2 cmd_f;
  cmd_f.x = ANGLE_FLOAT_OF_BFP(cmd->x);
  cmd_f.y = ANGLE_FLOAT_OF_BFP(cmd->y);

  /* Rotate horizontal commands to body frame by psi */
  float psi = stateGetNedToBodyEulers_f()->psi;
  float s_psi = sinf(psi);
  float c_psi = cosf(psi);
  stab_att_sp_euler.phi = -s_psi * cmd_f.x + c_psi * cmd_f.y;
  stab_att_sp_euler.theta = -c_psi * cmd_f.x - s_psi * cmd_f.y;
  stab_att_sp_euler.psi = ANGLE_FLOAT_OF_BFP(heading);
}



#define MAX_SUM_ERR 1000

#define MAX_DESIRED_RATE 100.0
void stabilization_attitude_run(bool_t  in_flight)
{

#if USE_ATT_REF
  static const float _dt = (1./PERIODIC_FREQUENCY);
  attitude_ref_euler_float_update(&att_ref_euler_f, &stab_att_sp_euler, _dt);
  att_ref_euler_f.euler.phi = stab_att_sp_euler.phi;
  att_ref_euler_f.euler.theta = stab_att_sp_euler.theta;
#else
  EULERS_COPY(att_ref_euler_f.euler, stab_att_sp_euler);
  FLOAT_RATES_ZERO(att_ref_euler_f.rate);
  FLOAT_RATES_ZERO(att_ref_euler_f.accel);
#endif

  /*below is double control loop from apm2.9*/
   float scaler = 1.0;

   static uint32_t _last_t = 0; 
	uint32_t tnow = get_sys_time_usec();
	int32_t dt = tnow - _last_t;
	if (dt < 0 || dt > 10000) {
		dt = 1000000/PERIODIC_FREQUENCY;
	}
	_last_t = tnow;
	float delta_time = (float)dt / 1000000.0;  //unit:second

   struct FloatEulers att_float = *stateGetNedToBodyEulers_f();
   struct FloatEulers att_err;
   EULERS_DIFF(att_err, att_ref_euler_f.euler, att_float);
   FLOAT_ANGLE_NORMALIZE(att_err.psi);

   /*rate filter*/
	const float RC = 1.0/(2*M_PI*20);  //use 20hz low filter
	const float s = delta_time / (RC + delta_time);
	static struct FloatRates _last_rate;
	struct FloatRates rate_float = *stateGetBodyRates_f();	
	struct FloatRates rate_err;
	RATES_DIFF(rate_err, rate_float, _last_rate);
	RATES_SUM_SCALED(rate_float, _last_rate, rate_err, s);
	RATES_COPY(_last_rate, rate_float);

	desired_rate.p = stabilization_gains.p.x  * att_err.phi;
	desired_rate.q = stabilization_gains.p.y  * att_err.theta;
	desired_rate.r = stabilization_gains.p.z  * att_err.psi;
	BoundAbs(desired_rate.p, MAX_DESIRED_RATE);
	BoundAbs(desired_rate.q, MAX_DESIRED_RATE);
	BoundAbs(desired_rate.r, MAX_DESIRED_RATE);

	//struct FloatVect3 rate_err;
	rate_err.p = desired_rate.p - rate_float.p;
	rate_err.q = desired_rate.q - rate_float.q;
	rate_err.r = desired_rate.r - rate_float.r;

	//struct FloatVect3 fb_cmd;
	//stabilization_att_fb_cmd[COMMAND_ROLL] = (stabilization_gains.d.x * rate_err.p  + stabilization_gains.dd.x * desired_rate.p); 
	//stabilization_att_fb_cmd[COMMAND_PITCH] = (stabilization_gains.d.y * rate_err.q  + stabilization_gains.dd.y * desired_rate.q); 
	//stabilization_att_fb_cmd[COMMAND_YAW] = (stabilization_gains.d.z * rate_err.r  + stabilization_gains.dd.z * desired_rate.r); 

	//cpz-rate_loop-P+D
	static struct FloatRates rate_float_prev;//???\BF??\C7\CB?\C8???
	static struct FloatRates desired_rate_prev;//???\BF??\C7\CB??\F8\B6\A8?

	const float inv_deta_t = 1.0/delta_time;
	
	stabilization_att_fb_cmd[COMMAND_ROLL] = stabilization_gains.p_rate.x * rate_err.p
		            + stabilization_gains.d_rate.x*(rate_float_prev.p-rate_float.p)*inv_deta_t 
		            + stabilization_gains.ff_rate.x*(desired_rate.p-desired_rate_prev.p)*inv_deta_t ;
	
	stabilization_att_fb_cmd[COMMAND_PITCH] = stabilization_gains.p_rate.y * rate_err.q 
		            + stabilization_gains.d_rate.y*(rate_float_prev.q-rate_float.q)*inv_deta_t  
		            + stabilization_gains.ff_rate.y*(desired_rate.q-desired_rate_prev.q)*inv_deta_t ;
	
	stabilization_att_fb_cmd[COMMAND_YAW] = stabilization_gains.p_rate.z * rate_err.r 
		            + stabilization_gains.d_rate.z*(rate_float_prev.r-rate_float.r)*inv_deta_t 
		            + stabilization_gains.ff_rate.z*(desired_rate.r-desired_rate_prev.r)*inv_deta_t ;
	RATES_COPY(desired_rate_prev, desired_rate);
    RATES_COPY(rate_float_prev, rate_float);    
	//cpz-rate_loop-P+D

	
    /*caculate intergrate*/
	if(in_flight)
	{
		if( stabilization_cmd[COMMAND_THRUST] > 4500 )
		{
			stab_sum_flag = TRUE;
		}
	}
	else
	{
		stab_sum_flag = FALSE;
	}
	
	if(stab_sum_flag)
	{
		stab_d_rate_sum_err.phi   += (stabilization_gains.i_rate.x*rate_err.p) * delta_time;
		stab_d_rate_sum_err.theta += (stabilization_gains.i_rate.y*rate_err.q) * delta_time;
		stab_d_rate_sum_err.psi   += (stabilization_gains.i_rate.z*rate_err.r) * delta_time;
		/*Bound(stab_d_rate_sum_err.phi,   
			-MAX_SUM_ERR-stabilization_att_fb_cmd[COMMAND_ROLL], MAX_SUM_ERR-stabilization_att_fb_cmd[COMMAND_ROLL]);
		Bound(stab_d_rate_sum_err.theta, 
			-MAX_SUM_ERR-stabilization_att_fb_cmd[COMMAND_PITCH], MAX_SUM_ERR-stabilization_att_fb_cmd[COMMAND_PITCH]);
		Bound(stab_d_rate_sum_err.psi,   
		    -MAX_SUM_ERR-stabilization_att_fb_cmd[COMMAND_YAW], MAX_SUM_ERR-stabilization_att_fb_cmd[COMMAND_YAW]);*/
		Bound(stab_d_rate_sum_err.phi,-MAX_SUM_ERR, MAX_SUM_ERR);
		Bound(stab_d_rate_sum_err.theta,-MAX_SUM_ERR, MAX_SUM_ERR);
		Bound(stab_d_rate_sum_err.psi,-MAX_SUM_ERR, MAX_SUM_ERR);
		
	}

	stabilization_cmd[COMMAND_ROLL]  = (int32_t)( (stabilization_att_fb_cmd[COMMAND_ROLL] + stab_d_rate_sum_err.phi) * scaler);
	stabilization_cmd[COMMAND_PITCH] = (int32_t)( (stabilization_att_fb_cmd[COMMAND_PITCH] + stab_d_rate_sum_err.theta) * scaler);
	stabilization_cmd[COMMAND_YAW]   = (int32_t)( (stabilization_att_fb_cmd[COMMAND_YAW] + stab_d_rate_sum_err.psi) * scaler);

  /* bound the result */
  BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);
  #if 0
  stabilization_cmd[COMMAND_ROLL] = 0;
  stabilization_cmd[COMMAND_PITCH] = 0;
  stabilization_cmd[COMMAND_YAW] = 0;
  #endif
}
