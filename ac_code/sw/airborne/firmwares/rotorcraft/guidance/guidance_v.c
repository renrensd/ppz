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

/** @file firmwares/rotorcraft/guidance/guidance_v.c
 *  Vertical guidance for rotorcrafts.
 *
 */
#include <math.h>
#include "generated/airframe.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/guidance/guidance_module.h"

#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/navigation.h"

#include "state.h"
#include"subsystems/gps.h"

#include "math/pprz_algebra_int.h"
#include "controllers/pid.h"
#include "filters/low_pass_filter.h"

#define GUIDANCE_V_LOOP_FREQ	(PERIODIC_FREQUENCY)

/* error if some gains are negative */
#if (GUIDANCE_V_HOVER_KP < 0) ||                   \
  (GUIDANCE_V_HOVER_KD < 0)   ||                   \
  (GUIDANCE_V_HOVER_KI < 0)   ||                   \
  (GUIDANCE_V_INSIDE_KP < 0)  ||                   \
  (GUIDANCE_V_INSIDE_KD < 0)  ||                   \
  (GUIDANCE_V_INSIDE_KI < 0)
#error "ALL control gains must be positive!!!"
#endif

/* If only GUIDANCE_V_NOMINAL_HOVER_THROTTLE is defined,
 * disable the adaptive throttle estimation by default.
 * Otherwise enable adaptive estimation by default.
 */
#ifdef GUIDANCE_V_NOMINAL_HOVER_THROTTLE
#  ifndef GUIDANCE_V_ADAPT_THROTTLE_ENABLED
#    define GUIDANCE_V_ADAPT_THROTTLE_ENABLED FALSE
#  endif
#else
#  define GUIDANCE_V_NOMINAL_HOVER_THROTTLE 0.4
#  ifndef GUIDANCE_V_ADAPT_THROTTLE_ENABLED
#    define GUIDANCE_V_ADAPT_THROTTLE_ENABLED TRUE
#  endif
#endif
PRINT_CONFIG_VAR(GUIDANCE_V_NOMINAL_HOVER_THROTTLE)
PRINT_CONFIG_VAR(GUIDANCE_V_ADAPT_THROTTLE_ENABLED)

#define Sign(_x) ((_x) > 0 ? 1 : (-1))
#ifndef GUIDANCE_V_CLIMB_RC_DEADBAND
#define GUIDANCE_V_CLIMB_RC_DEADBAND MAX_PPRZ/10
#endif

#ifndef GUIDANCE_V_MAX_RC_CLIMB_SPEED
#define GUIDANCE_V_MAX_RC_CLIMB_SPEED GUIDANCE_V_REF_MIN_ZD
#endif

#ifndef GUIDANCE_V_MAX_RC_DESCENT_SPEED
#define GUIDANCE_V_MAX_RC_DESCENT_SPEED GUIDANCE_V_REF_MAX_ZD
#endif

uint8_t guidance_v_mode;
int32_t guidance_v_ff_cmd;
int32_t guidance_v_fb_cmd;
int32_t guidance_v_delta_t;

float guidance_v_nominal_throttle;
bool_t guidance_v_adapt_throttle_enabled;

struct _s_guidance_v guid_v;

/** Direct throttle from radio control.
 *  range 0:#MAX_PPRZ
 */
int32_t guidance_v_rc_delta_t;

/** Vertical speed setpoint from radio control.
 *  fixed point representation: Q12.19
 *  accuracy 0.0000019, range +/-4096
 */
int32_t guidance_v_rc_zd_sp;

int32_t guidance_v_z_sp;
int32_t guidance_v_zd_sp;
int32_t guidance_v_z_ref;
int32_t guidance_v_zd_ref;
int32_t guidance_v_zdd_ref;

int32_t guidance_v_thrust_coeff;

int32_t c_z = 0; //waihuan gaodu shuchu neihuan sudu geiding
int32_t c_az = 0; //su du huan shuchu ,jia su du huan gei ding shuru

#define GuidanceVSetRef(_pos, _speed, _accel) { \
    gv_set_ref(_pos, _speed, _accel);        \
    guidance_v_z_ref = _pos;             \
    guidance_v_zd_ref = _speed;          \
    guidance_v_zdd_ref = _accel;             \
  }

static int32_t get_vertical_thrust_coeff(void);
static void run_hover_loop(bool_t in_flight);
static void guidance_v_controller_ini(void);
static void state_update_loop(void);

float fsg(float x, float d);
float fhan(float signal);
void Tracking_differntiator(float signal);

/*here is define the variables that do the differentiation of my control signal in inner loop*/
//*****************************************************//
float x1 = 0;
float x2 = 0;

float vh = 0.002 * 512 * 1025;
float vh0 = 0.5 * 512 * 1025;  //0.6
float r_vert = 100;
//*****************************************************//
//                define done here                                                    //

/*here is define the functions that do the differentiation of my control signal in inner loop*/
//*****************************************************//
float fsg(float x, float d)
{
	float f;
	f = (Sign(x+d) - Sign(x - d)) / 2;
	return f;
}

inline float fhan(float signal)
{
	float d, a0, y, a1, a2, a, out;
	d = r_vert * vh0 * vh0;
	a0 = vh0 * x2;
	y = (x1 - signal) + a0;
	a1 = sqrt(d * (d + 8 * fabs(y)));
	a2 = a0 + Sign(y) * (a1 - d) / 2;
	a = (a0 + y) * fsg(y, d) + a2 * (1 - fsg(y, d));
	out = -r_vert * (a / d) * fsg(a, d) - r_vert * Sign(a) * (1 - fsg(a, d));
	return out;
}

void Tracking_differntiator(float signal)
{
	float fh;
	fh = fhan(signal);

	x2 = x2 + vh * fh;
	x1 = x1 + vh * x2;
}
//*****************************************************//
//                define done here                                                    //

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_vert_loop(struct transport_tx *trans, struct link_device *dev)
{
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	pprz_msg_send_VERT_LOOP(trans, dev, AC_ID,
			&guid_v.NED_z_acc,
			&guid_v.NED_z_speed,
			&guid_v.NED_z_pos,
			&guid_v.ref_acc_z,
			&guid_v.ref_speed_z,
			&guid_v.ref_pos_z,
			&guid_v.acc_z_pid.out,
			&guid_v.speed_z_pid.out,
			&guid_v.pos_z_pid.out);
}

static void send_tune_vert(struct transport_tx *trans, struct link_device *dev)
{
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	pprz_msg_send_TUNE_VERT(trans, dev, AC_ID,
			&guidance_v_z_sp,
			&(stateGetPositionNed_i()->z),
			&guidance_v_z_ref,
			&guidance_v_zd_ref);
}
#endif

void guidance_v_init(void)
{

	guidance_v_mode = GUIDANCE_V_MODE_KILL;

	vh = GUIDANCE_V_TD_H;
	vh0 = GUIDANCE_V_TD_H0;
	r_vert = GUIDANCE_V_TD_R;

	guidance_v_nominal_throttle = GUIDANCE_V_NOMINAL_HOVER_THROTTLE;
	guidance_v_adapt_throttle_enabled = GUIDANCE_V_ADAPT_THROTTLE_ENABLED;

	gv_adapt_init();
	guidance_v_controller_ini();
#if GUIDANCE_V_MODE_MODULE_SETTING == GUIDANCE_V_MODE_MODULE
	guidance_v_module_init();
#endif

#if PERIODIC_TELEMETRY
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VERT_LOOP, send_vert_loop);
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_TUNE_VERT, send_tune_vert);
#endif
}

static void guidance_v_controller_ini(void)
{
	guid_v.pid_loop_mode_gcs = ACC_SPEED_POS;
	guid_v.pid_loop_mode_running = ACC_SPEED_POS;

	pid_ini(&guid_v.acc_z_pid, GUIDANCE_V_LOOP_FREQ);
	pid_ini(&guid_v.speed_z_pid, GUIDANCE_V_LOOP_FREQ);
	pid_ini(&guid_v.pos_z_pid, GUIDANCE_V_LOOP_FREQ);

	pid_set_out_range(&guid_v.acc_z_pid, 0, 1);
	pid_set_Ui_range(&guid_v.acc_z_pid, 0, 1);
	pid_set_out_range(&guid_v.speed_z_pid, -3, 3);
	pid_set_Ui_range(&guid_v.speed_z_pid, -3, 3);
	pid_set_out_range(&guid_v.pos_z_pid, -2, 2);
	pid_set_Ui_range(&guid_v.pos_z_pid, -2, 2);

	guid_v.acc_z_pid.Kp = 0.01f;
	guid_v.acc_z_pid.Ki = 0.3f;
	guid_v.speed_z_pid.Kp = 3.0f;
	guid_v.speed_z_pid.Kd = 0.05f;
	guid_v.pos_z_pid.Kp = 1.5f;
	guid_v.pos_z_pid.Kd = 0.0f;

	guid_v.acc_filter_fc = 5;
	init_butterworth_2_low_pass(&guid_v.NED_z_acc_filter,
			low_pass_filter_get_tau(guid_v.acc_filter_fc), 1.0f/(float)GUIDANCE_V_LOOP_FREQ, 0);
}

void guidance_v_SetAccCutoff(float fc)
{
	guid_v.acc_filter_fc = fc;
	init_butterworth_2_low_pass(&guid_v.NED_z_acc_filter,
				low_pass_filter_get_tau(fc), 1.0f/(float)GUIDANCE_V_LOOP_FREQ, guid_v.NED_z_acc_filter.i[0]);
}

void guidance_v_read_rc(void)
{

	/* used in RC_DIRECT directly and as saturation in CLIMB and HOVER */
	guidance_v_rc_delta_t = (int32_t) radio_control.values[RADIO_THROTTLE];

	/* used in RC_CLIMB */
	guidance_v_rc_zd_sp = (MAX_PPRZ / 2)
			- (int32_t) radio_control.values[RADIO_THROTTLE];
	DeadBand(guidance_v_rc_zd_sp, GUIDANCE_V_CLIMB_RC_DEADBAND);
	if (guidance_v_rc_zd_sp == 0)
	{
		guid_v.thr_in_deadband = TRUE;
	}
	else
	{
		guid_v.thr_in_deadband = FALSE;
	}
	if (guid_v.thr_in_deadband_prev != guid_v.thr_in_deadband)
	{
		if (guid_v.thr_in_deadband)
		{
		}
	}
	guid_v.thr_in_deadband_prev = guid_v.thr_in_deadband;

	static const int32_t climb_scale = ABS(
			SPEED_BFP_OF_REAL(GUIDANCE_V_MAX_RC_CLIMB_SPEED)
					/ (MAX_PPRZ / 2 - GUIDANCE_V_CLIMB_RC_DEADBAND));
	static const int32_t descent_scale = ABS(
			SPEED_BFP_OF_REAL(GUIDANCE_V_MAX_RC_DESCENT_SPEED)
					/ (MAX_PPRZ / 2 - GUIDANCE_V_CLIMB_RC_DEADBAND));

	if (guidance_v_rc_zd_sp > 0)
	{
		guidance_v_rc_zd_sp *= descent_scale;
	}
	else
	{
		guidance_v_rc_zd_sp *= climb_scale;
	}
	/*add z_sp adjust for GUIDANCE_V_MODE_HOVER
	 *radio_control.values[RADIO_THROTTLE] divide to three band for z_sp adjust
	 *by WHP
	 */
#if 0
	static uint8_t flag_throttle=0;
	uint8_t state_throttle=0;
	if(guidance_v_mode == GUIDANCE_V_MODE_HOVER)
	{
		if(radio_control.values[RADIO_THROTTLE] <3200) state_throttle=1;
		else if(radio_control.values[RADIO_THROTTLE] <6400) state_throttle=2;
		else state_throttle=3;
		switch(state_throttle)
		{	case 1:
			flag_throttle=1; break;
			case 2:
			if(flag_throttle ==1) guidance_v_z_sp+= POS_BFP_OF_REAL(0.5); //pre state_throttle is 1, descent height
			else if(flag_throttle ==2) guidance_v_z_sp-= POS_BFP_OF_REAL(0.5);//pre state_throttle is 3,climb height
			flag_throttle=0; break;//reset state
			case 3:
			flag_throttle=2; break;
			default: break;
		}
	}
#endif
}

void guidance_v_mode_changed(uint8_t new_mode)
{

	if (new_mode == guidance_v_mode)
	{
		return;
	}

	switch (new_mode)
	{
	case GUIDANCE_V_MODE_HOVER:
	case GUIDANCE_V_MODE_GUIDED:
		guid_v.rc_pos_z = guid_v.NED_z_pos;
		guid_v.acc_z_pid.Ui = (float)guidance_v_rc_delta_t * (1.0f/(float)MAX_PPRZ) * (1.0f/0.8f);
		guidance_v_z_sp = stateGetPositionNed_i()->z; // set current altitude as setpoint
		GuidanceVSetRef(guidance_v_z_sp, 0, 0);
		break;
	case GUIDANCE_V_MODE_RC_CLIMB:
	case GUIDANCE_V_MODE_CLIMB:
		guidance_v_zd_sp = 0;
	case GUIDANCE_V_MODE_NAV:
		GuidanceVSetRef(stateGetPositionNed_i()->z, stateGetSpeedNed_i()->z, 0);
		break;

#if GUIDANCE_V_MODE_MODULE_SETTING == GUIDANCE_V_MODE_MODULE
		case GUIDANCE_V_MODE_MODULE:
		guidance_v_module_enter();
		break;
#endif

	case GUIDANCE_V_MODE_FLIP:
		break;

	default:
		break;

	}
	guidance_v_mode = new_mode;
}

void guidance_v_notify_in_flight(bool_t in_flight)
{
	if (in_flight)
	{
		gv_adapt_init();
	}
}

void guidance_v_run(bool_t in_flight)
{
	state_update_loop();
	// FIXME... SATURATIONS NOT TAKEN INTO ACCOUNT
	// AKA SUPERVISION and co
	guidance_v_thrust_coeff = get_vertical_thrust_coeff();
	if (in_flight)
	{
		int32_t vertical_thrust = (stabilization_cmd[COMMAND_THRUST]
				* guidance_v_thrust_coeff) >> INT32_TRIG_FRAC;
		gv_adapt_run(stateGetAccelNed_i()->z, vertical_thrust, guidance_v_zd_ref);
	}
	else
	{
		/* reset estimate while not in_flight */
		gv_adapt_init();
	}

	switch (guidance_v_mode)
	{

	case GUIDANCE_V_MODE_RC_DIRECT:
		guidance_v_z_sp = stateGetPositionNed_i()->z; // for display only
		if ( guidance_v_rc_delta_t > 1400)
			stabilization_cmd[COMMAND_THRUST] = (guidance_v_rc_delta_t - 1400) / 5 * 4	+ 1400;
		else
			stabilization_cmd[COMMAND_THRUST] = guidance_v_rc_delta_t;
		break;

	case GUIDANCE_V_MODE_RC_CLIMB:
		guidance_v_zd_sp = guidance_v_rc_zd_sp;
		gv_update_ref_from_zd_sp(guidance_v_zd_sp, stateGetPositionNed_i()->z);
		run_hover_loop(in_flight);
		stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
		break;

	case GUIDANCE_V_MODE_CLIMB:
		gv_update_ref_from_zd_sp(guidance_v_zd_sp, stateGetPositionNed_i()->z);
		run_hover_loop(in_flight);
#if !NO_RC_THRUST_LIMIT
		/* use rc limitation if available */
		if (radio_control.status == RC_OK)
		{
			stabilization_cmd[COMMAND_THRUST] = Min(guidance_v_rc_delta_t, guidance_v_delta_t);
		}
		else
#endif
		stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
		break;

	case GUIDANCE_V_MODE_HOVER:
	case GUIDANCE_V_MODE_GUIDED:
		guidance_v_zd_sp = 0;
		gv_update_ref_from_z_sp(guidance_v_z_sp);
		run_hover_loop(in_flight);
#if 0//!NO_RC_THRUST_LIMIT   //TODOM:throttle using to adjust height
		/* use rc limitation if available */
		if (radio_control.status == RC_OK)
		{
			stabilization_cmd[COMMAND_THRUST] = Min(guidance_v_rc_delta_t, guidance_v_delta_t);
		}
		else
#endif
		stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
		break;

#if GUIDANCE_V_MODE_MODULE_SETTING == GUIDANCE_V_MODE_MODULE
		case GUIDANCE_V_MODE_MODULE:
		guidance_v_module_run(in_flight);
		break;
#endif

	case GUIDANCE_V_MODE_NAV:
	{
		if (vertical_mode == VERTICAL_MODE_ALT)
		{
			guidance_v_z_sp = -nav_flight_altitude;
			guidance_v_zd_sp = 0;

            #ifdef USE_FLIGHT_HEIGHT_LIMIT
			if(nav_flight_altitude >flight_limit_height)
			{	
				guidance_v_z_sp = -flight_limit_height;
			}
            #endif
			gv_update_ref_from_z_sp(guidance_v_z_sp);
			run_hover_loop(in_flight);
		}
		else if (vertical_mode == VERTICAL_MODE_CLIMB)
		{
			guidance_v_z_sp = stateGetPositionNed_i()->z;
			guidance_v_zd_sp = -nav_climb;

#ifdef USE_FLIGHT_HEIGHT_LIMIT
			if( guidance_v_z_sp <-(flight_limit_height+50) ) //add 50,avoid in limit height dump
			{
				/*change mode to alt,and set sp_pos=flight_limit_height. once climb cmd arrive,mode will become climb*/
				vertical_mode = VERTICAL_MODE_ALT;
				nav_altitude=flight_limit_height;
				guidance_v_z_sp = -flight_limit_height;
				guidance_v_zd_sp = 0;
			}
#endif

			gv_update_ref_from_zd_sp(guidance_v_zd_sp, guidance_v_z_sp);
			run_hover_loop(in_flight);
		}
		else if (vertical_mode == VERTICAL_MODE_MANUAL)
		{
			guidance_v_z_sp = stateGetPositionNed_i()->z;
			guidance_v_zd_sp = stateGetSpeedNed_i()->z;
			GuidanceVSetRef(guidance_v_z_sp, guidance_v_zd_sp, 0);
			guidance_v_delta_t = nav_throttle;
		}
#if 0//!NO_RC_THRUST_LIMIT    //TODOM:in nav mod,no RC_thrust_limit
		/* use rc limitation if available */
		if (radio_control.status == RC_OK)
		{
			stabilization_cmd[COMMAND_THRUST] = Min(guidance_v_rc_delta_t, guidance_v_delta_t);
		}
		else
#endif
		stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
		break;
	}

	case GUIDANCE_V_MODE_FLIP:
		break;

	default:
		break;
	}
}

/// get the cosine of the angle between thrust vector and gravity vector
static int32_t get_vertical_thrust_coeff(void)
{
	// cos(30°) = 0.8660254
	static const int32_t max_bank_coef = BFP_OF_REAL(0.8660254f, INT32_TRIG_FRAC);

	struct Int32RMat *att = stateGetNedToBodyRMat_i();
	/* thrust vector:
	 *  int32_rmat_vmult(&thrust_vect, &att, &zaxis)
	 * same as last colum of rmat with INT32_TRIG_FRAC
	 * struct Int32Vect thrust_vect = {att.m[2], att.m[5], att.m[8]};
	 *
	 * Angle between two vectors v1 and v2:
	 *  angle = acos(dot(v1, v2) / (norm(v1) * norm(v2)))
	 * since here both are already of unit length:
	 *  angle = acos(dot(v1, v2))
	 * since we we want the cosine of the angle we simply need
	 *  thrust_coeff = dot(v1, v2)
	 * also can be simplified considering: v1 is zaxis with (0,0,1)
	 *  dot(v1, v2) = v1.z * v2.z = v2.z
	 */
	int32_t coef = att->m[8];
	if (coef < max_bank_coef)
	{
		coef = max_bank_coef;
	}
	return coef;
}

#define FF_CMD_FRAC 18

#define MAX_MONITOR_ERROR_Z  256  //1m

uint8_t get_error_z(void)
{
	if (abs(guidance_v_z_ref - stateGetPositionNed_i()->z) > MAX_MONITOR_ERROR_Z)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

static void state_update_loop(void)
{
	guid_v.NED_z_acc = - ACCEL_FLOAT_OF_BFP(stateGetAccelNed_i()->z);
	guid_v.NED_z_speed = - SPEED_FLOAT_OF_BFP(stateGetSpeedNed_i()->z);
	guid_v.NED_z_pos = - POS_FLOAT_OF_BFP(stateGetPositionNed_i()->z);

	update_butterworth_2_low_pass(&guid_v.NED_z_acc_filter, guid_v.NED_z_acc);


	guid_v.rc_speed_z = - SPEED_FLOAT_OF_BFP(guidance_v_rc_zd_sp);
	guid_v.rc_acc_z = 3.0f * guid_v.rc_speed_z;
	if(!guid_v.thr_in_deadband)
	{
		guid_v.rc_pos_z += guid_v.rc_speed_z * guid_v.pos_z_pid.dT;
	}

	guid_v.thrust_coef = FLOAT_OF_BFP(get_vertical_thrust_coeff(), INT32_TRIG_FRAC);
	guid_v.thrust_coef = 1.0f / guid_v.thrust_coef;
	Bound(guid_v.thrust_coef, 0, 1.2f);
}

static void run_hover_loop(bool_t in_flight)
{
	/* convert our reference to generic representation */
	int64_t tmp = gv_z_ref >> (GV_Z_REF_FRAC - INT32_POS_FRAC);
	guidance_v_z_ref = (int32_t) tmp;
	guidance_v_zd_ref = gv_zd_ref << (INT32_SPEED_FRAC - GV_ZD_REF_FRAC);
	guidance_v_zdd_ref = gv_zdd_ref << (INT32_ACCEL_FRAC - GV_ZDD_REF_FRAC);
	/* compute the error to our reference */

	if (autopilot_motors_on)
	{
		if (guidance_v_mode == GUIDANCE_V_MODE_HOVER)
		{
			guid_v.pid_loop_mode_running = guid_v.pid_loop_mode_gcs;
			if ((guid_v.pid_loop_mode_running == ACC_SPEED) || (guid_v.pid_loop_mode_running == ACC_SPEED_POS))
			{
				if (guid_v.pid_loop_mode_running == ACC_SPEED_POS)
				{
					guid_v.ref_pos_z = guid_v.rc_pos_z;
					pid_loop_calc_2(&guid_v.pos_z_pid, guid_v.ref_pos_z, guid_v.NED_z_pos, 0, guid_v.NED_z_speed);
					guid_v.ref_speed_z = guid_v.pos_z_pid.out;
				}
				else
				{
					guid_v.ref_speed_z = guid_v.rc_speed_z;
				}
				//pid_loop_calc_2(&guid_v.speed_z_pid, guid_v.ref_speed_z, guid_v.NED_z_speed, 0, guid_v.NED_z_acc);
				pid_loop_calc_2(&guid_v.speed_z_pid, guid_v.ref_speed_z, guid_v.NED_z_speed, 0,
						get_butterworth_2_low_pass(&guid_v.NED_z_acc_filter));
				guid_v.ref_acc_z = guid_v.speed_z_pid.out;
			}
			else // RC ACC loop
			{
				guid_v.ref_acc_z = guid_v.rc_acc_z;
			}
			pid_loop_calc_2(&guid_v.acc_z_pid, guid_v.ref_acc_z, guid_v.NED_z_acc, 0, 0);

			guidance_v_delta_t = guid_v.acc_z_pid.out * guid_v.thrust_coef * (0.8f * (float) MAX_PPRZ);
			Bound(guidance_v_delta_t, 0, MAX_PPRZ);
		}
		else if (guidance_v_mode == GUIDANCE_V_MODE_NAV)
		{
			guid_v.ref_pos_z = - DOUBLE_OF_BFP(gv_z_ref, GV_Z_REF_FRAC);
			pid_loop_calc_2(&guid_v.pos_z_pid, guid_v.ref_pos_z, guid_v.NED_z_pos, 0, guid_v.NED_z_speed);
			if (vertical_mode == VERTICAL_MODE_ALT)
			{
				guid_v.ref_speed_z = guid_v.pos_z_pid.out;
			}
			else if (vertical_mode == VERTICAL_MODE_CLIMB)
			{
				guid_v.ref_speed_z = - FLOAT_OF_BFP(gv_zd_ref, GV_ZD_REF_FRAC);
			}
			else
			{
				guid_v.ref_speed_z = 0;
			}
			pid_loop_calc_2(&guid_v.speed_z_pid, guid_v.ref_speed_z, guid_v.NED_z_speed, 0,
									get_butterworth_2_low_pass(&guid_v.NED_z_acc_filter));
			guid_v.ref_acc_z = guid_v.speed_z_pid.out;
			pid_loop_calc_2(&guid_v.acc_z_pid, guid_v.ref_acc_z, guid_v.NED_z_acc, 0, 0);

			guidance_v_delta_t = guid_v.acc_z_pid.out * guid_v.thrust_coef * (0.8f * (float) MAX_PPRZ);
			Bound(guidance_v_delta_t, 0, MAX_PPRZ);
		}
		else
		{
			guidance_v_delta_t = 0;
		}
	}
	else
	{
		pid_reset(&guid_v.acc_z_pid);
		guidance_v_delta_t = 0;
	}
}

bool_t guidance_v_set_guided_z(float z)
{
	if (guidance_v_mode == GUIDANCE_V_MODE_GUIDED)
	{
		guidance_v_z_sp = POS_BFP_OF_REAL(z);
		return TRUE;
	}
	return FALSE;
}

