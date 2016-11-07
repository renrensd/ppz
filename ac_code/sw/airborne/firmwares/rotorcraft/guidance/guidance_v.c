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

#ifndef GUIDANCE_V_MIN_ERR_Z
#define GUIDANCE_V_MIN_ERR_Z POS_BFP_OF_REAL(-5.)
#endif

#ifndef GUIDANCE_V_MAX_ERR_Z
#define GUIDANCE_V_MAX_ERR_Z POS_BFP_OF_REAL(5.)
#endif

#ifndef GUIDANCE_V_MIN_ERR_ZD
#define GUIDANCE_V_MIN_ERR_ZD SPEED_BFP_OF_REAL(-5.)
#endif

#ifndef GUIDANCE_V_MAX_ERR_ZD
#define GUIDANCE_V_MAX_ERR_ZD SPEED_BFP_OF_REAL(5.)
#endif

#ifndef GUIDANCE_V_MAX_SUM_ERR
#define GUIDANCE_V_MAX_SUM_ERR  1500000  //default is 2000000
#endif

#define USE_ANTI_WIND_UP TRUE
int32_t wind_up_flag = 1;

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

int32_t guidance_v_kp;
int32_t guidance_v_kd;
int32_t guidance_v_ki;

int32_t guidance_vi_kp;
int32_t guidance_vi_kd;
int32_t guidance_vi_ki;

int32_t guidance_vii_kp; //nei huan pid can shu
int32_t guidance_vii_kd;
int32_t guidance_vii_ki;

int32_t guidance_v_z_sum_err;
int32_t guidance_v_zd_sum_err;
int32_t guidance_v_zdd_sum_err; //jia su du huan jifen xiang
int32_t guidance_v_thrust_coeff;

int32_t c_z = 0; //waihuan gaodu shuchu neihuan sudu geiding
int32_t c_z_pre = 0; //qian yi shi ke
int32_t c_az = 0; //su du huan shuchu ,jia su du huan gei ding shuru
int32_t c_az_pre = 0; //qian yi shi ke
int32_t num_to_acc = 400; // 2000/num_to_acc=(+-)5m/m2
int32_t v_pre = 0;
int32_t vd_pre = 0; //su du huan weifen lvbo
int32_t vdd_pre = 0; //jiasudu huan weifen lvbo
int32_t vdd_i = 0; //jiasudu huan jifen huanjie for display only
int32_t err_vvz_pre = 0; // qian yi shi ke jiasudu wucha
int32_t acc_z_pre = 0;
int32_t acc_z_speed_pre = 0;
#define time_inv 512
int32_t inside_isum = 0;

#define GuidanceVSetRef(_pos, _speed, _accel) { \
    gv_set_ref(_pos, _speed, _accel);        \
    guidance_v_z_ref = _pos;             \
    guidance_v_zd_ref = _speed;          \
    guidance_v_zdd_ref = _accel;             \
  }

static int32_t get_vertical_thrust_coeff(void);
static void run_hover_loop(bool_t in_flight);
static void dynamic_pid_check(void);
static void guidance_v_controller_ini(void);

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

	guidance_v_kp = GUIDANCE_V_HOVER_KP;
	guidance_v_kd = GUIDANCE_V_HOVER_KD;
	guidance_v_ki = GUIDANCE_V_HOVER_KI;
	guidance_vi_kp = GUIDANCE_V_INSIDE_KP;
	guidance_vi_kd = GUIDANCE_V_INSIDE_KD;
	guidance_vi_ki = GUIDANCE_V_INSIDE_KI;
	guidance_vii_kp = GUIDANCE_V_II_KP;
	guidance_vii_kd = GUIDANCE_V_II_KD;
	guidance_vii_ki = GUIDANCE_V_II_KI;
	vh = GUIDANCE_V_TD_H;
	vh0 = GUIDANCE_V_TD_H0;
	r_vert = GUIDANCE_V_TD_R;

	guidance_v_z_sum_err = 0;

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

static guidance_v_get_temp_v_z_sp(void)
{
	guid_v.z_sp = stateGetPositionNed_i()->z;
	guid_v.ref_pos_z = - POS_FLOAT_OF_BFP(stateGetPositionNed_i()->z);
}

static void guidance_v_controller_ini(void)
{
	guid_v.acc_filter_fc = 5;
	guid_v.speed_filter_fc = 10;
	guid_v.pid_loop_mode = ACC_SPEED_POS;

	pid_ini(&guid_v.acc_z_pid, 512);
	pid_ini(&guid_v.speed_z_pid, 512);
	pid_ini(&guid_v.pos_z_pid, 512);

	pid_set_out_range(&guid_v.acc_z_pid, 0, 1);
	pid_set_Ui_range(&guid_v.acc_z_pid, 0, 1);
	pid_set_out_range(&guid_v.speed_z_pid, -5, 5);
	pid_set_Ui_range(&guid_v.speed_z_pid, -5, 5);
	pid_set_out_range(&guid_v.pos_z_pid, -3, 3);
	pid_set_Ui_range(&guid_v.pos_z_pid, -3, 3);

	guid_v.acc_z_pid.Kp = 0.02f;
	guid_v.acc_z_pid.Ki = 0.5f;
	guid_v.speed_z_pid.Kp = 3.0f;
	guid_v.speed_z_pid.Kd = 0.3f;
	guid_v.pos_z_pid.Kp = 1.0f;
	guid_v.pos_z_pid.Kd = 0.5f;
	init_butterworth_2_low_pass(&guid_v.NED_z_acc_filter, low_pass_filter_get_tau(guid_v.acc_filter_fc), 1.0f/512.0f, 0);
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
			guidance_v_get_temp_v_z_sp();
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
		guidance_v_get_temp_v_z_sp();
		guidance_v_z_sp = stateGetPositionNed_i()->z; // set current altitude as setpoint
		guidance_v_z_sum_err = 0;
		GuidanceVSetRef(stateGetPositionNed_i()->z, 0, 0)
		;
		break;

	case GUIDANCE_V_MODE_RC_CLIMB:
	case GUIDANCE_V_MODE_CLIMB:
		guidance_v_zd_sp = 0;
	case GUIDANCE_V_MODE_NAV:
		guidance_v_z_sum_err = 0;
		GuidanceVSetRef(stateGetPositionNed_i()->z, stateGetSpeedNed_i()->z, 0)
		;
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
		//if ( guidance_v_rc_delta_t > 1400)
			//stabilization_cmd[COMMAND_THRUST] = (guidance_v_rc_delta_t - 1400) / 5 * 4	+ 1400;
		//else
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
			guidance_v_z_sum_err = 0;
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

static void run_hover_loop(bool_t in_flight)
{

	/* convert our reference to generic representation */
	int64_t tmp = gv_z_ref >> (GV_Z_REF_FRAC - INT32_POS_FRAC);
	guidance_v_z_ref = (int32_t) tmp;
	guidance_v_zd_ref = gv_zd_ref << (INT32_SPEED_FRAC - GV_ZD_REF_FRAC);
	guidance_v_zdd_ref = gv_zdd_ref << (INT32_ACCEL_FRAC - GV_ZDD_REF_FRAC);
	/* compute the error to our reference */

	int32_t err_z  = guidance_v_z_ref - stateGetPositionNed_i()->z;

	Bound(err_z, GUIDANCE_V_MIN_ERR_Z, GUIDANCE_V_MAX_ERR_Z);
	int32_t err_zd = guidance_v_zd_ref - stateGetSpeedNed_i()->z;
	Bound(err_zd, GUIDANCE_V_MIN_ERR_ZD, GUIDANCE_V_MAX_ERR_ZD);

	//int32_t err_dvz = x2-stateGetAccelNed_i()->z;
	if (in_flight)
	{
		/*here is the anti_windup block */
#if USE_ANTI_WIND_UP  //use sum wind up
		if ((abs(c_z) >= 2 * 1024 * 512) || (abs(guidance_v_fb_cmd) >= 2000))
		{
			wind_up_flag = 0;
		}
		else
		{
			wind_up_flag = 1;
		}

		guidance_v_z_sum_err  += wind_up_flag*err_z;
		//guidance_v_zd_sum_err += wind_up_flag*err_vz;
#else
		guidance_v_z_sum_err += err_z;
		Bound(guidance_v_z_sum_err, -GUIDANCE_V_MAX_SUM_ERR, GUIDANCE_V_MAX_SUM_ERR);
		guidance_v_zd_sum_err+= err_vz;
#endif
		/*here is the anti_windup block ends*/

	}
	else
	{
		guidance_v_z_sum_err = 0;
		guidance_v_zd_sum_err = 0;
		guidance_v_zdd_sum_err = 0;
	}

	//filter coef calculation
	guid_v.acc_filter_coef = 2.0f * 3.14f * (1.0f/512.0f) * guid_v.acc_filter_fc;
	guid_v.speed_filter_coef = 2.0f * 3.14f * (1.0f/512.0f) * guid_v.speed_filter_fc;

	guid_v.NED_z_acc = - ACCEL_FLOAT_OF_BFP(stateGetAccelNed_i()->z);
	update_butterworth_2_low_pass(&guid_v.NED_z_acc_filter, guid_v.NED_z_acc);
	//guid_v.NED_z_acc = update_butterworth_2_low_pass(&guid_v.NED_z_acc_filter, - ACCEL_FLOAT_OF_BFP(stateGetAccelNed_i()->z));
	//guid_v.NED_z_acc = pid_simple_filter(guid_v.acc_filter_coef, guid_v.NED_z_acc, - ACCEL_FLOAT_OF_BFP(stateGetAccelNed_i()->z));
	guid_v.NED_z_speed = - SPEED_FLOAT_OF_BFP(stateGetSpeedNed_i()->z);
	//guid_v.NED_z_speed = pid_simple_filter(guid_v.speed_filter_coef, guid_v.NED_z_speed, - SPEED_FLOAT_OF_BFP(stateGetSpeedNed_i()->z));
	guid_v.NED_z_pos = - POS_FLOAT_OF_BFP(stateGetPositionNed_i()->z);

	guid_v.rc_speed_z = - SPEED_FLOAT_OF_BFP(guidance_v_rc_zd_sp);
	guid_v.rc_acc_z = 3.0f * guid_v.rc_speed_z;

	if (guid_v.pid_loop_mode == ACC_SPEED_POS)
	{
		if (guid_v.thr_in_deadband)
		{
			guid_v.pid_loop_mode_now = ACC_SPEED_POS;
		}
		else
		{
			guid_v.pid_loop_mode_now = ACC_SPEED;
		}
	}
	else
	{
		guid_v.pid_loop_mode_now = guid_v.pid_loop_mode;
	}

	if ((guid_v.pid_loop_mode_now == ACC_SPEED)
			|| (guid_v.pid_loop_mode_now == ACC_SPEED_POS))
	{
		if (guid_v.pid_loop_mode_now == ACC_SPEED_POS)
		{
			pid_loop_calc_2(&guid_v.pos_z_pid, guid_v.ref_pos_z, guid_v.NED_z_pos, 0, guid_v.NED_z_speed);
			guid_v.ref_speed_z = guid_v.pos_z_pid.out;
		}
		else
		{
			guid_v.ref_speed_z = guid_v.rc_speed_z;
		}
		//pid_loop_calc_2(&guid_v.speed_z_pid, guid_v.ref_speed_z, guid_v.NED_z_speed, 0, guid_v.NED_z_acc);
		pid_loop_calc_2(&guid_v.speed_z_pid, guid_v.ref_speed_z, guid_v.NED_z_speed, 0, get_butterworth_2_low_pass(&guid_v.NED_z_acc_filter));
		guid_v.ref_acc_z = guid_v.speed_z_pid.out;
	}
	else // RC ACC loop
	{
		guid_v.ref_acc_z = guid_v.rc_acc_z;
	}
	pid_loop_calc_2(&guid_v.acc_z_pid, guid_v.ref_acc_z, guid_v.NED_z_acc, 0, 0);

	float thrust_coef = FLOAT_OF_BFP(get_vertical_thrust_coeff(), INT32_TRIG_FRAC);
	thrust_coef = 1.0f/thrust_coef;
	Bound(thrust_coef, 0, 1.2f);

	guidance_v_delta_t = guid_v.acc_z_pid.out * thrust_coef * (0.8f * (float)MAX_PPRZ);
	/* bound the result */
	Bound(guidance_v_delta_t, 0, MAX_PPRZ);

}

/*check loop pid*/
static void dynamic_pid_check(void)  //1
{
#ifdef USE_GPS_NMEA
	if(!gps.p_stable)
	{
		guidance_v_kp = 100;
		guidance_v_kd = 145;
		guidance_v_ki = 190;
		guidance_vi_kp = 95;
		guidance_vi_kd = 75;
		guidance_vi_ki = 25;
	}
	else
	{
		guidance_v_kp = 288;
		guidance_v_kd = 280;
		guidance_v_ki = 195;
		guidance_vi_kp = 77;
		guidance_vi_kd = 77;
		guidance_vi_ki = 49;
	}
#endif
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

