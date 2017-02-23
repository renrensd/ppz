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
#include "math/my_math.h"
#include "modules/ins/ins_ublox.h"
#include "subsystems/gps.h"
#include "subsystems/ins/ins_int.h"

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

#define PID_LOOP_MAX_TILT		(my_math_deg_to_rad * 10.0f)

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

struct Int32Vect2 guidance_h_trim_att_integrator;
struct Int32Vect2  guidance_h_cmd_earth;

int32_t rc_turn_rate;    //using in HORIZONTAL_MODE_RC,cmd of psi rate


static void guidance_h_update_reference(void);
#if !GUIDANCE_INDI
#endif
static void guidance_h_hover_enter(void);
static void guidance_h_nav_enter(void);
static inline void transition_run(void);
static void read_rc_setpoint_speed_i(struct Int32Vect2 *speed_sp, bool_t in_flight);

float fsg_h(float x,float d);
int32_t fhan_control(float pos,float vel,  float repul, float h1);
float fhan_h(float signal,float x1,float x2);
static void Tracking_differntiator_hx(float signal);
static void Tracking_differntiator_hy(float signal);
static void Tracking_differntiator_reset(void);

static void guidance_h_trajectory_tracking_update_state(void);
static void guidance_h_trajectory_tracking_loop(bool_t in_flight);
static void guidance_h_trajectory_tracking_ini(void);
static void traj_pid_loop_reset(void);

#define Sign(_x) ((_x) > 0 ? 1 : (-1))

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

static void Tracking_differntiator_hx(float signal)
{
	float fh;
	fh = fhan_h(signal,xh1,xh2);
	
	xh2 = xh2 + hh*fh;
	xh1 = xh1 + hh*xh2;
}

static void Tracking_differntiator_hy(float signal)
{
	float fh;
	fh = fhan_h(signal,yh1,yh2);
	
	yh2 = yh2 + hh*fh;
	yh1 = yh1 + hh*yh2;
}

static void Tracking_differntiator_reset(void)
{
	xh1 = 0;
	xh2 = 0;
	yh1 = 0;
	yh2 = 0;
}

//*****************************************************//
//                define done here                                                    //


#ifdef PERIODIC_TELEMETRY
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

}

static void send_href(struct transport_tx *trans, struct link_device *dev)
{
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  pprz_msg_send_GUIDANCE_H_REF_INT(trans, dev, AC_ID,
                                   &guidance_h.sp.pos.x, &guidance_h.ref.pos.x,
                                   &guidance_h.sp.speed.x, &guidance_h.ref.speed.x,
                                   &guidance_h.ref.accel.x,
                                   &guidance_h.sp.pos.y, &guidance_h.ref.pos.y,
                                   &guidance_h.sp.speed.y, &guidance_h.ref.speed.y,
                                   &guidance_h.ref.accel.y);
}

static void send_tune_hover(struct transport_tx *trans, struct link_device *dev)
{ 
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  pprz_msg_send_ROTORCRAFT_TUNE_HOVER(trans, dev, AC_ID,
                                      &traj.mode,
																			&traj.state,
																			&traj.hover_point.x,
																			&traj.hover_point.y,
																			&traj.segment.start.x,
																			&traj.segment.start.y,
																			&traj.segment.end.x,
																			&traj.segment.end.y,
																			&traj.vel_t.x,
																			&traj.vel_t.y,
																			&traj.pos_t.x,
																			&traj.pos_t.y,
																			&guidance_h.ned_vel.x,
																			&guidance_h.ned_vel.y,
																			&guidance_h.ned_pos.x,
																			&guidance_h.ned_pos.y,
																			&traj.pos_along_pid.ref,
																			&traj.pos_cross_pid.ref,
																			&traj.vel_along_pid.ref,
																			&traj.vel_cross_pid.ref,
																			&traj.cmd_t.x,
																			&traj.cmd_t.y,
																			&traj.cmd_t_comp.x,
																			&traj.cmd_t_comp.y
                                      );
}

#endif

void guidance_h_init(void)
{

  guidance_h.mode = GUIDANCE_H_MODE_KILL;
  guidance_h.use_ref = GUIDANCE_H_USE_REF;
  guidance_h.approx_force_by_thrust = GUIDANCE_H_APPROX_FORCE_BY_THRUST;

  INT_VECT2_ZERO(guidance_h.sp.pos);
  INT_VECT2_ZERO(guidance_h_trim_att_integrator);
  INT_EULERS_ZERO(guidance_h.rc_sp);
  guidance_h.sp.heading = 0;

  hh0 = GUIDANCE_H_TD_H0;
  hh = GUIDANCE_H_TD_H;
  r_h = GUIDANCE_H_TD_R;

	guidance_h.ned_acc_filter_fc = 1;
	guidance_h.ned_vel_filter_fc = 5;

	init_butterworth_2_low_pass(&guidance_h.ned_acc_x_filter, low_pass_filter_get_tau(guidance_h.ned_acc_filter_fc),
			1.0f / (float) PERIODIC_FREQUENCY, 0);
	init_butterworth_2_low_pass(&guidance_h.ned_acc_y_filter, low_pass_filter_get_tau(guidance_h.ned_acc_filter_fc),
			1.0f / (float) PERIODIC_FREQUENCY, 0);
	init_butterworth_2_low_pass(&guidance_h.ned_vel_x_filter, low_pass_filter_get_tau(guidance_h.ned_vel_filter_fc),
			1.0f / (float) PERIODIC_FREQUENCY, 0);
	init_butterworth_2_low_pass(&guidance_h.ned_vel_y_filter, low_pass_filter_get_tau(guidance_h.ned_vel_filter_fc),
			1.0f / (float) PERIODIC_FREQUENCY, 0);

	guidance_h_ned_pos_rc_need_reset();
	guidance_h.pid_loop_mode_running = POS_VEL;
	guidance_h.pid_loop_mode_gcs = POS_VEL;

  transition_percentage = 0;
  transition_theta_offset = 0;
  rc_turn_rate = 0;

  gh_ref_init();
  guidance_h_trajectory_tracking_ini();

#if GUIDANCE_H_MODE_MODULE_SETTING == GUIDANCE_H_MODE_MODULE
  guidance_h_module_init();
#endif

#ifdef PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_H_INT, send_gh);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_HOVER_LOOP, send_hover_loop);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_H_REF_INT, send_href);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_TUNE_HOVER, send_tune_hover);
#endif

#if GUIDANCE_INDI
  guidance_indi_enter();
#endif
}

void guidance_h_SetNedAccFc(float Fc)
{
	guidance_h.ned_acc_filter_fc = Fc;

	init_butterworth_2_low_pass(&guidance_h.ned_acc_x_filter, low_pass_filter_get_tau(Fc),
			1.0f / (float) PERIODIC_FREQUENCY, get_butterworth_2_low_pass(&guidance_h.ned_acc_x_filter));
	init_butterworth_2_low_pass(&guidance_h.ned_acc_y_filter, low_pass_filter_get_tau(Fc),
			1.0f / (float) PERIODIC_FREQUENCY, get_butterworth_2_low_pass(&guidance_h.ned_acc_y_filter));
}

void guidance_h_SetNedVelFc(float Fc)
{
	guidance_h.ned_vel_filter_fc = Fc;

	init_butterworth_2_low_pass(&guidance_h.ned_vel_x_filter, low_pass_filter_get_tau(Fc),
			1.0f / (float) PERIODIC_FREQUENCY, get_butterworth_2_low_pass(&guidance_h.ned_vel_x_filter));
	init_butterworth_2_low_pass(&guidance_h.ned_vel_y_filter, low_pass_filter_get_tau(Fc),
			1.0f / (float) PERIODIC_FREQUENCY, get_butterworth_2_low_pass(&guidance_h.ned_vel_y_filter));
}

static void guidance_h_state_update(bool_t in_flight)
{
	static bool_t in_flight_last = FALSE;

	struct NedCoor_f *ned_acc = stateGetAccelNed_f();
	VECT2_COPY(guidance_h.ned_acc, *ned_acc);

	struct NedCoor_f *ned_vel = stateGetSpeedNed_f();
	VECT2_COPY(guidance_h.ned_vel, *ned_vel);

	struct NedCoor_f *ned_pos = stateGetPositionNed_f();
	VECT2_COPY(guidance_h.ned_pos, *ned_pos);

	guidance_h.ned_acc_filter.x = update_butterworth_2_low_pass(&guidance_h.ned_acc_x_filter, guidance_h.ned_acc.x);
	guidance_h.ned_acc_filter.y = update_butterworth_2_low_pass(&guidance_h.ned_acc_y_filter, guidance_h.ned_acc.y);
	guidance_h.ned_vel_filter.x = update_butterworth_2_low_pass(&guidance_h.ned_vel_x_filter, guidance_h.ned_vel.x);
	guidance_h.ned_vel_filter.y = update_butterworth_2_low_pass(&guidance_h.ned_vel_y_filter, guidance_h.ned_vel.y);

	guidance_h.ned_acc = guidance_h.ned_acc_filter;
	guidance_h.ned_vel = guidance_h.ned_vel_filter;

	guidance_h.ned_vel_rc.x = SPEED_FLOAT_OF_BFP(guidance_h.sp.speed.x);
	guidance_h.ned_vel_rc.y = SPEED_FLOAT_OF_BFP(guidance_h.sp.speed.y);

	if((traj.mode == TRAJ_MODE_HOVER) && (traj.state == TRAJ_STATUS_POS))
	{
		guidance_h.ned_pos_rc.x += guidance_h.ned_vel_rc.x * (1.0f/(float)PERIODIC_FREQUENCY);
		guidance_h.ned_pos_rc.y += guidance_h.ned_vel_rc.y * (1.0f/(float)PERIODIC_FREQUENCY);
	}

	if(in_flight)
	{
		if (!in_flight_last)
		{
			guidance_h_ned_pos_rc_need_reset();
		}
	}
	in_flight_last = in_flight;

	bool_t realign_done = FALSE;
	bool_t reset = FALSE;

	if(guidance_h.ned_pos_rc_reset)
	{
		if(ins_int.gps_type == GPS_RTK)
		{
			if(!ins_int.rtk_hf_realign)
			{
				realign_done = TRUE;
			}
		}
		else
		{
			if (!ins_int.ublox_hf_realign)
			{
				realign_done = TRUE;
			}
		}

		if(realign_done)
		{
			guidance_h.ned_pos_rc_reset = FALSE;
			reset = TRUE;
		}
	}
	else if(ins_int_check_realign())
	{
		reset = TRUE;
	}

	if (reset)
	{
		guidance_h.ned_pos_rc = guidance_h.ned_pos;
		traj_pid_loop_reset();
	}
}

void guidance_h_ned_pos_rc_need_reset(void)
{
	guidance_h.ned_pos_rc_reset = TRUE;
}

/*
 * code start of trajectory tracking
 */

struct _s_trajectory_tracking traj;

static void traj_update_R(float psi)
{
	// get all rotation matrix between 3 coordinate
	traj.psi = psi;

	float cos = cosf(psi);
	float sin = sinf(psi);

	traj.R_b2i.m11 = +cos; traj.R_b2i.m12 = -sin;
	traj.R_b2i.m21 = +sin; traj.R_b2i.m22 = +cos;

	Matrix22_trans(&traj.R_i2b, &traj.R_b2i);

	Matrix22_mult(&traj.R_t2b, &traj.R_i2b, &traj.segment.R_t2i);
	Matrix22_trans(&traj.R_b2t, &traj.R_t2b);
}

static void traj_point_i2t(struct FloatVect2 *vt, struct FloatVect2 *vi)
{
	VECT2_COPY(*vt, *vi);
	if(traj.mode != TRAJ_MODE_HOVER)
	{
		VECT2_DIFF(*vt, *vt, traj.segment.start);
	}
	Rotate_vect2(vt, &traj.segment.R_i2t, vt);
}

static void traj_point_t2i(struct FloatVect2 *vi, struct FloatVect2 *vt)
{
	Rotate_vect2(vi, &traj.segment.R_t2i, vt);
	if (traj.mode != TRAJ_MODE_HOVER)
	{
		VECT2_SUM(*vi, *vi, traj.segment.start);
	}
}

static void traj_vect_i2t(struct FloatVect2 *vt, struct FloatVect2 *vi)
{
	Rotate_vect2(vt, &traj.segment.R_i2t, vi);
}

static void traj_vect_t2i(struct FloatVect2 *vi, struct FloatVect2 *vt)
{
	Rotate_vect2(vi, &traj.segment.R_t2i, vt);
}

static void traj_vect_b2t(struct FloatVect2 *vt, struct FloatVect2 *vb)
{
	Rotate_vect2(vt, &traj.R_b2t, vb);
}

static void traj_vect_t2b(struct FloatVect2 *vb, struct FloatVect2 *vt)
{
	Rotate_vect2(vb, &traj.R_t2b, vt);
}

static void traj_pid_loop_reset(void)
{
	pid_reset(&traj.vel_along_pid);
	pid_reset(&traj.vel_cross_pid);
}

static void traj_pid_loop_i_term_transform(void)
{
	struct FloatVect2 icmd;
	icmd.x = traj.vel_along_pid.Ui;
	icmd.y = traj.vel_cross_pid.Ui;

	struct _s_matrix22 R;
	Matrix22_mult(&R, &traj.segment.R_i2t, &traj.segment_last.R_t2i);
	Rotate_vect2(&icmd, &R, &icmd);

	traj.vel_along_pid.Ui = icmd.x;
	traj.vel_cross_pid.Ui = icmd.y;
}


static bool_t traj_test_distance(void)
{
	bool_t ret = FALSE;

	if (point2_distance(&guidance_h.ned_pos, &traj.segment.end) < 0.2f)
	{
		if (float_vect2_norm(&guidance_h.ned_vel) < 0.2f)
		{
			ret = TRUE;
		}
	}

	return ret;
}

static void traj_test_loop(void)
{
	if (traj.test_mode == 1)	//line
	{
		if (traj_test_distance())
		{
			guidance_h_trajectory_tracking_set_segment(traj.segment.end, traj.segment.start);
		}
	}
	else if (traj.test_mode == 2)	//square
	{
		if(traj.test_square_index == 0)
		{
			if (traj_test_distance())
			{
				guidance_h_trajectory_tracking_set_segment(traj.test_square_c2, traj.test_square_c3);
				traj.test_square_index = 1;
			}
		}
		else if (traj.test_square_index == 1)
		{
			if (traj_test_distance())
			{
				guidance_h_trajectory_tracking_set_segment(traj.test_square_c3, traj.test_square_c4);
				traj.test_square_index = 2;
			}
		}
		else if (traj.test_square_index == 2)
		{
			if (traj_test_distance())
			{
				guidance_h_trajectory_tracking_set_segment(traj.test_square_c4, traj.test_square_c1);
				traj.test_square_index = 3;
			}
		}
		else if (traj.test_square_index == 3)
		{
			if (traj_test_distance())
			{
				guidance_h_trajectory_tracking_set_segment(traj.test_square_c1, traj.test_square_c2);
				traj.test_square_index = 0;
			}
		}
	}
	else
	{

	}
}

void guidance_h_SetTrajRefSpeed(float speed)
{
	guidance_h_trajectory_tracking_set_ref_speed(speed);
}

void guidance_h_SetTrajTest(uint8_t mode)
{
	traj.test_mode = mode;
	if(mode == 1)
	{
		traj.test_line_start.x = 0;
		traj.test_line_start.y = - traj.test_length;

		traj.test_line_end.x = 0;
		traj.test_line_end.y = + traj.test_length;

		Rotate_vect2(&traj.test_line_start, &traj.R_b2i, &traj.test_line_start);
		Rotate_vect2(&traj.test_line_end, &traj.R_b2i, &traj.test_line_end);

		VECT2_SUM(traj.test_line_start, traj.test_line_start, guidance_h.ned_pos);
		VECT2_SUM(traj.test_line_end, traj.test_line_end, guidance_h.ned_pos);

		guidance_h_trajectory_tracking_set_segment(traj.test_line_start, traj.test_line_end);
	}
	else if(mode == 2)
	{
		traj.test_square_c1.x = traj.test_square_c2.x = guidance_h.ned_pos.x + traj.test_length;
		traj.test_square_c3.x = traj.test_square_c4.x = guidance_h.ned_pos.x - traj.test_length;

		traj.test_square_c2.y = traj.test_square_c3.y = guidance_h.ned_pos.y + traj.test_length;
		traj.test_square_c1.y = traj.test_square_c4.y = guidance_h.ned_pos.y - traj.test_length;

		guidance_h_trajectory_tracking_set_segment(traj.test_square_c1, traj.test_square_c2);
		traj.test_square_index = 0;
	}
	else
	{
		guidance_h_trajectory_tracking_set_hover(guidance_h.ned_pos);
		guidance_h_ned_pos_rc_need_reset();
	}
}

void guidance_h_SetEmBrake(bool_t on)
{
	guidance_h_trajectory_tracking_set_emergency_brake(on);
}

void guidance_h_SetMinBrakeLen(float len)
{
	guidance_h_trajectory_tracking_set_min_brake_len(len);
}

void guidance_h_SetMaxAcc(float acc)
{
	guidance_h_trajectory_tracking_set_max_acc(acc);
}

void guidance_h_SetEmBrakeAcc(float acc)
{
	guidance_h_trajectory_tracking_set_emergency_brake_acc(acc);
}

void guidance_h_trajectory_tracking_set_ref_speed(float speed)
{
	traj.ref_speed = speed;
	Bound(traj.ref_speed, 0.5f, 10.0f);
}

void guidance_h_trajectory_tracking_set_emergency_brake_acc(float acc)
{
	traj.emergency_brake_acc = acc;
	Bound(traj.emergency_brake_acc, 0.1f, 3.0f);
}

void guidance_h_trajectory_tracking_set_max_acc(float acc)
{
	traj.max_acc = acc;
	Bound(traj.max_acc, 0.1f, 3.0f);
}

void guidance_h_trajectory_tracking_set_min_brake_len(float len)
{
	traj.min_brake_len = len;
	Bound(traj.min_brake_len, 1.0f, 5.0f);
}

void guidance_h_trajectory_tracking_set_emergency_brake(bool_t brake)
{
	struct FloatVect2 brake_vect;
	struct FloatVect2 brake_start;
	struct FloatVect2 brake_end;
	float brake_len;
	float vt;

	if(brake)
	{
		traj.emergency_brake_cnt = 0;
		traj.emergency_brake_x = 0;
		if(traj.mode == TRAJ_MODE_HOVER)
		{
			if(point2_distance(&traj.hover_point, &guidance_h.ned_pos) > traj.min_brake_len)
			{
				guidance_h_trajectory_tracking_set_segment(guidance_h.ned_pos, traj.hover_point);
			}
		}
		traj.state = TRAJ_STATUS_BRAKE;
	}
	else
	{
		guidance_h_ned_pos_rc_need_reset();
	}
	traj.emergency_brake = brake;
}

void guidance_h_trajectory_tracking_set_hover(struct FloatVect2 point)
{
	guidance_h_trajectory_tracking_set_segment(point, point);
}

void guidance_h_trajectory_tracking_set_segment(struct FloatVect2 start, struct FloatVect2 end)
{
	static bool_t ini = FALSE;
	struct FloatVect2 seg;

	if(traj.emergency_brake)
	{
		return;
	}

	if(ini)
	{
		if(VECT2_IS_EQUAL(traj.segment.start, start) && VECT2_IS_EQUAL(traj.segment.end, end))
		{
			return;
		}
	}

	traj.segment_last = traj.segment;

	VECT2_COPY(traj.segment.start, start);
	VECT2_COPY(traj.segment.end, end);
	VECT2_DIFF(seg, end, start);
	traj.segment.length = float_vect2_norm(&seg);

	if(traj.segment.length < 0.5f)
	{
		Matrix22_set_i(&traj.segment.R_t2i);
		Matrix22_set_i(&traj.segment.R_i2t);

		traj.hover_point = traj.segment.start;

		traj.mode = TRAJ_MODE_HOVER;
	}
	else
	{
		VECT2_SDIV(seg, seg, traj.segment.length);

		VECT2_COPY(traj.segment.along, seg);
		traj.segment.cross.x = -seg.y;
		traj.segment.cross.y = +seg.x;

		float cos = traj.segment.along.x;
		float sin = traj.segment.along.y;

		traj.segment.R_t2i.m11 = +cos;
		traj.segment.R_t2i.m12 = -sin;
		traj.segment.R_t2i.m21 = +sin;
		traj.segment.R_t2i.m22 = +cos;

		Matrix22_trans(&traj.segment.R_i2t, &traj.segment.R_t2i);

		traj.mode = TRAJ_MODE_SEGMENT;
	}
	traj.state = TRAJ_STATUS_VEL;
	guidance_h_trajectory_tracking_update_state();
	traj.guid_speed = traj.vel_t.x;

	if(!ini)
	{
		ini = TRUE;
	}
	else
	{
		traj_pid_loop_i_term_transform();
	}
}

static void guidance_h_trajectory_tracking_ini(void)
{
	VECT2_ASSIGN(traj.segment.start, 0, 0);
	VECT2_ASSIGN(traj.segment.end, 0, 0);
	Matrix22_set_i(&traj.segment.R_t2i);
	Matrix22_set_i(&traj.segment.R_i2t);

	pid_ini(&traj.vel_along_pid, PERIODIC_FREQUENCY);
	pid_ini(&traj.vel_cross_pid, PERIODIC_FREQUENCY);
	pid_ini(&traj.pos_along_pid, PERIODIC_FREQUENCY);
	pid_ini(&traj.pos_cross_pid, PERIODIC_FREQUENCY);
	pid_set_out_range(&traj.vel_along_pid, -PID_LOOP_MAX_TILT, +PID_LOOP_MAX_TILT);
	pid_set_Ui_range(&traj.vel_along_pid, -PID_LOOP_MAX_TILT, +PID_LOOP_MAX_TILT);
	pid_set_out_range(&traj.vel_cross_pid, -PID_LOOP_MAX_TILT, +PID_LOOP_MAX_TILT);
	pid_set_Ui_range(&traj.vel_cross_pid, -PID_LOOP_MAX_TILT, +PID_LOOP_MAX_TILT);
	pid_set_out_range(&traj.pos_along_pid, -10, +10);
	pid_set_Ui_range(&traj.pos_along_pid, -0.5, +0.5);
	pid_set_out_range(&traj.pos_cross_pid, -5, +5);
	pid_set_Ui_range(&traj.pos_cross_pid, -0.5, +0.5);

	traj.vel_along_pid.Kp = 0.2f;
	traj.vel_along_pid.Ki = 0.02f;
	traj.vel_along_pid.Kd = 0.05f;

	traj.vel_cross_pid.Kp = 0.2f;
	traj.vel_cross_pid.Ki = 0.04f;
	traj.vel_cross_pid.Kd = 0.05f;

	traj.pos_along_pid.Kp = 0.6f;
	traj.pos_along_pid.Ki = 0.0f;
	traj.pos_along_pid.Kd = 0.2f;

	traj.pos_cross_pid.Kp = 0.8f;
	traj.pos_cross_pid.Ki = 0.0f;
	traj.pos_cross_pid.Kd = 0.25f;

	init_first_order_low_pass(&traj.thrust_cmd_filter, low_pass_filter_get_tau(1.0f), PERIODIC_FREQUENCY, 0);

	guidance_h_trajectory_tracking_set_ref_speed(3.0f);
	guidance_h_trajectory_tracking_set_max_acc(2.0f);
	guidance_h_trajectory_tracking_set_min_brake_len(2.0f);
	guidance_h_trajectory_tracking_set_emergency_brake_acc(3.0f);
	traj.test_length = 20.0f;
}

static void guidance_h_trajectory_tracking_state_machine(void)
{
	if (traj.mode == TRAJ_MODE_HOVER)
	{
		traj.state = TRAJ_STATUS_POS;
	}
	else if (traj.mode == TRAJ_MODE_SEGMENT)
	{
		float left_len = traj.segment.length - traj.pos_t.x;
		float vel_inc;

		if(traj.emergency_brake)
		{
			vel_inc = traj.emergency_brake_acc / (float) PERIODIC_FREQUENCY;
		}
		else
		{
			vel_inc = traj.max_acc / (float) PERIODIC_FREQUENCY;
			if(left_len < traj.min_brake_len)
			{
				traj.state = TRAJ_STATUS_POS;
			}
			else
			{
				traj.state = TRAJ_STATUS_VEL;
			}
		}

		if (traj.state == TRAJ_STATUS_VEL)
		{
			float ref_speed = traj.pos_along_pid.out;
			Bound(ref_speed, traj.pos_along_pid.outMin, traj.ref_speed);
			if(traj.guid_speed < ref_speed)
			{
				traj.guid_speed += vel_inc;
			}
			else
			{
				traj.guid_speed -= vel_inc;
			}
		}
		else if (traj.state == TRAJ_STATUS_BRAKE)
		{
			if (traj.guid_speed < 0)
			{
				traj.guid_speed = 0;
			}
			else if(traj.guid_speed > 0)
			{
				traj.guid_speed -= vel_inc;
			}

			if(fabsf(traj.vel_t.x) < 0.2f)
			{
				traj.emergency_brake_x += traj.pos_t.x;
				if(++traj.emergency_brake_cnt >= (2*PERIODIC_FREQUENCY))
				{
					traj.state = TRAJ_STATUS_BRAKE_DONE;
					traj.emergency_brake_x /= (float)(2*PERIODIC_FREQUENCY);
					if(fabsf(traj.emergency_brake_x - traj.pos_t.x) > 1.0f)
					{
						traj.emergency_brake_x = traj.pos_t.x;
					}
				}
			}
			else if(fabsf(traj.vel_t.x) > 0.2f)
			{
				traj.emergency_brake_cnt = 0;
			}
		}
		else if (traj.state == TRAJ_STATUS_BRAKE_DONE)
		{

		}
		else if (traj.state == TRAJ_STATUS_POS)
		{

		}
		else
		{
			traj.state = TRAJ_STATUS_POS;
		}
	}
	else
	{
		traj.mode = TRAJ_MODE_HOVER;
	}
}

static void guidance_h_trajectory_tracking_update_state(void)
{
	// NED state to trajectory
	traj_vect_i2t(&traj.acc_t, &guidance_h.ned_acc);
	traj_vect_i2t(&traj.vel_t, &guidance_h.ned_vel);
	traj_point_i2t(&traj.pos_t, &guidance_h.ned_pos);
}

static void guidance_h_trajectory_tracking_loop(bool_t in_flight)
{
	traj_update_R(stateGetNedToBodyEulers_f()->psi);

	guidance_h_trajectory_tracking_update_state();
	guidance_h_trajectory_tracking_state_machine();

	if (traj.mode == TRAJ_MODE_HOVER)
	{
		if (traj.pid_loop_test_mode == 0)
		{
			// along pid loop
			pid_loop_calc_2(&traj.pos_along_pid, traj.hover_point.x, traj.pos_t.x, 0, traj.vel_t.x);
			pid_loop_calc_2(&traj.vel_along_pid, traj.pos_along_pid.out, traj.vel_t.x, 0, traj.acc_t.x);

			// cross pid loop
			pid_loop_calc_2(&traj.pos_cross_pid, traj.hover_point.y, traj.pos_t.y, 0, traj.vel_t.y);
			pid_loop_calc_2(&traj.vel_cross_pid, traj.pos_cross_pid.out, traj.vel_t.y, 0, traj.acc_t.y);
		}
		else
		{
			struct FloatVect2 vel_rc_t;
			traj_vect_i2t(&vel_rc_t, &guidance_h.ned_vel_rc);

			pid_loop_calc_2(&traj.vel_along_pid, vel_rc_t.x, traj.vel_t.x, 0, traj.acc_t.x);
			pid_loop_calc_2(&traj.vel_cross_pid, vel_rc_t.y, traj.vel_t.y, 0, traj.acc_t.y);
		}
	}
	else
	{
		// along pid loop
		if (traj.state == TRAJ_STATUS_POS)
		{
			pid_loop_calc_2(&traj.pos_along_pid, traj.segment.length, traj.pos_t.x, 0, traj.vel_t.x);
			pid_loop_calc_2(&traj.vel_along_pid, traj.pos_along_pid.out, traj.vel_t.x, 0, traj.acc_t.x);
		}
		else if(traj.state == TRAJ_STATUS_BRAKE_DONE)
		{
			pid_loop_calc_2(&traj.pos_along_pid, traj.emergency_brake_x, traj.pos_t.x, 0, traj.vel_t.x);
			pid_loop_calc_2(&traj.vel_along_pid, traj.pos_along_pid.out, traj.vel_t.x, 0, traj.acc_t.x);
		}
		else
		{
			pid_loop_calc_2(&traj.pos_along_pid, traj.segment.length, traj.pos_t.x, 0, traj.vel_t.x);
			pid_loop_calc_2(&traj.vel_along_pid, traj.guid_speed, traj.vel_t.x, 0, traj.acc_t.x);
		}

		// cross pid loop
		pid_loop_calc_2(&traj.pos_cross_pid, 0, traj.pos_t.y, 0, traj.vel_t.y);
		pid_loop_calc_2(&traj.vel_cross_pid, traj.pos_cross_pid.out, traj.vel_t.y, 0, traj.acc_t.y);
	}

	traj.cmd_t.x = traj.vel_along_pid.out;
	traj.cmd_t.y = traj.vel_cross_pid.out;

	float thrust = update_first_order_low_pass(&traj.thrust_cmd_filter, stabilization_cmd[COMMAND_THRUST]);
	thrust = thrust / (float)MAX_PPRZ;
	if (guidance_h.approx_force_by_thrust && in_flight)
	{
		traj.cmd_t_comp.x = atan2f((traj.cmd_t.x / (my_math_pi/2.0f)), thrust);
		traj.cmd_t_comp.y = atan2f((traj.cmd_t.y / (my_math_pi/2.0f)), thrust);
	}

	//traj_vect_t2b(&traj.cmd_b, &traj.cmd_t);
	traj_vect_t2b(&traj.cmd_b, &traj.cmd_t_comp);

	if ((stateGetPositionEnu_f()->z < (DISTANCE_ABOVE_GROUNG)) || stateGetHorizontalSpeedNorm_f() > 30.0)
	{
		VECT2_STRIM(traj.cmd_b, (-PID_LOOP_MAX_TILT/4.0f), (PID_LOOP_MAX_TILT/4.0f));
	}

	VECT2_STRIM(traj.cmd_b, -PID_LOOP_MAX_TILT, PID_LOOP_MAX_TILT);
}

/*
 * code end of trajectory tracking
 */

static inline void reset_guidance_reference_from_current_position(void)
{
  VECT2_COPY(guidance_h.ref.pos, *stateGetPositionNed_i());
  VECT2_COPY(guidance_h.ref.speed, *stateGetSpeedNed_i());
  INT_VECT2_ZERO(guidance_h.ref.accel);
  gh_set_ref(guidance_h.ref.pos, guidance_h.ref.speed, guidance_h.ref.accel);

  INT_VECT2_ZERO(guidance_h_trim_att_integrator);
  traj_pid_loop_reset();
}

void guidance_h_mode_changed(uint8_t new_mode)
{
	if (new_mode == guidance_h.mode)
	{
		return;
	}

	if (new_mode != GUIDANCE_H_MODE_FORWARD && new_mode != GUIDANCE_H_MODE_RATE)
	{
		transition_percentage = 0;
		transition_theta_offset = 0;
	}

	switch (new_mode)
	{
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

void guidance_h_read_rc(bool_t in_flight)
{

	switch (guidance_h.mode)
	{

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
		if (radio_control.status == RC_OK)
		{
			stabilization_attitude_read_rc_setpoint_eulers(&guidance_h.rc_sp, in_flight, FALSE, FALSE);
		}
		else
		{
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
	guidance_h_state_update(in_flight);

	switch (guidance_h.mode)
	{
	case GUIDANCE_H_MODE_RC_DIRECT:
		stabilization_none_run(in_flight);
		break;

	case GUIDANCE_H_MODE_RATE:
		stabilization_rate_run(in_flight);
		break;

	case GUIDANCE_H_MODE_FORWARD:
		if (transition_percentage < (100 << INT32_PERCENTAGE_FRAC))
		{
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
		if(traj.test_mode == 0)
		{
			guidance_h_trajectory_tracking_set_hover(guidance_h.ned_pos_rc);
		}

	case GUIDANCE_H_MODE_GUIDED:
		/* guidance_h.sp.pos and guidance_h.sp.heading need to be set from external source */
		if (!in_flight)
		{
			guidance_h_hover_enter();
		}

		guidance_h_update_reference();

#if GUIDANCE_INDI
		guidance_indi_run(in_flight, guidance_h.sp.heading);
#else

		traj_test_loop();
		guidance_h_trajectory_tracking_loop(in_flight);
		stabilization_attitude_set_body_cmd_f(traj.cmd_b.y, -traj.cmd_b.x, ANGLE_FLOAT_OF_BFP(guidance_h.sp.heading));
#endif
		stabilization_attitude_run(in_flight);
		break;

	case GUIDANCE_H_MODE_NAV:
		if (!in_flight)
		{
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
			nav_heading += (rc_turn_rate / PERIODIC_FREQUENCY); //add deta heading
			guidance_h.sp.heading = nav_heading;

			INT32_ANGLE_NORMALIZE(guidance_h.sp.heading);

			struct FloatVect2 ref_point;
			ref_point.x = POS_FLOAT_OF_BFP(guidance_h.ref.pos.x);
			ref_point.y = POS_FLOAT_OF_BFP(guidance_h.ref.pos.y);
			guidance_h_trajectory_tracking_set_hover(ref_point);

			guidance_h_trajectory_tracking_loop(in_flight);
			stabilization_attitude_set_body_cmd_f(traj.cmd_b.y, -traj.cmd_b.x, ANGLE_FLOAT_OF_BFP(guidance_h.sp.heading));
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
			guidance_h_trajectory_tracking_loop(in_flight);
			stabilization_attitude_set_body_cmd_f(traj.cmd_b.y, -traj.cmd_b.x, ANGLE_FLOAT_OF_BFP(guidance_h.sp.heading));
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
#include "firmwares/rotorcraft/nav_flight.h"
#endif

static void guidance_h_hover_enter(void)
{
  /* set horizontal setpoint to current position */
  guidance_h.sp.heading = stateGetNedToBodyEulers_i()->psi;
  guidance_h.rc_sp.psi = stateGetNedToBodyEulers_i()->psi;

  guidance_h_ned_pos_rc_need_reset();
  guidance_h_trajectory_tracking_set_hover(guidance_h.ned_pos);

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
	guidance_h_ned_pos_rc_need_reset();
	guidance_h_trajectory_tracking_set_hover(guidance_h.ned_pos);

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
