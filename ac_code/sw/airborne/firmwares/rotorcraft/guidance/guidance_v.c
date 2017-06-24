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

#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/nav_flight.h"

#include "state.h"
#include "subsystems/gps.h"

#include "math/pprz_algebra_int.h"
#include "controllers/pid.h"
#include "filters/low_pass_filter.h"

#define GUIDANCE_V_LOOP_FREQ	(PERIODIC_FREQUENCY)

#define GUIDANCE_V_MAX_RC_CLIMB_SPEED (3)
#define GUIDANCE_V_MAX_RC_DESCENT_SPEED (1)

#define GUIDANCE_V_MAX_HEIGHT	(10.0f)
#define GUIDANCE_V_MAX_NORMALIZED_THROTTLE	(0.8f)
#define GUIDANCE_V_HOVER_MAX_NORMALIZED_THROTTLE	(0.85f)
#define GUIDANCE_V_MAX_ACC_LOOP_ERR	(0.8f)

struct _s_guidance_v guid_v;

static int32_t get_vertical_thrust_coeff(void);
static void run_hover_loop(bool_t in_flight);
static void guidance_v_controller_ini(void);
static void state_update_loop(void);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

//extern uint8_t vertical_mode;
static void send_vert_loop(struct transport_tx *trans, struct link_device *dev)
{
	uint8_t gv_mode = guid_v.mode;
	uint8_t vert_mode = vertical_mode;
	uint8_t err1 = guidance_v_get_thrust_error_1();
	uint8_t err2 = guidance_v_get_thrust_error_2();

	xbee_tx_header(XBEE_NACK, XBEE_ADDR_PC);
	pprz_msg_send_VERT_LOOP(trans, dev, AC_ID, &gv_mode, &vert_mode, &guid_v.UP_z_acc, &guid_v.UP_z_speed,
													&guid_v.UP_z_pos, &guid_v.ref_acc_z, &guid_v.ref_speed_z, &guid_v.ref_pos_z, &guid_v.acc_z_pid.out,
													&guid_v.speed_z_pid.out, &guid_v.pos_z_pid.out, &err1, &err2);
}

#include "subsystems/ins/ins_int.h"
#include "subsystems/imu.h"
#include "subsystems/ins/vf_extended_float.h"

static void send_tune_vert(struct transport_tx *trans, struct link_device *dev)
{
	float accel_scaled_z = -imu.accel_scaled.z;
	float accel_z = -imu.accel.z;
	float zdotdot = -vff.zdotdot;
	float bias = -vff.bias;
	float zdot = -vff.zdot;
	float z = -vff.z;
	float gps_body_z = -ins_int.rtk_ned_z;
	float hover_throttle = get_butterworth_2_low_pass(&guid_v.hover_throttle_filter);

	xbee_tx_header(XBEE_NACK, XBEE_ADDR_PC);
	pprz_msg_send_TUNE_VERT(trans, dev, AC_ID, &accel_scaled_z, &accel_z, &zdotdot, &bias, &zdot, &z, &gps_body_z,
													&guid_v.ref_acc_z, &guid_v.ref_speed_z, &guid_v.ref_pos_z, &guid_v.acc_z_pid.out, &hover_throttle);
}
#endif

void guidance_v_init(void)
{
	guid_v.mode = GUIDANCE_V_MODE_KILL;

	guidance_v_controller_ini();

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
	pid_set_out_range(&guid_v.speed_z_pid, -2, 2);
	pid_set_Ui_range(&guid_v.speed_z_pid, -2, 2);
	pid_set_out_range(&guid_v.pos_z_pid, -1, 1);
	pid_set_Ui_range(&guid_v.pos_z_pid, -1, 1);

	guid_v.acc_z_pid.Kp = 0.01f;
	guid_v.acc_z_pid.Ki = 0.4f;
	guid_v.speed_z_pid.Kp = 2.0f;
	guid_v.speed_z_pid.Kd = 0.01f;
	guid_v.pos_z_pid.Kp = 1.0f;
	guid_v.pos_z_pid.Kd = 0.3f;

	guid_v.acc_filter_fc = 1;
	init_butterworth_2_low_pass(&guid_v.UP_z_acc_filter, low_pass_filter_get_tau(guid_v.acc_filter_fc),
															1.0f / (float) GUIDANCE_V_LOOP_FREQ, 0);
	init_butterworth_2_low_pass(&guid_v.hover_throttle_filter, 1, 1.0f / (float) GUIDANCE_V_LOOP_FREQ, 0);
}

void guidance_v_SetAccCutoff(float fc)
{
	guid_v.acc_filter_fc = fc;
	init_butterworth_2_low_pass(&guid_v.UP_z_acc_filter, low_pass_filter_get_tau(fc), 1.0f / (float) GUIDANCE_V_LOOP_FREQ,
															guid_v.UP_z_acc_filter.o[0]);
}

void guidance_v_read_rc(void)
{
	/* used in RC_DIRECT directly and as saturation in CLIMB and HOVER */
	guid_v.src_direct_throttle = (int32_t) radio_control.values[RADIO_THROTTLE];
	guid_v.src_direct_throttle = (float) guid_v.src_direct_throttle * GUIDANCE_V_MAX_NORMALIZED_THROTTLE;
	Bound(guid_v.src_direct_throttle, 0, (MAX_PPRZ * GUIDANCE_V_MAX_NORMALIZED_THROTTLE));

	guid_v.src_speed_sp = (float) (radio_control.values[RADIO_THROTTLE] - (MAX_PPRZ / 2)) / (float) (MAX_PPRZ / 2);
	Bound(guid_v.src_speed_sp, -1, +1);

	// get acc sp
	guid_v.src_acc_sp = guid_v.src_speed_sp;
	guid_v.src_acc_sp = guid_v.src_acc_sp * 5.0f;
	Bound(guid_v.src_acc_sp, -5, +5);

	// get speed sp
	DeadBand(guid_v.src_speed_sp, 0.1);
	if (guid_v.src_speed_sp > 0)
	{
		guid_v.src_speed_sp *= GUIDANCE_V_MAX_RC_CLIMB_SPEED;
	}
	else
	{
		guid_v.src_speed_sp *= GUIDANCE_V_MAX_RC_DESCENT_SPEED;
	}
	Bound(guid_v.src_speed_sp, - GUIDANCE_V_MAX_RC_DESCENT_SPEED, + GUIDANCE_V_MAX_RC_CLIMB_SPEED);
}

static void guidance_v_src_thr_to_acc_loop(void)
{
	guid_v.acc_z_pid.Ui = guid_v.src_direct_throttle/(float)MAX_PPRZ;
}

void guidance_v_mode_changed(uint8_t new_mode)
{
	if (new_mode == guid_v.mode)
	{
		return;
	}

	switch (new_mode)
	{
	case GUIDANCE_V_MODE_HOVER:
	case GUIDANCE_V_MODE_GUIDED:
		guid_v.rc_pos_sp = guid_v.UP_z_pos;
		if (guid_v.mode == GUIDANCE_V_MODE_RC_DIRECT)
		{
			guidance_v_src_thr_to_acc_loop();
		}
		break;
	case GUIDANCE_V_MODE_RC_CLIMB:
	case GUIDANCE_V_MODE_CLIMB:
	case GUIDANCE_V_MODE_NAV:
		guid_v.rc_pos_sp = guid_v.UP_z_pos;
		if (guid_v.mode == GUIDANCE_V_MODE_RC_DIRECT)
		{
			guidance_v_src_thr_to_acc_loop();
		}
		break;
	case GUIDANCE_V_MODE_FLIP:
		break;
	default:
		break;

	}
	guid_v.mode = new_mode;
}

static void guidance_v_vertical_mode_change(void)
{
	static uint8_t last_vertical_mode = VERTICAL_MODE_ALT;

	if(last_vertical_mode != vertical_mode)
	{
		if(vertical_mode == VERTICAL_MODE_CLIMB)
		{
			guid_v.rc_pos_sp = guid_v.UP_z_pos;
		}
	}

	last_vertical_mode = vertical_mode;
}

void guidance_v_run(bool_t in_flight)
{
	state_update_loop();
	guidance_v_vertical_mode_change();

	switch (guid_v.mode)
	{
	case GUIDANCE_V_MODE_RC_DIRECT:
	case GUIDANCE_V_MODE_RC_CLIMB:
		stabilization_cmd[COMMAND_THRUST] = guid_v.src_direct_throttle;
		break;
	case GUIDANCE_V_MODE_CLIMB:
	case GUIDANCE_V_MODE_HOVER:
	case GUIDANCE_V_MODE_GUIDED:
	case GUIDANCE_V_MODE_ACC_LAND:
	case GUIDANCE_V_MODE_NAV:
		run_hover_loop(in_flight);
		stabilization_cmd[COMMAND_THRUST] = guid_v.loop_throttle_cmd;
		break;
	case GUIDANCE_V_MODE_FLIP:
		stabilization_cmd[COMMAND_THRUST] = 0;
		break;
	default:
		stabilization_cmd[COMMAND_THRUST] = 0;
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

bool_t guidance_v_get_thrust_error_1(void)
{
	bool_t err = FALSE;

	if (!autopilot_in_flight)
	{
		return FALSE;
	}

	if ((guid_v.mode == GUIDANCE_V_MODE_HOVER) || (guid_v.mode == GUIDANCE_V_MODE_NAV))
	{
		if (fabsf(guid_v.acc_z_pid.err) > GUIDANCE_V_MAX_ACC_LOOP_ERR)
		{
			err = TRUE;
		}
	}

	return err;
}

bool_t guidance_v_get_thrust_error_2(void)
{
	bool_t err = FALSE;

	if (!autopilot_in_flight)
	{
		return FALSE;
	}

	if ((guid_v.mode == GUIDANCE_V_MODE_HOVER) || (guid_v.mode == GUIDANCE_V_MODE_NAV))
	{
		if (guid_v.acc_z_pid.Ui > GUIDANCE_V_HOVER_MAX_NORMALIZED_THROTTLE)
		{
			err = TRUE;
		}
	}

	return err;
}

static void state_update_loop(void)
{
	guid_v.UP_z_acc = -ACCEL_FLOAT_OF_BFP(stateGetAccelNed_i()->z);
	guid_v.UP_z_speed = -SPEED_FLOAT_OF_BFP(stateGetSpeedNed_i()->z);
	guid_v.UP_z_pos = -POS_FLOAT_OF_BFP(stateGetPositionNed_i()->z);

	update_butterworth_2_low_pass(&guid_v.UP_z_acc_filter, guid_v.UP_z_acc);
	update_butterworth_2_low_pass(&guid_v.hover_throttle_filter, guid_v.acc_z_pid.Ui);

	guid_v.thrust_coef = FLOAT_OF_BFP(get_vertical_thrust_coeff(), INT32_TRIG_FRAC);
	Bound(guid_v.thrust_coef, 0.8f, 1.0f);

	if(guid_v.mode != GUIDANCE_V_MODE_CLIMB)
	{
		guid_v.climb_speed_sp = 0;
	}
}

static void run_hover_loop(bool_t in_flight)
{
	float normalized_cmd = 0;

	if (autopilot_motors_on)
	{
		if (guid_v.mode == GUIDANCE_V_MODE_HOVER)
		{
			guid_v.pid_loop_mode_running = guid_v.pid_loop_mode_gcs;
			if ((guid_v.pid_loop_mode_running == ACC_SPEED) || (guid_v.pid_loop_mode_running == ACC_SPEED_POS))
			{
				if (guid_v.pid_loop_mode_running == ACC_SPEED_POS)
				{
					guid_v.rc_pos_sp += guid_v.src_speed_sp * (1.0f / (float) GUIDANCE_V_LOOP_FREQ);
					if (guid_v.rc_pos_sp > GUIDANCE_V_MAX_HEIGHT)
					{
						guid_v.rc_pos_sp = GUIDANCE_V_MAX_HEIGHT;
					}
					guid_v.ref_pos_z = guid_v.rc_pos_sp;
					pid_loop_calc_2(&guid_v.pos_z_pid, guid_v.ref_pos_z, guid_v.UP_z_pos, 0, guid_v.UP_z_speed);
					guid_v.ref_speed_z = guid_v.pos_z_pid.out;
				}
				else
				{
					guid_v.ref_speed_z = guid_v.src_speed_sp;
				}
				//pid_loop_calc_2(&guid_v.speed_z_pid, guid_v.ref_speed_z, guid_v.NED_z_speed, 0, guid_v.NED_z_acc);
				pid_loop_calc_2(&guid_v.speed_z_pid, guid_v.ref_speed_z, guid_v.UP_z_speed, 0,
												get_butterworth_2_low_pass(&guid_v.UP_z_acc_filter));
				guid_v.ref_acc_z = guid_v.speed_z_pid.out;
			}
			else // RC ACC loop
			{
				guid_v.ref_acc_z = guid_v.src_acc_sp;
			}
			pid_loop_calc_2(&guid_v.acc_z_pid, guid_v.ref_acc_z, guid_v.UP_z_acc, 0, 0);

			normalized_cmd = guid_v.acc_z_pid.out;
		}
		else if (guid_v.mode == GUIDANCE_V_MODE_CLIMB)
		{
			if (guid_v.UP_z_pos > GUIDANCE_V_MAX_HEIGHT)
			{
				guid_v.ref_speed_z = 0;
			}
			else
			{
				guid_v.ref_speed_z = guid_v.climb_speed_sp;
			}

			pid_loop_calc_2(&guid_v.speed_z_pid, guid_v.ref_speed_z, guid_v.UP_z_speed, 0,
											get_butterworth_2_low_pass(&guid_v.UP_z_acc_filter));

			guid_v.ref_acc_z = guid_v.speed_z_pid.out;
			pid_loop_calc_2(&guid_v.acc_z_pid, guid_v.ref_acc_z, guid_v.UP_z_acc, 0, 0);

			normalized_cmd = guid_v.acc_z_pid.out;
		}
		else if (guid_v.mode == GUIDANCE_V_MODE_ACC_LAND)
		{
			pid_loop_calc_2(&guid_v.acc_z_pid, -0.1f, guid_v.UP_z_acc, 0, 0);

			normalized_cmd = guid_v.acc_z_pid.out;
		}
		else if (guid_v.mode == GUIDANCE_V_MODE_NAV)
		{
			if (vertical_mode == VERTICAL_MODE_ALT)
			{
				float nav_alt_sp = POS_FLOAT_OF_BFP(nav_flight_altitude);
				if (nav_alt_sp > GUIDANCE_V_MAX_HEIGHT)
				{
					nav_alt_sp = GUIDANCE_V_MAX_HEIGHT;
				}
				guid_v.ref_pos_z = nav_alt_sp;
				pid_loop_calc_2(&guid_v.pos_z_pid, guid_v.ref_pos_z, guid_v.UP_z_pos, 0, guid_v.UP_z_speed);
				guid_v.ref_speed_z = guid_v.pos_z_pid.out;
			}
			else if (vertical_mode == VERTICAL_MODE_CLIMB)
			{
				if(flight_mode == nav_rc_mode)
				{
					guid_v.rc_pos_sp += SPEED_FLOAT_OF_BFP(nav_climb) * (1.0f / (float) GUIDANCE_V_LOOP_FREQ);
					if (guid_v.rc_pos_sp > GUIDANCE_V_MAX_HEIGHT)
					{
						guid_v.rc_pos_sp = GUIDANCE_V_MAX_HEIGHT;
					}
					guid_v.ref_pos_z = guid_v.rc_pos_sp;
					pid_loop_calc_2(&guid_v.pos_z_pid, guid_v.ref_pos_z, guid_v.UP_z_pos, 0, guid_v.UP_z_speed);
					guid_v.ref_speed_z = guid_v.pos_z_pid.out;
				}
				else
				{
					guid_v.ref_speed_z = SPEED_FLOAT_OF_BFP(nav_climb);
				}
			}
			else
			{
				guid_v.ref_speed_z = 0;
			}
			pid_loop_calc_2(&guid_v.speed_z_pid, guid_v.ref_speed_z, guid_v.UP_z_speed, 0,
											get_butterworth_2_low_pass(&guid_v.UP_z_acc_filter));
			guid_v.ref_acc_z = guid_v.speed_z_pid.out;
			pid_loop_calc_2(&guid_v.acc_z_pid, guid_v.ref_acc_z, guid_v.UP_z_acc, 0, 0);

			normalized_cmd = guid_v.acc_z_pid.out;
		}
		else
		{
			normalized_cmd = 0;
		}
	}
	else
	{
		pid_reset(&guid_v.acc_z_pid);
		normalized_cmd = 0;
	}

	guid_v.loop_throttle_cmd = (normalized_cmd / guid_v.thrust_coef)
														 * (GUIDANCE_V_MAX_NORMALIZED_THROTTLE * (float) MAX_PPRZ);
	Bound(guid_v.loop_throttle_cmd, 0, (GUIDANCE_V_MAX_NORMALIZED_THROTTLE * (float) MAX_PPRZ));
}

bool_t guidance_v_set_guided_z(float z)
{
	if (guid_v.mode == GUIDANCE_V_MODE_GUIDED)
	{
		guid_v.guided_pos_sp = POS_BFP_OF_REAL(z);
		return TRUE;
	}
	return FALSE;
}

