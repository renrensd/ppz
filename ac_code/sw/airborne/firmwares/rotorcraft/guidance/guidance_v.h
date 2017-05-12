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

/** @file firmwares/rotorcraft/guidance/guidance_v.h
 *  Vertical guidance for rotorcrafts.
 *
 */

#ifndef GUIDANCE_V_H
#define GUIDANCE_V_H

#include "std.h"
#include "controllers/pid.h"
#include "filters/low_pass_filter.h"

#define GUIDANCE_V_MODE_KILL      0
#define GUIDANCE_V_MODE_RC_DIRECT 1
#define GUIDANCE_V_MODE_RC_CLIMB  2
#define GUIDANCE_V_MODE_CLIMB     3
#define GUIDANCE_V_MODE_HOVER     4
#define GUIDANCE_V_MODE_NAV       5
#define GUIDANCE_V_MODE_MODULE    6
#define GUIDANCE_V_MODE_FLIP      7
#define GUIDANCE_V_MODE_GUIDED    8
#define GUIDANCE_V_MODE_ACC_LAND  9

enum _e_v_pid_loop_mode
{
	ACC = 0,
	ACC_SPEED,
	ACC_SPEED_POS
};

struct _s_guidance_v
{
	uint8_t mode;

	enum _e_v_pid_loop_mode pid_loop_mode_gcs;
	enum _e_v_pid_loop_mode pid_loop_mode_running;
	float UP_z_acc;
	float UP_z_speed;
	float UP_z_pos;
	struct _s_pid acc_z_pid;
	struct _s_pid speed_z_pid;
	struct _s_pid pos_z_pid;
	int32_t loop_throttle_cmd;

	float ref_pos_z;
	float ref_speed_z;
	float ref_acc_z;
	float acc_filter_fc;
	Butterworth2LowPass UP_z_acc_filter;
	Butterworth2LowPass hover_throttle_filter;
	float thrust_coef;

	// src and vrc
	int32_t src_direct_throttle;
	float src_speed_sp;
	float src_acc_sp;
	float rc_pos_sp;

	//
	float guided_pos_sp;
	float climb_speed_sp;
};

extern struct _s_guidance_v guid_v;

extern void guidance_v_init(void);
extern void guidance_v_read_rc(void);
extern void guidance_v_mode_changed(uint8_t new_mode);
extern void guidance_v_run(bool_t in_flight);

extern void guidance_v_SetAccCutoff(float fc);

extern bool_t guidance_v_get_thrust_error_1(void);
extern bool_t guidance_v_get_thrust_error_2(void);

/** Set z setpoint in GUIDED mode.
 * @param z Setpoint (down is positive) in meters.
 * @return TRUE if setpoint was set (currently in GUIDANCE_V_MODE_GUIDED)
 */
extern bool_t guidance_v_set_guided_z(float z);

#endif /* GUIDANCE_V_H */
