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

/** @file firmwares/rotorcraft/guidance/guidance_h.h
 *  Horizontal guidance for rotorcrafts.
 *
 */

#ifndef GUIDANCE_H_H
#define GUIDANCE_H_H

#include "math/pprz_algebra_int.h"

#include "firmwares/rotorcraft/guidance/guidance_h_ref.h"
#include "generated/airframe.h"
#include "std.h"
#include "filters/low_pass_filter.h"
#include "controllers/pid.h"
#include "math/dim2_algebra.h"

/** Use horizontal guidance reference trajectory.
 * Default is TRUE, define to FALSE to always disable it.
 */
#ifndef GUIDANCE_H_USE_REF
#define GUIDANCE_H_USE_REF TRUE
#endif

/** Use horizontal guidance speed reference.
 * This also allows to give velocity commands via RC in GUIDANCE_H_MODE_HOVER.
 * Default is TRUE, define to FALSE to always disable it.
 */
#ifndef GUIDANCE_H_USE_SPEED_REF
#define GUIDANCE_H_USE_SPEED_REF TRUE
#endif

#define GUIDANCE_H_MODE_KILL        0
#define GUIDANCE_H_MODE_RATE        1
#define GUIDANCE_H_MODE_ATTITUDE    2
#define GUIDANCE_H_MODE_HOVER       3
#define GUIDANCE_H_MODE_NAV         4
#define GUIDANCE_H_MODE_RC_DIRECT   5
#define GUIDANCE_H_MODE_CARE_FREE   6
#define GUIDANCE_H_MODE_FORWARD     7
#define GUIDANCE_H_MODE_MODULE      8
#define GUIDANCE_H_MODE_FLIP        9
#define GUIDANCE_H_MODE_GUIDED      10


struct HorizontalGuidanceSetpoint {
  /** horizontal position setpoint in NED.
   *  fixed point representation: Q23.8
   *  accuracy 0.0039, range 8388km
   */
  struct Int32Vect2 pos;
  struct Int32Vect2 speed;  ///< only used if GUIDANCE_H_USE_SPEED_REF
  int32_t heading;          ///< with #INT32_ANGLE_FRAC
};

struct HorizontalGuidanceReference {
  struct Int32Vect2 pos;     ///< with #INT32_POS_FRAC
  struct Int32Vect2 speed;   ///< with #INT32_SPEED_FRAC
  struct Int32Vect2 accel;   ///< with #INT32_ACCEL_FRAC
};

struct HorizontalGuidanceGains {
  int32_t p;
  int32_t d;
  int32_t i;
  int32_t v;
  int32_t a;
};

struct HorizontalGuidanceGains_f {
  float p;
  float d;
  float i;
  float in_p;
  float in_i;
  float in_d;
};


enum _e_h_pid_loop_mode
{
	VEL = 0,
	POS_VEL
};

enum _e_traj_mode
{
	TRAJ_MODE_HOVER = 0,
	TRAJ_MODE_SEGMENT
};

enum _e_traj_status
{
	TRAJ_STATUS_VEL = 0,
	TRAJ_STATUS_POS,
	TRAJ_STATUS_BRAKE,
	TRAJ_STATUS_BRAKE_DONE,
};

struct _s_segment
{
	// tracking segment start and end point in LTP NED (i coordinate)
	struct FloatVect2 start;
	struct FloatVect2 end;
	//
	float length;
	// tracking segment along direction and cross direction (orthogonal unit vector) in LTP NED
	struct FloatVect2 along;
	struct FloatVect2 cross;
	// coordinate rotation matrix between i-t (LEP NED, segment)
	struct _s_matrix22 R_i2t;
	struct _s_matrix22 R_t2i;
};

struct _s_trajectory_tracking
{
	// test
	uint8_t test_mode;
	struct FloatVect2 test_line_start;
	struct FloatVect2 test_line_end;
	struct FloatVect2 test_square_c1;
	struct FloatVect2 test_square_c2;
	struct FloatVect2 test_square_c3;
	struct FloatVect2 test_square_c4;
	uint8_t test_square_index;
	float test_length;
	uint8_t pid_loop_test_mode;

	enum _e_traj_mode mode;
	enum _e_traj_status state;
	bool_t emergency_brake;
	uint16_t emergency_brake_cnt;
	float emergency_brake_x;
	float ref_speed;
	float pos_along_brake_len;
	float max_acc;
	float emergency_brake_acc;
	float min_brake_len;
	float brake_margin;
	float guid_speed;
	float pos_along_Kp;
	float pos_along_Kd;

	struct FloatVect2 hover_point;

	struct _s_segment segment;
	struct _s_segment segment_last;

	// coordinate rotation matrix between i-t-b (LEP NED, segment, body)
	struct _s_matrix22 R_i2b;
	struct _s_matrix22 R_b2i;
	struct _s_matrix22 R_t2b;
	struct _s_matrix22 R_b2t;

	// controller decomposition
	struct _s_pid vel_along_pid;
	struct _s_pid vel_cross_pid;
	struct _s_pid pos_along_pid;
	struct _s_pid pos_cross_pid;

	// state in segment coordinate
	struct FloatVect2 acc_t;
	struct FloatVect2 vel_t;
	struct FloatVect2 pos_t;

	//
	float psi;

	//
	struct FirstOrderLowPass thrust_cmd_filter;
	struct FloatVect2 cmd_t;
	struct FloatVect2 cmd_t_comp;
	struct FloatVect2 cmd_b;
};

struct HorizontalGuidance
{
  uint8_t mode;
  /* configuration options */
  bool_t use_ref;
  bool_t approx_force_by_thrust;

  struct HorizontalGuidanceSetpoint sp; ///< setpoints
  struct HorizontalGuidanceReference ref; ///< reference calculated from setpoints

  struct Int32Eulers rc_sp;    ///< with #INT32_ANGLE_FRAC

  // ublox pos pid loop
  struct FloatVect2 ned_acc;
  struct FloatVect2 ned_vel;
  struct FloatVect2 ned_pos;
  struct FloatVect2 ned_vel_rc;
  struct FloatVect2 ned_pos_rc;
  struct FloatVect2 ned_vel_ref;
  struct FloatVect2 ned_pos_ref;

  float ned_acc_filter_fc;
  float ned_vel_filter_fc;
  struct FloatVect2 ned_acc_filter;
  struct FloatVect2 ned_vel_filter;
  Butterworth2LowPass ned_acc_x_filter;
  Butterworth2LowPass ned_acc_y_filter;
  Butterworth2LowPass ned_vel_x_filter;
  Butterworth2LowPass ned_vel_y_filter;

  bool_t ned_pos_rc_reset;

  enum _e_h_pid_loop_mode pid_loop_mode_running;
  enum _e_h_pid_loop_mode pid_loop_mode_gcs;
};

extern int32_t rc_turn_rate;                        ///< with #INT32_RATE_FRAC

extern float hh;
extern float hh0;
extern float r_h;

extern struct HorizontalGuidance guidance_h;
extern struct _s_trajectory_tracking traj;
//#if GUIDANCE_H_USE_SPEED_REF
//extern struct Int32Vect2 guidance_h.sp.speed;
//#endif

extern int32_t transition_percentage;
extern int32_t transition_theta_offset;

extern void guidance_h_init(void);
extern void guidance_h_mode_changed(uint8_t new_mode);
extern void guidance_h_read_rc(bool_t in_flight);
extern void guidance_h_run(bool_t in_flight);

extern void guidance_h_nav_rc_enter(void); //use when nav_rc_mode enter

extern void guidance_h_trajectory_tracking_set_hover(struct FloatVect2 point);
extern void guidance_h_trajectory_tracking_set_segment(struct FloatVect2 start, struct FloatVect2 end);
extern void guidance_h_trajectory_tracking_set_ref_speed(float speed);
extern void guidance_h_trajectory_tracking_set_max_acc(float acc);
extern void guidance_h_trajectory_tracking_set_min_brake_len(float len);
extern void guidance_h_trajectory_tracking_set_emergency_brake_acc(float acc);
extern void guidance_h_trajectory_tracking_set_emergency_brake(bool_t brake);
extern void guidance_h_SetTrajTest(uint8_t mode);
extern void guidance_h_SetTrajRefSpeed(float speed);
extern void guidance_h_SetEmBrake(bool_t on);
extern void guidance_h_SetMinBrakeLen(float len);
extern void guidance_h_SetMaxAcc(float acc);
extern void guidance_h_SetEmBrakeAcc(float acc);

extern void guidance_h_SetNedAccFc(float Fc);
extern void guidance_h_SetNedVelFc(float Fc);

extern void guidance_h_ned_pos_rc_need_reset(void);

/** Set horizontal position setpoint in GUIDED mode.
 * @param x North position (local NED frame) in meters.
 * @param y East position (local NED frame) in meters.
 * @return TRUE if setpoints were set (currently in GUIDANCE_H_MODE_GUIDED)
 */
bool_t guidance_h_set_guided_pos(float x, float y);

/** Set heading setpoint in GUIDED mode.
 * @param heading Setpoint in radians.
 * @return TRUE if setpoint was set (currently in GUIDANCE_H_MODE_GUIDED)
 */
bool_t guidance_h_set_guided_heading(float heading);

/* Make sure that ref can only be temporarily disabled for testing,
 * but not enabled if GUIDANCE_H_USE_REF was defined to FALSE.
 */
#define guidance_h_SetUseRef(_val) {                    \
    guidance_h.use_ref = _val && GUIDANCE_H_USE_REF;    \
  }

static inline void guidance_h_SetMaxSpeed(float speed)
{
  gh_set_max_speed(speed);
}

static inline void guidance_h_SetOmega(float omega)
{
  gh_set_omega(omega);
}

static inline void guidance_h_SetZeta(float zeta)
{
  gh_set_zeta(zeta);
}

static inline void guidance_h_SetTau(float tau)
{
  gh_set_tau(tau);
}

#endif /* GUIDANCE_H_H */
