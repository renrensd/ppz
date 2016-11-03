/*
 * Copyright (C) 2008-2012 The Paparazzi Team
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
 * @file subsystems/ins/ins_int.h
 *
 * INS for rotorcrafts combining vertical and horizontal filters.
 *
 */

#ifndef INS_INT_H
#define INS_INT_H

#include "subsystems/ins.h"
#include "subsystems/gps.h"
#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_algebra_float.h"
#include "filters/low_pass_filter.h"

enum _e_ins_ekf_status
{
	INS_EKF_GPS = 0,
	INS_EKF_BARO,
	INS_EKF_BARO_TO_GPS
};

/** Ins implementation state (fixed point) */
struct InsInt {
  struct LtpDef_i  ltp_def;
  bool_t           ltp_initialized;

  uint32_t propagation_cnt; ///< number of propagation steps since the last measurement update

  /** request to realign horizontal filter.
   * Sets to current position (local origin unchanged).
   */
  bool_t hf_realign;

  /** request to reset vertical filter.
   * Sets the z-position to zero and resets the the z-reference to current altitude.
   */
  bool_t vf_realign;
  bool_t vf_stable;

  /* output LTP NED */
  struct NedCoor_i ltp_pos;
  struct NedCoor_i ltp_speed;
  struct NedCoor_i ltp_accel;

  // baro gps switch
  enum _e_ins_ekf_status ekf_state;
  float baro_z;  ///< z-position calculated from baro in meters (NED)
  float gps_body_z;
  float baro_offset_curr;
  struct FirstOrderLowPass baro_z_filter;
  bool_t baro_initialized;
  bool_t baro_valid;// not used !!!
  float R_baro;
  float R_baro_offset;
  uint32_t baro_to_gps_count;
  float baro_to_gps_offset_step;
  float baro_to_gps_z_step;
  int32_t virtual_p_stable;

  // gps telemetry
  uint8_t gps_qual;
  uint8_t p_stable;
  float gps_heading;
  float mag_heading;
  float gps_speed_z;

#if 1 //USE_SONAR
  bool_t update_on_agl; ///< use sonar to update agl if available
#endif
#if USE_RADAR24
  bool_t update_radar_agl; ///< use sonar to update agl if available
#endif
};

struct _s_move_filter
{
	unsigned short win_size;
	unsigned short data_index;
	float *data;
	float sum;
	float out;
};

/** global INS state */
extern struct InsInt ins_int;
extern float gps_noise_debug;

extern void ins_int_init(void);
extern void ins_int_propagate(struct Int32Vect3 *accel, float dt);
extern void ins_int_update_gps(struct GpsState *gps_s);


#ifndef DefaultInsImpl
#define DefaultInsImpl ins_int
#endif

extern void ins_int_register(void);

#endif /* INS_INT_H */
