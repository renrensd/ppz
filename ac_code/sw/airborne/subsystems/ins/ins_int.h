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

#define INS_INT_RTK_Z_HIST_SIZE	(25)

enum _e_ins_gpss_status
{
	RTK_UBLOX_INVALID = 0,
	UBLOX_VALID,
	RTK_VALID,
	RTK_UBLOX_VALID
};

enum _e_ins_ekf_status
{
	INS_EKF_GPS = 0,
	INS_EKF_GPS_TO_BARO,
	INS_EKF_BARO,
	INS_EKF_BARO_TO_GPS,
	INS_EKF_PURE_ACC,
};

enum _e_ins_gps_type
{
	GPS_NONE,
	GPS_RTK,
	GPS_UBLOX
};

/** Ins implementation state (fixed point) */
struct InsInt
{
  struct LtpDef_i  ltp_def;
  bool_t           ltp_initialized;

  /** request to realign horizontal filter.
   * Sets to current position (local origin unchanged).
   */
  bool_t rtk_hf_realign;

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
  float baro_ned_z;  ///< z-position calculated from baro in meters (NED)
  float rtk_ned_z;
  float rtk_ned_zd;
  float rtk_ned_z_hist[INS_INT_RTK_Z_HIST_SIZE];
  float rtk_ned_zd_hist[INS_INT_RTK_Z_HIST_SIZE];
  uint8_t rtk_ned_z_hist_index;
  bool_t rtk_z_hist_ok;
  float raw_baro_offset;
  struct FirstOrderLowPass baro_z_filter;
  bool_t baro_initialized;
  bool_t baro_valid;// not used !!!
  float R_baro;
  float R_baro_offset;
  uint32_t baro_to_gps_count;
  float baro_to_gps_offset_step;
  float baro_to_gps_z_step;
  bool_t virtual_rtk_pos_z_valid;
  bool_t virtual_rtk_pos_xy_valid;
  bool_t virtual_ublox_pos_valid;
  //
  float R_rtk_pos_z_setting;
  float R_rtk_pos_z;
  float R_ublox_pos;
  float R_ublox_vel;
  bool_t ublox_hf_realign;
  bool_t hf_realign_done;
  enum _e_ins_gps_type gps_type;
  enum _e_ins_gpss_status gpss_state;

  //
  struct NedCoor_i rtk_gps_pos_cm_ned;
  struct NedCoor_i rtk_gps_speed_cm_ned;
  struct NedCoor_i rtk_gps_sd_ned;
  bool_t rtk_gps_update;
  bool_t ublox_gps_update;

  struct FloatVect2 gps_pos_m_ned;
  struct FloatVect2 gps_speed_m_ned;

  uint8_t force_use_redundency;

#if 1 //USE_SONAR
  bool_t update_on_agl; ///< use sonar to update agl if available
#endif
#if USE_RADAR24
  bool_t update_radar_agl; ///< use sonar to update agl if available
#endif
};

/** global INS state */
extern struct InsInt ins_int;

#ifndef DefaultInsImpl
#define DefaultInsImpl ins_int
#endif

extern void ins_int_register(void);
extern void ins_int_SetType(enum _e_ins_gps_type type);
extern void ins_int_task(void);
extern bool_t ins_int_check_hf_realign_done(void);
extern bool_t ins_int_check_hf_has_realigned(void);
extern bool_t ins_int_is_rtk_pos_xy_valid(void);
extern bool_t ins_int_is_rtk_pos_z_valid(void);
extern bool_t ins_int_is_ublox_pos_valid(void);
extern bool_t ins_int_is_rtk_best_accu(void);
extern bool_t ins_int_v_ekf_open_loop(void);
extern void ins_int_SetForceRedun(uint8_t force);

#endif /* INS_INT_H */
