/*
 * Copyright (C) 2008-2010 The Paparazzi Team
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
 * @file subsystems/ins/ins_int.c
 *
 * INS for rotorcrafts combining vertical and horizontal filters.
 *
 */

#include "subsystems/ins/ins_int.h"
#include <stdlib.h>
#include "subsystems/abi.h"

#include "subsystems/imu.h"

#include "subsystems/gps.h"
#ifdef USE_GPS_NMEA
 #include "subsystems/gps/gps_nmea.h"
#endif


#include "generated/airframe.h"

#if USE_VFF_EXTENDED
#include "subsystems/ins/vf_extended_float.h"
#else
#include "subsystems/ins/vf_float.h"
#endif

#if USE_HFF
#include "subsystems/ins/hf_float.h"
#endif

#if defined SITL && USE_NPS
//#include "nps_fdm.h"
#include "nps_autopilot.h"
#include <stdio.h>
#endif

#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_isa.h"
#include "math/pprz_algebra_int.h"

#include "generated/flight_plan.h"
#include "subsystems/ahrs/ahrs_float_mlkf.h"
#include "math/my_math.h"
#include "modules/ins/ins_ublox.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/rc_nav/rc_nav_xbee.h"
#include "firmwares/rotorcraft/nav_flight.h"
#include "firmwares/rotorcraft/autopilot.h"

#define R_BARO					(1000.0f)
#define R_BARO_OFFSET		    (10.0f)
#define R_RTK_POS_Z			(0.0004f)
#define R_UBLOX					(0.1f)

#define RTK_GPS		gps
#define UBLOX_GPS	    gps2

#if USE_SONAR
#if !USE_VFF_EXTENDED
#error USE_SONAR needs USE_VFF_EXTENDED
#endif

/** default sonar to use in INS */
#ifndef INS_SONAR_ID
#define INS_SONAR_ID ABI_BROADCAST
#endif
abi_event sonar_ev;
static void sonar_cb(uint8_t sender_id, float distance);

#ifdef INS_SONAR_THROTTLE_THRESHOLD
#include "firmwares/rotorcraft/stabilization.h"
#endif

#ifndef INS_SONAR_OFFSET
#define INS_SONAR_OFFSET 0.
#endif
#ifndef INS_SONAR_MIN_RANGE
#define INS_SONAR_MIN_RANGE 0.18
#endif
#ifndef INS_SONAR_MAX_RANGE
#define INS_SONAR_MAX_RANGE 2.0
#endif
#define VFF_R_SONAR_0 0.5
#ifndef VFF_R_SONAR_OF_M
#define VFF_R_SONAR_OF_M 0.2
#endif
#ifndef INS_SONAR_DETA_RANGE
#define INS_SONAR_DETA_RANGE 0.5
#endif

#ifndef INS_SONAR_UPDATE_ON_AGL
#define INS_SONAR_UPDATE_ON_AGL FALSE
PRINT_CONFIG_MSG("INS_SONAR_UPDATE_ON_AGL defaulting to FALSE")
#endif
#endif // USE_SONAR


#if USE_RADAR24

#ifndef INS_RADAR24_MIN_RANGE
#define INS_RADAR24_MIN_RANGE 0.20
#endif

#ifndef INS_RADAR24_MAX_RANGE
#define INS_RADAR24_MAX_RANGE 20.00
#endif

#define VFF_R_RADAR24_0  3.0

#ifndef VFF_R_RADAR24_OF_M
#define VFF_R_RADAR24_OF_M  0.3
#endif

#endif// USE_RADAR24

#ifdef USE_GPS_NMEA
 #ifndef INS_VFF_R_GPS
 #define INS_VFF_R_GPS 3.0
 #endif
 #define INS_USE_GPS_ALT TRUE
 #define USE_INS_NAV_INIT FALSE
#else
 #define INS_USE_GPS_ALT FALSE
#endif

/** maximum number of propagation steps without any updates in between */
#ifndef INS_MAX_PROPAGATION_STEPS
#define INS_MAX_PROPAGATION_STEPS 200
#endif

#ifndef USE_INS_NAV_INIT
 #define USE_INS_NAV_INIT FALSE
PRINT_CONFIG_MSG("USE_INS_NAV_INIT defaulting to FALSE")
#endif

#ifdef INS_BARO_SENS
#warning INS_BARO_SENS is obsolete, please remove it from your airframe file.
#endif

/** default barometer to use in INS */
#ifndef INS_BARO_ID
#if USE_BARO_BOARD
#define INS_BARO_ID BARO_BOARD_SENDER_ID
#else
#define INS_BARO_ID ABI_BROADCAST
#endif
#endif
PRINT_CONFIG_VAR(INS_BARO_ID)
abi_event baro_ev;
static void baro_cb(uint8_t __attribute__((unused)) sender_id,
										uint32_t stamp,
										float pressure,
										float temperature);

/** ABI binding for IMU data.
 * Used accel ABI messages.
 */
#ifndef INS_INT_IMU_ID
#define INS_INT_IMU_ID ABI_BROADCAST
#endif
static abi_event accel_ev;
static abi_event gps_ev;
static abi_event gps2_ev;

#if USE_RADAR24
abi_event radar24_ev;
static void radar24_cb(uint8_t sender_id, float distance);
#endif

struct InsInt ins_int;

int32_t ins_body_rate_z;

/** ABI binding for VELOCITY_ESTIMATE.
 * Usually this is coming from opticflow.
 */
#ifndef INS_INT_VEL_ID
#define INS_INT_VEL_ID ABI_BROADCAST
#endif
static abi_event vel_est_ev;
static void vel_est_cb(uint8_t sender_id, uint32_t stamp, float x, float y, float z, float noise);

static void ins_init_origin_from_flightplan(void);
static void ins_ned_to_state(void);
static void ins_update_from_vff(void);
#if USE_HFF
static void ins_update_from_hff(void);
#endif

static void ins_int_init(void);
static void ins_int_propagate(struct Int32Vect3 *accel, float dt);
static void ins_int_gps_switch(enum _e_ins_gps_type type);
static inline void gpss_state_update(void);
static void ins_int_set_hf_realign_done(bool_t done);
static void ins_int_set_rtk_hf_realign(bool_t need_realign);
static void ins_int_set_ublox_hf_realign(bool_t need_realign);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  pprz_msg_send_INS(trans, dev, AC_ID,
                    &ins_int.ltp_pos.x, &ins_int.ltp_pos.y, &ins_int.ltp_pos.z,
                    &ins_int.ltp_speed.x, &ins_int.ltp_speed.y, &ins_int.ltp_speed.z,
                    &ins_int.ltp_accel.x, &ins_int.ltp_accel.y, &ins_int.ltp_accel.z);
}

static void send_ins_z(struct transport_tx *trans, struct link_device *dev)
{
	float raw_baro_offset = ins_int.rtk_ned_z - ins_int.baro_ned_z;
	float baro_filter = get_first_order_low_pass(&ins_int.baro_z_filter);

  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  pprz_msg_send_INS_Z(trans, dev, AC_ID,
  		  		&ins_int.baro_ned_z,
  					&baro_filter,
  					&ins_int.rtk_ned_z,
  					&vff.z,
					&ins_int.rtk_ned_zd,
					&vff.zdot,
					&vff.bias,
					&raw_baro_offset,
					&vff.offset            );
}

static void send_ins_ref(struct transport_tx *trans, struct link_device *dev)
{
  if (ins_int.ltp_initialized) {
  	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
    pprz_msg_send_INS_REF(trans, dev, AC_ID,
                          &ins_int.ltp_def.ecef.x, &ins_int.ltp_def.ecef.y, &ins_int.ltp_def.ecef.z,
                          &ins_int.ltp_def.lla.lat, &ins_int.ltp_def.lla.lon, &ins_int.ltp_def.lla.alt,
                          &ins_int.ltp_def.hmsl);
  }
}

static void send_debug_gps(struct transport_tx *trans, struct link_device *dev)
{
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	  DOWNLINK_SEND_DEBUG_GPS(DefaultChannel, DefaultDevice,
	  		&ins_int.gpss_state,
	  		&ins_int.ekf_state,
				&ahrs_mlkf.heading_state,
				&ins_int.gps_type,
	  		&gps_nmea.gps_qual,
				&gps_nmea.pos_type,
				&RTK_GPS.p_stable,
				&gps_nmea.sol_tatus,
				&RTK_GPS.h_stable,
				&RTK_GPS.num_sv,
				&RTK_GPS.head_stanum,
				&RTK_GPS.heading,
				&ahrs_mlkf.mag_heading,
				&ahrs_mlkf.mlkf_heading,
				&ins_int.rtk_gps_sd_ned.x,
				&ins_int.rtk_gps_sd_ned.y,
				&ins_int.rtk_gps_sd_ned.z,
				&UBLOX_GPS.tow,
				&UBLOX_GPS.num_sv,
				&UBLOX_GPS.fix);
}

#endif

static void ins_int_init(void)
{

#if USE_INS_NAV_INIT
  ins_init_origin_from_flightplan();
  ins_int.ltp_initialized = TRUE;
#else
  ins_int.ltp_initialized  = FALSE;  //via NavSetGroundReferenceHere() set true
#endif

  // Bind to BARO_ABS message,not default
#if USE_BARO_BOARD
  AbiBindMsgBARO_ABS(INS_BARO_ID, &baro_ev, baro_cb);
  ins_int.baro_initialized = FALSE;
  ins_int.R_baro = R_BARO;
  ins_int.R_baro_offset = R_BARO_OFFSET;
  ins_int.ekf_state = INS_EKF_PURE_ACC;
  ins_int.virtual_rtk_pos_z_valid = TRUE;
  ins_int.virtual_rtk_pos_xy_valid = TRUE;
  ins_int.virtual_ublox_pos_valid = TRUE;
  ins_int.virtual_baro_valid = TRUE;
#endif

  ins_int.force_use_redundency = FALSE;

#if USE_SONAR
  ins_int.update_on_agl = FALSE;  //INS_SONAR_UPDATE_ON_AGL; default FALSE, it will change in mornitoring
  // Bind to AGL message
  AbiBindMsgAGL(INS_SONAR_ID, &sonar_ev, sonar_cb);
#endif
#if USE_RADAR24
    ins_int.update_radar_agl = FALSE; 
    AbiBindMsgRADAR_24(AGL_NRA_24_ID,&radar24_ev,radar24_cb);
#endif

  ins_int.vf_realign = FALSE;
  ins_int.vf_stable = FALSE;
  ins_int.rtk_z_hist_ok = FALSE;
  ins_int.rtk_ned_z_hist_index = 0;
  ins_int_set_rtk_hf_realign(FALSE);
  ins_int_set_ublox_hf_realign(FALSE);
  ins_int_set_hf_realign_done(FALSE);

  /* init vertical and horizontal filters   all set 0 */
  vff_first_init();
#if USE_HFF
  b2_hff_init(0., 0., 0., 0.);
#endif

  INT32_VECT3_ZERO(ins_int.ltp_pos);
  INT32_VECT3_ZERO(ins_int.ltp_speed);
  INT32_VECT3_ZERO(ins_int.ltp_accel);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS, send_ins);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_Z, send_ins_z);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_REF, send_ins_ref);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DEBUG_GPS, send_debug_gps);
#endif

  ins_int.R_rtk_pos_z = R_RTK_POS_Z;
  ins_int.R_rtk_pos_z_setting = R_RTK_POS_Z;
  ins_int.R_ublox_pos = R_UBLOX;
  ins_int.R_ublox_vel = R_UBLOX;

	//ins_int_gps_switch(GPS_UBLOX);
  //ins_int_gps_switch(GPS_RTK);
  ins_int.gps_type = GPS_NONE;
}

void ins_int_SetForceRedun(uint8_t force)
{
	ins_int.force_use_redundency = force;
	force_use_all_redundency_and_vrc(force);
}

static inline void gpss_state_update(void)
{
	if(RTK_GPS.p_stable)
	{
		if(ins_int_is_ublox_pos_valid())
		{
			ins_int.gpss_state = RTK_UBLOX_VALID;
		}
		else
		{
			ins_int.gpss_state = RTK_VALID;
		}
	}
	else
	{
		if(ins_int_is_ublox_pos_valid())
		{
			ins_int.gpss_state = UBLOX_VALID;
		}
		else
		{
			ins_int.gpss_state = RTK_UBLOX_INVALID;
		}
	}
}

static void ins_int_set_rtk_hf_realign(bool_t need_realign)
{
	ins_int.rtk_hf_realign = need_realign;
	if(need_realign)
	{
		ins_int_set_hf_realign_done(FALSE);
	}
}

static void ins_int_set_ublox_hf_realign(bool_t need_realign)
{
	ins_int.ublox_hf_realign = need_realign;
	if (need_realign)
	{
		ins_int_set_hf_realign_done(FALSE);
	}
}

static void ins_int_set_hf_realign_done(bool_t done)
{
	ins_int.hf_realign_done = done;
}

bool_t ins_int_check_hf_has_realigned(void)
{
	static bool_t last_realign_done = FALSE;
	if((!last_realign_done) && ins_int.hf_realign_done)
	{
		last_realign_done = ins_int.hf_realign_done;
		return TRUE;
	}
	else
	{
		last_realign_done = ins_int.hf_realign_done;
		return FALSE;
	}

}

bool_t ins_int_check_hf_realign_done(void)
{
	if(ins_int.hf_realign_done)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

bool_t ins_int_is_rtk_best_accu(void)
{
	if((gps_nmea.gps_qual == 52) && (gps.num_sv >= RTK_MIN_POS_SV_NUM) && gps.p_stable)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

bool_t ins_int_is_rtk_pos_xy_valid(void)
{
	return (gps.p_stable && ins_int.ltp_initialized && (gps_nmea.pos_type > PSRDIFF) && ins_int.virtual_rtk_pos_xy_valid);
}

bool_t ins_int_is_rtk_pos_z_valid(void)
{
	if(gps.p_stable && ins_int.ltp_initialized && ins_int.vf_stable && ins_int.virtual_rtk_pos_z_valid)
	{
		float r_unstable = (float) ins_int.rtk_gps_sd_ned.z / 10000.0;
		r_unstable = r_unstable * r_unstable;
		return (r_unstable < 0.04f);
	}
	else
	{
		return FALSE;
	}
}

bool_t ins_int_is_baro_valid(void)
{
	return ins_int.virtual_baro_valid && ins_int.baro_valid;
}

bool_t ins_int_is_ublox_pos_valid(void)
{
	return (UBLOX_GPS.p_stable && ins_int.virtual_ublox_pos_valid);
}

static void ins_int_record_rtk_z_hist(void)
{
	if(ins_int.rtk_ned_z_hist_index >= INS_INT_RTK_Z_HIST_SIZE)
	{
		ins_int.rtk_z_hist_ok = TRUE;
		ins_int.rtk_ned_z_hist_index = 0;
	}
	ins_int.rtk_ned_z_hist[ins_int.rtk_ned_z_hist_index] = ins_int.rtk_ned_z;
	ins_int.rtk_ned_zd_hist[ins_int.rtk_ned_z_hist_index] = ins_int.rtk_ned_zd;
	++ins_int.rtk_ned_z_hist_index;
}

static uint8_t find_index(int16_t base, int16_t delta , int16_t max)
{
	int8_t index;

	index = base + delta;

  while (index > max) index -= max;
  while (index < 0) index += max;

  Bound(index, 0, max);
	return index;
}

static void ins_int_get_recent_valid_rtk_z(float *z, float *zd)
{
#define GET_HIST_SIZE	(5)

	uint8_t i, j, index;
	float z_sum = 0;
	float zd_sum = 0;
	float z_hist_mean;
	float zd_hist_mean;

	if ((z == NULL) || (zd == NULL))
	{
		return;
	}

	*z = ins_int.rtk_ned_z;
	*zd = ins_int.rtk_ned_zd;

	if (ins_int.rtk_z_hist_ok)
	{
		z_sum = 0;
		zd_sum = 0;
		for (i = 0; i < GET_HIST_SIZE; ++i)
		{
			index = find_index(ins_int.rtk_ned_z_hist_index, i+1, INS_INT_RTK_Z_HIST_SIZE);
			z_sum += ins_int.rtk_ned_z_hist[index];
			zd_sum += ins_int.rtk_ned_zd_hist[index];
		}
		z_hist_mean = z_sum / (float) GET_HIST_SIZE;
		zd_hist_mean = zd_sum / (float) GET_HIST_SIZE;

		if (fabsf(z_hist_mean) < 5)
		{
			*z = z_hist_mean;
		}
		if (fabsf(zd_hist_mean) < 2)
		{
			*zd = zd_hist_mean;
		}
	}

	ins_int.rtk_z_hist_ok = FALSE;
	ins_int.rtk_ned_z_hist_index = 0;
}

bool_t ins_int_v_ekf_open_loop(void)
{
	return (ins_int.ekf_state == INS_EKF_PURE_ACC);
}

static void switch_to_baro(void)
{
	if (!ins_int.baro_initialized)
	{
		return;
	}

	if (ins_int_is_baro_valid())
	{
		if (ins_int.ekf_state != INS_EKF_BARO)
		{
			ins_int.ekf_state = INS_EKF_GPS_TO_BARO;
		}
	}
	else
	{
		ins_int.ekf_state = INS_EKF_PURE_ACC;
		autopilot_set_mode(AP_MODE_FAILSAFE);
	}
}

static void switch_to_ublox(void)
{
	if(ins_int_is_ublox_pos_valid())
	{
		if (ins_int.gps_type != GPS_UBLOX)
		{
			ins_int_gps_switch(GPS_UBLOX);
		}
	}
	else
	{
		ins_int.gps_type = GPS_NONE;
		autopilot_set_mode(AP_MODE_FAILSAFE);
	}
}


void ins_int_task(void)
{
	gpss_state_update();

	if((ins_int.gpss_state == RTK_VALID) || (ins_int.gpss_state == RTK_UBLOX_VALID)) // RTK valid
	{
		if (ins_int.rtk_gps_update)
		{
			ins_int.rtk_gps_update = FALSE;

#if INS_USE_GPS_ALT
#define BARO_TO_GPS_TIME	(10)

			if (ins_int.vf_realign)
			{
				if (ins_int_is_rtk_best_accu())
				{
					ins_int.vf_realign = FALSE;
					vff_init(- GPS_B2G_DISTANCE, 0, 0, (- GPS_B2G_DISTANCE - get_first_order_low_pass(&ins_int.baro_z_filter)));
					ins_int.vf_stable = TRUE;
				}
			}

			if (ins_int_is_rtk_pos_z_valid())
			{
				if (ins_int.ekf_state == INS_EKF_BARO)
				{
					ins_int.baro_to_gps_count = 0;
					ins_int.baro_to_gps_offset_step = ((ins_int.rtk_ned_z - get_first_order_low_pass(&ins_int.baro_z_filter))
							- vff.offset) / (float) BARO_TO_GPS_TIME;
					ins_int.baro_to_gps_z_step = (ins_int.rtk_ned_z - vff.z) / (float) BARO_TO_GPS_TIME;
					if (!autopilot_in_flight)
					{
						ins_int.ekf_state = INS_EKF_BARO_TO_GPS;
					}
				}
				else if (ins_int.ekf_state == INS_EKF_BARO_TO_GPS)
				{
					vff.offset += ins_int.baro_to_gps_offset_step;
					vff.z += ins_int.baro_to_gps_z_step;
					//vff_update_z_conf(ins_int.gps_body_z, 10.0f);
					ins_update_from_vff();
					if (++ins_int.baro_to_gps_count >= BARO_TO_GPS_TIME) // 2s
					{
						ins_int.baro_to_gps_count = 0;
						ins_int.ekf_state = INS_EKF_GPS;
					}
				}
				else if (ins_int.ekf_state == INS_EKF_GPS)
				{
					//vff_update_z_conf(ins_int.gps_body_z, ins_int.R_rtk_pos_z);
					vff_update_z_conf(ins_int.rtk_ned_z, ins_int.R_rtk_pos_z_setting);
					vff_update_offset_conf(ins_int.rtk_ned_z - ins_int.baro_ned_z, ins_int.R_baro_offset);
					ins_update_from_vff();
					ins_ned_to_state();
				}
				ins_int_record_rtk_z_hist();
			}
			else
			{
				switch_to_baro();
			}

#endif
			/****************end of vertical infomation fuse*****************/


			/****************start of horizontal infomation fuse*****************/
			if ( ins_int_is_rtk_pos_xy_valid() )
			{
				if (ins_int.gps_type != GPS_RTK)
				{
					if (ins_int_is_rtk_best_accu() && (!autopilot_in_flight))
					{
						ins_int_gps_switch(GPS_RTK);
					}
				}

#if USE_HFF
				VECT2_COPY(ins_int.gps_pos_m_ned, ins_int.rtk_gps_pos_cm_ned);
				VECT2_SDIV(ins_int.gps_pos_m_ned, ins_int.gps_pos_m_ned, 100.0f);

				VECT2_COPY(ins_int.gps_speed_m_ned, ins_int.rtk_gps_speed_cm_ned);
				VECT2_SDIV(ins_int.gps_speed_m_ned, ins_int.gps_speed_m_ned, 100.0f);

				if (ins_int.rtk_hf_realign)
				{
					ins_int.rtk_hf_realign = FALSE;
					b2_hff_realign(ins_int.gps_pos_m_ned, ins_int.gps_speed_m_ned);
					ins_int_set_hf_realign_done(TRUE);
				}

				/*run horizontal filter*/
				float pos_r = ((float) ins_int.rtk_gps_sd_ned.x / 10000.0) * ((float) ins_int.rtk_gps_sd_ned.y / 10000.0);
				pos_r = fabs(pos_r);
				//TEST_CASE : need to add speed_sd
				b2_hff_set_gps_r(pos_r, 5.0f * pos_r);
				b2_hff_update_gps(&ins_int.gps_pos_m_ned, &ins_int.gps_speed_m_ned);
				// convert and copy result to ins_int
				ins_update_from_hff();
				ins_ned_to_state();

#else  /* hff not used */
				/* simply copy horizontal pos/speed from gps */
				INT32_VECT2_SCALE_2(ins_int.ltp_pos, gps_pos_cm_ned,
						INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
				INT32_VECT2_SCALE_2(ins_int.ltp_speed, gps_speed_cm_s_ned,
						INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);
#endif /* USE_HFF */
			}
			else
			{
				switch_to_ublox();
			}
		}
	}
	else
	{
		switch_to_baro();
		switch_to_ublox();
	}

	if((ins_int.gpss_state == UBLOX_VALID) || (ins_int.gpss_state == RTK_UBLOX_VALID))	// UBLOX valid
	{
		if (ins_int.ublox_gps_update)
		{
			ins_int.ublox_gps_update = FALSE;

			if (ins_int.gps_type == GPS_UBLOX)
			{
				VECT2_COPY(ins_int.gps_pos_m_ned, ins_ublox.ned_pos);
				VECT2_COPY(ins_int.gps_speed_m_ned, ins_ublox.ned_vel);

				if (ins_int.ublox_hf_realign)
				{
					ins_int.ublox_hf_realign = FALSE;
					b2_hff_realign(ins_int.gps_pos_m_ned, ins_int.gps_speed_m_ned);
					ins_int_set_hf_realign_done(TRUE);
				}

				b2_hff_set_gps_r(ins_int.R_ublox_pos, ins_int.R_ublox_vel);
				b2_hff_update_gps(&ins_int.gps_pos_m_ned, &ins_int.gps_speed_m_ned);
				ins_update_from_hff();
				ins_ned_to_state();
			}
		}
	}
}

void ins_int_SetType(enum _e_ins_gps_type type)
{
	ins_int_gps_switch(type);
}

void ins_reset_local_origin(void)
{
#if USE_GPS  //called by flightplan init,set gps's postion as local ins (ltp_def is base point of ins)
	if(ins_int_is_rtk_best_accu())
	{
		ltp_def_from_ecef_i(&ins_int.ltp_def, &RTK_GPS.ecef_pos);
		ins_int.ltp_def.lla.alt = RTK_GPS.lla_pos.alt;
		ins_int.ltp_def.hmsl = RTK_GPS.hmsl;
		ins_int.ltp_initialized = TRUE;
		stateSetLocalOrigin_i(&ins_int.ltp_def);
	}
	else
	{
		ins_int.ltp_initialized = FALSE;
	}
#else
	ins_int.ltp_initialized = FALSE;
#endif

#if USE_HFF
	ins_int_set_rtk_hf_realign(TRUE);
#endif
	ins_int.vf_realign = TRUE;
}

void ins_reset_altitude_ref(void)
{
#if USE_GPS //using state.ned_origin to set ins origin
	struct LlaCoor_i lla =
	{ .lat = state.ned_origin_i.lla.lat, .lon = state.ned_origin_i.lla.lon, .alt = RTK_GPS.lla_pos.alt };
	ltp_def_from_lla_i(&ins_int.ltp_def, &lla);
	ins_int.ltp_def.hmsl = RTK_GPS.hmsl;
	stateSetLocalOrigin_i(&ins_int.ltp_def);
#endif
	ins_int.vf_realign = TRUE;
}

//TODOM: start delay time to avoid without accel only caculate G 
static void ins_int_propagate(struct Int32Vect3 *accel, float dt)
{ 
  /*not propagate until system time > 6s*/
  if( get_sys_time_msec()< 6000 )  return;
  if(!ahrs_mlkf.is_aligned)
  {
  	return;
  }
  
  /* untilt accels */
  struct Int32Vect3 accel_meas_body;
  struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&imu.body_to_imu);
  int32_rmat_transp_vmult(&accel_meas_body, body_to_imu_rmat, accel);
  struct Int32Vect3 accel_meas_ltp;
  int32_rmat_transp_vmult(&accel_meas_ltp, stateGetNedToBodyRMat_i(), &accel_meas_body);
  struct FloatVect3 accel_meas_ltp_f;
  ACCELS_FLOAT_OF_BFP(accel_meas_ltp_f, accel_meas_ltp);

	vff_propagate(accel_meas_ltp_f.z, dt);
	ins_update_from_vff();

#if USE_HFF
  /* propagate horizontal filter */
  b2_hff_propagate(accel_meas_ltp_f.x, accel_meas_ltp_f.y);
  /* convert and copy result to ins_int */
  ins_update_from_hff();
#else
  ins_int.ltp_accel.x = accel_meas_ltp.x;
  ins_int.ltp_accel.y = accel_meas_ltp.y;
#endif /* USE_HFF */

  ins_ned_to_state();

  /*filter rate information*/
  ins_body_rate_z = (ins_body_rate_z*9 + stateGetBodyRates_i()->r)/10;
}

#if USE_BARO_BOARD   //change default using bar

static float baro_get_height(float press, float temp)
{
	float tmp_float;
	float alt;

	if ((press < 50000.0f) || (press > 150000.0f))
	{
		return 0;
	}
	tmp_float = (PPRZ_ISA_SEA_LEVEL_PRESSURE / press);
	tmp_float = powf(tmp_float, 1.0f / 5.257f);
	alt = (tmp_float - 1.0f) * (temp + 273.15f) * 153.84615f;

	return alt;
}

static void baro_cb(uint8_t __attribute__((unused)) sender_id,
										uint32_t stamp,
										float pressure,
										float temperature)
{
	static uint32_t last_stamp = 0;

	if (last_stamp > 0)
	{
		float dt = (float) (stamp - last_stamp) * 1e-6;
		ins_int.baro_ned_z = -baro_get_height(pressure, temperature); // - for NED

		if (ins_int.baro_initialized)
		{
			update_first_order_low_pass(&ins_int.baro_z_filter, ins_int.baro_ned_z);
		}

		if (ins_int.ekf_state == INS_EKF_PURE_ACC)
		{
			if ((!autopilot_in_flight) && ins_int_is_baro_valid() && (!ins_int.baro_initialized))
			{
				init_first_order_low_pass(&ins_int.baro_z_filter, low_pass_filter_get_tau(1.0f), 0.05f, ins_int.baro_ned_z);
				vff_init(- GPS_B2G_DISTANCE, 0, 0, (- GPS_B2G_DISTANCE - get_first_order_low_pass(&ins_int.baro_z_filter)));
				ins_int.baro_initialized = TRUE;
				ins_int.ekf_state = INS_EKF_BARO;
			}
		}
		else if (ins_int.ekf_state == INS_EKF_GPS_TO_BARO)
		{
			if (ins_int_is_baro_valid())
			{
				float valid_z, valid_zd;
				ins_int_get_recent_valid_rtk_z(&valid_z, &valid_zd);
				vff_init(valid_z, valid_zd, vff.bias, vff.offset);
				ins_int.ekf_state = INS_EKF_BARO;
			}
			else
			{
				ins_int.ekf_state = INS_EKF_PURE_ACC;
			}
		}
		else if (ins_int.ekf_state == INS_EKF_BARO)
		{
			if (ins_int_is_baro_valid())
			{
				vff_update_baro_conf(ins_int.baro_ned_z, ins_int.R_baro);
				ins_update_from_vff();
				ins_ned_to_state();
			}
			else
			{
				ins_int.ekf_state = INS_EKF_PURE_ACC;
			}
		}
	}
	last_stamp = stamp;
}
#else
static void baro_cb(uint8_t __attribute__((unused)) sender_id,
										uint32_t stamp,
										float pressure,
										float temperature)
{
}
#endif


#if USE_SONAR  //without default bar
static void sonar_cb(uint8_t __attribute__((unused)) sender_id, float distance)
{
  static float last_offset = 0.;
  
  /*pulse filter*/
  /*
  static float last_distance =0.0;

  float deta_distance = distance-last_distance;
  if( fabs(deta_distance) >INS_SONAR_DETA_RANGE )
  {
  	  distance = 0.9*last_distance + 0.1*deta_distance;
  }
  last_distance = distance;
  */
  
  /* update filter assuming a flat ground */
  if (   distance < INS_SONAR_MAX_RANGE 
  	  && distance > INS_SONAR_MIN_RANGE
#ifdef INS_SONAR_THROTTLE_THRESHOLD
      && stabilization_cmd[COMMAND_THRUST] < INS_SONAR_THROTTLE_THRESHOLD
#endif
#ifdef INS_SONAR_BARO_THRESHOLD
      && ins_int.baro_ned_z > -INS_SONAR_BARO_THRESHOLD /* z down */
#endif
      && ins_int.update_on_agl
	  #if 0 //USE_BARO_BOARD
      && ins_int.baro_initialized
      && ins_int.baro_ned_z >-5    //use sonar meas,request baro_z below 5m  --by whp
	  #endif
     ) 
   {
    vff_update_z_conf(-(distance), VFF_R_SONAR_0 + VFF_R_SONAR_OF_M * fabsf(distance));
    last_offset = vff.offset;
  } 
  else 
  {
    /* update offset with last value to avoid divergence */
  	vff_update_offset_conf(last_offset, ins_int.R_baro_offset);
  }

  /* reset the counter to indicate we just had a measurement update */
  ins_int.propagation_cnt = 0;
}
#endif // USE_SONAR


#if USE_RADAR24  //without default bar
static void radar24_cb(uint8_t __attribute__((unused)) sender_id, float distance)
{
  static float last_radar_offset = 0.;
  static float last_distance = 0;
  //static float distance_avr = 0;
  
  /*set deta distance < 1.0*/  
  #if 1
  /* update filter assuming a flat ground */
  if (  distance < INS_RADAR24_MAX_RANGE 
  	 && distance > INS_RADAR24_MIN_RANGE
  	 && fabs(distance-last_distance) < 1.0
  	 && ins_int.update_radar_agl           ) 
   {
   		//distance_avr = 0.8*distance_avr + 0.2*distance;
		//distance = distance_avr;
        vff_update_z_conf(-(distance),  VFF_R_RADAR24_0 + VFF_R_RADAR24_OF_M * fabs(distance));
        last_radar_offset = vff.offset;
  } 
  else if(ins_int.update_radar_agl)
  {
    /* update offset with last value to avoid divergence */
  	vff_update_offset_conf(last_radar_offset, ins_int.R_baro_offset);
  }
  
  last_distance = distance;
  #else
  vff_update_z_conf(-(distance), 0.5);  // VFF_R_SONAR_OF_M * fabsf(distance));
  #endif

  /* reset the counter to indicate we just had a measurement update */
  ins_int.propagation_cnt = 0;
}
#endif // USE_RADAR24

/** initialize the local origin (ltp_def) from flight plan position */
static void ins_init_origin_from_flightplan(void)
{
  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_i(&ins_int.ltp_def, &ecef_nav0);
  ins_int.ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(&ins_int.ltp_def);
}

/** copy position and speed to state interface */
static void ins_ned_to_state(void)
{
  stateSetPositionNed_i(&ins_int.ltp_pos);
  stateSetSpeedNed_i(&ins_int.ltp_speed);
  stateSetAccelNed_i(&ins_int.ltp_accel);

#if defined SITL && USE_NPS
  if (nps_bypass_ins) {
    sim_overwrite_ins();
  }
#endif
}

/** update ins state from vertical filter */
static void ins_update_from_vff(void)
{
  ins_int.ltp_accel.z = ACCEL_BFP_OF_REAL(vff.zdotdot);
  ins_int.ltp_speed.z = SPEED_BFP_OF_REAL(vff.zdot);
  ins_int.ltp_pos.z   = POS_BFP_OF_REAL(vff.z);
}

#if USE_HFF
/** update ins state from horizontal filter */
static void ins_update_from_hff(void)
{
  ins_int.ltp_accel.x = ACCEL_BFP_OF_REAL(b2_hff_state.xdotdot);
  ins_int.ltp_accel.y = ACCEL_BFP_OF_REAL(b2_hff_state.ydotdot);
  ins_int.ltp_speed.x = SPEED_BFP_OF_REAL(b2_hff_state.xdot);
  ins_int.ltp_speed.y = SPEED_BFP_OF_REAL(b2_hff_state.ydot);
  ins_int.ltp_pos.x   = POS_BFP_OF_REAL(b2_hff_state.x);
  ins_int.ltp_pos.y   = POS_BFP_OF_REAL(b2_hff_state.y);
}
#endif


static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp, struct Int32Vect3 *accel)
{
  PRINT_CONFIG_MSG("Calculating dt for INS int propagation.")
  /* timestamp in usec when last callback was received */
  static uint32_t last_stamp = 0;

  if (last_stamp > 0)
  {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ins_int_propagate(accel, dt);
  }
  last_stamp = stamp;
}

static bool_t gps_pos_inspect(struct NedCoor_i data)
{
	static struct NedCoor_i last_pos;
	if(  abs(last_pos.x-data.x) > 1000 || abs(last_pos.y-data.y) > 1000)
	{
		VECT3_COPY(last_pos, data);
		return FALSE;
	}
	else
	{
		VECT3_COPY(last_pos, data);
		return TRUE;
	}
}

static bool_t gps_speed_inspect(struct NedCoor_i data)
{
	if( abs(data.x) > 2000 || abs(data.y) >2000 )
	{
		return FALSE;
	}
	return TRUE;
}

#ifdef GPS_INSTALL_BIAS
/*unit :cm, body frame*/
  #define  INS_BODY_TO_GPS_X  0
  #define  INS_BODY_TO_GPS_Y  33
  #define  INS_BODY_TO_GPS_Z  0
#endif

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  static uint32_t last_stamp = 0;

	if (last_stamp > 0)
	{
		float dt = (float)(stamp - last_stamp) * 1e-6;
	}
	last_stamp = stamp;

	ins_int.rtk_gps_update = TRUE;

	if(gps_s->p_stable)
	{
		if (!ins_int.ltp_initialized)
		{
			ins_reset_local_origin();
		}
		else
		{
			struct NedCoor_i gps_pos_cm_ned;
			ned_of_ecef_point_i(&gps_pos_cm_ned, &ins_int.ltp_def, &gps_s->ecef_pos);
			/*add pos diff inspect, request pos diff <10m*/
			if (gps_pos_inspect(gps_pos_cm_ned))
			{
				VECT3_COPY(ins_int.rtk_gps_pos_cm_ned, gps_pos_cm_ned);
			}

			/*add AC relative height*/
			ins_int.rtk_gps_pos_cm_ned.z = ins_int.rtk_gps_pos_cm_ned.z - (int32_t) (GPS_B2G_DISTANCE * 100);

			struct NedCoor_i gps_speed_cm_ned;
			ned_of_ecef_vect_i(&gps_speed_cm_ned, &ins_int.ltp_def, &gps_s->ecef_vel);
			/* add gps speed inspect,request speed <20m/s*/
			if (gps_speed_inspect(gps_speed_cm_ned))
			{
				VECT3_COPY(ins_int.rtk_gps_speed_cm_ned, gps_speed_cm_ned);
			}

			ned_of_ecef_vect_i(&ins_int.rtk_gps_sd_ned, &ins_int.ltp_def, &gps_s->ecef_pos_sd);

			/* calculate body frame position taking BODY_TO_GPS translation (in cm) into account */
#ifdef GPS_INSTALL_BIAS
			/* body2gps translation in body frame */
			struct Int32Vect3 b2g_b =
			{ .x = INS_BODY_TO_GPS_X, .y = INS_BODY_TO_GPS_Y, .z = INS_BODY_TO_GPS_Z };
			/* rotate offset given in body frame to navigation/ltp frame using current attitude */
			struct Int32Quat q_b2n = *stateGetNedToBodyQuat_i();
			QUAT_INVERT(q_b2n, q_b2n);
			struct Int32Vect3 b2g_n;
			int32_quat_vmult(&b2g_n, &q_b2n, &b2g_b);
			/* subtract body2gps translation in ltp from gps position */
			VECT3_SUB(ins_int.rtk_gps_pos_cm_ned, b2g_n);

			struct Int32Vect3 delta_speed_b, delta_speed_n;
			delta_speed_b.x = (ins_body_rate_z * (-b2g_b.y)) >> INT32_RATE_FRAC;
			delta_speed_b.y = 0;
			delta_speed_b.z = 0;
			int32_quat_vmult(&delta_speed_n, &q_b2n, &delta_speed_b);
			VECT3_SUB(ins_int.rtk_gps_speed_cm_ned, delta_speed_n);
#endif

			ins_int.rtk_ned_z = (float) ins_int.rtk_gps_pos_cm_ned.z * 0.01f;
			ins_int.rtk_ned_zd = (float) ins_int.rtk_gps_speed_cm_ned.z * 0.01f;
		}
	}
}

static void gps2_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  static uint32_t last_stamp = 0;

	if (last_stamp > 0)
	{
		float dt = (float)(stamp - last_stamp) * 1e-6;
	}
	last_stamp = stamp;

	if (gps_s->p_stable && ins_ublox.ltpDef_initialized)
	{
		ins_int.ublox_gps_update = TRUE;
	}
}

static void ins_int_gps_switch(enum _e_ins_gps_type type)
{
	ins_int.gps_type = type;

	if(type == GPS_RTK)
	{
		ins_int_set_rtk_hf_realign(TRUE);
	}
	else
	{
		ins_int_set_ublox_hf_realign(TRUE);
	}
}

static void vel_est_cb(uint8_t sender_id __attribute__((unused)),
                       uint32_t stamp __attribute__((unused)),
                       float x, float y, float z,
                       float noise __attribute__((unused)))
{

  struct FloatVect3 vel_body = {x, y, z};

  /* rotate velocity estimate to nav/ltp frame */
  struct FloatQuat q_b2n = *stateGetNedToBodyQuat_f();
  QUAT_INVERT(q_b2n, q_b2n);
  struct FloatVect3 vel_ned;
  float_quat_vmult(&vel_ned, &q_b2n, &vel_body);

#if USE_HFF
  struct FloatVect2 vel = {vel_ned.x, vel_ned.y};
  struct FloatVect2 Rvel = {noise, noise};

  b2_hff_update_vel(vel,  Rvel);
  ins_update_from_hff();
#else
  ins_int.ltp_speed.x = SPEED_BFP_OF_REAL(vel_ned.x);
  ins_int.ltp_speed.y = SPEED_BFP_OF_REAL(vel_ned.y);
#endif

  ins_ned_to_state();
}


void ins_int_register(void)
{
  ins_register_impl(ins_int_init);

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgIMU_ACCEL_INT32(INS_INT_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgGPS_POS(ABI_BROADCAST, &gps_ev, gps_cb);
  AbiBindMsgGPS_UBX(ABI_BROADCAST, &gps2_ev, gps2_cb);
  AbiBindMsgVELOCITY_ESTIMATE(INS_INT_VEL_ID, &vel_est_ev, vel_est_cb);
}




