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

#if USE_FLOW
#include "modules/optical_flow/px4_flow_i2c.h"
#include "subsystems/ins/flow_hf_float.h"
#include "subsystems/ins/hf_float.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
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

//float   sonar_distance,sonar_distance_i;

#if USE_BARO_BOARD
#define BARO_OFFSET 0.3   //baro upper agl 0.3m
#endif

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

abi_event radar24_ev;
static void radar24_cb(uint8_t sender_id, float distance);

struct InsInt ins_int;

float gps_noise_debug;

#if USE_FLOW
static abi_event flow_ev;  //add for flow
static void flow_cb(uint8_t sender_id, struct Px4_flow_Data *flow_data);
static void flow_to_state(struct HfilterFloat *flow_hff);
#else
void ins_int_update_flow(struct Px4_flow_Data *flow_data __attribute__((unused)));
#endif
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

static void ins_int_z_cmpl_update(float dt);
static void ins_int_z_cmpl_corr_gps(struct GpsState *gps_s, float dt);
static void ins_int_z_cmpl_corr_baro(float height, float dt);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  pprz_msg_send_INS(trans, dev, AC_ID,
                    &ins_int.ltp_pos.x, &ins_int.ltp_pos.y, &ins_int.ltp_pos.z,
                    &ins_int.ltp_speed.x, &ins_int.ltp_speed.y, &ins_int.ltp_speed.z,
                    &ins_int.ltp_accel.x, &ins_int.ltp_accel.y, &ins_int.ltp_accel.z,
										&ins_int.zp_baro,
										&ins_int.zp_est,
										&ins_int.zv_est,
										&ins_int.za_est);
}

static void send_ins_z(struct transport_tx *trans, struct link_device *dev)
{
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  pprz_msg_send_INS_Z(trans, dev, AC_ID,
                      &ins_int.baro_z, &ins_int.ltp_pos.z, &ins_int.ltp_speed.z, &ins_int.ltp_accel.z);
}

static void send_ins_ref(struct transport_tx *trans, struct link_device *dev)
{
  if (ins_int.ltp_initialized) {
  	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
    pprz_msg_send_INS_REF(trans, dev, AC_ID,
                          &ins_int.ltp_def.ecef.x, &ins_int.ltp_def.ecef.y, &ins_int.ltp_def.ecef.z,
                          &ins_int.ltp_def.lla.lat, &ins_int.ltp_def.lla.lon, &ins_int.ltp_def.lla.alt,
                          &ins_int.ltp_def.hmsl, &ins_int.qfe);
  }
}
#endif

void ins_int_init(void)
{

#if USE_INS_NAV_INIT
  ins_init_origin_from_flightplan();
  ins_int.ltp_initialized = TRUE;
#else
  ins_int.ltp_initialized  = FALSE;  //via NavSetGroundReferenceHere() set true
#endif

  /* we haven't had any measurement updates yet, so set the counter to max */
  ins_int.propagation_cnt = INS_MAX_PROPAGATION_STEPS;

  // Bind to BARO_ABS message,not default
#if USE_BARO_BOARD
  AbiBindMsgBARO_ABS(INS_BARO_ID, &baro_ev, baro_cb);
  ins_int.baro_initialized = FALSE;
  ins_int.baro_valid =FALSE;
  ins_int.R_baro = 1000.0f;
#endif

#if USE_SONAR
  ins_int.update_on_agl = FALSE;  //INS_SONAR_UPDATE_ON_AGL; default FALSE, it will change in mornitoring
  // Bind to AGL message
  AbiBindMsgAGL(INS_SONAR_ID, &sonar_ev, sonar_cb);
#endif
#if USE_RADAR24
    ins_int.update_radar_agl = FALSE; 
    AbiBindMsgRADAR_24(AGL_NRA_24_ID,&radar24_ev,radar24_cb);
#endif

  ins_int.vf_reset = FALSE;
  ins_int.hf_realign = FALSE;

  /* init vertical and horizontal filters   all set 0 */
  vff_init_zero();
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
#endif

  ins_int.za_corr_k = 0.5f;
  ins_int.zv_corr_k = 2.0f;
  ins_int.zp_corr_k = 4.0f;
  gps_noise_debug = 0.0004;
}

void ins_reset_local_origin(void)
{
#if USE_GPS  //called by flightplan init,set gps's postion as local ins (ltp_def is base point of ins)
  if (GpsFixValid()) {
    ltp_def_from_ecef_i(&ins_int.ltp_def, &gps.ecef_pos);
    ins_int.ltp_def.lla.alt = gps.lla_pos.alt;
    ins_int.ltp_def.hmsl = gps.hmsl;
    ins_int.ltp_initialized = TRUE;
    stateSetLocalOrigin_i(&ins_int.ltp_def);
  }
  else {
    ins_int.ltp_initialized = FALSE;
  }
#else
  ins_int.ltp_initialized = FALSE;
#endif

#if USE_HFF
  ins_int.hf_realign = TRUE;
#endif
  ins_int.vf_reset = TRUE;
}

void ins_reset_altitude_ref(void)
{
#if USE_GPS //using state.ned_origin to set ins origin
  struct LlaCoor_i lla = {
    .lat = state.ned_origin_i.lla.lat,
    .lon = state.ned_origin_i.lla.lon,
    .alt = gps.lla_pos.alt
  };
  ltp_def_from_lla_i(&ins_int.ltp_def, &lla);
  ins_int.ltp_def.hmsl = gps.hmsl;
  stateSetLocalOrigin_i(&ins_int.ltp_def);
#endif
  ins_int.vf_reset = TRUE;
}

//TODOM: start delay time to avoid without accel only caculate G 
void ins_int_propagate(struct Int32Vect3 *accel, float dt)
{ 
  /*not propagate until system time > 6s*/
  if( get_sys_time_msec()< 6000 )  return;  
  
  /* untilt accels */
  struct Int32Vect3 accel_meas_body;
  struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&imu.body_to_imu);
  int32_rmat_transp_vmult(&accel_meas_body, body_to_imu_rmat, accel);
  struct Int32Vect3 accel_meas_ltp;
  int32_rmat_transp_vmult(&accel_meas_ltp, stateGetNedToBodyRMat_i(), &accel_meas_body);

  float z_accel_meas_float = ACCEL_FLOAT_OF_BFP(accel_meas_ltp.z);

  /* Propagate only if we got any measurement during the last INS_MAX_PROPAGATION_STEPS.
   * Otherwise halt the propagation to not diverge and only set the acceleration.
   * This should only be relevant in the startup phase when the baro is not yet initialized
   * and there is no gps fix yet...
   */
  if (ins_int.propagation_cnt < INS_MAX_PROPAGATION_STEPS) 
  {
    vff_propagate(z_accel_meas_float, dt);
    ins_update_from_vff();
  } 
  else 
  {
    // feed accel from the sensors
    // subtract -9.81m/s2 (acceleration measured due to gravity,
    // but vehicle not accelerating in ltp)
    ins_int.ltp_accel.z = accel_meas_ltp.z + ACCEL_BFP_OF_REAL(9.81);
  }

#if USE_HFF
  /* propagate horizontal filter */
  b2_hff_propagate();
  /* convert and copy result to ins_int */
  ins_update_from_hff();
#else
  ins_int.ltp_accel.x = accel_meas_ltp.x;
  ins_int.ltp_accel.y = accel_meas_ltp.y;
#endif /* USE_HFF */

  ins_ned_to_state();

  /* increment the propagation counter, while making sure it doesn't overflow */
  if (ins_int.propagation_cnt < 100 * INS_MAX_PROPAGATION_STEPS) {
    ins_int.propagation_cnt++;
  }
  
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
		float dt = (float)(stamp - last_stamp) * 1e-6;
		ins_int.baro_z = baro_get_height(pressure, temperature);
		//ins_int_z_cmpl_corr_baro( ins_int.baro_z, dt);
		//vff_update_baro(-baro_height);
		if(!ins_int.baro_initialized)
		{
			ins_int.baro_initialized = true;
			vff_realign(0);
			ins_update_from_vff();
			ins_ned_to_state();
		}
		else
		{
			vff_update_baro_conf(-ins_int.baro_z, ins_int.R_baro);
			ins_ned_to_state();
			ins_int.propagation_cnt = 0;
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


//#if USE_BARO_BOARD   //change default using bar
//static void baro_cb(uint8_t __attribute__((unused)) sender_id, float pressure)
//{
//  return;
//
//
//
//  static uint8_t baro_modify_counter=0;
//  //static float last_agl_dist=0.0;
//  static float pressure_origin;
//  float pressure_caculate;
//  if (!ins_int.baro_initialized && pressure > 1e-7 && ins_int.baro_valid) {
//    // wait for a first positive value
//    ins_int.qfe = pressure;
//	pressure_origin=pressure;    //record origin pressure
//    ins_int.baro_initialized = TRUE;
//  }
//
//  if (ins_int.baro_initialized)
//  {
//    if (ins_int.vf_reset)
//    {
//      ins_int.vf_reset = FALSE;
//      ins_int.qfe = pressure;
//	  pressure_origin=pressure;    //record origin pressure
//      vff_realign(-BARO_OFFSET);    //init state
//      ins_update_from_vff();
//      ins_ned_to_state();
//    }
//    else if(ins_int.baro_valid)
//	{
//	  #if 0 //false, not use sonar calibrate baro
//   	  /*use sonar dist modify ins_int.qfe, by whp*/
//   	  if( ins_int.update_on_agl && agl_dist_value_filtered <2.0 )
//	  //( (last_agl_dist!=agl_dist_value_filtered) && (agl_dist_value_filtered <2.2) )
//	  {
//	  	baro_modify_counter++;
//		if(baro_modify_counter==100)
//		{
//			pressure_caculate = pprz_isa_pressure_of_height(agl_dist_value_filtered-BARO_OFFSET, ins_int.qfe);
//			ins_int.qfe -=(pressure_caculate - pressure);
//			//if( ins_int.qfe>(pressure_origin+0.2) )  ins_int.qfe=pressure_origin+0.2;
//			//else if( ins_int.qfe<(pressure_origin-0.1) )  ins_int.qfe=pressure_origin-0.1;
//			Bound( (ins_int.qfe), (pressure_origin-10.0), (pressure_origin+20.0) );   //about limit in about -1 to 2m
//			baro_modify_counter=0;
//		}
//	  }
//	  else
//	  {
//	  	baro_modify_counter=0;   //make sure sonar information is stable
//	  }
//	  //last_agl_dist=agl_dist_value_filtered;
//
//      ins_int.baro_z = -pprz_isa_height_of_pressure(pressure, ins_int.qfe) - BARO_OFFSET;  //ISA conditions
//
//      //baro_z below 1.0m or sonar dist below 2.0m,only use sonar. --by whp
//      if( ins_int.update_on_agl )   //sonar useful
//      {
//	  	  if( ins_int.baro_z> -1.0 || agl_dist_value_filtered< 2.0 )   return;
//      }
//	  //else must use baro_z to update ins
//	  #else
//	  ins_int.baro_z = -pprz_isa_height_of_pressure(pressure, ins_int.qfe) - BARO_OFFSET;  //ISA conditions
//	  #endif
//
//#if USE_VFF_EXTENDED
//      vff_update_baro(ins_int.baro_z);
//#else
//      vff_update(ins_int.baro_z);
//#endif
//      ins_ned_to_state();
//    }
//
//    /* reset the counter to indicate we just had a measurement update */
//    ins_int.propagation_cnt = 0;
//  }
//
//  //TODOM:
//  //if(pressure<1e-7) {ins_int.qfe=5000.0;ins_int.baro_z=120;}
//}
//#else
//static void baro_cb(uint8_t __attribute__((unused)) sender_id, float pressure) {}
//#endif


/**********************************************/
/*************here is gps cb function**********/
/**********************************************/
#if USE_GPS

#ifdef GPS_INSTALL_BIAS
/*unit :cm, body frame*/
  #define  INS_BODY_TO_GPS_X  0
  #define  INS_BODY_TO_GPS_Y  33
  #define  INS_BODY_TO_GPS_Z  0
#endif
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

void ins_int_update_gps(struct GpsState *gps_s)
{
  if (gps_s->fix < GPS_FIX_3D) {
    return;
  }
  
#if USE_FLOW
  if ( guidance_h.mode==GUIDANCE_H_MODE_HOVER || guidance_h.mode==GUIDANCE_H_MODE_ATTITUDE) {   //using flow in hover mode,GPS data giving up
    return;
  }
#endif

  if (!ins_int.ltp_initialized && gps.p_stable)
  {
    ins_reset_local_origin();
  }

  /*get relative postion*/
  struct NedCoor_i gps_pos_cm_ned;
  ned_of_ecef_point_i(&gps_pos_cm_ned, &ins_int.ltp_def, &gps_s->ecef_pos);
  /*add pos diff inspect, request pos diff <10m*/
  if( !gps_pos_inspect(gps_pos_cm_ned) ) 
  {
  	return;  
  }

   /*get relative speed*/
  struct NedCoor_i gps_speed_cm_s_ned;
  ned_of_ecef_vect_i(&gps_speed_cm_s_ned, &ins_int.ltp_def, &gps_s->ecef_vel);
  /* add gps speed inspect,request speed <20m/s*/
  if( !gps_speed_inspect(gps_speed_cm_s_ned) ) 
  {
  	return;
  }

  /*get pos standard deviation*/
  struct NedCoor_i gps_pos_sd;   //unit 1000*m
  ned_of_ecef_vect_i(&gps_pos_sd, &ins_int.ltp_def, &gps_s->ecef_pos_sd);
  
  /* calculate body frame position taking BODY_TO_GPS translation (in cm) into account */
#ifdef GPS_INSTALL_BIAS
  /* body2gps translation in body frame */
  struct Int32Vect3 b2g_b = {
    .x = INS_BODY_TO_GPS_X,
    .y = INS_BODY_TO_GPS_Y,
    .z = INS_BODY_TO_GPS_Z
  };
  /* rotate offset given in body frame to navigation/ltp frame using current attitude */
  struct Int32Quat q_b2n = *stateGetNedToBodyQuat_i();
  QUAT_INVERT(q_b2n, q_b2n);
  struct Int32Vect3 b2g_n;
  int32_quat_vmult(&b2g_n, &q_b2n, &b2g_b);
  /* subtract body2gps translation in ltp from gps position */
  VECT3_SUB(gps_pos_cm_ned, b2g_n);

  /*filter rate information*/
  static int32_t ins_body_rate_z;
  int32_t ins_body_rate_z_now;
  ins_body_rate_z_now =  stateGetBodyRates_i()->r;  
  ins_body_rate_z =  (ins_body_rate_z*3 + ins_body_rate_z_now)/4;
  struct Int32Vect3 delta_speed_b, delta_speed_n;
  delta_speed_b.x = (ins_body_rate_z * (-b2g_b.y)) >>INT32_RATE_FRAC;
  delta_speed_b.y = 0;
  delta_speed_b.z = 0;
  int32_quat_vmult(&delta_speed_n, &q_b2n, &delta_speed_b);
  VECT3_SUB(gps_speed_cm_s_ned, delta_speed_n);
  
#endif

/*airframe height*/
 gps_pos_cm_ned.z = gps_pos_cm_ned.z - (int32_t)(DISTANCE_B2G*100 + 2);  

#if INS_USE_GPS_ALT
  if(gps.p_stable)//gps_nmea.gps_qual==52)
  {
  	ins_int.update_radar_agl = FALSE;
  	vff_update_z_conf(((float)gps_pos_cm_ned.z) / 100.0, gps_noise_debug);
    //vff_update_z_conf(((float)gps_pos_cm_ned.z) / 100.0, INS_VFF_R_GPS*0.15);
	//vff_update_zd_conf(((float)gps_speed_cm_s_ned.z) / 100.0, INS_VFF_R_GPS*0.15);
  }
  /*
  else
  {
  	ins_int.update_radar_agl = TRUE;
  }
*/
  else  /*if R <1.0, use the RTK Z info*/
  {
  	float r_unstable =(float)gps_pos_sd.z /10000.0;
	r_unstable = r_unstable * r_unstable;
	if( r_unstable< 0.001)
	{
		r_unstable = 0.001;
	}
	if(r_unstable <0.25)
	{
		 vff_update_z_conf(((float)gps_pos_cm_ned.z) / 100.0, r_unstable );
	}
	else
	{
		/*need use baro*/
	}
  }

 #if PERIODIC_TELEMETRY
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  DOWNLINK_SEND_DEBUG_GPS(DefaultChannel, DefaultDevice, &gps_pos_cm_ned.z, &gps_speed_cm_s_ned.z, &gps_nmea.gps_qual,
  	                                                     &gps_pos_sd.x, &gps_pos_sd.y, &gps_pos_sd.z);
  //DOWNLINK_SEND_DEBUG_GPS(DefaultChannel, DefaultDevice, &gps_pos_cm_ned.x, &gps_pos_cm_ned.y, &gps_nmea.gps_qual,
  //	                                                     &gps_speed_cm_s_ned.x, &gps_speed_cm_s_ned.y, &gps_speed_cm_s_ned.z);
 #endif
#endif

#if USE_HFF
  /* horizontal gps transformed to NED in meters as float */
  struct FloatVect2 gps_pos_m_ned;
  VECT2_ASSIGN(gps_pos_m_ned, gps_pos_cm_ned.x, gps_pos_cm_ned.y);
  VECT2_SDIV(gps_pos_m_ned, gps_pos_m_ned, 100.0f);

  struct FloatVect2 gps_speed_m_s_ned;
  VECT2_ASSIGN(gps_speed_m_s_ned, gps_speed_cm_s_ned.x, gps_speed_cm_s_ned.y);
  VECT2_SDIV(gps_speed_m_s_ned, gps_speed_m_s_ned, 100.);

  if (ins_int.hf_realign) {
    ins_int.hf_realign = FALSE;
    const struct FloatVect2 zero = {0.0f, 0.0f};
    b2_hff_realign(gps_pos_m_ned, zero);
  }
  
  /*run horizontal filter*/
  float pos_r =((float)gps_pos_sd.x /10000.0) * ((float)gps_pos_sd.y /10000.0);
  pos_r = fabs(pos_r);
  b2_hff_update_gps_r(pos_r);
  b2_hff_update_gps(&gps_pos_m_ned, &gps_speed_m_s_ned);
  // convert and copy result to ins_int
  ins_update_from_hff();

#else  /* hff not used */
  /* simply copy horizontal pos/speed from gps */
  INT32_VECT2_SCALE_2(ins_int.ltp_pos, gps_pos_cm_ned,
                      INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
  INT32_VECT2_SCALE_2(ins_int.ltp_speed, gps_speed_cm_s_ned,
                      INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);
#endif /* USE_HFF */

  ins_ned_to_state();

  /* reset the counter to indicate we just had a measurement update */
  ins_int.propagation_cnt = 0;
}
#else
void ins_int_update_gps(struct GpsState *gps_s __attribute__((unused))) {}
#endif /* USE_GPS */



#if USE_FLOW
/**** predic convert flow coords pos to NED *****/
static void flow_to_state(struct HfilterFloat *flow_hff)   
{  
   struct NedCoor_f pos_flow, speed_flow;
   float psi= stateGetNedToBodyEulers_f()->psi; //yaw angle
   float s_psi = sinf(psi);
   float c_psi = cosf(psi);
   pos_flow.x = c_psi * flow_hff->x - s_psi * flow_hff->y;
   pos_flow.y = s_psi * flow_hff->x + c_psi * flow_hff->y;
   speed_flow.x = c_psi * flow_hff->xdot - s_psi * flow_hff->ydot;
   speed_flow.y = s_psi * flow_hff->xdot + c_psi * flow_hff->ydot;
		   //pos_flow.z=stateGetPositionNed_f()->z;
		   //speed_flow.z=stateGetSpeedNed_f()->z;	   
   VECT2_ADD(pos_flow,local_flow_pos);  //add local pos to absolute
#if 1
   b2_hff_state.x = pos_flow.x;
   b2_hff_state.y = pos_flow.y;
   b2_hff_state.xdot=(b2_hff_state.xdot * 4.0 + speed_flow.x) /5.0; //smooth the speed
   b2_hff_state.ydot=(b2_hff_state.ydot * 4.0 + speed_flow.y) /5.0;
   ins_update_from_hff();
   ins_ned_to_state();
#else
		   //stateSetPositionNed_f(&pos_flow);
		   //stateSetSpeedNed_f(&speed_flow);  
   #if PERIODIC_TELEMETRY
   RunOnceEvery(10, (xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
       DOWNLINK_SEND_FLOW_NED(DefaultChannel, DefaultDevice, 
         &pos_flow.x, &pos_flow.y, &speed_flow.x, &speed_flow.y) ));
   #endif
#endif
}

void ins_int_update_flow(struct Px4_flow_Data *flow_data)
{ static uint8_t count_time=0;
  
  if(guidance_h_mode==GUIDANCE_H_MODE_HOVER||guidance_h_mode==GUIDANCE_H_MODE_ATTITUDE)  //running in hover mode
  { 
  	//if(flow_data->qual<50) return;  //flow quality lower,give up
  	count_time++;
#if 0//USE_HFF
  /* horizontal gps transformed to NED in meters as float */
   struct FloatVect2 *pos_flow, *speed_flow;
   pos_flow->x=flow_data->sum_x;
   pos_flow->y=flow_data->sum_y;
   speed_flow->x=flow_data->flow_comp_m_x/1000.0;
   speed_flow->y=flow_data->flow_comp_m_y/1000.0;  
  // run horizontal filter
   hff_update_flow(pos_flow, speed_flow, flow_data->qual);
  
#else  /* hff not used */
  /* simply add horizontal pos/speed from flow */
   if(!flow_hff_state.rollback) flow_hff_init(0.0, 0.0, 0.0, 0.0);  //init local point
   flow_hff_state.x+=flow_data->sum_x;
   flow_hff_state.y+=flow_data->sum_y;
   flow_hff_state.xdot=flow_data->flow_comp_m_x/1000.0;
   flow_hff_state.ydot=flow_data->flow_comp_m_y/1000.0;  
#endif /* USE_HFF */

  if(count_time==10)   //lower the fre calculate NED pos to 10Hzna
  { count_time=0;
    //conver flow coords to ned,and send pos to state
    flow_to_state(&flow_hff_state);
  }
    /* reset the counter to indicate we just had a measurement update */
  ins_int.propagation_cnt = 0;
  }
}

#else
void ins_int_update_flow(struct Px4_flow_Data *flow_data __attribute__((unused))) {}
#endif/* USE_FLOW */


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
      && ins_int.baro_z > -INS_SONAR_BARO_THRESHOLD /* z down */
#endif
      && ins_int.update_on_agl
	  #if 0 //USE_BARO_BOARD
      && ins_int.baro_initialized
      && ins_int.baro_z >-5    //use sonar meas,request baro_z below 5m  --by whp
	  #endif
     ) 
   {
    vff_update_z_conf(-(distance), VFF_R_SONAR_0 + VFF_R_SONAR_OF_M * fabsf(distance));
    last_offset = vff.offset;
  } 
  else 
  {
    /* update offset with last value to avoid divergence */
    vff_update_offset(last_offset);
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
        vff_update_offset(last_radar_offset);
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

  if (last_stamp > 0) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ins_int_propagate(accel, dt);
    ins_int_z_cmpl_update(dt);
  }
  last_stamp = stamp;
}

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  //ins_int_update_gps(gps_s);

  static uint32_t last_stamp = 0;

	if (last_stamp > 0)
	{
		float dt = (float)(stamp - last_stamp) * 1e-6;
		//ins_int_z_cmpl_corr_gps(gps_s, dt);
	}
	last_stamp = stamp;
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

  /* reset the counter to indicate we just had a measurement update */
  ins_int.propagation_cnt = 0;
}

#if USE_FLOW
static void flow_cb(uint8_t sender_id __attribute__((unused)), struct Px4_flow_Data *flow_data)
{
  ins_int_update_flow(flow_data);
}
#endif

void ins_int_register(void)
{
  ins_register_impl(ins_int_init);

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgIMU_ACCEL_INT32(INS_INT_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, gps_cb);
  AbiBindMsgVELOCITY_ESTIMATE(INS_INT_VEL_ID, &vel_est_ev, vel_est_cb);
  #if USE_FLOW
  AbiBindMsgFLOW(ABI_BROADCAST, &flow_ev, flow_cb);
  #endif
}

static void ins_int_z_cmpl_update(float dt)
{
	int32_t z = stateGetAccelNed_i()->z;
	ins_int.za_acc = ACCEL_FLOAT_OF_BFP(z);
	ins_int.za_est = ins_int.za_acc + ins_int.za_corr;
	ins_int.zv_inc = ins_int.za_est * dt;
	ins_int.zp_base += (ins_int.zv_est + ins_int.zv_inc * 0.5f) * dt;
	ins_int.zp_est = ins_int.zp_base + ins_int.zp_corr;
	ins_int.zv_est += ins_int.zv_inc;
}

static void ins_int_z_cmpl_corr_gps(struct GpsState *gps_s, float dt)
{
	float zv_err;

	struct NedCoor_i gps_speed_cm_s_ned;
	ned_of_ecef_vect_i(&gps_speed_cm_s_ned, &ins_int.ltp_def, &gps_s->ecef_vel);

	ins_int.zv_gps = gps_speed_cm_s_ned.z / 100.0f;
	zv_err = ins_int.zv_gps - ins_int.zv_est;
	ins_int.za_corr += zv_err * ins_int.za_corr_k * dt;
	ins_int.zv_est += zv_err * ins_int.zv_corr_k * dt;
}

static void ins_int_z_cmpl_corr_baro(float height, float dt)
{
	float zp_err;

	ins_int.zp_baro = -height;
	zp_err = ins_int.zp_baro - ins_int.zp_est;

	ins_int.za_corr += zp_err * ins_int.za_corr_k * dt;
	ins_int.zv_est += zp_err * ins_int.zv_corr_k * dt;
	ins_int.zp_corr += zp_err * ins_int.zp_corr_k * dt;
}




