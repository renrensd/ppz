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

#include "subsystems/abi.h"

#include "subsystems/imu.h"
//#include "subsystems/ahrs/ahrs_float_cmpl.h"    //add for get ahrs aligher
#include "subsystems/gps.h"

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

#include "generated/flight_plan.h"

float   sonar_distance,sonar_distance_i;

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
#define VFF_R_SONAR_0 0.1
#ifndef VFF_R_SONAR_OF_M
#define VFF_R_SONAR_OF_M 0.1//old is 0.2
#endif

#ifndef INS_SONAR_UPDATE_ON_AGL
#define INS_SONAR_UPDATE_ON_AGL FALSE
PRINT_CONFIG_MSG("INS_SONAR_UPDATE_ON_AGL defaulting to FALSE")
#endif
#endif // USE_SONAR

#ifndef INS_VFF_R_GPS
#define INS_VFF_R_GPS 2.0
#endif

/** maximum number of propagation steps without any updates in between */
#ifndef INS_MAX_PROPAGATION_STEPS
#define INS_MAX_PROPAGATION_STEPS 200
#endif

#ifndef USE_INS_NAV_INIT
#define USE_INS_NAV_INIT TRUE
PRINT_CONFIG_MSG("USE_INS_NAV_INIT defaulting to TRUE")
#endif

#ifdef INS_BARO_SENS
#warning INS_BARO_SENS is obsolete, please remove it from your airframe file.
#endif

/** default barometer to use in INS */
#ifndef INS_BARO_ID
#if USE_BARO_BOARD
#define INS_BARO_ID    BARO_BOARD_SENDER_ID   //BARO_BMP_SENDER_ID   //TODOM: is not BARO_BOARD_SENDER_ID,without temp compensated
#else
#define INS_BARO_ID ABI_BROADCAST
#endif
#endif
PRINT_CONFIG_VAR(INS_BARO_ID)
abi_event baro_ev;
static void baro_cb(uint8_t sender_id, float pressure);

/** ABI binding for IMU data.
 * Used accel ABI messages.
 */
#ifndef INS_INT_IMU_ID
#define INS_INT_IMU_ID ABI_BROADCAST
#endif
static abi_event accel_ev;
static abi_event gps_ev;
#if USE_FLOW
static abi_event flow_ev;  //add for flow
static void flow_cb(uint8_t sender_id, struct Px4_flow_Data *flow_data);
static void flow_to_state(struct HfilterFloat *flow_hff);
#endif

struct InsInt ins_int;

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

static void ins_init_origin_from_flightplan(void);
static void ins_ned_to_state(void);
static void ins_update_from_vff(void);
#if USE_HFF
static void ins_update_from_hff(void);
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
#endif

#if USE_SONAR
  ins_int.update_on_agl = INS_SONAR_UPDATE_ON_AGL;
  // Bind to AGL message
  AbiBindMsgAGL(INS_SONAR_ID, &sonar_ev, sonar_cb);
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
  register_periodic_telemetry(DefaultPeriodic, "INS", send_ins);
  register_periodic_telemetry(DefaultPeriodic, "INS_Z", send_ins_z);
  register_periodic_telemetry(DefaultPeriodic, "INS_REF", send_ins_ref);
#endif
}

void ins_reset_local_origin(void)
{
#if USE_GPS  //called by flightplan init,set gps's postion as local ins (ltp_def is base point of ins)
  #if 0
    gps.ecef_pos.x=-239003472;
    gps.ecef_pos.y=-538745928;
	gps.ecef_pos.z=-242981776;
	gps.lla_pos.alt=12250;
	gps.hmsl=15004;
    ltp_def_from_ecef_i(&ins_int.ltp_def, &gps.ecef_pos);
    ins_int.ltp_def.lla.alt = gps.lla_pos.alt;
    ins_int.ltp_def.hmsl = gps.hmsl;
    ins_int.ltp_initialized = TRUE;
    stateSetLocalOrigin_i(&ins_int.ltp_def);
  #else
  if (gps.fix == GPS_FIX_3D) {
    ltp_def_from_ecef_i(&ins_int.ltp_def, &gps.ecef_pos);
    ins_int.ltp_def.lla.alt = gps.lla_pos.alt;
    ins_int.ltp_def.hmsl = gps.hmsl;
    ins_int.ltp_initialized = TRUE;
    stateSetLocalOrigin_i(&ins_int.ltp_def);
  }
  else {
    ins_int.ltp_initialized = FALSE;
  }
  #endif
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

void ins_int_propagate(struct Int32Vect3 *accel, float dt)
{ static int32_t i=6000;    //TODOM: start delay time to avoid without accel only caculate G 
  if(i==0){
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
  if (ins_int.propagation_cnt < INS_MAX_PROPAGATION_STEPS)  {   
  	    vff_propagate(z_accel_meas_float, dt);   
  	    ins_update_from_vff();
  } else {
    // feed accel from the sensors
    // subtract -9.81m/s2 (acceleration measured due to gravity,
    // but vehicle not accelerating in ltp)
    ins_int.ltp_accel.z = accel_meas_ltp.z + ACCEL_BFP_OF_REAL(9.81);
  }

#if  USE_HFF
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
  if (ins_int.propagation_cnt < 100*INS_MAX_PROPAGATION_STEPS) {
    ins_int.propagation_cnt++;
  }
 }
 else i--;
}

#if USE_BARO_BOARD   //change default using bar
static void baro_cb(uint8_t __attribute__((unused)) sender_id, float pressure)
{
  static uint8_t baro_modify_conter=0;
  static float last_agl_dist=0.0;
  static float pressure_origin;
  float pressure_caculate;
  if (!ins_int.baro_initialized && pressure > 1e-7) {
    // wait for a first positive value
    ins_int.qfe = pressure;
	pressure_origin=pressure;    //record origin pressure
    ins_int.baro_initialized = TRUE;
  }

  if (ins_int.baro_initialized)
  {
    if (ins_int.vf_reset)
    {
      ins_int.vf_reset = FALSE;
      ins_int.qfe = pressure;
	  pressure_origin=pressure;    //record origin pressure
      vff_realign(0.);  //init state
      ins_update_from_vff();
      ins_ned_to_state();
    }
   else {
   	  /*use sonar dist modify ins_int.qfe*/
	  /*by whp*/	   
   	  if( (last_agl_dist!=agl_dist_value_filtered) && (agl_dist_value_filtered <2.2) )
	  { 
	  	baro_modify_conter++;
		if(baro_modify_conter==100)
		{   
			pressure_caculate = pprz_isa_pressure_of_height(agl_dist_value_filtered, ins_int.qfe);
			ins_int.qfe -=(pressure_caculate - pressure);
			//if( ins_int.qfe>(pressure_origin+0.2) )  ins_int.qfe=pressure_origin+0.2;
			//else if( ins_int.qfe<(pressure_origin-0.1) )  ins_int.qfe=pressure_origin-0.1;
			Bound( (ins_int.qfe), (pressure_origin-10.0), (pressure_origin+20.0) );   //limit in about -1 to 2m
			baro_modify_conter=0;
		}
	  }
	  last_agl_dist=agl_dist_value_filtered;
	  
      ins_int.baro_z = -pprz_isa_height_of_pressure(pressure, ins_int.qfe);  //ISA conditions
      //baro_z below 1.0m or sonar dist below 2.0m,only use sonar. --by whp
      if( (ins_int.baro_z> -1.0)||(agl_dist_value_filtered< 2.0)  )   return;      
#if USE_VFF_EXTENDED
      vff_update_baro(ins_int.baro_z);
#else
      vff_update(ins_int.baro_z);
#endif
      ins_ned_to_state();
    }
    
    /* reset the counter to indicate we just had a measurement update */
    ins_int.propagation_cnt = 0; 
  }
  
  //TODOM:
  //if(pressure<1e-7) {ins_int.qfe=5000.0;ins_int.baro_z=120;}
}
#else 
static void baro_cb(uint8_t __attribute__((unused)) sender_id, float pressure) {}
#endif


#if USE_GPS
void ins_int_update_gps(struct GpsState *gps_s)
{
  if (gps_s->fix != GPS_FIX_3D ) {
    return;
  }
  
#if USE_FLOW
  if ( guidance_h_mode==GUIDANCE_H_MODE_HOVER || guidance_h_mode==GUIDANCE_H_MODE_ATTITUDE) {   //using flow in hover mode,GPS data giving up
    return;
  }
#endif

  if (!ins_int.ltp_initialized) {
    ins_reset_local_origin();
  }

  struct NedCoor_i gps_pos_cm_ned;
  ned_of_ecef_point_i(&gps_pos_cm_ned, &ins_int.ltp_def, &gps_s->ecef_pos);

  /* calculate body frame position taking BODY_TO_GPS translation (in cm) into account */
#ifdef INS_BODY_TO_GPS_X
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
#endif

  /// @todo maybe use gps_s->ned_vel directly??
  struct NedCoor_i gps_speed_cm_s_ned;
  ned_of_ecef_vect_i(&gps_speed_cm_s_ned, &ins_int.ltp_def, &gps_s->ecef_vel);

#if INS_USE_GPS_ALT
  vff_update_z_conf(((float)gps_pos_cm_ned.z) / 100.0, INS_VFF_R_GPS);
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
  // run horizontal filter
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
   speed_flow.x=c_psi * flow_hff->xdot - s_psi * flow_hff->ydot;
   speed_flow.y = s_psi * flow_hff->xdot + c_psi * flow_hff->ydot;
		   //pos_flow.z=stateGetPositionNed_f()->z;
		   //speed_flow.z=stateGetSpeedNed_f()->z;	   
   VECT2_ADD(pos_flow,local_flow_pos);  //add local pos to absolute
#if 1
   b2_hff_state.x=pos_flow.x;
   b2_hff_state.y=pos_flow.y;
   b2_hff_state.xdot=(b2_hff_state.xdot * 4.0 + speed_flow.x) /5.0; //smooth the speed
   b2_hff_state.ydot=(b2_hff_state.ydot * 4.0 + speed_flow.y) /5.0;
   ins_update_from_hff();
   ins_ned_to_state();
#else
		   //stateSetPositionNed_f(&pos_flow);
		   //stateSetSpeedNed_f(&speed_flow);  
   RunOnceEvery(10, (xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
       DOWNLINK_SEND_FLOW_NED(DefaultChannel, DefaultDevice, 
         &pos_flow.x, &pos_flow.y, &speed_flow.x, &speed_flow.y) ));
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

  /* update filter assuming a flat ground */
  if (distance < INS_SONAR_MAX_RANGE && distance > INS_SONAR_MIN_RANGE && ins_int.update_on_agl  
#ifdef INS_SONAR_THROTTLE_THRESHOLD
      && stabilization_cmd[COMMAND_THRUST] < INS_SONAR_THROTTLE_THRESHOLD
#endif
#ifdef INS_SONAR_BARO_THRESHOLD
      &&(ins_int.baro_z > -INS_SONAR_BARO_THRESHOLD) // z down 
#endif   
#if USE_BARO_BOARD
      &&ins_int.baro_initialized
      &&(ins_int.baro_z > -6)    //use sonar meas,request baro_z below 6m  --by whp
#endif
     ) 
  {
    vff_update_z_conf(-(distance), VFF_R_SONAR_0 + VFF_R_SONAR_OF_M * fabsf(distance));
    last_offset = vff.offset;
    //sonar_distance=distance;
    //sonar_distance_i=(sonar_distance_i*3.0+sonar_distance*2.0)/5.0;	
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
#if 0   //USE_SONER
  ins_int.ltp_pos.z=-POS_BFP_OF_REAL(  sonar_distance_i  );
#endif
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
  }
  last_stamp = stamp;
}

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  ins_int_update_gps(gps_s);
}

#if USE_FLOW
static void flow_cb(uint8_t sender_id __attribute__((unused)), struct Px4_flow_Data *flow_data)
{
  ins_int_update_flow(flow_data);
  //RunOnceEvery(25, DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &flow_data->flow_comp_m_x, &flow_data->ground_distance) );
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
  #if USE_FLOW
  AbiBindMsgFLOW(ABI_BROADCAST, &flow_ev, flow_cb);
  #endif
}
