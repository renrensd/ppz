/*
 * Copyright (C) 2008-2010 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file firmwares/rotorcraft/main.c
 *
 * Rotorcraft main loop.
 */

#define MODULES_C

#define ABI_C

#include <inttypes.h>
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

#include "subsystems/datalink/telemetry.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/settings.h"

#include "subsystems/commands.h"
#include "subsystems/actuators.h"
#if USE_MOTOR_MIXING
#include "subsystems/actuators/motor_mixing.h"
#endif

#if USE_IMU
#include "subsystems/imu.h"
#endif
#if USE_GPS
#include "subsystems/gps.h"
#endif

#if USE_BARO_BOARD
#include "subsystems/sensors/baro.h"
PRINT_CONFIG_MSG_VALUE("USE_BARO_BOARD is TRUE, reading onboard baro: ", BARO_BOARD)
#endif

#include "subsystems/electrical.h"

#include "firmwares/rotorcraft/autopilot.h"

#include "subsystems/radio_control.h"

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance.h"

#include "subsystems/ahrs.h"
#if USE_AHRS_ALIGNER
#include "subsystems/ahrs/ahrs_aligner.h"
#endif
#include "subsystems/ins.h"

#include "state.h"

#include "firmwares/rotorcraft/main.h"

#ifdef SITL
#include "nps_autopilot.h"
#endif

#include "generated/modules.h"
#include "subsystems/abi.h"

#include "subsystems/datalink/xbee.h"

#ifdef OPS_OPTION
#include"subsystems/ops/ops_app_if.h"
#endif	/* OPS_OPTION */

#ifdef MONITORING_OPTION
#include "subsystems/monitoring/monitoring.h"
#endif

#ifdef WDG_OPTION
#include "wdg.h"
#endif

/* if PRINT_CONFIG is defined, print some config options */
PRINT_CONFIG_VAR(PERIODIC_FREQUENCY)

/* TELEMETRY_FREQUENCY is defined in generated/periodic_telemetry.h
 * defaults to 60Hz or set by TELEMETRY_FREQUENCY configure option in airframe file
 */
PRINT_CONFIG_VAR(TELEMETRY_FREQUENCY)

/* MODULES_FREQUENCY is defined in generated/modules.h
 * according to main_freq parameter set for modules in airframe file
 */
PRINT_CONFIG_VAR(MODULES_FREQUENCY)

#ifndef BARO_PERIODIC_FREQUENCY
#define BARO_PERIODIC_FREQUENCY 50
#endif
PRINT_CONFIG_VAR(BARO_PERIODIC_FREQUENCY)

#if USE_AHRS && USE_IMU && (defined AHRS_PROPAGATE_FREQUENCY)
#if (AHRS_PROPAGATE_FREQUENCY > PERIODIC_FREQUENCY)
#warning "PERIODIC_FREQUENCY should be least equal or greater than AHRS_PROPAGATE_FREQUENCY"
INFO_VALUE("it is recommended to configure in your airframe PERIODIC_FREQUENCY to at least ", AHRS_PROPAGATE_FREQUENCY)
#endif
#endif

tid_t main_periodic_tid; ///< id for main_periodic() timer
tid_t modules_tid;       ///< id for modules_periodic_task() timer
tid_t failsafe_tid;      ///< id for failsafe_check() timer
#ifndef WITHOUT_RADIO
tid_t radio_control_tid; ///< id for radio_control_periodic_task() timer
#endif
tid_t electrical_tid;    ///< id for electrical_periodic() timer
tid_t telemetry_tid;     ///< id for telemetry_periodic() timer
#if USE_BARO_BOARD
tid_t baro_tid;          ///< id for baro_periodic() timer
#endif
#ifdef OPS_OPTION
tid_t ops_tid;           ///< id for ops_task() timer
#endif	/* OPS_OPTION */
#ifdef MONITORING_OPTION
tid_t monitor_tid;         ///< id for monitoring_periodic() timer
#endif

#ifndef SITL
int main(void)
{
  main_init();

#if LIMIT_EVENT_POLLING
  /* Limit main loop frequency to 1kHz.
   * This is a kludge until we can better leverage threads and have real events.
   * Without this limit the event flags will constantly polled as fast as possible,
   * resulting on 100% cpu load on boards with an (RT)OS.
   * On bare metal boards this is not an issue, as you have nothing else running anyway.
   */
  uint32_t t_begin = 0;
  uint32_t t_diff = 0;
  while (1) {
    t_begin = get_sys_time_usec();

    handle_periodic_tasks();
    main_event();

    /* sleep remaining time to limit to 1kHz */
    t_diff = get_sys_time_usec() - t_begin;
    if (t_diff < 1000) {
      sys_time_usleep(1000 - t_diff);
    }
  }
#else
  while (1) 
  {
	  handle_periodic_tasks();
      main_event();
  }
#endif

  return 0;
}
#endif /* SITL */

STATIC_INLINE void main_init(void)
{
  mcu_init();

#if defined(PPRZ_TRIG_INT_COMPR_FLASH)
  pprz_trig_int_init();
#endif

  electrical_init();

  stateInit();

#ifndef INTER_MCU_AP
  actuators_init();
#else
  intermcu_init();
#endif

#if USE_MOTOR_MIXING
  motor_mixing_init();
#endif

#ifndef INTER_MCU_AP
 #ifndef WITHOUT_RADIO
  radio_control_init();
 #endif
#endif

#if USE_BARO_BOARD
  baro_init();
#endif
#if USE_IMU
  imu_init();
#endif
#if USE_AHRS_ALIGNER
  ahrs_aligner_init();
#endif

#if USE_AHRS
  ahrs_init();
#endif

  ins_init();

#if USE_GPS
  gps_init();
#endif

  autopilot_init();

  modules_init();

  settings_init();

  mcu_int_enable();

#if DOWNLINK
  downlink_init();  //requested after settings_init(),to get flash data set var
#endif

#ifdef INTER_MCU_AP
  intermcu_init();
#endif

#ifdef OPS_OPTION
	ops_init();
#endif	/* OPS_OPTION */

#ifdef MONITORING_OPTION
    monitoring_init();
#endif

#ifdef WDG_OPTION
  wdg_enable(); 
#endif
  // register the timers for the periodic functions
  main_periodic_tid = sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
  modules_tid = sys_time_register_timer(1. / MODULES_FREQUENCY, NULL);
  #ifndef WITHOUT_RADIO
  radio_control_tid = sys_time_register_timer((1. / 60.), NULL);
  #endif
  failsafe_tid = sys_time_register_timer(0.05, NULL);
  electrical_tid = sys_time_register_timer(0.1, NULL);
  telemetry_tid = sys_time_register_timer((1. / TELEMETRY_FREQUENCY), NULL);
#if USE_BARO_BOARD
  baro_tid = sys_time_register_timer(1. / BARO_PERIODIC_FREQUENCY, NULL);
#endif

#ifdef OPS_OPTION
  ops_tid = sys_time_register_timer(1. / OPS_PERIODIC_FREQUENCY, NULL);
#endif	/* OPS_OPTION */

#ifdef MONITORING_OPTION
  monitor_tid = sys_time_register_timer(1. /MONITORING_FREQUENCY, NULL);
#endif
#if USE_IMU
  // send body_to_imu from here for now
  AbiSendMsgBODY_TO_IMU_QUAT(1, orientationGetQuat_f(&imu.body_to_imu));
#endif


  // Do a failsafe check first
  failsafe_check();
}

STATIC_INLINE void handle_periodic_tasks(void)
{
  if (sys_time_check_and_ack_timer(main_periodic_tid)) {
    main_periodic();
  }
  if (sys_time_check_and_ack_timer(modules_tid)) 
  {
	#ifdef WDG_OPTION
	mcu_set_task_wdg_flag(WDG_TASK_MODULES);
	#endif	/* WDG_OPTION */
	modules_periodic_task();
  }
  #ifndef WITHOUT_RADIO
  if (sys_time_check_and_ack_timer(radio_control_tid)) {
    radio_control_periodic_task();
  }
  #endif
  if (sys_time_check_and_ack_timer(failsafe_tid)) {
    failsafe_check();
  }
  if (sys_time_check_and_ack_timer(electrical_tid)) {
    electrical_periodic();
  }
  if (sys_time_check_and_ack_timer(telemetry_tid)) {
    telemetry_periodic();
  }
#if USE_BARO_BOARD
  if (sys_time_check_and_ack_timer(baro_tid)) 
  {
    #ifdef WDG_OPTION
	mcu_set_task_wdg_flag(WDG_TASK_BARO);
	#endif	/* WDG_OPTION */
	baro_periodic();
  }
#endif

#ifdef OPS_OPTION
  if (sys_time_check_and_ack_timer(ops_tid)) 
  {
    ops_task();
  }
#endif	/* OPS_OPTION */

#ifdef MONITORING_OPTION
  if (sys_time_check_and_ack_timer(monitor_tid)) 
  {
	 #ifdef WDG_OPTION
	 mcu_set_task_wdg_flag(WDG_TASK_MONITORING);
	 #endif	/* WDG_OPTION */
	 monitoring_periodic();
  }
#endif
}

STATIC_INLINE void main_periodic(void)
{
#ifdef WDG_OPTION
	mcu_set_task_wdg_flag(WDG_TASK_MAIN);
	wdg_feed_handle();
#endif	/* WDG_OPTION */

#if USE_IMU
  imu_periodic();
#endif

  //FIXME: temporary hack, remove me
#ifdef InsPeriodic
  InsPeriodic();
#endif

  /* run control loops */
  autopilot_periodic();
  /* set actuators     */
  //actuators_set(autopilot_motors_on);
#ifndef INTER_MCU_AP
  SetActuatorsFromCommands(commands, autopilot_mode);
#else
  intermcu_set_actuators(commands, autopilot_mode);
#endif

  if (autopilot_in_flight) {
    RunOnceEvery(PERIODIC_FREQUENCY, autopilot_flight_time++);
  }

#if defined DATALINK || defined SITL
  RunOnceEvery(PERIODIC_FREQUENCY, datalink_time++);
#endif

  RunOnceEvery(10, LED_PERIODIC());
}

STATIC_INLINE void telemetry_periodic(void)
{
  static uint8_t boot = TRUE;

	#ifdef WDG_OPTION
	mcu_set_task_wdg_flag(WDG_TASK_TELEMETRY);
	#endif	/* WDG_OPTION */
  /* initialisation phase during boot */
  if (boot) {
#if DOWNLINK
 #if PERIODIC_TELEMETRY
    send_autopilot_version(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
 #endif
#endif
    boot = FALSE;
  }
  /* then report periodicly */
  else {
#if PERIODIC_TELEMETRY
    periodic_telemetry_send_Main(DefaultPeriodic, &(DefaultChannel).trans_tx, &(DefaultDevice).device);
#endif
  }

#if DOWNLINK
  downlink_periodic();
#endif //DOWNLINK
}

/** mode to enter when RC is lost while using a mode with RC input (not AP_MODE_NAV) */
#ifndef RC_LOST_MODE
#define RC_LOST_MODE AP_MODE_FAILSAFE
#endif

STATIC_INLINE void failsafe_check(void)
{
	#ifdef WDG_OPTION
	mcu_set_task_wdg_flag(WDG_TASK_FAILSAFE);
	#endif	/* WDG_OPTION */
	autopilot_check_in_flight(autopilot_motors_on);
   #if USE_GPS
    gps_periodic_check();
   #endif

   #ifndef USE_MISSION   //our flight use mission,not use radio and only AP_MODE_NAV
   
	  if (radio_control.status == RC_REALLY_LOST &&
	      autopilot_mode != AP_MODE_KILL &&
	      autopilot_mode != AP_MODE_HOME &&
	      autopilot_mode != AP_MODE_FAILSAFE &&
	      autopilot_mode != AP_MODE_NAV) {   //radio lost,will set mode kill
	    autopilot_set_mode(RC_LOST_MODE);
	  }

    #if FAILSAFE_ON_BAT_CRITICAL
	  if (autopilot_mode != AP_MODE_KILL &&
	      electrical.bat_critical) {
	    autopilot_set_mode(AP_MODE_FAILSAFE);
	  }
    #endif

    #if USE_GPS
	  if (autopilot_mode == AP_MODE_NAV &&
	      autopilot_motors_on &&
    #if NO_GPS_LOST_WITH_RC_VALID
	      radio_control.status != RC_OK &&
    #endif
	      GpsIsLost()) {
	    autopilot_set_mode(AP_MODE_FAILSAFE);
	  }

	  if (autopilot_mode == AP_MODE_HOME &&
	      autopilot_motors_on && GpsIsLost()) {
	    autopilot_set_mode(AP_MODE_FAILSAFE);
	  }
    #endif

  #endif

}



STATIC_INLINE void main_event(void)
{
	#ifdef WDG_OPTION
	mcu_set_task_wdg_flag(WDG_EVENT_ALL);
	#endif	/* WDG_OPTION */

  /* event functions for mcu peripherals: i2c, usb_serial.. */
  mcu_event();

  DatalinkEvent();

  if(autopilot_rc) 
  {
    RadioControlEvent(autopilot_on_rc_frame);
  }
  else  //use ground_check_pass to set mode,instead of radio control
  {
  	autopilot_ready_check();
  }

#if USE_IMU
  ImuEvent();
#endif

#ifdef InsEvent
  TODO("calling InsEvent, remove me..")
  InsEvent();
#endif

#if USE_BARO_BOARD
  BaroEvent();
#endif

#if USE_GPS
  GpsEvent();
#endif

#if FAILSAFE_GROUND_DETECT || KILL_ON_GROUND_DETECT
  DetectGroundEvent();
#endif

  modules_event_task();
}
