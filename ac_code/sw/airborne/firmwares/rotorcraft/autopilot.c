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
 * @file firmwares/rotorcraft/autopilot.c
 *
 * Autopilot.
 *
 */

#include <stdint.h>
#include "math.h"
#include "mcu.h"
#include "firmwares/rotorcraft/autopilot.h"

#include "mcu_periph/uart.h"
#include "subsystems/radio_control.h"
#include "subsystems/commands.h"
#include "subsystems/actuators.h"
#include "subsystems/electrical.h"
#include "subsystems/settings.h"
#include "subsystems/datalink/telemetry.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance.h"

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_none.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

#include "subsystems/monitoring/monitoring.h"
#include "generated/settings.h"

#include "subsystems/ops/ops_msg_if.h"
#include "subsystems/ops/ops_app_if.h"

#if USE_MOTOR_MIXING
#include "subsystems/actuators/motor_mixing.h"
#endif

#if USE_GPS
#include "subsystems/gps.h"
#else
#if NO_GPS_NEEDED_FOR_NAV
#define GpsIsLost() FALSE
#else
#define GpsIsLost() TRUE
#endif
#endif

#ifdef POWER_SWITCH_GPIO
#include "mcu_periph/gpio.h"
#endif

#include "pprz_version.h"

uint8_t  autopilot_mode;
uint8_t  autopilot_mode_auto2;

bool_t   autopilot_in_flight;
uint32_t autopilot_in_flight_counter;
uint16_t autopilot_flight_time;

bool_t   above_ground;
uint32_t above_ground_counter;

bool_t   autopilot_motors_on;
bool_t   kill_throttle;

bool_t   autopilot_rc;
bool_t   autopilot_power_switch;

bool_t   autopilot_ground_detected;
bool_t   autopilot_detect_ground_once;

/** time steps for in_flight detection (at 20Hz, so 20=1second) */
#ifndef AUTOPILOT_IN_FLIGHT_TIME
#define AUTOPILOT_IN_FLIGHT_TIME    20
#endif

/** minimum vertical speed for in_flight condition in m/s */
#ifndef AUTOPILOT_IN_FLIGHT_MIN_SPEED
#define AUTOPILOT_IN_FLIGHT_MIN_SPEED 0.2
#endif

/** minimum vertical acceleration for in_flight condition in m/s^2 */
#ifndef AUTOPILOT_IN_FLIGHT_MIN_ACCEL
#define AUTOPILOT_IN_FLIGHT_MIN_ACCEL 2.0
#endif

/** minimum thrust for in_flight condition in pprz_t units (max = 9600) */
#ifndef AUTOPILOT_IN_FLIGHT_MIN_THRUST
#define AUTOPILOT_IN_FLIGHT_MIN_THRUST 500
#endif

#ifndef AUTOPILOT_DISABLE_AHRS_KILL
static inline int ahrs_is_aligned(void)
{
	return stateIsAttitudeValid();
}
#else
PRINT_CONFIG_MSG("Using AUTOPILOT_DISABLE_AHRS_KILL")
static inline int ahrs_is_aligned(void)
{
	return TRUE;
}
#endif

/** Set descent speed in failsafe mode */
#define FAILSAFE_DESCENT_SPEED (-0.8) //1.5   //for our aircraft,default 1.5m/s maybe too fast

/** Mode that is set when the plane is really too far from home */
#ifndef FAILSAFE_MODE_TOO_FAR_FROM_HOME
#define FAILSAFE_MODE_TOO_FAR_FROM_HOME AP_MODE_FAILSAFE
#endif


#if USE_KILL_SWITCH_FOR_MOTOR_ARMING
#include "autopilot_arming_switch.h"
PRINT_CONFIG_MSG("Using kill switch for motor arming")
#elif USE_THROTTLE_FOR_MOTOR_ARMING
#include "autopilot_arming_throttle.h"
PRINT_CONFIG_MSG("Using throttle for motor arming")
#else
#include "autopilot_arming_yaw.h"
PRINT_CONFIG_MSG("Using 2 sec yaw for motor arming")
#endif

#ifndef MODE_STARTUP
#define MODE_STARTUP AP_MODE_KILL
PRINT_CONFIG_MSG("Using default AP_MODE_KILL as MODE_STARTUP")
#endif

#ifndef UNLOCKED_HOME_MODE
#if MODE_AUTO1 == AP_MODE_HOME
#define UNLOCKED_HOME_MODE TRUE
PRINT_CONFIG_MSG("Enabled UNLOCKED_HOME_MODE since MODE_AUTO1 is AP_MODE_HOME")
#elif MODE_AUTO2 == AP_MODE_HOME
#define UNLOCKED_HOME_MODE TRUE
PRINT_CONFIG_MSG("Enabled UNLOCKED_HOME_MODE since MODE_AUTO2 is AP_MODE_HOME")
#else
#define UNLOCKED_HOME_MODE FALSE
#endif
#endif

#if MODE_MANUAL == AP_MODE_NAV
#error "MODE_MANUAL mustn't be AP_MODE_NAV"
#endif

#if PERIODIC_TELEMETRY
void send_autopilot_version(struct transport_tx *trans, struct link_device *dev)
{
#ifndef GCS_V1_OPTION
	static uint32_t ap_version = PPRZ_VERSION_INT;
	static char *ver_desc = PPRZ_VERSION_DESC;
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	pprz_msg_send_AUTOPILOT_VERSION(trans, dev, AC_ID, &ap_version, strlen(ver_desc), ver_desc);
#endif	//GCS_V1_OPTION
}

static void send_alive(struct transport_tx *trans, struct link_device *dev)
{
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	pprz_msg_send_ALIVE(trans, dev, AC_ID, 16, MD5SUM);
#if OPEN_PC_DATALINK
	pprz_msg_send_ALIVE(&((DOWNLINK_TRANSPORT).trans_tx), &((DOWNLINK_DEVICE).device), AC_ID, 16, MD5SUM);
#endif
}
#ifndef NPS_SIMU
static void send_mcu_fault(struct transport_tx *trans, struct link_device *dev)
{
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	pprz_msg_send_MCU_FAULT(trans, dev, AC_ID, &mcu_info.reset_src);
}
#endif
#ifdef BBOX_OPTION
static void send_utc_time(struct transport_tx *trans, struct link_device *dev)
{
	if(get_utc_day())
	{
		uint32_t time = get_utc_time_decimal();
		pprz_msg_send_UTC_TIME(trans, dev, AC_ID, &time);
	}
}

static void send_actuators_pwm(struct transport_tx *trans, struct link_device *dev)
{
	pprz_msg_send_ACTUATORS_PWM(trans, dev, AC_ID ,
															&actuators_pwm_values[0],
															&actuators_pwm_values[1],
															&actuators_pwm_values[2],
															&actuators_pwm_values[3],
															&actuators_pwm_values[4],
															&actuators_pwm_values[5]      );
}
#endif
static void send_attitude(struct transport_tx *trans, struct link_device *dev)
{
	struct FloatEulers *att = stateGetNedToBodyEulers_f();
	pprz_msg_send_ATTITUDE(trans, dev, AC_ID, &(att->phi), &(att->psi), &(att->theta));
};

static void send_status(struct transport_tx *trans, struct link_device *dev)
{
	uint32_t imu_nb_err = 0;
#if USE_MOTOR_MIXING
	uint8_t _motor_nb_err = motor_mixing.nb_saturation + motor_mixing.nb_failure * 10;
#else
	uint8_t _motor_nb_err = 0;
#endif
#if USE_GPS
	uint8_t fix = gps.fix;
#else
	uint8_t fix = 0;
#endif
	uint8_t spray_state = get_spray_switch_state();
	uint16_t time_sec = sys_time.nb_sec;
	//int32_t ops_connect_info = (int32_t)((ops_info.con_flag)|(ops_info.sys_error)<<8);
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	pprz_msg_send_ROTORCRAFT_STATUS(trans, dev, AC_ID,
																	//&imu_nb_err,
																	&radio_control.link_status,
																	&radio_control.status, &radio_control.frame_rate,
																	&spray_state,
																	&fix, &autopilot_mode,
																	&autopilot_in_flight, &autopilot_motors_on,
																	&guidance_h.mode, &guid_v.mode,
																	&electrical.vsupply, &electrical.current, &electrical.rep_cap, &electrical.temper,
																	&time_sec);
#if OPEN_PC_DATALINK
	pprz_msg_send_ROTORCRAFT_STATUS(&((DOWNLINK_TRANSPORT).trans_tx), &((DOWNLINK_DEVICE).device), AC_ID,
																	&radio_control.link_status,
																	&radio_control.status, &radio_control.frame_rate,
																	&spray_state,
																	&fix, &autopilot_mode,
																	&autopilot_in_flight, &autopilot_motors_on,
																	&guidance_h.mode, &guid_v.mode,
																	&electrical.vsupply, &electrical.current, &electrical.rep_cap, &electrical.temper,
																	&time_sec);
#endif
}

static void send_energy(struct transport_tx *trans, struct link_device *dev)
{
	uint16_t e = electrical.energy;
	if (fabs(electrical.energy) >= INT16_MAX)
	{
		e = INT16_MAX;
	}
	float vsup = ((float)electrical.vsupply) / 10.0f;
	float curs = ((float)electrical.current) / 1000.0f;
	float power = vsup * curs;
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	pprz_msg_send_ENERGY(trans, dev, AC_ID, &vsup, &curs, &e, &power);
}

static void send_fp(struct transport_tx *trans, struct link_device *dev)
{
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	pprz_msg_send_ROTORCRAFT_FP(trans, dev, AC_ID,
															&(stateGetPositionEnu_i()->x),
															&(stateGetPositionEnu_i()->y),
															&(stateGetPositionEnu_i()->z),
															&(stateGetSpeedEnu_i()->x),
															&(stateGetSpeedEnu_i()->y),
															&(stateGetSpeedEnu_i()->z),
															&(stateGetNedToBodyEulers_i()->phi),
															&(stateGetNedToBodyEulers_i()->theta),
															&(stateGetNedToBodyEulers_i()->psi),
															&stabilization_cmd[COMMAND_THRUST],
															&autopilot_flight_time);
#if OPEN_PC_DATALINK
	pprz_msg_send_ROTORCRAFT_FP(&((DOWNLINK_TRANSPORT).trans_tx), &((DOWNLINK_DEVICE).device), AC_ID,
															&(stateGetPositionEnu_i()->x),
															&(stateGetPositionEnu_i()->y),
															&(stateGetPositionEnu_i()->z),
															&(stateGetSpeedEnu_i()->x),
															&(stateGetSpeedEnu_i()->y),
															&(stateGetSpeedEnu_i()->z),
															&(stateGetNedToBodyEulers_i()->phi),
															&(stateGetNedToBodyEulers_i()->theta),
															&(stateGetNedToBodyEulers_i()->psi),
															&guidance_h.sp.pos.y,
															&guidance_h.sp.pos.x,
															&carrot_up,
															&guidance_h.sp.heading,
															&stabilization_cmd[COMMAND_THRUST],
															&autopilot_flight_time);
#endif
}

static void send_attitude_data(struct transport_tx *trans, struct link_device *dev)
{
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	pprz_msg_send_ATTITUDE_DATA(trans, dev, AC_ID,
															&(stateGetNedToBodyEulers_i()->phi),
															&(stateGetNedToBodyEulers_i()->theta),
															&(stateGetNedToBodyEulers_i()->psi)       );
}

#ifdef RADIO_CONTROL
static void send_rc(struct transport_tx *trans, struct link_device *dev)
{
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	pprz_msg_send_RC(trans, dev, AC_ID, RADIO_CONTROL_NB_CHANNEL, radio_control.values);
}

static void send_rotorcraft_rc(struct transport_tx *trans, struct link_device *dev)
{
#ifdef RADIO_KILL_SWITCH
	int16_t _kill_switch = radio_control.values[RADIO_RADIO_KILL_SWITCH];
#else
	int16_t _kill_switch = 42;
#endif
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	pprz_msg_send_ROTORCRAFT_RADIO_CONTROL(trans, dev, AC_ID,
																				 &radio_control.values[RADIO_ROLL],
																				 &radio_control.values[RADIO_PITCH],
																				 &radio_control.values[RADIO_YAW],
																				 &radio_control.values[RADIO_THROTTLE],
																				 &radio_control.values[RADIO_MODE],
																				 &_kill_switch,
																				 &radio_control.status);
}
#endif

#ifdef ACTUATORS
static void send_actuators(struct transport_tx *trans, struct link_device *dev)
{
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	pprz_msg_send_ACTUATORS(trans, dev, AC_ID , ACTUATORS_NB, actuators);
}
#endif

static void send_dl_value(struct transport_tx *trans, struct link_device *dev)
{
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	PeriodicSendDlValue(trans, dev);
#if OPEN_PC_DATALINK
	PeriodicSendDlValue(&((DOWNLINK_TRANSPORT).trans_tx), &((DOWNLINK_DEVICE).device));
#endif
}

static void send_rotorcraft_cmd(struct transport_tx *trans, struct link_device *dev)
{
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	pprz_msg_send_ROTORCRAFT_CMD(trans, dev, AC_ID,
															 &stabilization_cmd[COMMAND_ROLL],
															 &stabilization_cmd[COMMAND_PITCH],
															 &stabilization_cmd[COMMAND_YAW],
															 &stabilization_cmd[COMMAND_THRUST]);
}
#endif

void check_joystick_enable(uint8_t status)		//add by lg
{
	if(ac_config_info.rocker_remote_status==1) autopilot_rc=TRUE;
	else if(ac_config_info.rocker_remote_status==0) autopilot_rc=FALSE;
}
void autopilot_init(void)
{
	/* mode is finally set at end of init if MODE_STARTUP is not KILL */
	autopilot_mode = AP_MODE_KILL;
	autopilot_motors_on = FALSE;
	kill_throttle = ! autopilot_motors_on;
	autopilot_in_flight = FALSE;
	autopilot_in_flight_counter = 0;
	autopilot_mode_auto2 = MODE_AUTO2;
	autopilot_ground_detected = FALSE;
	autopilot_detect_ground_once = FALSE;
	autopilot_flight_time = 0;
	/*#ifndef WITHOUT_RADIO
	  autopilot_rc = TRUE;
	#else
	  autopilot_rc = FALSE;   //stop radio event task
	#endif*/
	autopilot_power_switch = FALSE;
#ifdef POWER_SWITCH_GPIO
	gpio_setup_output(POWER_SWITCH_GPIO);
	gpio_clear(POWER_SWITCH_GPIO); // POWER OFF
#endif

	autopilot_rc = FALSE;   //stop radio event task add by lg
	autopilot_arming_init();

	nav_init();
	guidance_h_init();
	guidance_v_init();

	stabilization_init();
	stabilization_none_init();
	stabilization_rate_init();
	stabilization_attitude_init();

	/* set startup mode, propagates through to guidance h/v */
	autopilot_set_mode(MODE_STARTUP);
#if PERIODIC_TELEMETRY

#ifndef GCS_V1_OPTION
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AUTOPILOT_VERSION, send_autopilot_version);
#endif //GCS_V1_OPTION
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ALIVE, send_alive);
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_STATUS, send_status);
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ATTITUDE, send_attitude);
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ENERGY, send_energy);
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_FP, send_fp);
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_CMD, send_rotorcraft_cmd);
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DL_VALUE, send_dl_value);
#ifndef NPS_SIMU
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_MCU_FAULT, send_mcu_fault);
#endif
#ifdef BBOX_OPTION
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_UTC_TIME, send_utc_time);
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ACTUATORS_PWM, send_actuators_pwm);
#endif
#ifdef ACTUATORS
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ACTUATORS, send_actuators);
#endif
#ifdef RADIO_CONTROL
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RC, send_rc);
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_RADIO_CONTROL, send_rotorcraft_rc);
#endif

#endif
}


#define NAV_PRESCALER (PERIODIC_FREQUENCY / NAV_FREQ)
void autopilot_periodic(void)
{
#ifndef USE_MISSION   //our flight,not use AP_MODE_HOME
	RunOnceEvery(NAV_PRESCALER, compute_dist2_to_home());
	//***if too far from home, true set home mode
	if (autopilot_in_flight && autopilot_mode == AP_MODE_NAV)
	{
		if (too_far_from_home)
		{
			if (dist2_to_home > failsafe_mode_dist2)
			{
				autopilot_set_mode(FAILSAFE_MODE_TOO_FAR_FROM_HOME);
			}
			else
			{
				autopilot_set_mode(AP_MODE_HOME);
			}
		}
	}

	if (autopilot_mode == AP_MODE_HOME)
	{
		RunOnceEvery(NAV_PRESCALER, nav_home());
	}
	else
	{
		// otherwise always call nav_periodic_task so that carrot is always updated in GCS for other modes
		RunOnceEvery(NAV_PRESCALER, nav_periodic_task());
	}

#else
	RunOnceEvery(NAV_PRESCALER, nav_periodic_task());
#endif

	/* If in FAILSAFE mode and either already not in_flight anymore
	 * or just "detected" ground, go to KILL mode.
	 */
	if (autopilot_mode == AP_MODE_FAILSAFE)
	{
		if (!autopilot_in_flight)
		{
			autopilot_set_mode(AP_MODE_KILL);
		}
#if FAILSAFE_GROUND_DETECT
		INFO("Using FAILSAFE_GROUND_DETECT: KILL")
		if (autopilot_ground_detected)
		{
			autopilot_set_mode(AP_MODE_KILL);
		}
#endif
	}

	/* Reset ground detection _after_ running flight plan
	 */
	if (!autopilot_in_flight)
	{
		autopilot_ground_detected = FALSE;
		autopilot_detect_ground_once = FALSE;
	}

	/* Set fixed "failsafe" commands from airframe file if in KILL mode.
	 * If in FAILSAFE mode, run normal loops with failsafe attitude and
	 * downwards velocity setpoints.
	 */
	if (autopilot_mode == AP_MODE_KILL)
	{
		SetCommands(commands_failsafe);
	}
	else
	{
		guidance_v_run(autopilot_in_flight);
		guidance_h_run(autopilot_in_flight);
		/*set 4 channel of sta_cmd, for motor_mix_run*/
		SetRotorcraftCommands(stabilization_cmd, autopilot_in_flight, autopilot_motors_on);
	}

}


void autopilot_set_mode(uint8_t new_autopilot_mode)
{
	/* force startup mode (default is kill) as long as AHRS is not aligned */
	if (!ahrs_is_aligned())
	{
		new_autopilot_mode = MODE_STARTUP;
	}

	if(new_autopilot_mode == AP_MODE_FAILSAFE)
	{
		if((autopilot_mode == AP_MODE_ATTITUDE_DIRECT) || (autopilot_mode == AP_MODE_KILL))
		{
			return;
		}
	}

	if (autopilot_mode == AP_MODE_FAILSAFE)
	{
		if ((new_autopilot_mode != AP_MODE_ATTITUDE_DIRECT) && (new_autopilot_mode != AP_MODE_KILL))
		{
			return;
		}
	}

	if (new_autopilot_mode != autopilot_mode)
	{
		/* horizontal mode */
		switch (new_autopilot_mode)
		{
		case AP_MODE_FAILSAFE:
#ifndef KILL_AS_FAILSAFE
			stabilization_attitude_set_failsafe_setpoint();
			guidance_h_mode_changed(GUIDANCE_H_MODE_ATTITUDE);
			break;
#endif
		case AP_MODE_KILL:
			autopilot_in_flight = FALSE;
			autopilot_in_flight_counter = 0;
			guidance_h_mode_changed(GUIDANCE_H_MODE_KILL);
			break;
		case AP_MODE_RC_DIRECT:
			guidance_h_mode_changed(GUIDANCE_H_MODE_RC_DIRECT);
			break;
		case AP_MODE_RATE_DIRECT:
		case AP_MODE_RATE_Z_HOLD:
			guidance_h_mode_changed(GUIDANCE_H_MODE_RATE);
			break;
		case AP_MODE_ATTITUDE_RC_CLIMB:
		case AP_MODE_ATTITUDE_DIRECT:
		case AP_MODE_ATTITUDE_CLIMB:
		case AP_MODE_ATTITUDE_Z_HOLD:
			guidance_h_mode_changed(GUIDANCE_H_MODE_ATTITUDE);
			break;
		case AP_MODE_FORWARD:
			guidance_h_mode_changed(GUIDANCE_H_MODE_FORWARD);
			break;
		case AP_MODE_CARE_FREE_DIRECT:
			guidance_h_mode_changed(GUIDANCE_H_MODE_CARE_FREE);
			break;
		case AP_MODE_HOVER_DIRECT:
		case AP_MODE_HOVER_CLIMB:
		case AP_MODE_HOVER_Z_HOLD:
			guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
			break;
		case AP_MODE_HOME:
		case AP_MODE_NAV:
			guidance_h_mode_changed(GUIDANCE_H_MODE_NAV);
			break;
		case AP_MODE_MODULE:
#ifdef GUIDANCE_H_MODE_MODULE_SETTING
			guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE_SETTING);
#endif
			break;
		case AP_MODE_FLIP:
			guidance_h_mode_changed(GUIDANCE_H_MODE_FLIP);
			break;
		case AP_MODE_GUIDED:
			guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
			break;
		default:
			break;
		}
		/* vertical mode */
		switch (new_autopilot_mode)
		{
		case AP_MODE_FAILSAFE:
#ifndef KILL_AS_FAILSAFE
			if (ins_int_v_ekf_open_loop())
			{
				guidance_v_mode_changed(GUIDANCE_V_MODE_ACC_LAND);
			}
			else
			{
				guidance_v_mode_changed(GUIDANCE_V_MODE_CLIMB);
				guid_v.climb_speed_sp = FAILSAFE_DESCENT_SPEED;
			}
			break;
#endif
		case AP_MODE_KILL:
			autopilot_set_motors_on(FALSE);
			stabilization_cmd[COMMAND_THRUST] = 0;
			guidance_v_mode_changed(GUIDANCE_V_MODE_KILL);
			break;
		case AP_MODE_RC_DIRECT:
		case AP_MODE_RATE_DIRECT:
		case AP_MODE_ATTITUDE_DIRECT:
		case AP_MODE_HOVER_DIRECT:
		case AP_MODE_CARE_FREE_DIRECT:
		case AP_MODE_FORWARD:
			guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
			break;
		case AP_MODE_RATE_RC_CLIMB:
		case AP_MODE_ATTITUDE_RC_CLIMB:
			guidance_v_mode_changed(GUIDANCE_V_MODE_RC_CLIMB);
			break;
		case AP_MODE_ATTITUDE_CLIMB:
		case AP_MODE_HOVER_CLIMB:
			guidance_v_mode_changed(GUIDANCE_V_MODE_CLIMB);
			break;
		case AP_MODE_RATE_Z_HOLD:
		case AP_MODE_ATTITUDE_Z_HOLD:
		case AP_MODE_HOVER_Z_HOLD:
			guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
			break;
		case AP_MODE_HOME:
		case AP_MODE_NAV:
			guidance_v_mode_changed(GUIDANCE_V_MODE_NAV);
			break;
		case AP_MODE_MODULE:
#ifdef GUIDANCE_V_MODE_MODULE_SETTING
			guidance_v_mode_changed(GUIDANCE_V_MODE_MODULE_SETTING);
#endif
			break;
		case AP_MODE_FLIP:
			guidance_v_mode_changed(GUIDANCE_V_MODE_FLIP);
			break;
		case AP_MODE_GUIDED:
			guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
			break;
		default:
			break;
		}
		autopilot_mode = new_autopilot_mode;
	}

}

bool_t autopilot_guided_goto_ned(float x, float y, float z, float heading)
{
	if (autopilot_mode == AP_MODE_GUIDED)
	{
		guidance_h_set_guided_pos(x, y);
		guidance_h_set_guided_heading(heading);
		guidance_v_set_guided_z(z);
		return TRUE;
	}
	return FALSE;
}

bool_t autopilot_guided_goto_ned_relative(float dx, float dy, float dz, float dyaw)
{
	if (autopilot_mode == AP_MODE_GUIDED && stateIsLocalCoordinateValid())
	{
		float x = stateGetPositionNed_f()->x + dx;
		float y = stateGetPositionNed_f()->y + dy;
		float z = stateGetPositionNed_f()->z + dz;
		float heading = stateGetNedToBodyEulers_f()->psi + dyaw;
		return autopilot_guided_goto_ned(x, y, z, heading);
	}
	return FALSE;
}

bool_t autopilot_guided_goto_body_relative(float dx, float dy, float dz, float dyaw)
{
	if (autopilot_mode == AP_MODE_GUIDED && stateIsLocalCoordinateValid())
	{
		float psi = stateGetNedToBodyEulers_f()->psi;
		float x = stateGetPositionNed_f()->x + cosf(-psi) * dx + sinf(-psi) * dy;
		float y = stateGetPositionNed_f()->y - sinf(-psi) * dx + cosf(-psi) * dy;
		float z = stateGetPositionNed_f()->z + dz;
		float heading = psi + dyaw;
		return autopilot_guided_goto_ned(x, y, z, heading);
	}
	return FALSE;
}

void autopilot_check_in_flight(bool_t motors_on)
{
	if (autopilot_in_flight)
	{
		if (autopilot_in_flight_counter > 0)
		{
			/* probably in_flight if thrust, speed and accel above IN_FLIGHT_MIN thresholds */
			if ((stabilization_cmd[COMMAND_THRUST] <= AUTOPILOT_IN_FLIGHT_MIN_THRUST) &&
					(fabsf(stateGetSpeedNed_f()->z) < AUTOPILOT_IN_FLIGHT_MIN_SPEED) &&
					(fabsf(stateGetAccelNed_f()->z) < AUTOPILOT_IN_FLIGHT_MIN_ACCEL))
			{
				autopilot_in_flight_counter--;
				if (autopilot_in_flight_counter == 0)
				{
					autopilot_in_flight = FALSE;
				}
			}
			else     /* thrust, speed or accel not above min threshold, reset counter */
			{
				autopilot_in_flight_counter = AUTOPILOT_IN_FLIGHT_TIME;
			}
		}
	}
	else
	{
		/* currently not in flight */
		if (autopilot_in_flight_counter < AUTOPILOT_IN_FLIGHT_TIME && motors_on)
		{
			/* if thrust above min threshold, assume in_flight.
			 * Don't check for velocity and acceleration above threshold here...
			 */
			if (stabilization_cmd[COMMAND_THRUST] > AUTOPILOT_IN_FLIGHT_MIN_THRUST)
			{
				autopilot_in_flight_counter++;
				if (autopilot_in_flight_counter == AUTOPILOT_IN_FLIGHT_TIME)
				{
					autopilot_in_flight = TRUE;
				}
			}
			else
			{
				/* currently not in_flight and thrust below threshold, reset counter */
				autopilot_in_flight_counter = 0;
			}
		}
	}

	/*add above_ground signal to ajust attitude intergrater,by whp*/
	if (above_ground)
	{
		if (!autopilot_in_flight)
		{
			above_ground = FALSE;
		}

	}
	else
	{
		/* currently not in flight */
		if (autopilot_in_flight)
		{
			if (stabilization_cmd[COMMAND_THRUST] >3000 && stateGetPositionEnu_f()->z >DISTANCE_ABOVE_GROUNG)
			{
				above_ground_counter++;
				if (above_ground_counter > (1*AUTOPILOT_IN_FLIGHT_TIME) )   //continue 1s
				{
					above_ground = TRUE;
					above_ground_counter = 0;
				}
			}
			else
			{
				/*reset counter */
				above_ground_counter = 0;
			}
		}
	}
}


void autopilot_set_motors_on(bool_t motors_on)
{
	if (autopilot_mode != AP_MODE_KILL && ahrs_is_aligned() && motors_on)
	{
		autopilot_motors_on = TRUE;
	}
	else
	{
		autopilot_motors_on = FALSE;
	}
	kill_throttle = ! autopilot_motors_on;
	autopilot_arming_set(autopilot_motors_on);
}


#define THRESHOLD_1_PPRZ (MIN_PPRZ / 2)
#define THRESHOLD_2_PPRZ (MAX_PPRZ / 2)

#define AP_MODE_OF_PPRZ(_rc, _mode) {    \
    if      (_rc > THRESHOLD_2_PPRZ)     \
      _mode = autopilot_mode_auto2;      \
    else if (_rc > THRESHOLD_1_PPRZ)     \
      _mode = MODE_AUTO1;                \
    else                                 \
      _mode = MODE_MANUAL;               \
  }


/** get autopilot mode as set by RADIO_MODE 3-way switch */
static uint8_t ap_mode_of_3way_switch(void)
{
	if (radio_control.values[RADIO_MODE] > THRESHOLD_2_PPRZ)
	{
		return autopilot_mode_auto2;
	}
	else if (radio_control.values[RADIO_MODE] > THRESHOLD_1_PPRZ)
	{
		return MODE_AUTO1;
	}
	else
	{
		return MODE_MANUAL;
	}
}

/**
 * Get autopilot mode from two 2way switches.
 * RADIO_MODE switch just selectes between MANUAL and AUTO.
 * If not MANUAL, the RADIO_AUTO_MODE switch selects between AUTO1 and AUTO2.
 *
 * This is mainly a cludge for entry level radios with no three-way switch,
 * but two available two-way switches which can be used.
 */
#if defined RADIO_AUTO_MODE || defined(__DOXYGEN__)
static uint8_t ap_mode_of_two_switches(void)
{
	if (radio_control.values[RADIO_MODE] < THRESHOLD_1_PPRZ)
	{
		/* RADIO_MODE in MANUAL position */
		return MODE_MANUAL;
	}
	else
	{
		/* RADIO_MODE not in MANUAL position.
		 * Select AUTO mode bassed on RADIO_AUTO_MODE channel
		 */
		if (radio_control.values[RADIO_AUTO_MODE] > THRESHOLD_2_PPRZ)
		{
			return autopilot_mode_auto2;
		}
		else
			return MODE_AUTO1;
	}
}
#endif

void autopilot_on_rc_frame(void)
{
	uint8_t new_autopilot_mode = 0;

	if (radio_kill_switch) //(kill_switch_is_on())
	{
		autopilot_set_mode(AP_MODE_KILL);
	}
	else
	{
#if 0 // def RADIO_AUTO_MODE
		INFO("Using RADIO_AUTO_MODE to switch between AUTO1 and AUTO2.")
		new_autopilot_mode = ap_mode_of_two_switches();
		AP_MODE_OF_PPRZ(radio_control.values[RADIO_MODE], new_autopilot_mode);
#else
		//uint8_t new_autopilot_mode = ap_mode_of_3way_switch();
		//AP_MODE_OF_PPRZ(radio_control.values[RADIO_MODE], new_autopilot_mode); //TODOM: whp
		new_autopilot_mode = radio_ap_mode;
#endif
		/* don't enter NAV mode if GPS is lost (this also prevents mode oscillations) */
		if (!(new_autopilot_mode == AP_MODE_NAV && GpsIsLost()))
		{
			/* always allow to switch to manual */
			if (new_autopilot_mode == MODE_MANUAL)
			{
				autopilot_set_mode(new_autopilot_mode);
			}
			/* if in HOME mode, don't allow switching to non-manual modes */
			else if ((autopilot_mode != AP_MODE_HOME)
#if UNLOCKED_HOME_MODE
							 /* Allowed to leave home mode when UNLOCKED_HOME_MODE */
							 || !too_far_from_home
#endif
							)
			{
				autopilot_set_mode(new_autopilot_mode);
			}
		}
	}

	/* an arming sequence is used to start/stop motors.
	 * only allow arming if ahrs is aligned
	 */
	if (ahrs_is_aligned())
	{
		autopilot_arming_check_motors_on();
		kill_throttle = ! autopilot_motors_on;
	}

	/* if not in FAILSAFE or HOME mode, read RC and set commands accordingly */
	if (autopilot_mode != AP_MODE_FAILSAFE && autopilot_mode != AP_MODE_HOME)
	{

		/* if there are some commands that should always be set from RC, do it */
#ifdef SetAutoCommandsFromRC
		SetAutoCommandsFromRC(commands, radio_control.values);
#endif

		/* if not in NAV_MODE set commands from the rc */
#ifdef SetCommandsFromRC
		if (autopilot_mode != AP_MODE_NAV)
		{
			SetCommandsFromRC(commands, radio_control.values);
		}
#endif

		guidance_v_read_rc();
		guidance_h_read_rc(autopilot_in_flight);
	}

}

void autopilot_ready_check(void)
{
#ifdef USE_MISSION
	if((ground_check_pass)&&(Flag_AC_Flight_Ready==TRUE))		//add by lg
	{
		autopilot_set_mode(AP_MODE_NAV);
	}
	else
	{
		autopilot_set_mode(AP_MODE_KILL);
	}
#endif
}

bool_t autopilot_check_is_pairing_mode(void)
{
	if(autopilot_in_flight || !kill_throttle)  return FALSE;
	float rc_roll = stateGetNedToBodyEulers_f()->phi;
	float rc_pitch = stateGetNedToBodyEulers_f()->theta;

	if( fabs(rc_roll) >0.5 || fabs(rc_pitch) >0.5 )
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

#ifdef CALIBRATION_OPTION
void autopilot_mag_cali_store(void)
{
	if(kill_throttle)
	{
		settings_store_flag = TRUE;
		settings_store();
	}
}
#endif
