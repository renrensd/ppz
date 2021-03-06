/*
 * Copyright (C) 2013 Gautier Hattenberger, Alexandre Bustico
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
 * @file firmwares/rotorcraft/main_chibios_libopencm3.c
 */
#define MODULES_C

#define ABI_C

#include "firmwares/fixedwing/chibios-libopencm3/chibios_init.h"
#include "chibios_stub.h"
#include "subsystems/chibios-libopencm3/chibios_sdlog.h"
#include "sdLog.h"


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

#include "subsystems/imu.h"
#include "subsystems/gps.h"

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
tid_t radio_control_tid; ///< id for radio_control_periodic_task() timer
tid_t electrical_tid;    ///< id for electrical_periodic() timer
tid_t telemetry_tid;     ///< id for telemetry_periodic() timer
#if USE_BARO_BOARD
tid_t baro_tid;          ///< id for baro_periodic() timer
#endif


static int32_t pprz_thd(void *arg);
static bool_t sdlogOk ;
bool_t pprzReady = FALSE;

#ifndef SITL
int main(void)
{
	mcu_init();
	// Init ChibiOS
	sdlogOk = chibios_init();

	main_init();

	chibios_chThdSleepMilliseconds(100);

	launch_pprz_thd(&pprz_thd);
	pprzReady = TRUE;
	// Call PPRZ periodic and event functions
	while (TRUE)
	{
		chibios_chThdSleepMilliseconds(1000);
	}
	return 0;
}
#endif /* SITL */

static int32_t pprz_thd(void *arg)
{
	/*
	   To be compatible with rtos architecture, each of this 4 workers should
	   be implemented in differents threads, each of them waiting for job to be done:
	   periodic task should sleep, and event task should wait for event
	   */
	(void) arg;
	chibios_chRegSetThreadName("pprz big loop");

	while (!chThdShouldTerminate())
	{
		handle_periodic_tasks();
		main_event();
		chibios_chThdSleepMilliseconds(1);
	}

	return 0;
}

STATIC_INLINE void main_init(void)
{
	//mcu_init();

	electrical_init();

	stateInit();

	actuators_init();
#if USE_MOTOR_MIXING
	motor_mixing_init();
#endif

	radio_control_init();

#if USE_BARO_BOARD
	baro_init();
#endif
	imu_init();
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
	downlink_init();
#endif

	// register the timers for the periodic functions
	main_periodic_tid = sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
	modules_tid = sys_time_register_timer(1. / MODULES_FREQUENCY, NULL);
	radio_control_tid = sys_time_register_timer((1. / 60.), NULL);
	failsafe_tid = sys_time_register_timer(0.05, NULL);
	electrical_tid = sys_time_register_timer(0.1, NULL);
	telemetry_tid = sys_time_register_timer((1. / TELEMETRY_FREQUENCY), NULL);
#if USE_BARO_BOARD
	baro_tid = sys_time_register_timer(1. / BARO_PERIODIC_FREQUENCY, NULL);
#endif

	// send body_to_imu from here for now
	AbiSendMsgBODY_TO_IMU_QUAT(1, orientationGetQuat_f(&imu.body_to_imu));
}

STATIC_INLINE void handle_periodic_tasks(void)
{
	if (sys_time_check_and_ack_timer(main_periodic_tid))
	{
		main_periodic();
	}
	if (sys_time_check_and_ack_timer(modules_tid))
	{
		modules_periodic_task();
	}
	if (sys_time_check_and_ack_timer(radio_control_tid))
	{
		radio_control_periodic_task();
	}
	if (sys_time_check_and_ack_timer(failsafe_tid))
	{
		failsafe_check();
	}
	if (sys_time_check_and_ack_timer(electrical_tid))
	{
		electrical_periodic();
	}
	if (sys_time_check_and_ack_timer(telemetry_tid))
	{
		telemetry_periodic();
	}
#if USE_BARO_BOARD
	if (sys_time_check_and_ack_timer(baro_tid))
	{
		baro_periodic();
	}
#endif
}

STATIC_INLINE void main_periodic(void)
{

	imu_periodic();

	//FIXME: temporary hack, remove me
#ifdef InsPeriodic
	InsPeriodic();
#endif

	/* run control loops */
	autopilot_periodic();
	/* set actuators     */
	//actuators_set(autopilot_motors_on);
	SetActuatorsFromCommands(commands, autopilot_mode);

	if (autopilot_in_flight)
	{
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

	/* initialisation phase during boot */
	if (boot)
	{
#if DOWNLINK
		send_autopilot_version(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
#endif
		boot = FALSE;
	}
	/* then report periodicly */
	else
	{
#if PERIODIC_TELEMETRY
		periodic_telemetry_send_Main(DefaultPeriodic, &(DefaultChannel).trans_tx, &(DefaultDevice).device);
#endif
	}
}

/** mode to enter when RC is lost while using a mode with RC input (not AP_MODE_NAV) */
#ifndef RC_LOST_MODE
#define RC_LOST_MODE AP_MODE_FAILSAFE
#endif

STATIC_INLINE void failsafe_check(void)
{
	if (radio_control.status == RC_REALLY_LOST &&
			autopilot_mode != AP_MODE_KILL &&
			autopilot_mode != AP_MODE_HOME &&
			autopilot_mode != AP_MODE_FAILSAFE &&
			autopilot_mode != AP_MODE_NAV)
	{
		autopilot_set_mode(RC_LOST_MODE);
	}

#if FAILSAFE_ON_BAT_CRITICAL
	if (autopilot_mode != AP_MODE_KILL &&
			electrical.bat_critical)
	{
		autopilot_set_mode(AP_MODE_FAILSAFE);
	}
#endif

#if USE_GPS
	gps_periodic_check();
	if (autopilot_mode == AP_MODE_NAV &&
			autopilot_motors_on &&
#if NO_GPS_LOST_WITH_RC_VALID
			radio_control.status != RC_OK &&
#endif
			GpsIsLost())
	{
		autopilot_set_mode(AP_MODE_FAILSAFE);
	}

	if (autopilot_mode == AP_MODE_HOME &&
			autopilot_motors_on && GpsIsLost())
	{
		autopilot_set_mode(AP_MODE_FAILSAFE);
	}
#endif

	autopilot_check_in_flight(autopilot_motors_on);
}

STATIC_INLINE void main_event(void)
{
	/* event functions for mcu peripherals: i2c, usb_serial.. */
	mcu_event();

	DatalinkEvent();

	if (autopilot_rc)
	{
		RadioControlEvent(autopilot_on_rc_frame);
	}

	ImuEvent();

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


