/*
 * Copyright (C) 2012 The Paparazzi Team
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
 * @file firmwares/rotorcraft/autopilot_rc_helpers.h
 *
 * Some helper functions to check RC sticks.
 */

#ifndef AUTOPILOT_RC_HELPERS_H
#define AUTOPILOT_RC_HELPERS_H

#include "generated/airframe.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/autopilot.h"

#define AUTOPILOT_THROTTLE_THRESHOLD      (MAX_PPRZ / 20)
#define AUTOPILOT_YAW_THRESHOLD           (MAX_PPRZ * 19 / 20)
#ifndef AUTOPILOT_STICK_CENTER_THRESHOLD
#define AUTOPILOT_STICK_CENTER_THRESHOLD  (MAX_PPRZ * 1 / 10)
#endif

#define THROTTLE_STICK_DOWN()                                           \
  (radio_control.values[RADIO_THROTTLE] < AUTOPILOT_THROTTLE_THRESHOLD)
#define YAW_STICK_PUSHED()                                      \
  (radio_control.values[RADIO_YAW] > AUTOPILOT_YAW_THRESHOLD ||  \
   radio_control.values[RADIO_YAW] < -AUTOPILOT_YAW_THRESHOLD)
#define YAW_STICK_CENTERED()                                            \
  (radio_control.values[RADIO_YAW] < AUTOPILOT_STICK_CENTER_THRESHOLD && \
   radio_control.values[RADIO_YAW] > -AUTOPILOT_STICK_CENTER_THRESHOLD)
#define PITCH_STICK_CENTERED()                                          \
  (radio_control.values[RADIO_PITCH] < AUTOPILOT_STICK_CENTER_THRESHOLD && \
   radio_control.values[RADIO_PITCH] > -AUTOPILOT_STICK_CENTER_THRESHOLD)
#define ROLL_STICK_CENTERED()                                           \
  (radio_control.values[RADIO_ROLL] < AUTOPILOT_STICK_CENTER_THRESHOLD && \
   radio_control.values[RADIO_ROLL] > -AUTOPILOT_STICK_CENTER_THRESHOLD)

static inline bool_t rc_attitude_sticks_centered(void)
{
	return ROLL_STICK_CENTERED() && PITCH_STICK_CENTERED() && YAW_STICK_CENTERED();
}

#ifdef RADIO_KILL_SWITCH
static inline bool_t kill_switch_is_on(void)
{
	static uint8_t counter = 0;

	if(radio_control.link_status != RC_LINK_LOST
			&& radio_control.status != RC_REALLY_LOST)
	{
		if( radio_control.values[RADIO_RADIO_KILL_SWITCH] < 0)
		{
			//must be RADIO_RADIO_KILL_SWITCH,RADIO_KILL_SWITCH is defined true in airframe
			counter++;
			if( counter > 20)
			{
				counter = 20;
				return TRUE;
			}
			return FALSE;
		}
		else
		{
			counter = 0;
			return FALSE;
		}
	}
	else  //link lost,must set kill,except nav
	{
		if(autopilot_mode == AP_MODE_NAV)
		{
			return FALSE;
		}
		return TRUE;
	}
}
#else
static inline bool_t kill_switch_is_on(void)
{
	return FALSE;
}
#endif

#define THRESHOLD_1_PPRZ (MIN_PPRZ / 2)
#define THRESHOLD_2_PPRZ (MAX_PPRZ / 2)
static inline uint8_t get_ap_mode_of_radio(void)
{
	static uint8_t counter = 0;
	static uint8_t ap_mode_return = 0;
	static uint8_t ap_mode_last = 0;
	uint8_t ap_mode_now;
	pprz_t values = radio_control.values[RADIO_MODE];
	if (values > THRESHOLD_2_PPRZ)
	{
		ap_mode_now = autopilot_mode_auto2;
	}
	else if (values > THRESHOLD_1_PPRZ)
	{
		ap_mode_now = MODE_AUTO1;
	}
	else
	{
		ap_mode_now = MODE_MANUAL;
	}

	if(ap_mode_now != ap_mode_return)
	{
		if(ap_mode_now == ap_mode_last)
		{
			counter++;
		}
		else
		{
			counter = 0;
		}
		if(counter > 10)
		{
			counter = 10;
			ap_mode_return = ap_mode_last;
		}
	}
	ap_mode_last = ap_mode_now;

	return ap_mode_return;
}

static inline uint8_t percent_from_rc(int channel)
{
	int per = (MAX_PPRZ + (int32_t)radio_control.values[channel]) * 50 / MAX_PPRZ;
	if (per < 0)
	{
		per = 0;
	}
	else if (per > 100)
	{
		per = 100;
	}
	return per;
}


#endif /* AUTOPILOT_RC_HELPERS_H */
