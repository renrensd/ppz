/*
 * Copyright (C) 2006-2014 The Paparazzi Team
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file subsystems/radio_control.c
 *
 * Implementation independant radio control handing.
 *
 */

#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"

struct RadioControl radio_control;

bool_t radio_kill_switch = TRUE;
uint8_t radio_ap_mode = 0;

void radio_control_init(void)
{
	uint8_t i;
	for (i = 0; i < RADIO_CONTROL_NB_CHANNEL; i++)
	{
		radio_control.values[i] = 0;
	}
	radio_control.link_status = RC_LINK_LOST;
	radio_control.status = RC_REALLY_LOST;
	radio_control.time_since_last_frame = RC_REALLY_LOST_TIME;
	radio_control.link_counter = 0;
	radio_control.radio_ok_cpt = 0;
	radio_control.frame_rate = 0;
	radio_control.frame_cpt = 0;
	radio_control_impl_init();
}

void radio_control_periodic_task(void)
{
	static uint8_t _1Hz;
	_1Hz++;

	if (_1Hz >= 60)
	{
		_1Hz = 0;
		radio_control.frame_rate = radio_control.frame_cpt;
		radio_control.frame_cpt = 0;
		if(radio_control.link_counter > 40 )   //current 1s have 40times upper lost
		{
			radio_control.link_status++;
			if(radio_control.link_status >= RC_LINK_LOST)  //keep 5s will set link_lost
			{
				radio_control.link_status = RC_LINK_LOST;
			}
		}
		else
		{
			radio_control.link_status = RC_LINK_OK;  //once recover,set link_ok
		}
		radio_control.link_counter = 0; //reset
	}

	if (radio_control.time_since_last_frame >= RC_REALLY_LOST_TIME)
	{
		radio_control.status = RC_REALLY_LOST;
	}
	else
	{
		if (radio_control.time_since_last_frame >= RC_LOST_TIME)
		{
			radio_control.status = RC_LOST;
			radio_control.radio_ok_cpt = RC_OK_CPT;
		}
		radio_control.time_since_last_frame++;
	}

	/*use RADIO_RADIO_KILL_SWITCH channel lost status as lost time counter*/
	if( (USEC_OF_RC_PPM_TICKS(ppm_pulses[RADIO_GAIN1]) < 1780)
			&&(USEC_OF_RC_PPM_TICKS(ppm_pulses[RADIO_GAIN1]) > 1600) )
	{
		radio_control.link_counter++;
	}

#if defined RADIO_CONTROL_LED
	if (radio_control.status == RC_OK)
	{
		LED_ON(RADIO_CONTROL_LED);
	}
	else
	{
		LED_OFF(RADIO_CONTROL_LED);
	}
#endif
#ifdef RADIO_KILL_SWITCH
	radio_kill_switch = kill_switch_is_on();
#endif
	radio_ap_mode = get_ap_mode_of_radio();


}
