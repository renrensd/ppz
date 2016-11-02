/*
 * Copyright (C) 2010-2014 The Paparazzi Team
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
 * @file subsystems/radio_control/ppm.c
 *
 * Architecture independent functions for PPM radio control.
 *
 */

#include "subsystems/radio_control.h"
#include "subsystems/radio_control/ppm.h"
#include "filters/median_filter.h"
#include "filters/low_pass_filter.h"
#include "controllers/pid.h"

uint16_t ppm_pulses[RADIO_CTL_NB];
//struct MedianFilterInt ppm_pulses_filter[RADIO_CTL_NB];
//Butterworth2LowPass_int ppm_pulses_filter2[RADIO_CTL_NB];
//float ppm_pulses_filter3[RADIO_CTL_NB];
volatile bool_t ppm_frame_available;

/*
 * State machine for decoding ppm frames
 */
static uint8_t  ppm_cur_pulse;
static uint32_t ppm_last_pulse_time;
static bool_t   ppm_data_valid;

/**
 * RssiValid test macro.
 * This macro has to be defined to test the validity of ppm frame
 * from an other source (ex: GPIO).
 * By default, always true.
 */
#ifndef RssiValid
#define RssiValid() TRUE
#endif


#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_ppm(struct transport_tx *trans, struct link_device *dev)
{
  uint16_t ppm_pulses_usec[RADIO_CTL_NB];
  for (int i = 0; i < RADIO_CTL_NB; i++) {
    ppm_pulses_usec[i] = USEC_OF_RC_PPM_TICKS(ppm_pulses[i]);
  }
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  pprz_msg_send_PPM(trans, dev, AC_ID,
                    &radio_control.frame_rate, RADIO_CTL_NB, ppm_pulses_usec);
}
#endif

void radio_control_impl_init(void)
{
  ppm_frame_available = FALSE;
  ppm_last_pulse_time = 0;
  ppm_cur_pulse = RADIO_CTL_NB;
  ppm_data_valid = FALSE;

  for (int i = 0; i < RADIO_CTL_NB; i++)
  {
//  	init_median_filter(&(ppm_pulses_filter[i]));
//  	init_butterworth_2_low_pass_int(&(ppm_pulses_filter2[i]), 1, 0.02f, 0);
  }

  ppm_arch_init();

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PPM, send_ppm);
#endif
}

void radio_control_impl_event(void (* _received_frame_handler)(void))
{
  if (ppm_frame_available) {
    radio_control.frame_cpt++;
    radio_control.time_since_last_frame = 0;
    if (radio_control.radio_ok_cpt > 0) {
      radio_control.radio_ok_cpt--;
    } 
	else {
      radio_control.status = RC_OK;
      NormalizePpmIIR(ppm_pulses, radio_control);
      _received_frame_handler();
    }
    ppm_frame_available = FALSE;
  }
  //else radio_control.time_since_last_frame will add
}

/**
 * Decode a PPM frame.
 * A valid ppm frame:
 * - synchro blank
 * - correct number of channels
 * - synchro blank
 */
void ppm_decode_frame(uint32_t ppm_time)
{
	uint32_t length;

	if (ppm_time < ppm_last_pulse_time)
	{
		length = (1 << 16) - (ppm_last_pulse_time - ppm_time);
	}
	else
	{
		length = ppm_time - ppm_last_pulse_time;
	}
	ppm_last_pulse_time = ppm_time;

	if (ppm_cur_pulse == RADIO_CTL_NB)
	{
		//sync sign is coming
		if (length > RC_PPM_TICKS_OF_USEC(PPM_SYNC_MIN_LEN)
				&& length < RC_PPM_TICKS_OF_USEC(PPM_SYNC_MAX_LEN))
		{
			if (ppm_data_valid && RssiValid())
			{
				ppm_frame_available = TRUE;
				ppm_data_valid = FALSE;
			}
			ppm_cur_pulse = 0;
		}
		else
		{
			ppm_data_valid = FALSE;
			//ppm_frame_available = FALSE;  //add:if RC lost,pulse will lower 1010
		}
	}
	else
	{
		if (length > RC_PPM_TICKS_OF_USEC(PPM_DATA_MIN_LEN)
				&& length < RC_PPM_TICKS_OF_USEC(PPM_DATA_MAX_LEN))
		{
			ppm_pulses[ppm_cur_pulse] = length;
//			ppm_pulses[ppm_cur_pulse] = update_median_filter(
//					&(ppm_pulses_filter[ppm_cur_pulse]), length);
//			ppm_pulses[ppm_cur_pulse] =
//					update_butterworth_2_low_pass_int(&(ppm_pulses_filter2[ppm_cur_pulse]), length);
//			ppm_pulses_filter3[ppm_cur_pulse] = pid_simple_filter(0.05f, ppm_pulses_filter3[ppm_cur_pulse], length);
//			ppm_pulses[ppm_cur_pulse] = ppm_pulses_filter3[ppm_cur_pulse];
			ppm_cur_pulse++;
			if (ppm_cur_pulse == RADIO_CTL_NB)
			{
				ppm_data_valid = TRUE;
			}
		}
		else
		{
			ppm_cur_pulse = RADIO_CTL_NB;
			ppm_data_valid = FALSE;
		}
	}
}
