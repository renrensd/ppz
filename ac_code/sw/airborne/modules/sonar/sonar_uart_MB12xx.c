/*
 * Copyright (C) 2010  Gautier Hattenberger, 2013 Tobias Münch
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
 *
 */

#include "modules/sonar/sonar_uart_MB12xx.h"
#include "generated/airframe.h"
//#include "mcu_periph/adc.h"
#include "subsystems/abi.h"
//#include "subsystems/abi_sender_ids.h"
#ifdef SITL
#include "state.h"
#endif
#include"modules/sonar/agl_dist.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
//#include "mcu_periph/gpio.h"
//#include "arch/lpc21/efs/inc/plibc.h"
#include "std.h"

#define SonarLinkDev  (uart3.device)
#define SonarLinkTransmit(c) SonarLinkDev.put_byte(SonarLinkDev.periph, c)
#define SonarLinkChAvailable() SonarLinkDev.char_available(SonarLinkDev.periph)
#define SonarLinkGetch() SonarLinkDev.get_byte(SonarLinkDev.periph)

struct sonar_data Sonar_MB12;

#define SONAR_STX 0X52
#define SONAR_FRAME_LEN 0x03
#define SONAR_ETX 0x0D

// sonar parsing state machine
#define SONAR_UNINIT      0
#define SONAR_GOT_STX     1
#define SONAR_GOT_PAYLOAD 2
#define SONAR_GOT_ETX    3


static inline void parse_sonar(struct sonar_data *t, uint8_t c)
{
	switch (t->status)
	{
	case SONAR_UNINIT:
		if (c == SONAR_STX)
		{
			t->status++;
			t->payload_idx = 0;
		}
		break;
	case SONAR_GOT_STX:
		if (t->msg_received)
		{
			t->ovrn++;
			goto error;
		}
		t->payload[t->payload_idx] = c;
		t->payload_idx++;
		if (t->payload_idx == SONAR_FRAME_LEN)
		{
			t->status++;
		}
		break;
	case SONAR_GOT_PAYLOAD:
		if (c != SONAR_ETX)
		{
			goto error;
		}
		t->msg_received = TRUE;
		goto restart;
	default:
		goto error;
	}
	return;
error:
	t->error++;
restart:
	t->status = UNINIT;
	return;
}

void sonar_uart_read(void)
{
	if ( SonarLinkChAvailable() )
	{
		while ( SonarLinkChAvailable() && !Sonar_MB12.msg_received )
		{
			parse_sonar( &Sonar_MB12, SonarLinkGetch() );
		}
		if (Sonar_MB12.msg_received)
		{
			Sonar_MB12.distance_cm = (Sonar_MB12.payload[0]-0x30)*100 + (Sonar_MB12.payload[1]-0x30)*10
															 + (Sonar_MB12.payload[2]-0x30);

			Sonar_MB12.msg_received = FALSE;
#if 0
			if ( Sonar_MB12.distance_cm < 20 )
				return;

			if ( Sonar_MB12.distance_cm > 700 )
				return;
#endif

			Sonar_MB12.distance_m=(float)Sonar_MB12.distance_cm/100.0;

			// Send ABI message
			AbiSendMsgAGL(AGL_SONAR_ADC_ID, Sonar_MB12.distance_m);

#if PERIODIC_TELEMETRY
			xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
			DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &Sonar_MB12.distance_cm, &agl_dist_value_filtered);
#endif
		}
	}
}
