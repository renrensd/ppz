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

#include "modules/laser/laser_r2100.h"
#include "generated/airframe.h"
//#include "mcu_periph/adc.h"
#include "subsystems/abi.h"
//#include "subsystems/abi_sender_ids.h"
#ifdef SITL
#include "state.h"
#endif

#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "std.h"

#define laserDev  (uart1.device)
#define laserTransmit(c) laserDev.put_byte(laserDev.periph, c)
#define laserChAvailable() laserDev.char_available(laserDev.periph)
#define laserGetch() laserDev.get_byte(laserDev.periph)

struct LASER_R2100 laser_r2100;
struct LASER_R2100_DATA laser_data;


#define LASER_R2100_STX 	0x01
#define LASER_R2100_TXID 	0xDE
#define LASER_R2100_LEN 	0x32
#define LASER_R2100_CMD 	0x11

// laser parsing state machine
#define LASER_R2100_UNINIT      	0
#define LASER_R2100_GOT_STX   		1
#define LASER_R2100_GOT_TXID   		2
#define LASER_R2100_GOT_LEN     	3
#define LASER_R2100_GOT_CMD     	4
#define LASER_R2100_GOT_PAYLOAD 	5
#define LASER_R2100_GOT_REV			6


static inline void parse_laser(struct LASER_R2100 *t, uint8_t c)
{
	switch (t->status)
	{
	case LASER_R2100_UNINIT:
		if (c == LASER_R2100_STX)
		{
			t->cs = 0;
			t->status++;
			t->cs ^= c;
		}
		break;
	case LASER_R2100_GOT_STX:
		if (c == LASER_R2100_TXID)
		{
			t->status++;
			t->cs ^= c;
		}
		else
		{
			t->status = LASER_R2100_UNINIT;
		}
		break;
	case LASER_R2100_GOT_TXID:
		if (c == LASER_R2100_LEN)
		{
			t->status++;
			t->cs ^= c;
			t->payload_idx = 0;
		}
		else
		{
			t->status = LASER_R2100_UNINIT;
		}
		break;
	case LASER_R2100_GOT_LEN:
		if (c == LASER_R2100_CMD)
		{
			t->status++;
			t->cs ^= c;
			t->payload_idx = 0;
		}
		else
		{
			t->status = LASER_R2100_UNINIT;
		}
		break;
	case LASER_R2100_GOT_CMD:
		t->payload[t->payload_idx] = c;
		t->cs ^= c;
		t->payload_idx++;
		if (t->payload_idx == LASER_R2100_PAYLOAD_LEN)
		{
			t->status++;
		}
		break;
	case LASER_R2100_GOT_PAYLOAD:
		t->status++;
		t->cs ^= c;
		break;
	case LASER_R2100_GOT_REV:
		if (c != t->cs)
		{
			goto error;
		}
		t->msg_received = TRUE;
		t->cs = 0;
		goto restart;

	default:
		goto error;
	}
	return;
error:
	t->error++;
restart:
	t->status = LASER_R2100_UNINIT;
	return;
}

void laser_r2100_event(void)
{
	uint8_t i;

	if ( laserChAvailable() )
	{
		while ( laserChAvailable() && !laser_r2100.msg_received )
		{
			parse_laser( &laser_r2100, laserGetch() );
		}
		if (laser_r2100.msg_received)
		{
			laser_r2100.msg_received = FALSE;
			for(i=0; i<R2100_DIS_NUM; i++)
			{
				laser_data.dis[i] = laser_r2100.payload[i*4] | (laser_r2100.payload[i*4+1] << 8);
				laser_data.echo[i] = laser_r2100.payload[i*4+2] | (laser_r2100.payload[i*4+3] << 8);
			}
#if 0//PERIODIC_TELEMETRY
			RunOnceEvery(10,
			{
				xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
				DOWNLINK_SEND_LASER_DATA(DefaultChannel, DefaultDevice, &laser_data.dis[0],&laser_data.echo[0]);
			}    );
#endif
		}
	}
}

void laser_r2100_send_msg(void)
{
	laserTransmit(0xDE);
	laserTransmit(0x01);
	laserTransmit(0x05);
	laserTransmit(0x59);
	laserTransmit(0x83);
}
