/*
 * Copyright (C) 2006 Pascal Brisset, Antoine Drouin
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

/** \file downlink.c
 *  \brief Common code for AP and FBW telemetry
 *
 */


#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/datalink.h"
#ifdef GCS_V1_OPTION
#include "subsystems/datalink/downlink_xbee_periodic.h"
#endif	/* GCS_V1_OPTION */

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#include "subsystems/datalink/datalink.h"
#include "mcu_periph/sys_time.h"
#include "modules/system/timer_if.h"

static uint32_t last_down_nb_bytes = 0;  // previous number of bytes sent
static uint32_t last_up_nb_msgs = 0;  // previous number of received messages
static uint32_t last_ts = 0;  // timestamp in usec when last message was send

static void send_downlink(struct transport_tx *trans, struct link_device *dev)
{
	// current timestamp
	uint32_t now_ts = get_sys_time_msec();
	// compute downlink byte rate
	if (now_ts > last_ts)
	{
		uint16_t down_rate = (1000 * ((uint32_t)dev->nb_bytes - last_down_nb_bytes)) / (now_ts - last_ts);
		uint16_t up_rate = (1000 * ((uint32_t)datalink_nb_msgs - last_up_nb_msgs)) / (now_ts - last_ts);

		last_ts = now_ts;
#if defined DATALINK || defined SITL
		last_down_nb_bytes = dev->nb_bytes;
		last_up_nb_msgs = datalink_nb_msgs;
#else
		last_down_nb_bytes = 0;
		last_up_nb_msgs = 0;
#endif
		xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
		pprz_msg_send_DATALINK_REPORT(trans, dev, AC_ID, &datalink_time, &datalink_nb_msgs, &dev->nb_msgs, &down_rate,
																	&up_rate, &dev->nb_ovrn);
	}
}
#endif

void downlink_init(void)
{
	// Set initial counters
	(DefaultDevice).device.nb_ovrn = 0;
	(DefaultDevice).device.nb_bytes = 0;
	(DefaultDevice).device.nb_msgs = 0;
#if DATALINK == TRANSPTA
	(SecondDevice).device.nb_ovrn = 0;
	(SecondDevice).device.nb_bytes = 0;
	(SecondDevice).device.nb_msgs = 0;
#endif

#if defined DATALINK

	datalink_nb_msgs = 0;

#if DATALINK == PPRZ || DATALINK == SUPERBITRF || DATALINK == W5100 || DATALINK == BLUEGIGA || DATALINK == TRANSPTA
	pprz_transport_init(&pprz_tp);
#endif

#if DATALINK == XBEE
	xbee_init();
#if USE_MANU_DEBUG
	pprz_transport_init(&mdebug_tp);
#endif
#endif
#if DATALINK == TRANSPTA
	pta_init();
#endif
#if DATALINK == W5100
	w5100_init();
#endif
#if DATALINK == BLUEGIGA
	bluegiga_init(&bluegiga_p);
#endif

#endif

#if USE_PPRZLOG
	pprzlog_transport_init();
#endif

#if SITL && !USE_NPS
	ivy_transport_init();
#endif

#if PERIODIC_TELEMETRY
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DATALINK_REPORT, send_downlink);
#endif
}

void downlink_periodic(void)
{
#if DATALINK == XBEE
	xbee_periodic();
#endif

#if DATALINK == TRANSPTA
	pta_periodic();
#endif

#ifdef GCS_V1_OPTION
	downlink_gcs_periodic();
	downlink_pc_periodic();
#endif

#if DATALINK == PPRZ
	tm_stimulate(TIMER_TASK_TELEMETRY);
#endif
}

void xbee_tx_header(uint8_t ack, uint8_t addr)
{
#if DATALINK == XBEE
	xbee_tx_frame_header(ack, addr);
#endif

#if DATALINK == TRANSPTA
	pta_set_tx_frame_addr(ack, addr);
#endif
}

