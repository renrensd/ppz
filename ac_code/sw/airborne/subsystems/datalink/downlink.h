/*
 * Copyright (C) 2003-2006  Pascal Brisset, Antoine Drouin
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

/** \file downlink.h
 *  \brief Common code for AP and FBW telemetry
 *
 */

#ifndef DOWNLINK_H
#define DOWNLINK_H

#include <inttypes.h>

#ifndef PPRZ_DATALINK_EXPORT

#include "generated/modules.h"
#include "messages.h"
#include "generated/airframe.h" // AC_ID is required

#if defined SITL && !USE_NPS
/** Software In The Loop simulation uses IVY bus directly as the transport layer */
#include "ivy_transport.h"

#else /** SITL */
#include "subsystems/datalink/datalink.h"

#include "subsystems/datalink/pprz_transport.h"
#include "subsystems/datalink/pprzlog_transport.h"
#ifdef BBOX_OPTION
#include "subsystems/datalink/can_transport.h"
#endif	/* BBOX_OPTION */
#if DATALINK == XBEE
#include "subsystems/datalink/xbee.h"
#endif	/* XBEE */
#include "subsystems/datalink/w5100.h"
#if DATALINK == TRANSPTA
#include "subsystems/datalink/transpta.h"
#endif	/* TRANSPTA */
#if DATALINK == BLUEGIGA
#include "subsystems/datalink/bluegiga.h"
#endif
#if USE_SUPERBITRF
#include "subsystems/datalink/superbitrf.h"
#endif
#if USE_AUDIO_TELEMETRY
#include "subsystems/datalink/audio_telemetry.h"
#endif
#if USE_USB_SERIAL
#include "mcu_periph/usb_serial.h"
#endif
#ifdef USE_UDP
#include "mcu_periph/udp.h"
#endif
#include "mcu_periph/uart.h"

#endif /** !SITL */

#else /* PPRZ_DATALINK_EXPORT defined */

#include "messages.h"
#include "pprz_transport.h"
#ifndef AC_ID
#define AC_ID 1
#endif

#endif

#ifdef GCS_V1_OPTION
#include"uplink_ac.h"
#include"uplink_gcs.h"
#include"uplink_rc.h"
#endif


#if 0//def BBOX_OPTION
#define DefaultChannel can_tp	//pprz_tp\can_tp\pta_tp.
#define DefaultDevice can_tp	//uart2\can_tp.
#else
#define DefaultChannel DOWNLINK_TRANSPORT	//pprz_tp\can_tp\pta_tp.
#define DefaultDevice DOWNLINK_DEVICE	//uart2\can_tp.
#endif	/* BBOX_OPTION */

#if DATALINK==TRANSPTA
#define SecondChannel DOWNLINK_TRANSPORT_PTA
#define SecondDevice DOWNLINK_DEVICE_PTA
#else
#define SecondChannel DOWNLINK_TRANSPORT
#define SecondDevice DOWNLINK_DEVICE
#endif	/* TRANSPTA */

// Init function
extern void downlink_init(void);
extern void downlink_periodic(void);
extern void xbee_tx_header(uint8_t ack, uint8_t addr);

#endif /* DOWNLINK_H */
