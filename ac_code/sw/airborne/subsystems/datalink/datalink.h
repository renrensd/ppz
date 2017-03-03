/*
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
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
/** \file datalink.h
 *  \brief Handling of messages coming from ground and other A/Cs
 *
 */

#ifndef DATALINK_H
#define DATALINK_H

#ifdef DATALINK_C
#define EXTERN
#else
#define EXTERN extern
#endif

#ifdef __IEEE_BIG_ENDIAN /* From machine/ieeefp.h */
#define Swap32IfBigEndian(_u) { _u = (_u << 32) | (_u >> 32); }
#else
#define Swap32IfBigEndian(_) {}
#endif

#include "std.h"
#include "dl_protocol.h"
#include "subsystems/datalink/xbee_msg_def.h"


/** Datalink kinds */
#define PPRZ 1
#define XBEE 2
#define SUPERBITRF 3
#define W5100 4
#define BLUEGIGA 5
#define TRANSPTA 6

#ifdef UPGRADE_OPTION
enum UPGRADE_TYPE_PARAM
{
	UPGRADE_TYPE_AC = 0x01,
	UPGRADE_TYPE_OPS,
	UPGRADE_TYPE_BBOX,
};

enum UPGRADE_RESPONSE_PARAM
{
	UPGRADE_RES_OK,
	UPGRADE_RES_FAIL,
};
#endif	/* UPGRADE_OPTION */

/** Flag provided to control calls to ::dl_parse_msg. NOT used in this module*/
EXTERN bool_t dl_msg_available;

/** time in seconds since last datalink message was received */
EXTERN uint16_t datalink_time;

/** number of datalink/uplink messages received */
EXTERN uint16_t datalink_nb_msgs;

#define MSG_SIZE 256
EXTERN uint8_t dl_buffer[MSG_SIZE]  __attribute__((aligned));

/** Should be called when chars are available in dl_buffer */
EXTERN void dl_parse_msg(void);

#if USE_NPS
EXTERN bool_t datalink_enabled;
#endif

/** Check for new message and parse */
static inline void DlCheckAndParse(void)
{
  // make it possible to disable datalink in NPS sim
#if USE_NPS
  if (!datalink_enabled) {
    return;
  }
#endif

  if (dl_msg_available) {
    datalink_time = 0;
    datalink_nb_msgs++;
    dl_parse_msg();
    dl_msg_available = FALSE;
  }
}

#if defined DATALINK && DATALINK == PPRZ

#define DatalinkEvent() {                       \
    PprzCheckAndParse(PPRZ_UART, pprz_tp);      \
    DlCheckAndParse();                          \
  }

#elif defined DATALINK && DATALINK == XBEE
#if USE_MANU_DEBUG
#define DatalinkEvent() {                       \
    XBeeCheckAndParse(XBEE_UART, xbee_tp);      \
    DlCheckAndParse();                          \
	PprzCheckAndParse(MDEBUG_UART, mdebug_tp);  \
    DlCheckAndParse();                          \
  }
#else
#define DatalinkEvent() {                       \
    XBeeCheckAndParse(XBEE_UART, xbee_tp);      \
    DlCheckAndParse();                          \
  }
#endif  //end USE_MANU_MDEBUG
#elif defined DATALINK && DATALINK == TRANSPTA

#define DatalinkEvent() {                       \
    PtaCheckAndParse(PTA_UART, pta_tp);      \
    DlCheckAndParse();                          \
  }

#elif defined DATALINK && DATALINK == W5100

#define DatalinkEvent() {                       \
    W5100CheckAndParse(W5100, pprz_tp);         \
    DlCheckAndParse();                          \
  }

#elif defined DATALINK && DATALINK == SUPERBITRF

#define DatalinkEvent() {                       \
    SuperbitRFCheckAndParse();                  \
    DlCheckAndParse();                          \
  }

#elif defined DATALINK && DATALINK == BLUEGIGA

#define DatalinkEvent() {                       \
    BlueGigaCheckAndParse(DOWNLINK_DEVICE, pprz_tp);   \
    DlCheckAndParse();                          \
  }

#else

// Unknown DATALINK
#define DatalinkEvent() {}

#endif /* DATALINK == */

#endif /* DATALINK_H */
