/*
 * Copyright (C) 2006  Pascal Brisset, Antoine Drouin
 * Copyright (C) 2014  Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 *
 */

/**
 * @file subsystems/datalink/xbee.h
 * Maxstream XBee serial input and output
 */

#ifndef XBEE_H
#define XBEE_H

#include "subsystems/datalink/datalink.h"
#include "generated/airframe.h"
#include "subsystems/datalink/transport.h"

//#include "subsystems/datalink/downlink.h"

#define XBEE868
#ifdef XBEE868
#include "subsystems/datalink/xbee868.h"
#else /* Not 868 */
#include "subsystems/datalink/xbee24.h"
#endif
#include "subsystems/datalink/xbee_msg_def.h"

#include <stdio.h>
#include <string.h>

#define XBEE_START 0x7E

#ifdef GCS_V1_OPTION   
#include "firmwares/rotorcraft/autopilot.h"
#include "subsystems/settings.h"

//extern tid_t xbee_bc_tid; //id for xbee_msg_aircraft_ready_broadcast() timer.
//extern tid_t xbee_heart_beat_tid;

#define A2G_SERIAL_CODE 	    {0x09,0x24,0x52,0x73,0x5D,0x45,0x68,0x21,0x29,0x70}
#define A2R_SERIAL_CODE 	    {0x09,0x74,0x12,0x83,0x9D,0x15,0x08,0x41,0x21,0x65}
#define GCS_SERIAL_CODE 		{0x09,0x35,0x40,0x72,0x55,0x76,0x64,0x2A,0x2C,0x6E}
#define RC_SERIAL_CODE 		{0x09,0x58,0x4E,0x7B,0x4A,0x5B,0x6D,0x62,0x3B,0x57}

#define AC_SN_CODE    {0x45,0x46,0x41,0x31,0x35,0x30,0x00,0x00,0x00,0x00}    //"EFA150"
//#define PPZCENTER_ADDR          {0x00,0x13,0xA2,0x00,0x40,0xF5,0x4A,0xE2}  //no3
//#define PPZCENTER_ADDR          {0x00,0x13,0xA2,0x00,0x41,0x46,0xF1,0xC1}  //no4
//#define PPZCENTER_ADDR          {0x00,0x13,0xA2,0x00,0x40,0xFC,0xE0,0x7D}  //no5
//#define PPZCENTER_ADDR          {0x00,0x13,0xA2,0x00,0x41,0x4E,0x7E,0x91}   //no8
#define PPZCENTER_ADDR					{0x00,0x13,0xA2,0x00,0x40,0xD4,0x2E,0x23} //long-arm and small-2
//#define PPZCENTER_ADDR          {0x00,0x13,0xA2,0x00,0x40,0xF1,0xEB,0x1B}
//#define PPZCENTER_ADDR         {0x00,0x13,0xA2,0x00,0x40,0xf3,0x3b,0x3f}
#define XBEE_ADDR_OFFSET 1
#define XBEE_ADDR_LEN 8
#define XBEE_SERIAL_CODE_OFFSET 2
#define XBEE_SYSTEM_ID_OFFSET 0
#define XBEE_SERIAL_CODE_LEN 10
#define XBEE_BC_PERIODIC_FREQUENCY 0.5
#define XBEE_HEART_BEAT_FREQUENCY 0.5
#endif //GCS_V1_OPTION

/** Initialisation in API mode and setting of the local address
 * FIXME: busy wait */
void xbee_init(void);

#ifdef GCS_V1_OPTION
void xbee_set_tx_header(uint8_t seq, uint8_t *addr, uint8_t option);
void xbee_msg_rc_ready_response(void);
#endif//GCS_V1_OPTION

extern void xbee_tx_frame_header(uint8_t ack, uint8_t addr);
extern void xbee_periodic(void);


/** Status of the API packet receiver automata */
#define XBEE_UNINIT         0
#define XBEE_GOT_START      1
#define XBEE_GOT_LENGTH_MSB 2
#define XBEE_GOT_LENGTH_LSB 3
#define XBEE_GOT_PAYLOAD    4

struct xbee_transport {
  // generic reception interface
  struct transport_rx trans_rx;
  // specific xbee transport variables
  uint8_t status;
  uint8_t payload_idx;
  uint8_t cs_rx;
  uint8_t rssi;
  // generic transmission interface
  struct transport_tx trans_tx;
  // specific pprz transport_tx variables
  uint8_t cs_tx;
};

#ifdef GCS_V1_OPTION
struct xbee_connect_info
{
	bool_t gcs_con_available;
	uint8_t gcs_addr[3][8];
	uint8_t gcs_num;
	bool_t rc_con_available;
	uint8_t rc_addr[3][8];
	uint8_t rc_num;
	bool_t ppzcenter_con_available;
	uint8_t ppzcenter_addr[3][8];
	uint8_t ppzcenter_num;
};
extern uint8_t pprzcenter_addr[8];
extern struct xbee_connect_info xbee_con_info;
#endif //GCS_V1_OPTION

void xbee_check_and_parse(struct link_device *dev, struct xbee_transport *trans);

#define XBeeCheckAndParse(_dev, _trans) xbee_check_and_parse(&(_dev).device, &(_trans))

extern struct xbee_transport xbee_tp;

extern void XbeeSetRcConFalse(void);
extern void XbeeSetGcsConFalse(void);
extern void XbeeSetFailBind(void);
extern void XbeeSetSuccessBind(void);

#endif /* XBEE_H */
