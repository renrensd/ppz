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

//#define GCS_V1_OPTION  //defined, we can use gcs/rc/ubuntu_gcs

#include "modules/system/timer_if.h"
#include "modules/system/timer_class.h"
#include "modules/system/timer_def.h"
//#include "modules/system/types.h"

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

#define AC_SN_CODE    {0x45,0x46,0x41,0x31,0x31,0x35,0x00,0x00,0x00,0x00}    //"EFA115"
//#define PPZCENTER_ADDR          {0x00,0x13,0xA2,0x00,0x41,0x46,0xF1,0xC1}
#define PPZCENTER_ADDR          {0x00,0x13,0xA2,0x00,0x40,0xD6,0xCF,0x02}
//#define PPZCENTER_ADDR          {0x00,0x13,0xA2,0x00,0x40,0xFB,0xA0,0x78}
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
void xbee_msg_aircraft_ready_broadcast(void);
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

extern struct xbee_transport xbee_tp;

/** Parsing a XBee API frame */
static inline void parse_xbee(struct xbee_transport *t, uint8_t c)
{
  switch (t->status) {
    case XBEE_UNINIT:
      if (c == XBEE_START) {
        t->status++;
      }
      break;
    case XBEE_GOT_START:
      if (t->trans_rx.msg_received) {
        t->trans_rx.ovrn++;
        goto error;
      }
      t->trans_rx.payload_len = c << 8;
      t->status++;
      break;
    case XBEE_GOT_LENGTH_MSB:
      t->trans_rx.payload_len |= c;
      t->status++;
      t->payload_idx = 0;
      t->cs_rx = 0;
	  if(t->trans_rx.payload_len > (TRANSPORT_PAYLOAD_LEN -10) )
	  {
		goto error;	 
	  }
      break;
    case XBEE_GOT_LENGTH_LSB:
      t->trans_rx.payload[t->payload_idx] = c;
      t->cs_rx += c;
      t->payload_idx++;
	  if(t->payload_idx > (TRANSPORT_PAYLOAD_LEN -10) )
	  {
		goto error;	 
	  }
	  
      if (t->payload_idx >= t->trans_rx.payload_len) {
        t->status++;
      }
      break;
    case XBEE_GOT_PAYLOAD:
      if (c + t->cs_rx != 0xff) {
        goto error;
      }
      t->trans_rx.msg_received = TRUE;
      goto restart;
      break;
    default:
      goto error;
  }
  return;
error:
  t->trans_rx.error++;
  t->payload_idx = 0;
  t->cs_rx = 0;
  t->status = XBEE_UNINIT;
restart:
  t->status = XBEE_UNINIT;
  t->payload_idx = 0;
  t->cs_rx = 0;
  return;
}

/** Parsing a frame data and copy the payload to the datalink buffer */
static inline void xbee_parse_payload(struct xbee_transport *t)
{
#ifdef GCS_V1_OPTION
  const uint8_t gcskey[] = GCS_SERIAL_CODE;
  uint8_t gcs_equal;
  const uint8_t rckey[] = RC_SERIAL_CODE;
  uint8_t rc_equal;
#endif //GCS_V1_OPTION

  switch (t->trans_rx.payload[0]) 
  {
    case XBEE_RX_ID:
    case XBEE_TX_ID: /* Useful if A/C is connected to the PC with a cable */
      XbeeGetRSSI(t->trans_rx.payload);
      uint8_t i;
      //payload save to dl_buffer
      for (i = XBEE_RFDATA_OFFSET; i < t->trans_rx.payload_len; i++) 
	  {
        dl_buffer[i - XBEE_RFDATA_OFFSET] = t->trans_rx.payload[i];
      }
	  
	  #ifdef GCS_V1_OPTION
		//check gcs serial code.
		if(xbee_con_info.gcs_con_available == FALSE)
		{
		  gcs_equal = TRUE;
		  for(i=0; i<XBEE_SERIAL_CODE_LEN; i++)
		  {
		  	if(dl_buffer[i+XBEE_SERIAL_CODE_OFFSET] != gcskey[i])
		  	{
				gcs_equal = FALSE;
				break;
			}
		  }
		  if(gcs_equal == TRUE)
		  {
			xbee_con_info.gcs_con_available = TRUE;
			xbee_con_info.gcs_num++;
			for (i = XBEE_ADDR_OFFSET; i <= XBEE_ADDR_LEN; i++) 
			{
		       xbee_con_info.gcs_addr[0][i-XBEE_ADDR_OFFSET] = t->trans_rx.payload[i];
		    }

			//sys_time_cancel_timer(xbee_bc_tid);  //cancel broadcast message
			tm_kill_timer(TIMER_XBEE_HEARTBEAT_MSG);
		  }
		}

		//rc broadcast message,check rc serial code.
		if ( xbee_con_info.rc_con_available == FALSE )
		{ 
		  rc_equal = TRUE;
		  for(i=0; i<XBEE_SERIAL_CODE_LEN; i++)
		  {
		  	if(dl_buffer[i+XBEE_SERIAL_CODE_OFFSET] != rckey[i])
		  	{
				rc_equal = FALSE;		
				break;
			}
		  }
		  
		  if(rc_equal == TRUE)
		  {
			//xbee_con_info.rc_con_available = TRUE;
			//xbee_con_info.rc_num++;
			for (i = XBEE_ADDR_OFFSET; i <= XBEE_ADDR_LEN; i++) 
			{
		       if( xbee_con_info.rc_addr[0][i-XBEE_ADDR_OFFSET] != t->trans_rx.payload[i] )
		       {   
			   	    rc_equal = FALSE;  //indecate addr is different
			   		if(1)// autopilot_check_rc_bind() )
			   		{  //AC attitude >30deg,record address
						for (i = XBEE_ADDR_OFFSET; i <= XBEE_ADDR_LEN; i++) 
						{
							xbee_con_info.rc_addr[0][i-XBEE_ADDR_OFFSET] = t->trans_rx.payload[i];
						}
						//settings_StoreSettings(1);  //save addr to flash
						xbee_con_info.rc_con_available = TRUE;
			   		}
					//else  give up	
					break;  //jump out the loop of check address
		       }
		    }
			if(rc_equal == TRUE)
			{  //old RC connect
				xbee_con_info.rc_con_available = TRUE;   
			}
			
		  }
		}

		//ubuntu gcs bind without serial code,only add fixed address
	   if(xbee_con_info.ppzcenter_con_available == FALSE)
		{
          xbee_con_info.ppzcenter_con_available = TRUE;
		  xbee_con_info.ppzcenter_num++;
		  for (i=0; i<XBEE_ADDR_LEN; i++)
		  {
		     xbee_con_info.ppzcenter_addr[0][i]=*(pprzcenter_addr+i);
		  }
		}
	   if( (xbee_con_info.gcs_con_available == TRUE) || 
	   	   (xbee_con_info.rc_con_available == TRUE)  ||
	   	   (xbee_con_info.ppzcenter_con_available == TRUE)    )			 
	    {
	      dl_msg_available = TRUE;
		}
		
	  #else
	    dl_msg_available = TRUE;
	  #endif //GCS_V1_OPTION
      break;
    default:
      return;
  }
}

#define XBeeCheckAndParse(_dev, _trans) xbee_check_and_parse(&(_dev).device, &(_trans))

static inline void xbee_check_and_parse(struct link_device *dev, struct xbee_transport *trans)
{
  if (dev->char_available(dev->periph)) 
  	{
    while (dev->char_available(dev->periph) && !trans->trans_rx.msg_received) 
	{
      parse_xbee(trans, dev->get_byte(dev->periph));
    }
    if (trans->trans_rx.msg_received) {
      xbee_parse_payload(trans);
      trans->trans_rx.msg_received = FALSE;
    }
  }
}

#ifdef GCS_V1_OPTION
#define XbeeSetRcConFalse()  { xbee_con_info.rc_con_available = FALSE; }

#define XbeeSetGcsConFalse()  { tm_create_timer(TIMER_XBEE_HEARTBEAT_MSG, (2000 MSECONDS), TIMER_PERIODIC,0); \
                                xbee_con_info.gcs_con_available = FALSE; }


#define XbeeSetFailBind()  { tm_create_timer(TIMER_XBEE_HEARTBEAT_MSG, (2000 MSECONDS), TIMER_PERIODIC,0); \
	                          xbee_con_info.gcs_con_available = FALSE; }

#define XbeeSetSuccessBind()  { tm_kill_timer(TIMER_XBEE_HEARTBEAT_MSG); }
#endif
#endif /* XBEE_H */
