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
 * @file subsystems/datalink/xbee.c
 * Maxstream XBee serial input and output
 */

#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/uart_print.h"
#include "subsystems/datalink/xbee.h"
#include "subsystems/datalink/downlink.h"
#include "uplink_ac.h"
#include "modules/system/timer_if.h"
#include "modules/system/timer_class.h"
#include "modules/system/timer_def.h"

#ifdef XBEE_RESET_GPIO
#include "mcu_periph/gpio.h"

#ifndef XBEE_RESET_DISABLE
#define XBEE_RESET_DISABLE gpio_set
#endif
#endif //XBEE_RESET_GPIO

/** Ground station address */
#define GROUND_STATION_ADDR 0x3b3f
/** Aircraft address */
#define XBEE_MY_ADDR AC_ID

/** Constants for the API protocol */
#define TX_OPTIONS 0x00
#define NO_FRAME_ID 0x00
#define XBEE_API_OVERHEAD 5 /* start + len_msb + len_lsb + API_id + checksum */

struct xbee_transport xbee_tp;

#ifdef GCS_V1_OPTION
struct xbee_connect_info xbee_con_info;
#endif//GCS_V1_OPTION

#define AT_COMMAND_SEQUENCE "+++"
#define AT_SET_MY "ATMY"
#define AT_AP_MODE "ATAP1\r"
#define AT_WR "ATWR\r"
#define AT_AC "ATAC\r"
#define AT_EXIT "ATCN\r"

#ifdef GCS_V1_OPTION
uint8_t txAddr_bc[10] = {0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff};
uint8_t pprzcenter_addr[8] = PPZCENTER_ADDR;
#endif	//GCS_V1_OPTION


static inline void xbee_parse_payload(struct xbee_transport *t);


/** Xbee protocol implementation */

static void put_1byte(struct xbee_transport *trans, struct link_device *dev, const uint8_t byte)
{
#ifdef GCS_V1_OPTION
  if( (xbee_con_info.gcs_con_available == TRUE) || (xbee_con_info.rc_con_available == TRUE) 
  	  || (xbee_tp.trans_tx.tx_addr_type == XBEE_ADDR_BC)||(xbee_tp.trans_tx.tx_addr_type == XBEE_ADDR_PC) )
#endif //GCS_V1_OPTION
  {
	  trans->cs_tx += byte;
	  dev->put_byte(dev->periph, byte);
  }
}

static void put_bytes(struct xbee_transport *trans, struct link_device *dev,
                      enum TransportDataType type __attribute__((unused)), enum TransportDataFormat format __attribute__((unused)),
                      uint8_t len, const void *bytes)
{
  const uint8_t *b = (const uint8_t *) bytes;
  int i;
  for (i = 0; i < len; i++) {
    put_1byte(trans, dev, b[i]);
  }
}

static void put_named_byte(struct xbee_transport *trans, struct link_device *dev,
                           enum TransportDataType type __attribute__((unused)), enum TransportDataFormat format __attribute__((unused)),
                           uint8_t byte, const char *name __attribute__((unused)))
{
  put_1byte(trans, dev, byte);
}

static uint8_t size_of(struct xbee_transport *trans __attribute__((unused)), uint8_t len)
{ 
  // message length: payload + API overhead + XBEE TX overhead (868 or 2.4)
#ifdef GCS_V1_OPTION
  if( (xbee_con_info.gcs_con_available == TRUE) || (xbee_con_info.rc_con_available == TRUE) 
  	  || (xbee_tp.trans_tx.tx_addr_type == XBEE_ADDR_BC)||(xbee_tp.trans_tx.tx_addr_type == XBEE_ADDR_PC) )
  return len + XBEE_API_OVERHEAD + XBEE_TX_OVERHEAD;
  else return 0;
#else
  return len + XBEE_API_OVERHEAD + XBEE_TX_OVERHEAD;
#endif 
  
}

void xbee_tx_frame_header(uint8_t ack, uint8_t addr)
{
#ifdef GCS_V1_OPTION
    xbee_tp.trans_tx.tx_addr_type = addr;

	if(addr == XBEE_ADDR_BC)
	{
		xbee_set_tx_header(0x00,&txAddr_bc[0],0x00);
	}
	else if(addr == XBEE_ADDR_GCS)
	{
		xbee_set_tx_header(ack,&xbee_con_info.gcs_addr[0][0],0x00);
	}
	else if(addr == XBEE_ADDR_PC)  //pprzcenter
	{
		//xbee_set_tx_header(ack,&(xbee_con_info.ppzcenter_addr[0][0]),0x00);
		xbee_set_tx_header(ack, pprzcenter_addr, 0x00);
	}
	else if(addr == XBEE_ADDR_RC)
	{
		xbee_set_tx_header(ack,&xbee_con_info.rc_addr[0][0],0x00);
	}
#endif
}

#ifdef GCS_V1_OPTION

void xbee_set_tx_header(uint8_t seq, uint8_t *addr, uint8_t option)
{
	xbee_tp.trans_tx.tx_header[0] = XBEE_TX_ID;
	if(seq > XBEE_NACK)
	{
		xbee_tp.trans_tx.tx_seq++;
		xbee_tp.trans_tx.tx_header[1] = xbee_tp.trans_tx.tx_seq;
	}
	else if(seq == XBEE_NACK)
	{
		xbee_tp.trans_tx.tx_header[1] = 0x00;
	}
	
	if(xbee_tp.trans_tx.tx_seq == 0xff)
	{
		xbee_tp.trans_tx.tx_seq = 0x00;
	}

	xbee_tp.trans_tx.tx_header[2] = addr[0];
	xbee_tp.trans_tx.tx_header[3] = addr[1];
	xbee_tp.trans_tx.tx_header[4] = addr[2];
	xbee_tp.trans_tx.tx_header[5] = addr[3];
	xbee_tp.trans_tx.tx_header[6] = addr[4];
	xbee_tp.trans_tx.tx_header[7] = addr[5];
	xbee_tp.trans_tx.tx_header[8] = addr[6];
	xbee_tp.trans_tx.tx_header[9] = addr[7];

	xbee_tp.trans_tx.tx_header[10] = 0xff;
	xbee_tp.trans_tx.tx_header[11] = 0xfe;
	xbee_tp.trans_tx.tx_header[12] = 0x00;
	xbee_tp.trans_tx.tx_header[13] = option;	
}
#endif	//GCS_V1_OPTION

static void start_message(struct xbee_transport *trans, struct link_device *dev, uint8_t payload_len)
{
	#ifdef GCS_V1_OPTION
    if( (xbee_con_info.gcs_con_available == TRUE) || (xbee_con_info.rc_con_available == TRUE) 
  	     ||(xbee_tp.trans_tx.tx_addr_type == XBEE_ADDR_BC)||(xbee_tp.trans_tx.tx_addr_type == XBEE_ADDR_PC) )
	#endif //GCS_V1_OPTION
	{
	  dev->nb_msgs++;
	  dev->put_byte(dev->periph, XBEE_START);
	  const uint16_t len = payload_len + XBEE_TX_OVERHEAD + 1;
	  dev->put_byte(dev->periph, (len >> 8));
	  dev->put_byte(dev->periph, (len & 0xff));
	  trans->cs_tx = 0;
	  #ifdef GCS_V1_OPTION
	  put_bytes(trans, dev, DL_TYPE_UINT8, DL_FORMAT_SCALAR, XBEE_TX_OVERHEAD + 1, trans->trans_tx.tx_header);
	  #else
	  const uint8_t header[] = XBEE_TX_HEADER;
	  put_bytes(trans, dev, DL_TYPE_UINT8, DL_FORMAT_SCALAR, XBEE_TX_OVERHEAD + 1, header);
	  #endif //GCS_V1_OPTION
	}
}

static void end_message(struct xbee_transport *trans, struct link_device *dev)
{
  #ifdef GCS_V1_OPTION
  if( (xbee_con_info.gcs_con_available == TRUE) || (xbee_con_info.rc_con_available == TRUE) 
  	  || (xbee_tp.trans_tx.tx_addr_type == XBEE_ADDR_BC)||(xbee_tp.trans_tx.tx_addr_type == XBEE_ADDR_PC) )
  #endif //GCS_V1_OPTION
  {
	  trans->cs_tx = 0xff - trans->cs_tx;
	  dev->put_byte(dev->periph, trans->cs_tx);
	  dev->send_message(dev->periph);
  }
}

static void overrun(struct xbee_transport *trans __attribute__((unused)),
                    struct link_device *dev __attribute__((unused)))
{
  dev->nb_ovrn++;
}

static void count_bytes(struct xbee_transport *trans __attribute__((unused)),
                        struct link_device *dev __attribute__((unused)), uint8_t bytes)
{
  dev->nb_bytes += bytes;
}

static int check_available_space(struct xbee_transport *trans __attribute__((unused)), struct link_device *dev,
                                 uint8_t bytes)
{
  return dev->check_free_space(dev->periph, bytes);
}

static uint8_t xbee_text_reply_is_ok(struct link_device *dev)
{
  char c[2];
  int count = 0;

  while (dev->char_available(dev->periph)) {
    char cc = dev->get_byte(dev->periph);
    if (count < 2) {
      c[count] = cc;
    }
    count++;
  }

  if ((count > 2) && (c[0] == 'O') && (c[1] == 'K')) {
    return TRUE;
  }

  return FALSE;
}

static uint8_t xbee_try_to_enter_api(struct link_device *dev)
{

  /** Switching to AT mode (FIXME: busy waiting) */
  print_string(dev, AT_COMMAND_SEQUENCE);

  /** - busy wait 1.25s */
  sys_time_usleep(1250000);

  return xbee_text_reply_is_ok(dev);
}


#if XBEE_BAUD == B9600
#define XBEE_BAUD_ALTERNATE B115200
#define XBEE_ATBD_CODE "ATBD3\rATWR\r"
#pragma message "Experimental: XBEE-API@9k6 auto-baudrate 57k6 -> 9k6 (stop ground link for correct operation)"
#elif XBEE_BAUD == B115200
#define XBEE_BAUD_ALTERNATE B9600
#define XBEE_ATBD_CODE "ATBD6\rATWR\r"
#pragma message "Experimental: XBEE-API@57k6 auto-baudrate 9k6 -> 57k6 (stop ground link for correct operation)"
#else
#warning XBEE-API Non default baudrate: auto-baud disabled
#endif

void xbee_init(void)
{
  xbee_tp.status = XBEE_UNINIT;
  xbee_tp.trans_rx.msg_received = FALSE;
  xbee_tp.trans_tx.size_of = (size_of_t) size_of;
  xbee_tp.trans_tx.check_available_space = (check_available_space_t) check_available_space;
  xbee_tp.trans_tx.put_bytes = (put_bytes_t) put_bytes;
  xbee_tp.trans_tx.put_named_byte = (put_named_byte_t) put_named_byte;
  xbee_tp.trans_tx.start_message = (start_message_t) start_message;
  xbee_tp.trans_tx.end_message = (end_message_t) end_message;
  xbee_tp.trans_tx.overrun = (overrun_t) overrun;
  xbee_tp.trans_tx.count_bytes = (count_bytes_t) count_bytes;
  xbee_tp.trans_tx.impl = (void *)(&xbee_tp);

  #ifdef GCS_V1_OPTION
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_BC);
  #endif

  #ifdef XBEE_RESET_GPIO
  gpio_setup_output(XBEE_RESET_GPIO);
  XBEE_RESET_DISABLE(XBEE_RESET_GPIO);
  #endif //XBEE_RESET_GPIO

  struct link_device *dev = &((XBEE_UART).device);

  // Empty buffer before init process
  while (dev->char_available(dev->periph)) {
    dev->get_byte(dev->periph);
  }

#ifndef NO_XBEE_API_INIT
  /** - busy wait 1.25s */
  sys_time_usleep(1250000);

  if (! xbee_try_to_enter_api(dev)) {
#ifdef XBEE_BAUD_ALTERNATE

    // Badly configured... try the alternate baudrate:
    uart_periph_set_baudrate(&(XBEE_UART), XBEE_BAUD_ALTERNATE); // FIXME add set_baudrate to generic device, assuming uart for now
    if (xbee_try_to_enter_api(dev)) {
      // The alternate baudrate worked,
      print_string(dev, XBEE_ATBD_CODE);
    } else {
      // Complete failure, none of the 2 baudrates result in any reply
      // TODO: set LED?

      // Set the default baudrate, just in case everything is right
      uart_periph_set_baudrate(&(XBEE_UART), XBEE_BAUD); // FIXME add set_baudrate to generic device, assuming uart for now
      print_string(dev, "\r");
    }

#endif
    // Continue changing settings until the EXIT is issued.
  }

  /** Setting my address */
  #if 0
  print_string(dev, AT_SET_MY);
  uint16_t addr = XBEE_MY_ADDR;
  print_hex16(dev, addr);
  print_string(dev, "\r");
  #endif

  print_string(dev, AT_AP_MODE);
#ifdef XBEE_INIT
  print_string(dev, XBEE_INIT);
#endif

  /** Switching back to normal mode */
  print_string(dev, AT_EXIT);

  uart_periph_set_baudrate(&(XBEE_UART), XBEE_BAUD); // FIXME add set_baudrate to generic device, assuming uart for now

#endif

#ifdef GCS_V1_OPTION
  xbee_con_info.rc_con_available = TRUE;   //set TRUE only for heartbeat communication
  uint8_t rc_add[]= {0x00, 0x13, 0xA2, 0x00, 0x40, 0xF1, 0xEB, 0x1B};
  for (uint8_t i = 0; i < 8; i++) 
  {
	  xbee_con_info.rc_addr[0][i] = rc_add[i];
  }


  

  xbee_con_info.gcs_con_available = FALSE;
  xbee_con_info.ppzcenter_con_available = FALSE;
/*stop register timer call back,use timer.c*/  //xbee_bc_tid = sys_time_register_timer(1./XBEE_BC_PERIODIC_FREQUENCY, (sys_time_cb)xbee_msg_aircraft_ready_broadcast);
  tm_create_timer(TIMER_XBEE_HEARTBEAT_MSG, (2000 MSECONDS), TIMER_PERIODIC,0);
#endif //GCS_V1_OPTION
}

void xbee_periodic(void)
{
#ifdef GCS_V1_OPTION
 tm_stimulate(TIMER_TASK_TELEMETRY);
#endif
}

#ifdef GCS_V1_OPTION
void xbee_msg_aircraft_ready_broadcast(void)  //call by xbee_init,priodic send until binded
{
	const char serialcode[] = A2G_SERIAL_CODE;
	const char ac_sn[] = AC_SN_CODE;
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_BC);
	DOWNLINK_SEND_AIRCRAFT_BIND_STATE(DefaultChannel, DefaultDevice, serialcode, ac_sn);
}
#endif

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

void xbee_check_and_parse(struct link_device *dev, struct xbee_transport *trans)
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

void XbeeSetRcConFalse(void)
{
	xbee_con_info.rc_con_available = FALSE;
}

void XbeeSetGcsConFalse(void)  
{ 
	tm_create_timer(TIMER_XBEE_HEARTBEAT_MSG, (2000 MSECONDS), TIMER_PERIODIC,0);
    xbee_con_info.gcs_con_available = FALSE; 
}

void XbeeSetFailBind(void)  
{ 
	tm_create_timer(TIMER_XBEE_HEARTBEAT_MSG, (2000 MSECONDS), TIMER_PERIODIC,0);
	xbee_con_info.gcs_con_available = FALSE; 
}

void XbeeSetSuccessBind(void)  
{ 
	tm_kill_timer(TIMER_XBEE_HEARTBEAT_MSG); 
}


