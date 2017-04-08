/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : R&D SW      									   *
*   AUTHOR	   :             										   *
************************************************************************
* Object        : 
* Module        : 
* Instance      : 
* Description   : 
*-----------------------------------------------------------------------
* Version: 
* Date: 
* Author: 
***********************************************************************/
/*-History--------------------------------------------------------------
* Version       Date    Name    Changes and comments
* 
*=====================================================================*/

#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/can_transport.h"
#include "subsystems/datalink/downlink.h"
#include "can_drv.h"
#ifdef BBOX_OPTION
#include "subsystems/bbox/bbox_if.h"
#include "subsystems/bbox/bbox_msg_if.h"
#endif	/* BBOX_OPTION */


struct can_transport can_tp;
//uint32_t can_t1,can_t2,can_tt,can_tta[300];
//uint8_t can_tt_index = 0;

static void can_increment_buf(uint16_t *buf_idx, uint16_t len);
static void put_1byte(struct can_transport *trans, struct link_device *dev, const uint8_t byte);
static void put_bytes(struct can_transport *trans, struct link_device *dev,
                      enum TransportDataType type __attribute__((unused)), enum TransportDataFormat format __attribute__((unused)),
                      uint8_t len, const void *bytes);
static void put_named_byte(struct can_transport *trans, struct link_device *dev,
                           enum TransportDataType type __attribute__((unused)), enum TransportDataFormat format __attribute__((unused)),
                           uint8_t byte, const char *name __attribute__((unused)));
static uint8_t size_of(struct can_transport *trans, uint8_t len);
static void start_message(struct can_transport *trans, struct link_device *dev, uint8_t payload_len);
static void end_message(struct can_transport *trans, struct link_device *dev);
static void overrun(struct can_transport *trans __attribute__((unused)),
                    struct link_device *dev __attribute__((unused)));
static void count_bytes(struct can_transport *trans __attribute__((unused)),
                        struct link_device *dev __attribute__((unused)), uint8_t bytes);
static int check_available_space(struct can_transport *trans __attribute__((unused)), struct link_device *dev, uint8_t bytes);
static bool_t can_check_free_space(struct can_transport *p, uint8_t len);
static uint8_t can_getch(struct can_transport *p);
static uint16_t can_char_available(struct can_transport *p);
static void can_send_message(struct can_transport *p);
static void can_transport_tx_proc(struct can_transport *p);
static void can_transport_rx_proc(struct can_transport *p,  uint32_t id, uint8_t *buf, int len);
static bool_t can_has_frame_send(struct can_transport *p);
static bool_t can_has_frame_received(struct can_transport *p);
static void can_transport_parse(struct can_transport *t, uint8_t c);
static bool_t can_is_rx_frame_idle(struct can_transport *p);
static void can_transport_send_msg(struct can_transport *p, uint8_t len);



/** Transparent can private protocol_a protocol implementation */

static void put_1byte(struct can_transport *trans, struct link_device *dev, const uint8_t byte)
{
	if(bbox_info.start_log == TRUE)
  	{
	  	dev->put_byte(dev->periph, byte);
  	}
}

static void put_bytes(struct can_transport *trans, struct link_device *dev,
                      enum TransportDataType type __attribute__((unused)), enum TransportDataFormat format __attribute__((unused)),
                      uint8_t len, const void *bytes)
{
  	const uint8_t *b = (const uint8_t *) bytes;
  	int i;
  	for (i = 0; i < len; i++) 
  	{
    	put_1byte(trans, dev, b[i]);
  	}
}

static void put_named_byte(struct can_transport *trans, struct link_device *dev,
                           enum TransportDataType type __attribute__((unused)), enum TransportDataFormat format __attribute__((unused)),
                           uint8_t byte, const char *name __attribute__((unused)))
{
  	put_1byte(trans, dev, byte);
}

static uint8_t size_of(struct can_transport *trans, uint8_t len)
{ 
  	if(bbox_info.start_log == TRUE)
  	{
		trans->tx_frame_len = len + 12;
		return (len + 12);
  	}
	else
	{
		return 0;
	}
}

static void start_message(struct can_transport *trans, struct link_device *dev, uint8_t payload_len)
{
   	if(bbox_info.start_log == TRUE)
	{
		//can_t1 = get_sys_time_usec();
		dev->nb_msgs++;
		dev->put_byte(dev->periph, (uint8_t)trans->tx_frame_len);
		dev->put_byte(dev->periph, CAN_VAL_STX);
		dev->put_byte(dev->periph, CAN_VAL_FIX1);
		trans->cs_tx = 0;
		const uint16_t len = payload_len + CAN_FRAME_EXTRA_LEN;
		dev->put_byte(dev->periph, (uint8_t)(len>>8) );
		dev->put_byte(dev->periph, (uint8_t)(len&0xFF) );
		dev->put_byte(dev->periph, trans->trans_tx.tx_seq++);
		dev->put_byte( dev->periph, (uint8_t)(trans->tx_canid >> 8) );
		dev->put_byte( dev->periph, (uint8_t)(trans->tx_canid & 0xFF) );
		uint32_t ts = get_sys_time_usec() / 100;
  		put_bytes(trans, dev, DL_TYPE_TIMESTAMP, DL_FORMAT_SCALAR, 4, (uint8_t *)(&ts));
	}
}

static void end_message(struct can_transport *trans, struct link_device *dev)
{
  	if(bbox_info.start_log == TRUE)
  	{
		//if( can_check_free_space(p, 1) )
		{
			dev->put_byte(dev->periph, trans->cs_tx);
			trans->tx_frame_counter++;
		}
  	}
}

static void overrun(struct can_transport *trans __attribute__((unused)),
                    struct link_device *dev __attribute__((unused)))
{
	dev->nb_ovrn++;
}

static void count_bytes(struct can_transport *trans __attribute__((unused)),
                        struct link_device *dev __attribute__((unused)), uint8_t bytes)
{
  	dev->nb_bytes += bytes;
}

uint8_t bbox_upgrade_status=FALSE;
static int check_available_space(struct can_transport *trans __attribute__((unused)), struct link_device *dev, uint8_t bytes)
{
	if(bbox_upgrade_status==FALSE)
	{
		return dev->check_free_space(dev->periph, bytes);
	}
	else if(bbox_upgrade_status==TRUE)
	{
		return FALSE;
	}
	
}

static bool_t can_check_free_space(struct can_transport *p, uint8_t len)
{
  	int16_t space = p->tx_extract_idx - p->tx_insert_idx;
  	if (space <= 0) 
  	{
    	space += CAN_TX_BUFFER_SIZE;
  	}
  	return (uint16_t)(space - 2) >= len;
}

static uint8_t can_getch(struct can_transport *p)
{
  	uint8_t ret = p->rx_buf[p->rx_extract_idx];
  	p->rx_extract_idx = (p->rx_extract_idx + 1) % CAN_RX_BUFFER_SIZE;
  	return ret;
}

static uint16_t can_char_available(struct can_transport *p)
{
	int16_t available = p->rx_insert_idx - p->rx_extract_idx;
	if (available < 0) 
	{
	  available += CAN_RX_BUFFER_SIZE;
	}
	return (uint16_t)available;
}

/***********************************************************************
* FUNCTION    : can_increment_buf
* DESCRIPTION : safe increment of circular buffer
* INPUTS      : none
* RETURN      : none
***********************************************************************/
static void can_increment_buf(uint16_t *buf_idx, uint16_t len)
{
  	*buf_idx = (*buf_idx + len) % CAN_TX_BUFFER_SIZE;
}

/***********************************************************************
* FUNCTION    : can_put_byte
* DESCRIPTION : Add one byte to the end of tx circular buffer
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void can_put_byte(struct can_transport *p, uint8_t byte)
{
	if( can_check_free_space(p, 1) )
	{
		p->tx_buf[p->tx_insert_idx] = byte;
		p->cs_tx ^= byte;
	  	can_increment_buf(&p->tx_insert_idx, 1);
	}
	else
	{
		//p->device.nb_ovrn++;	//TODOM:
	}
}

/***********************************************************************
* FUNCTION    : can_transport_send_msg
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
static void can_transport_send_msg(struct can_transport *p, uint8_t len)
{
	uint8_t data[8];
	uint8_t i;

	for(i=0; i<len; i++)
	{
		p->tx_extract_idx %= CAN_TX_BUFFER_SIZE;
		data[i] = p->tx_buf[p->tx_extract_idx];
		can_increment_buf(&p->tx_extract_idx, 1);
	}
	
	can_drv_transmit(p->tx_canid, data, len);
}

static void can_send_message(struct can_transport *p)
{
	if(p->tx_cur_frame_len <= 8 )
	{
		can_transport_send_msg(p, p->tx_cur_frame_len);
		p->tx_rem_len = 0;
	}
	else
	{
		can_transport_send_msg(p, 8);
		p->tx_rem_len = p->tx_cur_frame_len - 8;
	}
}

static void can_transport_tx_proc(struct can_transport *p)
{
	if(p->tx_rem_len > 0)
	{
		 if(p->tx_rem_len <= 8)
		 {
			can_transport_send_msg(p, p->tx_rem_len);
			p->tx_rem_len = 0;

			//can_t2 = get_sys_time_usec();
			//can_tta[can_tt_index++] = can_t2 - can_t1;
		 }
		 else
		 {
			can_transport_send_msg(p, 8);
			p->tx_rem_len -= 8;
		 }
	}
	else
	{
		p->tx_status = CAN_TX_END;	//one frame data is sent succefully.
		p->tx_frame_handled_counter++;
		p->tx_status = CAN_TX_IDLE;
	}
}

static void can_transport_rx_proc(struct can_transport *p,  uint32_t id, uint8_t *buf, int len)
{
	uint8_t i;
	uint16_t temp;
	
	p->rx_canid = (uint16_t)id;
	for(i=0; i<len; i++)
	{
		can_transport_parse(p, buf[i]);
	}
}

void can_drv_tx_callback(void)
{
	can_transport_tx_proc(&can_tp);
}

void can_drv_rx_callback(uint32_t id, uint8_t *buf, int len)
{
	can_transport_rx_proc(&can_tp, id, buf, len);
}


/***********************************************************************
*  Name         : can_has_frame_send
*  Description : to confirm if has frame to be send. 
*  Parameter  : None
*  Returns      : BOOL
***********************************************************************/
static bool_t can_has_frame_send(struct can_transport *p)
{
    return (p->tx_frame_counter != p->tx_frame_handled_counter);
}

/***********************************************************************
*  Name         : can_has_frame_received
*  Description : to confirm if has frame to be received. 
*  Parameter  : None
*  Returns      : BOOL
***********************************************************************/
static bool_t can_has_frame_received(struct can_transport *p)
{
    return (p->rx_frame_counter != p->rx_frame_handled_counter);
}

/***********************************************************************
*  Name        : can_transport_parse
*  Description : Parsing a transparent uart private protocol frame. 
*  Parameter   : None
*  Returns     : 
***********************************************************************/
static void can_transport_parse(struct can_transport *t, uint8_t c)
{
  	switch (t->rx_status) 
  	{
    	case CAN_RX_IDLE:
	    if (c == CAN_VAL_STX) 
		{
	       	t->rx_status = CAN_RX_FIX1;
	    }
	    break;
	case CAN_RX_START:
	    t->rx_status = CAN_RX_IDLE;
	    break;
	case CAN_RX_FIX1:
	    if (c == CAN_VAL_FIX1) 
		{
	       	t->rx_status = CAN_RX_LEN_H;
	    }
		else if (c == CAN_VAL_STX) 
		{
	       	t->rx_status = CAN_RX_FIX1;
	    }
		else
		{
	       	t->rx_status = CAN_RX_IDLE;
	    }
	    break;
	case CAN_RX_LEN_H:
		t->len = c << 8;
		t->cs_rx = c;	//init the cs value.
		t->rx_last_frame_insert_idx = t->rx_insert_idx;
		t->rx_status = CAN_RX_LEN_L;
	    break;
	case CAN_RX_LEN_L:	
		t->len = c & 0xFF;	//TODOM: data length limit to 255.
		t->cs_rx ^= c;	//init the cs value.
		t->len -= CAN_FRAME_EXTRA_LEN;
		t->rx_status = CAN_RX_FS;

		t->rx_buf[t->rx_insert_idx] = t->len + 2;//canid_l and canid_h.
		can_increment_buf(&t->rx_insert_idx, 1);
	    break;
	case CAN_RX_FS:
		t->rx_seq = c;
		t->cs_rx ^= c;
		t->rx_status = CAN_RX_ID_H;
	    break;
	case CAN_RX_ID_H:
		t->rx_canid = c << 8;
		t->cs_rx ^= c;
		t->rx_status = CAN_RX_ID_L;
		t->rx_buf[t->rx_insert_idx] = c;
		can_increment_buf(&t->rx_insert_idx, 1);
	    break;
	case CAN_RX_ID_L:
		t->rx_canid |= c;
		t->cs_rx ^= c;
		t->rx_status = CAN_RX_TIME_LSB;
		t->rx_buf[t->rx_insert_idx] = c;
		can_increment_buf(&t->rx_insert_idx, 1);
	    break;

	case CAN_RX_TIME_LSB:
		t->rx_ts = c;
		t->cs_rx ^= c;
		t->rx_status = CAN_RX_TIME2;
	    break;
	case CAN_RX_TIME2:
		t->rx_ts |= c<<8;
		t->cs_rx ^= c;
		t->rx_status = CAN_RX_TIME3;
	    break;
	case CAN_RX_TIME3:
		t->rx_ts |= c<<16;
		t->cs_rx ^= c;
		t->rx_status = CAN_RX_TIME_MSB;
	    break;
	case CAN_RX_TIME_MSB:
		t->rx_ts |= c<<24;
		t->cs_rx ^= c;
		t->rx_status = CAN_RX_DATA;
      	break;
    case CAN_RX_DATA:
		t->rx_buf[t->rx_insert_idx] = c;
		can_increment_buf(&t->rx_insert_idx, 1);
		t->cs_rx ^= c;
		if(t->len > 0)
		{
			if(--t->len == 0)
			{
				t->rx_status = CAN_RX_CS;
			}
		}
		else
		{
			t->rx_status = CAN_RX_IDLE;
		}
		break;
	case CAN_RX_CS:
	  	if(c == t->cs_rx)
	  	{
			if(t->last_rx_seq == t->rx_seq) /* is the last frame */
			{
				//TODOM:
				t->rx_insert_idx = t->rx_last_frame_insert_idx;
			}
			else
			{
				t->last_rx_seq = t->rx_seq;
				t->rx_last_frame_insert_idx = t->rx_insert_idx;
				t->rx_frame_counter++;
				t->msg_received = TRUE;
			}
		}
		else
		{
			t->rx_insert_idx = t->rx_last_frame_insert_idx;
			t->ore++;
		}
	  	t->rx_status = CAN_RX_IDLE;
	  	break;
	default:
		t->rx_status = CAN_RX_IDLE;
		t->ore++;
	  	break;
  	}
}

void can_transport_init(struct can_transport *p)
{
	can_drv_init();

	p->rx_status = CAN_RX_IDLE;
	p->rx_frame_status = CAN_RX_FRAME_IDLE;
	p->tx_status = CAN_TX_IDLE;
	p->msg_received = FALSE;
	p->trans_tx.size_of = (size_of_t) size_of;
	p->trans_tx.check_available_space = (check_available_space_t) check_available_space;
	p->trans_tx.put_bytes = (put_bytes_t) put_bytes;
	p->trans_tx.put_named_byte = (put_named_byte_t) put_named_byte;
	p->trans_tx.start_message = (start_message_t) start_message;
	p->trans_tx.end_message = (end_message_t) end_message;
	p->trans_tx.overrun = (overrun_t) overrun;
	p->trans_tx.count_bytes = (count_bytes_t) count_bytes;
	p->trans_tx.impl = (void *)(p);
	p->tx_canid = CANID_AC;
	p->tx_frame_counter = 0;
	p->tx_frame_handled_counter = 0;
	p->rx_frame_counter = 0;
	p->rx_frame_handled_counter = 0;

	p->rx_insert_idx = 0;
	p->rx_extract_idx = 0;
	p->rx_seq = 0;
	p->last_rx_seq = 0xff;
	p->trans_tx.tx_seq = 0x01;
	p->tx_insert_idx = 0;
	p->tx_extract_idx = 0;
	p->ore = 0;
	p->device.periph = (void *)p;
	p->device.check_free_space = (check_free_space_t)can_check_free_space;
	p->device.put_byte = (put_byte_t)can_put_byte;
	p->device.send_message = (send_message_t)can_send_message;
	p->device.char_available = (char_available_t)can_char_available;
	p->device.get_byte = (get_byte_t)can_getch;

}

/***********************************************************************
*  Name         : can_is_tx_idle
*  Description : is tx idle ?
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static bool_t can_is_tx_idle(struct can_transport *p)
{
	return (CAN_TX_IDLE == p->tx_status);
}

/***********************************************************************
*  Name         : can_is_rx_frame_idle
*  Description : is tx idle ?
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static bool_t can_is_rx_frame_idle(struct can_transport *p)
{
	return (CAN_RX_FRAME_IDLE == p->rx_frame_status);
}

/***********************************************************************
*  Name         : can_send_polling
*  Description : send a frame to bbox periodically if has frame to be send.       
*  Parameter  : None
*  Returns      : None
***********************************************************************/
void can_send_polling(struct can_transport *p)
{
	if( can_is_tx_idle(p) & can_has_frame_send(p) )
	{
		p->tx_extract_idx %= CAN_TX_BUFFER_SIZE;
		p->tx_cur_frame_len = p->tx_buf[p->tx_extract_idx];
		can_increment_buf(&p->tx_extract_idx, 1);
		p->tx_status = CAN_TX_SENDING;
		can_send_message(p);
	}
}

/***********************************************************************
*  Name         : can_read_polling
*  Description : read a frame from bbox periodically if has frame to be received.       
*  Parameter  : None
*  Returns      : None
***********************************************************************/
void can_read_polling(struct can_transport *p)
{
	uint8_t i;

	if( can_is_rx_frame_idle(p) & can_has_frame_received(p) )
	{
		p->rx_frame_status = CAN_RX_FRAME_HANDLING;
		p->rx_cur_frame_len = p->rx_buf[p->rx_extract_idx];
		p->rx_cur_frame_len %= CAN_FRAME_SIZE;
		can_increment_buf(&p->rx_extract_idx, 1);

		for (i = 0; i < p->rx_cur_frame_len; i++) 
		{
	    	p->rx_data[i] = p->rx_buf[p->rx_extract_idx];
			can_increment_buf(&p->rx_extract_idx, 1);
	  	}

		can_msg_handle(p, (uint8_t)p->rx_cur_frame_len);
		p->rx_frame_handled_counter++;
		p->rx_frame_status = CAN_RX_FRAME_IDLE;
	}
}

#ifdef BBOX_OPTION
/***********************************************************************
*  Name         : bbox_send_polling
*  Description :        
*  Parameter  : None
*  Returns      : None
***********************************************************************/
void bbox_send_polling(void)
{
	can_send_polling(&can_tp);
}

/***********************************************************************
*  Name         : bbox_read_polling
*  Description :        
*  Parameter  : None
*  Returns      : None
***********************************************************************/
void bbox_read_polling(void)
{
	can_read_polling(&can_tp);
}

/***********************************************************************
*  Name         : bbox_can_init
*  Description :        
*  Parameter  : None
*  Returns      : None
***********************************************************************/
void bbox_can_init(void)
{
	can_transport_init(&can_tp);
}

/***********************************************************************
*  Name         : bbox_can_init
*  Description :        
*  Parameter  : None
*  Returns      : None
***********************************************************************/
void can_msg_handle(struct can_transport *p, uint8_t len)
{
	bbox_msg_handle(p->rx_canid, p->rx_data, len);
}

#endif	/* BBOX_OPTION */

