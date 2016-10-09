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

#ifndef __CAN_H__
#define __CAN_H__

//#include "modules/system/types.h"

#include "subsystems/datalink/datalink.h"
#include "generated/airframe.h"
#include "subsystems/datalink/transport.h"

//#include "subsystems/datalink/downlink.h"

#include <stdio.h>
#include <string.h>

#include "firmwares/rotorcraft/autopilot.h"
#include "subsystems/settings.h"

/** Status of the API packet receiver automata */
enum CAN_RX_STATUS_PARAM
{
	CAN_RX_IDLE = 0x00,
	CAN_RX_START,
	CAN_RX_FIX1,
	CAN_RX_LEN_H,
	CAN_RX_LEN_L,
	CAN_RX_FS,
	CAN_RX_ID_H,
	CAN_RX_ID_L,
	CAN_RX_TIME_LSB,
	CAN_RX_TIME2,
	CAN_RX_TIME3,
	CAN_RX_TIME_MSB,
	CAN_RX_DATA,
	CAN_RX_CS,
};

enum CAN_RX_FRAME_PARAM
{
	CAN_RX_FRAME_IDLE = 0x00,
	CAN_RX_FRAME_HANDLING,
};

enum CAN_TX_FRAME_PARAM
{
	CAN_TX_IDLE = 0x00,
	CAN_TX_SENDING,
	CAN_TX_END,
};

#define CAN_VAL_STX 				0xFF
#define CAN_VAL_FIX1 				0xA5
#define CAN_FRAME_EXTRA_LEN			0x07
#define CAN_FRAME_MSG_EXTRA_LEN		0x0C

#define CAN_RX_BUFFER_SIZE			512
#define CAN_TX_BUFFER_SIZE			512

#define CAN_FRAME_SIZE				64

#define CANID_BBOX					0x0011
#define CANID_AC					0x0010


struct can_transport 
{
	// generic reception interface,can transport variables
  	uint8_t rx_status;
	uint8_t rx_frame_status;
  	uint16_t payload_idx;
 	uint8_t cs_rx;
  	volatile uint16_t len;
  	uint16_t rx_canid;
	volatile bool_t msg_received;           ///< message received flag
  	uint8_t rx_seq;	//rx frame sequence
 	uint8_t last_rx_seq;
	uint32_t rx_ts;
 	uint8_t rx_buf[CAN_RX_BUFFER_SIZE+5];
	uint8_t rx_data[CAN_FRAME_SIZE+5];
  	uint16_t rx_insert_idx;
	uint16_t rx_last_frame_insert_idx;
  	uint16_t rx_extract_idx;
	uint32_t rx_frame_counter;
	uint32_t rx_frame_handled_counter;
	uint16_t rx_cur_frame_len;
	 
 	 // generic transmission interface
 	uint8_t tx_status;
  	struct transport_tx trans_tx;
  	uint8_t cs_tx;
	uint16_t tx_canid;
  	uint8_t tx_buf[CAN_TX_BUFFER_SIZE+5];
  	uint16_t tx_insert_idx;
  	uint16_t tx_extract_idx;
	uint16_t tx_frame_len;
	uint16_t tx_cur_frame_len;
	uint16_t tx_rem_len;
	uint32_t tx_frame_counter;
	uint32_t tx_frame_handled_counter;
	
  	volatile uint16_t ore;    ///< overrun or error counter
  	/** Generic device interface */
 	 struct link_device device;
	
};

void can_msg_handle(struct can_transport *p, uint8_t len);
void can_read_polling(struct can_transport *p);
void can_send_polling(struct can_transport *p);


extern struct can_transport can_tp;

extern void can_transport_init(struct can_transport *p);
extern void can_drv_rx_callback(uint32_t id, uint8_t *buf, int len);
extern void can_drv_tx_callback(void);

extern void bbox_send_polling(void);
extern void bbox_read_polling(void);
extern void bbox_can_init(void);
extern void can_put_byte(struct can_transport *p, uint8_t byte);



#endif /*__ CAN_H__ */

