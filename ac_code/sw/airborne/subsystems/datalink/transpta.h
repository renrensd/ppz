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

#ifndef PTA_H
#define PTA_H

//#include "modules/system/types.h"

#include "subsystems/datalink/datalink.h"
#include "generated/airframe.h"
#include "subsystems/datalink/transport.h"

//#include "subsystems/datalink/downlink.h"

#include <stdio.h>
#include <string.h>

#include "firmwares/rotorcraft/autopilot.h"
#include "subsystems/settings.h"
#include "subsystems/datalink/transpta_msg_def.h"


/** Initialisation in API mode and setting of the local address
 * FIXME: busy wait */
void pta_init(void);
extern void pta_set_tx_frame_addr(uint8_t ack __attribute__((unused)), uint8_t addr);
extern void pta_periodic(void);


/** Status of the API packet receiver automata */
enum PTA_FRAME_PARAM
{
	/*transpta protocol */
	PTA_ST_IDLE = 0x00,
	PTA_ST_START,
	PTA_ST_FIX1,
	PTA_ST_FIX2,
	PTA_ST_LEN,
	PTA_ST_FS,
	PTA_ST_SN1,
	PTA_ST_SN2,
	PTA_ST_SN3,
	PTA_ST_SN4,
	PTA_ST_TYPE,
	PTA_ST_DATA,
	PTA_ST_CS,

	/*pprz transport protocol */
	PPRZ_ST_STX,
	PPRZ_ST_LENGTH,
	PPRZ_ST_PAYLOAD,
	PPRZ_ST_CRC1,

};

#define AC_MASTER_OPTION	//aircraft as master

#define PTA_VAL_STX 		0xFD
#define PTA_VAL_FIX1 		0x95
#define PTA_VAL_FIX2 		0x79

#define PPRZ_VAL_STX		0x99

#define PTA_FRAME_EXTRA_LEN		0x06

enum COMM_ADDR_SERIAL_NUM_PARAM
{
	COMM_SN_PC = 0,
	COMM_SN_AC,
	COMM_SN_GCS,
	COMM_SN_RC,
};

#define COMM_SERIAL_NUM	0x0000001

struct pta_transport
{
	// generic reception interface
	struct transport_rx trans_rx;
	// specific pta transport variables
	uint8_t status;
	uint8_t payload_idx;
	uint8_t cs_rx;
	uint8_t rssi;
	uint8_t ck_a_rx, ck_b_rx;
	// generic transmission interface
	struct transport_tx trans_tx;
	// specific pprz transport_tx variables
	uint8_t cs_tx;
	uint32_t ac_sn;	//p840 module serial number.
	uint32_t gcs_sn;
	uint32_t rc_sn;
	uint32_t pc_sn;
	uint32_t sn;
	uint8_t frame_type;
	uint8_t ck_a_tx, ck_b_tx;

};

struct xbee_connect_info
{
	bool_t gcs_con_available;
	bool_t rc_con_available;
	bool_t ppzcenter_con_available;
};
extern struct xbee_connect_info xbee_con_info;
extern struct pta_transport pta_tp;

/** Parsing a transparent uart private protocol frame */
static inline void parse_pta(struct pta_transport *t, uint8_t c)
{
	switch (t->status)
	{
	case PTA_ST_IDLE:
		if (c == PTA_VAL_STX)
		{
			t->status = PTA_ST_FIX1;
		}
		else if (c == PPRZ_VAL_STX)
		{
			t->status = PPRZ_ST_STX;
		}
		break;
	case PTA_ST_START:
		t->status = PTA_ST_IDLE;
		break;
	case PTA_ST_FIX1:
		if (c == PTA_VAL_FIX1)
		{
			t->status = PTA_ST_FIX2;
		}
		else if (c == PTA_VAL_STX)
		{
			t->status = PTA_ST_FIX1;
		}
		else
		{
			t->status = PTA_ST_IDLE;
		}
		break;

	case PTA_ST_FIX2:
		if (c == PTA_VAL_FIX2)
		{
			t->status = PTA_ST_LEN;
		}
		else if (c == PTA_VAL_STX)
		{
			t->status = PTA_ST_FIX1;
		}
		else
		{
			t->status = PTA_ST_IDLE;
		}
		break;
	case PTA_ST_LEN:
		if(c <= TRANSPORT_PAYLOAD_LEN)
		{
			t->trans_rx.payload_len = c ^ 0x39;
			t->cs_rx = t->trans_rx.payload_len;	//init the cs value.
			t->trans_rx.payload_len -= PTA_FRAME_EXTRA_LEN;
			t->status = PTA_ST_FS;
		}
		else
		{
			t->status = PTA_ST_IDLE;
		}
		break;
	case PTA_ST_FS:
		t->trans_rx.rx_seq = c ^ 0x45;
		t->cs_rx ^= t->trans_rx.rx_seq;
		t->status = PTA_ST_SN1;
		break;
	case PTA_ST_SN1:
		t->sn = (c << 24) ^ 0x34;
		t->cs_rx ^= c ^ 0x34;
		t->status = PTA_ST_SN2;
		break;
	case PTA_ST_SN2:
		t->sn |= (c << 16) ^ 0x34;
		t->cs_rx ^= c ^ 0x34;
		t->status = PTA_ST_SN3;
		break;
	case PTA_ST_SN3:
		t->sn |= (c << 8) ^ 0x34;
		t->cs_rx ^= c ^ 0x34;
		t->status = PTA_ST_SN4;
		break;
	case PTA_ST_SN4:
		t->sn |= c ^ 0x34;
		t->cs_rx ^= c ^ 0x34;
		t->status = PTA_ST_TYPE;
		break;
	case PTA_ST_TYPE:
		t->frame_type = c ^ 0x34;
		t->cs_rx ^= c ^ 0x34;
		t->payload_idx = 0;
		t->status = PTA_ST_DATA;
		break;
	case PTA_ST_DATA:
		t->trans_rx.payload[t->payload_idx++] = c ^ 0x51;
		t->cs_rx ^= c ^ 0x51;
		if(t->trans_rx.payload_len > 0)
		{
			if(--t->trans_rx.payload_len == 0)
			{
				t->status = PTA_ST_CS;
			}
		}
		else
		{
			t->status = PTA_ST_IDLE;
		}
		break;
	case PTA_ST_CS:
		if( (c^0x28) == t->cs_rx )
		{
			t->payload_idx = 0;
			if(t->trans_rx.last_rx_seq == t->trans_rx.rx_seq) /* is the last frame */
			{
				//TODOM:
			}
			else
			{
				t->trans_rx.last_rx_seq = t->trans_rx.rx_seq;
				t->trans_rx.msg_received = TRUE;
			}
		}
		else
		{
			t->trans_rx.error++;
		}
		t->status = PTA_ST_IDLE;
		t->payload_idx = 0;
		break;

		/* pprz transport protocol */
	case PPRZ_ST_STX:
		if(t->trans_rx.msg_received)
		{
			t->trans_rx.ovrn++;
			goto error;
		}
		t->trans_rx.payload_len = c - 4; /* Counting STX, LENGTH and CRC1 and CRC2 */
		t->ck_a_rx = t->ck_b_rx = c;
		t->status++;
		t->payload_idx = 0;
		break;
	case PPRZ_ST_LENGTH:
		t->trans_rx.payload[t->payload_idx] = c;
		t->ck_a_rx += c;
		t->ck_b_rx += t->ck_a_rx;
		t->payload_idx++;
		if (t->payload_idx == t->trans_rx.payload_len)
		{
			t->status++;
		}
		break;
	case PPRZ_ST_PAYLOAD:
		if (c != t->ck_a_rx)
		{
			goto error;
		}
		t->status++;
		break;
	case PPRZ_ST_CRC1:
		if (c != t->ck_b_rx)
		{
			goto error;
		}
		t->trans_rx.msg_received = TRUE;
		goto restart;

	default:
		t->status = PTA_ST_IDLE;
		t->payload_idx = 0;
		t->trans_rx.error++;
		break;
	}

	return;
error:
	t->trans_rx.error++;
restart:
	t->status = PTA_ST_IDLE;
	return;
}

/** Parsing a frame data and copy the payload to the datalink buffer */
static inline void pta_parse_payload(struct pta_transport *t)
{
	uint8_t i;
	for (i = 0; i < t->trans_rx.payload_len; i++)
	{
		dl_buffer[i] = t->trans_rx.payload[i];
	}
	dl_msg_available = TRUE;
}


#define PtaCheckAndParse(_dev, _trans) pta_check_and_parse(&(_dev).device, &(_trans))

static inline void pta_check_and_parse(struct link_device *dev, struct pta_transport *trans)
{
	if (dev->char_available(dev->periph))
	{
		while (dev->char_available(dev->periph) && !trans->trans_rx.msg_received)
		{
			parse_pta(trans, dev->get_byte(dev->periph));
		}
		if (trans->trans_rx.msg_received)
		{
			pta_parse_payload(trans);
			trans->trans_rx.msg_received = FALSE;
		}
	}
}

#define XbeeSetRcConFalse()  { xbee_con_info.rc_con_available = FALSE; }

#define XbeeSetGcsConFalse()  { xbee_con_info.gcs_con_available = FALSE; }

#define XbeeSetFailBind()  { xbee_con_info.gcs_con_available = FALSE; }

#define XbeeSetSuccessBind()  0

#endif /* PTA_H */

