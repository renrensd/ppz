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
#include "subsystems/datalink/uart_print.h"
#include "subsystems/datalink/transpta.h"
#include "subsystems/datalink/downlink.h"
#include "uplink_ac.h"

#include "modules/system/timer_if.h"
#include "modules/system/timer_class.h"
#include "modules/system/timer_def.h"

#ifdef PTA_RESET_GPIO
#include "mcu_periph/gpio.h"

#ifndef PTA_RESET_DISABLE
#define PTA_RESET_DISABLE gpio_set
#endif
#endif //PTA_RESET_GPIO

struct pta_transport pta_tp;
struct xbee_connect_info xbee_con_info;


#define AT_COMMAND_SEQUENCE "+++"
#define AT_SET_MY "ATMY"
#define AT_AP_MODE "ATAP1\r"
#define AT_WR "ATWR\r"
#define AT_AC "ATAC\r"
#define AT_EXIT "ATCN\r"

/** Transparent uart private protocol_a protocol implementation */

static void put_1byte(struct pta_transport *trans, struct link_device *dev, const uint8_t byte)
{
	dev->put_byte(dev->periph, byte ^ 0x51);
	trans->cs_tx ^= byte;
}

static void put_bytes(struct pta_transport *trans, struct link_device *dev,
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

static void put_named_byte(struct pta_transport *trans, struct link_device *dev,
													 enum TransportDataType type __attribute__((unused)), enum TransportDataFormat format __attribute__((unused)),
													 uint8_t byte, const char *name __attribute__((unused)))
{
	put_1byte(trans, dev, byte);
}

static uint8_t size_of(struct pta_transport *trans __attribute__((unused)), uint8_t len)
{
	return len + 11;
}

static void start_message(struct pta_transport *trans, struct link_device *dev, uint8_t payload_len)
{
	dev->nb_msgs++;
	trans->trans_tx.tx_seq++;
	dev->put_byte(dev->periph, PTA_VAL_STX);
	dev->put_byte(dev->periph, PTA_VAL_FIX1);
	dev->put_byte(dev->periph, PTA_VAL_FIX2);
	trans->cs_tx = 0;
	const uint8_t len = payload_len + PTA_FRAME_EXTRA_LEN;
	dev->put_byte(dev->periph, len ^ 0x39);
	trans->cs_tx ^= len;
	dev->put_byte(dev->periph, trans->trans_tx.tx_seq ^ 0x45);
	trans->cs_tx ^= trans->trans_tx.tx_seq;
	dev->put_byte( dev->periph, (uint8_t)(trans->sn >> 24) ^ 0x34);
	trans->cs_tx ^= (uint8_t)(trans->sn >> 24);
	dev->put_byte( dev->periph, (uint8_t)(trans->sn >> 16) ^ 0x34);
	trans->cs_tx ^= (uint8_t)(trans->sn >> 16);
	dev->put_byte( dev->periph, (uint8_t)(trans->sn >> 8) ^ 0x34);
	trans->cs_tx ^= (uint8_t)(trans->sn >> 8);
	dev->put_byte( dev->periph, (uint8_t)(trans->sn & 0xff) ^ 0x34);
	trans->cs_tx ^= (uint8_t)(trans->sn & 0xff);
	dev->put_byte( dev->periph, 0x00 ^ 0x34 );
	trans->cs_tx ^= 0x00;
}

static void end_message(struct pta_transport *trans, struct link_device *dev)
{

	dev->put_byte(dev->periph, trans->cs_tx ^ 0x28);
	dev->send_message(dev->periph);
}

static void overrun(struct pta_transport *trans __attribute__((unused)),
										struct link_device *dev __attribute__((unused)))
{
	dev->nb_ovrn++;
}

static void count_bytes(struct pta_transport *trans __attribute__((unused)),
												struct link_device *dev __attribute__((unused)), uint8_t bytes)
{
	dev->nb_bytes += bytes;
}

static int check_available_space(struct pta_transport *trans __attribute__((unused)), struct link_device *dev,
																 uint8_t bytes)
{
	return dev->check_free_space(dev->periph, bytes);
}

static uint8_t pta_text_reply_is_ok(struct link_device *dev)
{
	char c[2];
	int count = 0;

	while (dev->char_available(dev->periph))
	{
		char cc = dev->get_byte(dev->periph);
		if (count < 2)
		{
			c[count] = cc;
		}
		count++;
	}

	if ((count > 2) && (c[0] == 'O') && (c[1] == 'K'))
	{
		return TRUE;
	}

	return FALSE;
}

static uint8_t pta_try_to_enter_api(struct link_device *dev)
{
	/** Switching to AT mode (FIXME: busy waiting) */
	print_string(dev, AT_COMMAND_SEQUENCE);

	/** - busy wait 1.25s */
	sys_time_usleep(1250000);

	return pta_text_reply_is_ok(dev);
}

void pta_set_tx_frame_addr(uint8_t ack __attribute__((unused)), uint8_t addr)
{
	if(addr == COMM_ADDR_AC)
	{
		pta_tp.sn = pta_tp.ac_sn;
	}
	else if(addr == COMM_ADDR_GCS)
	{
#ifdef AC_MASTER_OPTION
		pta_tp.sn = pta_tp.gcs_sn;
#else
		pta_tp.sn = pta_tp.ac_sn;
#endif	/* AC_MASTER_OPTION	*/
	}
	else if(addr == COMM_ADDR_PC)  //pc pprz center
	{
		pta_tp.sn = pta_tp.pc_sn;
	}
	else if(addr == COMM_ADDR_RC)
	{
		pta_tp.sn = pta_tp.rc_sn;
	}
}

void pta_init(void)
{
	pta_tp.status = PTA_ST_IDLE;
	pta_tp.trans_rx.msg_received = FALSE;
	pta_tp.trans_tx.size_of = (size_of_t) size_of;
	pta_tp.trans_tx.check_available_space = (check_available_space_t) check_available_space;
	pta_tp.trans_tx.put_bytes = (put_bytes_t) put_bytes;
	pta_tp.trans_tx.put_named_byte = (put_named_byte_t) put_named_byte;
	pta_tp.trans_tx.start_message = (start_message_t) start_message;
	pta_tp.trans_tx.end_message = (end_message_t) end_message;
	pta_tp.trans_tx.overrun = (overrun_t) overrun;
	pta_tp.trans_tx.count_bytes = (count_bytes_t) count_bytes;
	pta_tp.trans_tx.impl = (void *)(&pta_tp);

	pta_tp.trans_rx.rx_seq = 0;
	pta_tp.trans_rx.last_rx_seq = 0xff;

#ifdef PTA_RESET_GPIO
	gpio_setup_output(PTA_RESET_GPIO);
	PTA_RESET_DISABLE(PTA_RESET_GPIO);
#endif //PTA_RESET_GPIO

	struct link_device *dev = &((PTA_UART).device);

	// Empty buffer before init process
	while (dev->char_available(dev->periph))
	{
		dev->get_byte(dev->periph);
	}

	xbee_con_info.rc_con_available = FALSE;
	xbee_con_info.gcs_con_available = FALSE;
	xbee_con_info.ppzcenter_con_available = TRUE;
	//TODOM: init for debug,should get value from factory fram.
	pta_tp.pc_sn = (COMM_SERIAL_NUM<<4) | COMM_SN_PC;
	pta_tp.ac_sn = (COMM_SERIAL_NUM<<4) | COMM_SN_AC;
	pta_tp.gcs_sn = (COMM_SERIAL_NUM<<4) | COMM_SN_GCS;
	pta_tp.rc_sn = (COMM_SERIAL_NUM<<4) | COMM_SN_RC;
}

uint8_t pta_test = 0;
uint16_t pta_val1 = 0x3421,pta_val2 = 0x5642;
void pta_periodic(void)
{
	tm_stimulate(TIMER_TASK_TELEMETRY);
	if(pta_test ==1)
	{
		pta_test = 0;
		DOWNLINK_SEND_TAKEOFF(pta_tp, uart2, &pta_val1, &pta_val2);
	}
	else if(pta_test ==2)
	{
		pta_test = 0;
		DOWNLINK_SEND_TAKEOFF(pprz_tp, uart2, &pta_val1, &pta_val1);
	}
}

