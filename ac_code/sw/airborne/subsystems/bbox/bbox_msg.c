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

/**** System include files ****/
#include "../../../include/std.h"
/*---Public include files---------------------------------------------*/
#include "mcu.h"

#include "modules/system/timer_if.h"
#include "modules/system/timer_class.h"
#include "modules/system/timer_def.h"
#include "subsystems/datalink/can_transport.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"

/*---Private include files--------------------------------------------*/
#include "bbox_msg.h"   
#include "bbox_msg_if.h"   
#include "bbox.h"   
#include "bbox_if.h"   
#include "bbox_msg_def.h"

/*===VARIABLES========================================================*/
/*---Global-----------------------------------------------------------*/
/*****************for upgrade***********************************/
extern uint8_t bbox_upgrade_status;
void bbox_msg_request_upgrade(void)
{
	uint8_t arg[2];
	arg[0] = BBOX_UPGRADE_SERVIC;   //request upgrade  0x05
	arg[1] = BBOX_UPDATE_REQ_DATA;	//0x01
	bbox_can_msg_send(2, &arg[0]);
}

void bbox_msg_ready_status(void)
{
	uint8_t arg[2];
	arg[0] = BBOX_UPGRADE_SERVIC;   //request ready status?	0x05
	arg[1] = BBOX_UPDATE_ASK_READY_DATA;	//0x03
	bbox_can_msg_send(2, &arg[0]);
}

void bbox_msg_send_frame(uint8_t *pt_value, uint8_t length)
{
	uint8_t arg[150], i = 0;
	arg[0] = BBOX_UPGRADE_SERVIC;   //request upgrade
	arg[1] = BBOX_UPDATE_DATA;
	for (i = 0; i < length; i++)
	{
		arg[2 + i] = *((uint8_t *) pt_value + i);
	}
	bbox_can_msg_send(2 + length, arg);
}

void bbox_msg_update_over(uint8_t num, uint8_t *arg)
{
	bbox_can_msg_send(num, &arg[0]);
}

/*******************************************************************************
 **  FUNCTION      : bbox_msg_heart_beat
 **  DESCRIPTION   : void
 **  PARAMETERS    : void
 **  RETURN        : void
 *******************************************************************************/
void bbox_msg_heart_beat(void)
{
	uint8_t arg[4];
	arg[0] = BBOX_MANAGE_SERVICE;
	arg[1] = 0x01;
	arg[2] = BBOX_POSITIVE_RESULT;

	bbox_can_msg_send(3, &arg[0]);
}

/*******************************************************************************
 **  FUNCTION      : bbox_msg_log_start
 **  DESCRIPTION   : void
 **  PARAMETERS    : void
 **  RETURN        : void
 *******************************************************************************/
void bbox_msg_log_start(void)
{
	uint8_t arg[9];
	arg[0] = BBOX_LOG_DATA_SERVICE;
	arg[1] = 0x00;
	arg[2] = 0x00;	//start log.
	if (get_utc_year())
	{ /*below is UTC time, 8hours delay beijing time*/
		arg[3] = get_utc_year();	//year
		arg[4] = get_utc_month();	//month
		arg[5] = get_utc_day();	    //day
		arg[6] = get_utc_hour();	//hour
		arg[7] = get_utc_minute();	//minute
		arg[8] = get_utc_second();	//second
	}
	else
	{
		uint32_t ts = sys_time.nb_sec;
		/*default 160927+cup_time*/
		arg[3] = 16;	//year
		arg[4] = 9;		//month
		arg[5] = 27;	//day
		arg[6] = (uint8_t) (ts / 3600);	//hour
		arg[7] = (uint8_t) (ts % 3600 / 60);	//minute
		arg[8] = (uint8_t) (ts % 3600 % 60);	//second
	}

	bbox_can_msg_send(9, &arg[0]);
}

/*******************************************************************************
 **  FUNCTION      : bbox_msg_log_end
 **  DESCRIPTION   : void
 **  PARAMETERS    : void
 **  RETURN        : void
 *******************************************************************************/
void bbox_msg_log_end(void)
{
	uint8_t arg[4];
	arg[0] = BBOX_LOG_DATA_SERVICE;
	arg[1] = 0x00;
	arg[2] = 0x01;	//end log.

	bbox_can_msg_send(3, &arg[0]);
}

/*******************************************************************************
 **  FUNCTION      : bbox_write_file_fault
 **  DESCRIPTION   : void
 **  PARAMETERS    : void
 **  RETURN        : void
 *******************************************************************************/
void bbox_write_file_fault(char *buf, uint8_t len)
{
	uint8_t arg[256];
	uint8_t i;
	arg[0] = BBOX_FAULT_DATA_SERVICE;
	arg[1] = 0x01;
	for (i = 0; i < len; i++)
	{
		arg[2 + i] = *buf++;
	}

	bbox_can_msg_send(len + 2, &arg[0]);
}

/*******************************************************************************
 **  FUNCTION      : bbox_request_software_version
 **  DESCRIPTION   : void
 **  PARAMETERS    : void
 **  RETURN        : void
 *******************************************************************************/
void bbox_request_software_version(void)
{
	uint8_t arg[2];
	arg[0] = BBOX_CONTROL_COMMAND_SERVICE;
	arg[1] = 0x01;

	bbox_can_msg_send(2, &arg[0]);
}

/***********************************************************************
 *  Name        : bbox_msg_handle
 *  Description :
 *  Parameter   :
 *  Returns     :
 *  Note		   :
 ***********************************************************************/
void bbox_msg_handle(uint16_t can_id, uint8_t *frame, uint8_t len)
{
	if (can_id == CANID_BBOX)
	{
		bbox_info.con_flag = TRUE;
		tm_create_timer(TIMER_BBOX_HEART_BEAT_TIMEOUT, (4000 MSECONDS), TIMER_ONE_SHOT, 0);
		switch (frame[2])
		{
		case BBOX_MANAGE_SERVICE:

			if (frame[4] == BBOX_IS_ERROR)
			{
				bbox_info.status = BBOX_IS_ERROR;
			}
			else if (frame[4] == BBOX_IS_NORMAL)
			{
				bbox_info.status = BBOX_IS_NORMAL;
			}

			bbox_msg_heart_beat();
			if (bbox_info.first_con == FALSE)
			{
				bbox_info.first_con = TRUE;
#ifdef FAULT_OPTION
				mcu_write_file_fault();
#endif	/* FAULT_OPTION */
				bbox_msg_log_start();
			}
			if (bbox_info.start_log == FALSE)
			{
				bbox_msg_log_start();
			}
			if (!bbox_info.gps_time)
			{
				if (get_utc_year())
				{
					bbox_msg_log_start();
					bbox_info.gps_time = TRUE;
				}
			}
			break;
		case BBOX_LOG_DATA_SERVICE:
			if (frame[3] == 0x00)
			{
				if (frame[4] == BBOX_POSITIVE_RESULT)
				{
					if (frame[5] == 0x00)	//start log response ok.
					{
						bbox_info.start_log = TRUE;
					}
				}
			}
			break;

		case BBOX_CONTROL_COMMAND_SERVICE:
			if (frame[3] == 0x02)
			{
				if ((len - 4) < MAX_BBOX_SV_VERSION_LEN)
				{
					for (uint8_t i = 0; i < len; i++)
					{
						bbox_info.version[i] = frame[i + 4];
					}
					bbox_info.sv_len = len - 4;
					bbox_info.sv_update = TRUE;
				}
			}
		case BBOX_UPGRADE_SERVIC:
		{
			if (frame[3] == BBOX_UPDATE_REQ_RES_DATA)		//request upgrade 0x02
			{

				if (frame[4] == BBOX_POSITIVE_RESULT)		//return successful request to GCS
				{
					//delay 100ms
					uint8_t type = UPGRADE_TYPE_BBOX;
					uint8_t ug_state = UPGRADE_RES_OK;
					xbee_tx_header(XBEE_ACK, XBEE_ADDR_GCS);
					DOWNLINK_SEND_UPGRADE_RESPONSE(SecondChannel, SecondDevice, &type, &ug_state);
				}
				else if (frame[4] == BBOX_BAD_RESULT)		//send request to bbox again
				{
					bbox_msg_request_upgrade();
				}
			}
			else if (frame[3] == BBOX_UPDATE_ASK_READY_RES_DATA)		//request ready status	0x04
			{

				if (frame[4] == BBOX_POSITIVE_RESULT)		//return successful request to GCS
				{
					bbox_upgrade_status = TRUE;
					uint8_t type = UPGRADE_TYPE_BBOX;
					xbee_tx_header(XBEE_ACK, XBEE_ADDR_GCS);
					DOWNLINK_SEND_UPGRADE_STATUS(SecondChannel, SecondDevice, &type);
					break;
				}
				else if (frame[4] == BBOX_BAD_RESULT)		//send request to bbox again
				{
					bbox_msg_ready_status();
				}
			}
			else if (frame[3] == BBOX_UPDATE_RES_DATA)		//return frame ack  0x08
			{
				if (frame[4] == BBOX_POSITIVE_RESULT)
				{
					uint8_t type = UPGRADE_TYPE_BBOX;
					uint8_t ug_state = UPGRADE_RES_PASS;   //next frame
					xbee_tx_header(XBEE_ACK, XBEE_ADDR_GCS);
					DOWNLINK_SEND_REQUESET_FIRMWARE(SecondChannel, SecondDevice, &type, &ug_state);
				}
				else if (frame[4] == BBOX_BAD_RESULT)
				{
					uint8_t type = UPGRADE_TYPE_BBOX;
					uint8_t ug_state = UPGRADE_RES_FAIL;    //current frame
					xbee_tx_header(XBEE_ACK, XBEE_ADDR_GCS);
					DOWNLINK_SEND_REQUESET_FIRMWARE(SecondChannel, SecondDevice, &type, &ug_state);
				}
			}
			else if (frame[3] == BBOX_UPDATE_OVER_RES_DATA)		//update over 0x0a
			{
				if (frame[4] == BBOX_POSITIVE_RESULT)
				{
					uint8_t type = UPGRADE_TYPE_BBOX;
					uint8_t ug_state = UPGRADE_RES_OK;
					bbox_upgrade_status = FALSE;
					xbee_tx_header(XBEE_ACK, XBEE_ADDR_GCS);
					DOWNLINK_SEND_UPGRADE_RESULT(SecondChannel, SecondDevice, &type, &ug_state);
				}
				else if (frame[4] == BBOX_BAD_RESULT)
				{
					uint8_t type = UPGRADE_TYPE_BBOX;
					uint8_t ug_state = UPGRADE_RES_FAIL;
					xbee_tx_header(XBEE_ACK, XBEE_ADDR_GCS);
					DOWNLINK_SEND_UPGRADE_RESULT(SecondChannel, SecondDevice, &type, &ug_state);

				}
			}
			break;
		}
		default:
			break;
		}
	}
}

/***********************************************************************
 *  Name        : bbox_msg_handle
 *  Description :
 *  Parameter   :
 *  Returns     :
 ***********************************************************************/
void bbox_heart_beat_timeout_handler(void)
{
	bbox_info.con_flag = FALSE;
}

/*---Private----------------------------------------------------------*/
/***********************************************************************
 *  Name         : bbox_can_msg_send
 *  Description : send onde frame msg to output fifo
 *  Parameter  :
 *  Returns      :
 ***********************************************************************/
void bbox_can_msg_send(uint8_t nArgs, uint8_t const *pArg)
{
	if (bbox_info.con_flag == TRUE)
	{
		bbox_can_send_frame(&can_tp, nArgs, pArg);
	}
}

/***********************************************************************
 *  Name         : bbox_can_send_frame
 *  Description :  put one frame data to tx fifo.
 *  Parameter  :
 *  Returns      :
 ***********************************************************************/
void bbox_can_send_frame(struct can_transport *p, uint8_t nArgs, uint8_t const *pArg)
{
	uint8_t i;

	if (p->device.check_free_space(p, nArgs + CAN_FRAME_MSG_EXTRA_LEN + 1))
	{
		can_put_byte(p, nArgs + CAN_FRAME_MSG_EXTRA_LEN);	//first put frame total length to tx fifo.
		can_put_byte(p, CAN_VAL_STX);
		can_put_byte(p, CAN_VAL_FIX1);
		p->cs_tx = 0;
		can_put_byte(p, 0x00);	//MSB frame length set to 0.
		can_put_byte(p, (uint8_t) (nArgs + CAN_FRAME_EXTRA_LEN));
		can_put_byte(p, p->trans_tx.tx_seq++);
		can_put_byte(p, (uint8_t) (p->tx_canid >> 8));
		can_put_byte(p, (uint8_t) (p->tx_canid & 0xFF));

		uint32_t ts = get_sys_time_usec() / 100;
		can_put_byte(p, (uint8_t) (ts & 0xFF));
		can_put_byte(p, (uint8_t) (ts >> 8 & 0xFF));
		can_put_byte(p, (uint8_t) (ts >> 16 & 0xFF));
		can_put_byte(p, (uint8_t) (ts >> 24 & 0xFF));

		for (i = 0; i < nArgs; i++)
		{
			can_put_byte(p, pArg[i]);
		}

		can_put_byte(p, p->cs_tx);
		p->tx_frame_counter++;	//successfully put one frame data to tx fifo.
	}
	else
	{
		p->device.nb_ovrn++;
	}
}
/**************** END OF FILE *****************************************/

