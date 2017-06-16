/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : R&D SW      									   *
*   AUTHOR	   :            										   *
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
#include "../../modules/system/types.h"

/*---Public include files---------------------------------------------*/
#include <libopencm3/stm32/usart.h>
#include "../../mcu_periph/uart.h"
#include "../../../include/std.h"
#include "../../modules/system/fifo_if.h"
#include "ops_app_if.h"
/*---Private include files--------------------------------------------*/
#include "uart_ops.h"
#include "uart_ops_if.h"
#include "ops_msg_if.h"
#include "ops_msg_uart_def.h"


/*===VARIABLES========================================================*/
#define opsDev  (OPS_DEVICE.device)
#define opsTransmit(c) opsDev.put_byte(opsDev.periph, c)
#define opsChAvailable() opsDev.char_available(opsDev.periph)
#define opsGetch() opsDev.get_byte(opsDev.periph)

/*---Global-----------------------------------------------------------*/

/*---Private----------------------------------------------------------*/
U8 uart_ops_rec_frame_counter;
U8 uart_ops_rec_frame_handled_counter;
U8 uart_ops_send_frame_counter;
U8 uart_ops_send_frame_handled_counter;

U8 uart_ops_lost_ack_counter;
U8 uart_ops_req_tx_timeout_counter;
U8 uart_ops_tx_timeout_counter;
U8 uart_ops_rx_timeout_counter;
U8 uart_ops_send_frame_interval_counter;

U8 uart_ops_in_buff_one[UART_OPS_INP_BUF_ONE_SIZE];
U8 uart_ops_in_buff_two[UART_OPS_INP_BUF_TWO_SIZE];
U8 uart_ops_in_buff_three[UART_OPS_INP_BUF_THREE_SIZE];
U8 uart_ops_in_buff_four[UART_OPS_INP_BUF_FOUR_SIZE];

U8 uart_ops_out_buff_one[UART_OPS_OUT_BUF_ONE_SIZE];
U8 uart_ops_out_buff_two[UART_OPS_OUT_BUF_TWO_SIZE];
U8 uart_ops_out_buff_three[UART_OPS_OUT_BUF_THREE_SIZE];
U8 uart_ops_out_buff_four[UART_OPS_OUT_BUF_FOUR_SIZE];

UART_OPS_FIFO_TYPE uart_ops_in_fifo[OPS_UART_MSG_PRIO_NUM],uart_ops_out_fifo[OPS_UART_MSG_PRIO_NUM];
UART_OPS_FRAME_TYPE uart_ops_in_frame,uart_ops_out_frame;

UART_OPS_RX_INFO_TYPE uart_ops_rx_info;
UART_OPS_TX_INFO_TYPE uart_ops_tx_info;

UART_OPS_TIMER_TYPE uart_ops_req_send_timer,uart_ops_enter_tx_timer,uart_ops_tx_timer,uart_ops_ack_timer,uart_ops_rx_timer;

/*===FUNCTIONS========================================================*/
/***********************************************************************
*  Name        : uart_ops_down
*  Description : kill eng uart comm.
*  Parameter   : void
*  Returns     : void
***********************************************************************/
void uart_ops_down(void)
{
	//Disable & kill ComPort here
	//UART_OPS_Disable();
}

/***********************************************************************
*  Name         : uart_ops_create
*  Description : creat spi navi
*  Parameter  : None
*  Returns      : None
***********************************************************************/
void uart_ops_create(void)
{
	uart_ops_config();
	uart_ops_init_var();
}

/***********************************************************************
*  Name         : uart_ops_send_frame
*  Description : send a frame to uart_ops_out_fifo[OPS_UART_MSG_PRIO_NUM]
*  Parameter  : U8 *frame,U8 length
*  Returns      : BOOL
***********************************************************************/
BOOL uart_ops_send_frame(U8 const *frame,U8 length)
{
	U8 ret_val = FALSE;
	U8 tx_frame_prio;
	/*put data into fifo bace on the priority of the data frame.*/
	tx_frame_prio = *(frame + OPS_CMD_PRIO);
	if(tx_frame_prio <= OPS_UART_MSG_PRIO_NUM)
	{
		if(fifo_input_frame(&uart_ops_out_fifo[tx_frame_prio - 1].fifo_info,(U8 *)frame,length))
		{
			uart_ops_send_frame_counter++;
			ret_val = TRUE;
		}
	}
	return ret_val;
}

/***********************************************************************
*  Name         : uart_ops_read_polling
*  Description : read from uart_ops_in_fifo periodically.
*  Parameter  : None
*  Returns      : None
***********************************************************************/
void uart_ops_read_polling(void)
{
	if(uart_ops_has_frame_rev())
	{
		U8 len;
		U8 frame[UART_OPS_FRAME_SIZE];
		if(uart_ops_read_frame(frame,&len))
		{
			uart_ops_handle_frame(frame);
		}
	}
}

/***********************************************************************
*  Name         : uart_ops_send_polling
*  Description : send a frame to navi periodically if has frame to be send.
*  Parameter  : None
*  Returns      : None
***********************************************************************/
void uart_ops_send_polling(void)
{
	BOOL bSend = FALSE;
	if(uart_ops_is_tx_idle())
	{
		if(0 == uart_ops_send_frame_interval_counter)
		{
			if(OPS_ACK_WAIT_TIMEOUT == uart_ops_tx_info.ack_wait_state)/*if ack timeout, we need retry sending last frame*/
			{
				uart_ops_tx_info.ack_wait_state = OPS_ACK_WAIT_NONE;
				uart_ops_tx_info.req_send_frame = TRUE;
				bSend = TRUE;
			}
			else if((OPS_ACK_WAIT_NONE == uart_ops_tx_info.ack_wait_state)&&(uart_ops_has_frame_send()))/*if no ack to be waited, send a new frame*/
			{
				uart_ops_tx_info.req_send_frame = TRUE;
				uart_ops_send_out_frame();
				bSend = TRUE;
			}
			else
			{
				//
			}
			if(bSend)
			{
				bSend = FALSE;
				uart_ops_trig_tx();/*to trigger tx intr,when has frame/ack to be send*/
			}
		}

		if((FALSE == uart_ops_tx_info.req_send_ack)&&(FALSE == uart_ops_tx_info.req_send_frame)) /*nothing is sending,and nothing need to send, we should check the crq */
		{
			//if(FALSE == IO_UP2NP_SPI_IRQ_GetVal())/**/
			{
				uart_ops_end_send();
			}
		}

	}
}

uint8_t uart_ops_poll_cnt = 0;
/***********************************************************************
*  Name         : uart_ops_polling
*  Description : It's called by timer interrupt periodically as spi timer.
*  Parameter  : None
*  Returns      : None
***********************************************************************/
void uart_ops_polling(void)
{
	U8 lost_frame_counter = 0;

	uart_ops_poll_cnt++;
	if(uart_ops_poll_cnt >= 2)
	{
		uart_ops_poll_cnt = 0;
		if(uart_ops_req_restart_crq())
		{
			if(uart_ops_is_tx_idle())
			{
				uart_ops_clr_restart_crq();
				uart_ops_start_send();
			}
		}

		if(uart_ops_send_frame_interval_counter)
		{
			uart_ops_send_frame_interval_counter--;
		}


		if(uart_ops_ack_timer.timer_counter )
		{
			if(--uart_ops_ack_timer.timer_counter == 0)
			{
				//req clock time out
				if(++uart_ops_ack_timer.timeout_counter > OPS_ACK_TIMEOUT_COUNTER_MAX) /**when >5, how to do ?**/
				{
					uart_ops_clr_ack_alarm();
					uart_ops_tx_info.ack_wait_state = OPS_ACK_WAIT_NONE;
					uart_ops_out_frame.bValid = FALSE;/**ignore the current frame,**/
					uart_ops_lost_ack_counter++;
					uart_ops_send_frame_handled_counter = uart_ops_tx_info.frame_id;//uart_ops_send_frame_handled_counter++;
				}
				else
				{
					uart_ops_tx_info.ack_wait_state = OPS_ACK_WAIT_TIMEOUT;
				}
			}
		}

		if(uart_ops_req_send_timer.timer_counter)
		{
			if(--uart_ops_req_send_timer.timer_counter  == 0)
			{
				//req clock time out
				if(++uart_ops_req_send_timer.timeout_counter > OPS_REQ_SEND_TIMEOUT_COUNTER_MAX) /**when >10, how to do ?**/
				{
					//uart_ops_clr_req_send_alarm();
					uart_ops_end_send();/**stop comm**/
					uart_ops_out_frame.bValid = FALSE;/**ignore the current frame,**/
					uart_ops_send_frame_handled_counter = uart_ops_tx_info.frame_id;
					uart_ops_req_tx_timeout_counter++;
				}
				else
				{
					uart_ops_set_req_send_alarm();
					uart_ops_crq_retry();
				}
			}
		}

		if(uart_ops_enter_tx_timer.timer_counter )
		{
			if(--uart_ops_enter_tx_timer.timer_counter  == 0)
			{
				uart_ops_clr_enter_tx_alarm();
				uart_ops_end_send();/**stop comm**/
				uart_ops_tx_info.status = OPS_ST_IDLE;
				uart_ops_tx_info.ack_wait_state = OPS_ACK_WAIT_NONE;
				uart_ops_out_frame.bValid = FALSE;/**ignore the current frame,**/
			}
		}

		if(uart_ops_tx_timer.timer_counter )
		{
			if(--uart_ops_tx_timer.timer_counter  == 0)
			{
				if((uart_ops_tx_info.req_send_ack)||(uart_ops_tx_info.req_send_frame))
				{
					//tx time out
					if(++uart_ops_tx_timer.timeout_counter > OPS_TX_TIMEOUT_COUNTER_MAX)
					{
						//uart_ops_clr_tx_alarm();
						uart_ops_end_send();/**stop comm**/
						uart_ops_tx_info.status = OPS_ST_IDLE;
						uart_ops_out_frame.bValid = FALSE;/**ignore the current frame,**/
						uart_ops_send_frame_handled_counter = uart_ops_tx_info.frame_id;
						uart_ops_tx_timeout_counter++;
					}
					else
					{
						uart_ops_crq_retry();
					}
				}
				else
				{
					//uart_ops_clr_tx_alarm();
					uart_ops_end_send();/**stop comm**/
					uart_ops_tx_info.status = OPS_ST_IDLE;
				}
			}
		}

		if(uart_ops_rx_timer.timer_counter )
		{
			if(--uart_ops_rx_timer.timer_counter  == 0)
			{
				//rx time out
				uart_ops_clr_rx_alarm();
				uart_ops_rx_info.status = OPS_ST_IDLE;
				uart_ops_rx_timeout_counter++;
			}
		}

		lost_frame_counter = uart_ops_lost_ack_counter+uart_ops_req_tx_timeout_counter+uart_ops_rx_timeout_counter+uart_ops_tx_timeout_counter;
		if(lost_frame_counter > OPS_FRAME_LOST_COUNTER_MAX)
		{
			//reset system
			//power_get_navi_reqeust_reset_msg();
		}
	}
}


/***********************************************************************
*  Name         : uart_ops_rx_proc
*  Description : it's called by the interrupt handler,and put char into buffer when valid.
*  Parameter  : None
*  Returns      : None
***********************************************************************/
void uart_ops_rx_proc(void)
{
	U8 data;
	U8 rx_frame_prio;
	uart_ops_rx_timeout_counter = 0;
	uart_ops_clr_rx_alarm();

#if 0	/*not used.*/
	if(TRUE == uart_ops_tx_info.req_send_sync_byte)/** trig to start sending a new frame **/
	{
		if(0 == UART_OPS_IS_TX_FLAG())
		{
			opsTransmit(0x00);/*not use oxff,because when receive frame,ops start with 0xff*/

			uart_ops_tx_info.status = OPS_ST_START;
			uart_ops_tx_info.req_send_sync_byte = FALSE;
			uart_ops_req_tx_timeout_counter = 0;
			uart_ops_clr_req_send_alarm();
			uart_ops_set_enter_tx_alarm();
		}
	}
#endif

	data = opsGetch();

	switch(uart_ops_rx_info.status)
	{
	case OPS_ST_IDLE:
		if(0xff == data)
		{
			uart_ops_rx_info.status = OPS_ST_A5;
		}
		break;

	case OPS_ST_START:/*No this state*/
		uart_ops_rx_info.status = OPS_ST_IDLE;
		break;

	case OPS_ST_A5:
		if(0xa5 == data)
		{
			uart_ops_rx_info.status = OPS_ST_5A;
		}
		else if(0xff == data)
		{
			uart_ops_rx_info.status = OPS_ST_A5;
		}
		else
		{
			uart_ops_rx_info.status = OPS_ST_IDLE;
		}
		break;
	case OPS_ST_5A:
		if(0x5a == data)
		{
			uart_ops_rx_info.status = OPS_ST_ID;
		}
		else if(0xff == data)
		{
			uart_ops_rx_info.status = OPS_ST_A5;
		}
		else
		{
			uart_ops_rx_info.status = OPS_ST_IDLE;
		}
		break;
	case OPS_ST_ID:
		uart_ops_rx_info.frame_id = data;
		uart_ops_rx_info.cs = uart_ops_rx_info.frame_id; //init the cs value
		uart_ops_rx_info.status = OPS_ST_LEN;
		break;
	case OPS_ST_LEN:
		if(data <= UART_OPS_FRAME_SIZE)
		{
			uart_ops_rx_info.frame_len = data;
			uart_ops_rx_info.cs ^= uart_ops_rx_info.frame_len;
			uart_ops_in_frame.length = data;
			uart_ops_rx_info.status = OPS_ST_TYPE;
		}
		else
		{
			uart_ops_rx_info.status =OPS_ST_IDLE;
		}
		break;
	case OPS_ST_TYPE:	/*the first data indicate frame type:0xff is ack,others is data frame.*/
		if(0xff == data) /** ack frame **/
		{
			uart_ops_rx_info.frame_type = data;
			uart_ops_rx_info.cs ^= uart_ops_rx_info.frame_type;
			--uart_ops_rx_info.frame_len;
			uart_ops_rx_info.status = OPS_ST_ACK_CS;
		}
		else if(data < OPS_FRAME_TYPE_MAX) /** data frame **/
		{
			uart_ops_rx_info.byte_index = 0;
			uart_ops_in_frame.buff[uart_ops_rx_info.byte_index++] = data;
			uart_ops_rx_info.frame_type = data;
			uart_ops_rx_info.cs ^= uart_ops_rx_info.frame_type;
			--uart_ops_rx_info.frame_len;
			uart_ops_rx_info.status = OPS_ST_DATA;
		}
		else
		{
			uart_ops_rx_info.status =OPS_ST_IDLE;
		}
		break;
	case OPS_ST_DATA:
		uart_ops_in_frame.buff[uart_ops_rx_info.byte_index++] = data;
		uart_ops_rx_info.cs ^= data;
		if(uart_ops_rx_info.frame_len > 0)
		{
			if(--uart_ops_rx_info.frame_len == 0)
			{
				uart_ops_rx_info.status = OPS_ST_DATA_CS;
			}
		}
		else
		{
			uart_ops_rx_info.status = OPS_ST_DATA_CS;
		}
		break;
	case OPS_ST_DATA_CS:
		if(data == uart_ops_rx_info.cs )/** a full frame received **/
		{
			uart_ops_rx_info.byte_index = 0; /**clear byte_index **/
			if(uart_ops_rx_info.last_frame_id == uart_ops_rx_info.frame_id)/** is the last frame **/
			{
				/**not put to fifo, only send ack again**/
				/*received frame need ack.*/
				if( (OPS_RES_ACK_NEEDED == uart_ops_rx_info.frame_type)
						||(OPS_REQ_ACK_NEEDED == uart_ops_rx_info.frame_type) )
				{
					uart_ops_send_ack(uart_ops_rx_info.frame_id);
				}
			}
			else
			{
				rx_frame_prio = uart_ops_in_frame.buff[OPS_CMD_PRIO];
				if(rx_frame_prio <= OPS_UART_MSG_PRIO_NUM)
				{
					if(TRUE == fifo_input_frame(&uart_ops_in_fifo[rx_frame_prio-1].fifo_info,&(uart_ops_in_frame.buff[0]),uart_ops_in_frame.length))
					{
						/**if fifo full, will not send ack, so navi will send again. **/
						uart_ops_rec_frame_counter++;
						uart_ops_rx_info.last_frame_id  = uart_ops_rx_info.frame_id;
						/*received frame need ack.*/
						if( (OPS_RES_ACK_NEEDED == uart_ops_rx_info.frame_type)
								||(OPS_REQ_ACK_NEEDED == uart_ops_rx_info.frame_type) )
						{
							uart_ops_send_ack(uart_ops_rx_info.frame_id);
						}
					}
					else
					{
						//fifo is full
					}
				}
			}
		}
		uart_ops_rx_info.status = OPS_ST_IDLE;
		break;
	case OPS_ST_ACK_CS:
		if((uart_ops_rx_info.frame_id == uart_ops_tx_info.frame_id)&&(data == uart_ops_rx_info.cs)) /** a new ack received **/
		{
			uart_ops_lost_ack_counter = 0;
			uart_ops_clr_ack_alarm(); /**kill the timer **/
			uart_ops_tx_info.ack_wait_state = OPS_ACK_WAIT_NONE;/**/
			uart_ops_out_frame.bValid = FALSE;/** uart_ops_out_frame has been send successfully**/
			uart_ops_send_frame_handled_counter = uart_ops_tx_info.frame_id;
		}
		uart_ops_rx_info.status = OPS_ST_IDLE;
		break ;
	default:
		uart_ops_rx_info.status = OPS_ST_IDLE;
		break;
	}
	if(uart_ops_rx_info.status > OPS_ST_START)
	{
		uart_ops_set_rx_alarm();
	}

}

/***********************************************************************
*  Name         : uart_ops_tx_proc
*  Description : it's called by the interrupt handler, and put char out of the buffer
*  Parameter  : None
*  Returns      : None
***********************************************************************/
void uart_ops_tx_proc(void)
{
	uart_ops_clr_enter_tx_alarm();
	uart_ops_tx_timeout_counter = 0;
	uart_ops_clr_tx_alarm();

	switch(uart_ops_tx_info.status)
	{
	case OPS_ST_IDLE:/*do nothing,error state*/
		break;
	case OPS_ST_START:
	case OPS_ST_END:
		if(uart_ops_tx_info.req_send_ack)
		{
			uart_ops_tx_info.req_send_ack = FALSE;
			uart_ops_tx_info.send_type = OPS_IS_SENDING_ACK;
			opsTransmit(0xff);
			uart_ops_tx_info.status = OPS_ST_A5;
		}
		else if((TRUE == uart_ops_tx_info.req_send_frame)&&(TRUE == uart_ops_out_frame.bValid))
		{
			uart_ops_tx_info.req_send_frame = FALSE;
			uart_ops_tx_info.send_type = OPS_IS_SENDING_FRAME;
			opsTransmit(0xff);
			uart_ops_tx_info.status = OPS_ST_A5;
		}
		else
		{
			/** it has nothing to send **/
			uart_ops_end_send(); /** one of the two places to call uart_ops_end_send() **/
			uart_ops_tx_info.status = OPS_ST_IDLE;
		}
		break;
	case OPS_ST_A5:
		opsTransmit(0xa5);
		uart_ops_tx_info.status = OPS_ST_5A;
		break;
	case OPS_ST_5A:
		opsTransmit(0x5a);
		uart_ops_tx_info.status =OPS_ST_ID;
		break;
	case OPS_ST_ID:
		if(OPS_IS_SENDING_ACK == uart_ops_tx_info.send_type) /*send ack*/
		{
			opsTransmit(uart_ops_tx_info.ack_id);
		}
		else/* send a data frame */
		{
			opsTransmit(uart_ops_tx_info.frame_id);
		}
		uart_ops_tx_info.status = OPS_ST_LEN;
		break ;
	case OPS_ST_LEN:
		if(OPS_IS_SENDING_ACK == uart_ops_tx_info.send_type) /*send ack*/
		{
			opsTransmit(0x01);/*ack len=0x01*/
			uart_ops_tx_info.status = OPS_ST_TYPE;
		}
		else/* send a data frame */
		{
			uart_ops_tx_info.byte_index = 0;
			uart_ops_tx_info.frame_len = uart_ops_out_frame.length;
			opsTransmit(uart_ops_out_frame.length);
			if(0==uart_ops_out_frame.length)
			{
				uart_ops_tx_info.status = OPS_ST_DATA_CS;
			}
			else
			{
				uart_ops_tx_info.status = OPS_ST_DATA;
			}
		}
		break ;
	case OPS_ST_TYPE:	/*the first data is frame type: 0xff indicate ack frame.*/
		if(OPS_IS_SENDING_ACK == uart_ops_tx_info.send_type) /*send ack*/
		{
			opsTransmit(0xff);/*ack msgtype=0xff*/
			uart_ops_tx_info.status = OPS_ST_ACK_CS;
		}
		else
		{
			uart_ops_tx_info.status = OPS_ST_END;
		}
		break;
	case OPS_ST_DATA:
		opsTransmit(uart_ops_out_frame.buff[uart_ops_tx_info.byte_index++]);
		if(--uart_ops_tx_info.frame_len == 0)
		{
			uart_ops_tx_info.status = OPS_ST_DATA_CS;
		}
		break;
	case OPS_ST_DATA_CS:
		opsTransmit(uart_ops_out_frame.buff[uart_ops_out_frame.length]); /*uart_ops_out_frame.length byte is cs*/
		/*if sended frame type is OPS_REQ_ACK_NEEDED,need to wait ack,otherwise don't have to wait ack.*/
		if( (OPS_REQ_ACK_NEEDED == uart_ops_out_frame.buff[0])
				||(OPS_RES_ACK_NEEDED == uart_ops_out_frame.buff[0]) )
		{
			/*set the bit prevent from send a new frame when sent a frame without receiving an ack back*/
			uart_ops_tx_info.ack_wait_state = OPS_ACK_WAITING;
			uart_ops_set_ack_alarm();/* timing */
		}
		else
		{
			uart_ops_lost_ack_counter = 0;
			uart_ops_clr_ack_alarm(); /**kill the timer **/
			uart_ops_tx_info.ack_wait_state = OPS_ACK_WAIT_NONE;/**/
			uart_ops_out_frame.bValid = FALSE;/** uart_ops_out_frame regard as been send successfully**/
			uart_ops_send_frame_handled_counter = uart_ops_tx_info.frame_id;
		}
		uart_ops_tx_info.send_type = OPS_IS_SENDING_NONE;
		uart_ops_tx_info.status = OPS_ST_END;
		break ;
	case OPS_ST_ACK_CS:
		opsTransmit(0x01^0xff^uart_ops_tx_info.ack_id) ;/*cs= RC^LEN^MSGTYPE.*/
		uart_ops_tx_info.send_type = OPS_IS_SENDING_NONE;
		uart_ops_tx_info.status = OPS_ST_END;
		break ;

	default:
		opsTransmit(0xff);
		uart_ops_tx_info.status = OPS_ST_END;
		break;
	}
	if(uart_ops_is_tx_idle())
	{
		uart_ops_send_frame_interval_counter = OPS_UART_SEND_FRAME_INTERVAL_MIN;
	}
	else
	{
		if(uart_ops_tx_info.status > OPS_ST_START)
		{
			uart_ops_set_tx_alarm();
		}
	}
}

/***********************************************************************
*  Name         : uart_ops_is_tx_idle
*  Description : is tx idle ?
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static BOOL uart_ops_is_tx_idle(void)
{
	return (OPS_ST_IDLE == uart_ops_tx_info.status);
}


/*---Private----------------------------------------------------------*/
/***********************************************************************
*  Name         : uart_ops_read_frame
*  Description : get frame from output fifo,and request sendding
*  Parameter  : U8 *frame, U8 *length
*  Returns      : BOOL
***********************************************************************/
static BOOL uart_ops_read_frame(U8 *frame, U8 *length)
{
	if (!fifo_is_empty(&uart_ops_in_fifo[0].fifo_info))
	{
		return (fifo_output_frame(&uart_ops_in_fifo[0].fifo_info, frame,length));
	}
	else if (!fifo_is_empty(&uart_ops_in_fifo[1].fifo_info))
	{
		return (fifo_output_frame(&uart_ops_in_fifo[1].fifo_info, frame,length));
	}
	else if (!fifo_is_empty(&uart_ops_in_fifo[2].fifo_info))
	{
		return (fifo_output_frame(&uart_ops_in_fifo[2].fifo_info, frame,length));
	}
	else if (!fifo_is_empty(&uart_ops_in_fifo[3].fifo_info))
	{
		return (fifo_output_frame(&uart_ops_in_fifo[3].fifo_info, frame,length));
	}
	else
	{
		return FALSE;
	}
}

/***********************************************************************
*  Name         : uart_ops_handle_frame
*  Description : get frame from output fifo,and request sendding
*  Parameter  : U8 *frame
*  Returns      : None
***********************************************************************/
static void uart_ops_handle_frame(U8 const *frame)
{
	ops_uart_msg_handle(frame);
	uart_ops_rec_frame_handled_counter++;
}


/***********************************************************************
*  Name         : uart_ops_config
*  Description : to config spi hardware
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void uart_ops_config(void)
{
	//UART_OPS_Init();//uart init in function: mcu_init().
	uart_ops_end_send();
}

/***********************************************************************
*  Name         : uart_ops_init_global_var
*  Description : init the global var
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void uart_ops_init_global_var(void)
{
	//init counter
	uart_ops_rec_frame_counter = 0;
	uart_ops_rec_frame_handled_counter = 0;
	uart_ops_send_frame_counter = 0;
	uart_ops_send_frame_handled_counter = 0;
	uart_ops_lost_ack_counter = 0;
	uart_ops_req_tx_timeout_counter = 0;
	uart_ops_tx_timeout_counter = 0;
	uart_ops_rx_timeout_counter = 0;
	uart_ops_send_frame_interval_counter = 0;
}

/***********************************************************************
*  Name         : uart_ops_init_struct_var
*  Description : init the struct var
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void uart_ops_init_struct_var(void)
{
	//init uart_ops_rx_info
	uart_ops_rx_info.status = 0;
	uart_ops_rx_info.cs = 0;
	uart_ops_rx_info.last_frame_id = 0xff;
	uart_ops_rx_info.frame_id = 0;
	uart_ops_rx_info.frame_len = 0;
	uart_ops_rx_info.byte_index = 0;

	//init uart_ops_tx_info
	uart_ops_tx_info.req_send_sync_byte = FALSE;
	uart_ops_tx_info.req_send_ack = FALSE;
	uart_ops_tx_info.req_send_frame = FALSE;
	uart_ops_tx_info.req_restart_crq = FALSE;
	uart_ops_tx_info.send_type = OPS_IS_SENDING_NONE;
	uart_ops_tx_info.ack_wait_state = OPS_ACK_WAIT_NONE;
	uart_ops_tx_info.status= 0;
	uart_ops_tx_info.cs= 0;
	uart_ops_tx_info.ack_id= 0;
	uart_ops_tx_info.frame_id= 0;
	uart_ops_tx_info.frame_len= 0;
	uart_ops_tx_info.byte_index= 0;

	//init frame
	uart_ops_in_frame.bValid = FALSE;
	uart_ops_in_frame.length= 0;
	uart_ops_out_frame.bValid = FALSE;
	uart_ops_out_frame.length = 0;

	//init fifo

	uart_ops_in_fifo[0].pBuff = uart_ops_in_buff_one;
	uart_ops_in_fifo[1].pBuff = uart_ops_in_buff_two;
	uart_ops_in_fifo[2].pBuff = uart_ops_in_buff_three;
	uart_ops_in_fifo[3].pBuff = uart_ops_in_buff_four;

	uart_ops_out_fifo[0].pBuff = uart_ops_out_buff_one;
	uart_ops_out_fifo[1].pBuff = uart_ops_out_buff_two;
	uart_ops_out_fifo[2].pBuff = uart_ops_out_buff_three;
	uart_ops_out_fifo[3].pBuff = uart_ops_out_buff_four;

	fifo_io_init(&uart_ops_in_fifo[0].fifo_info, uart_ops_in_fifo[0].pBuff , UART_OPS_INP_BUF_ONE_SIZE);
	fifo_io_init(&uart_ops_in_fifo[1].fifo_info, uart_ops_in_fifo[1].pBuff , UART_OPS_INP_BUF_TWO_SIZE);
	fifo_io_init(&uart_ops_in_fifo[2].fifo_info, uart_ops_in_fifo[2].pBuff , UART_OPS_INP_BUF_THREE_SIZE);
	fifo_io_init(&uart_ops_in_fifo[3].fifo_info, uart_ops_in_fifo[3].pBuff , UART_OPS_INP_BUF_FOUR_SIZE);

	fifo_io_init(&uart_ops_out_fifo[0].fifo_info, uart_ops_out_fifo[0].pBuff , UART_OPS_OUT_BUF_ONE_SIZE);
	fifo_io_init(&uart_ops_out_fifo[1].fifo_info, uart_ops_out_fifo[1].pBuff , UART_OPS_OUT_BUF_TWO_SIZE);
	fifo_io_init(&uart_ops_out_fifo[2].fifo_info, uart_ops_out_fifo[2].pBuff , UART_OPS_OUT_BUF_THREE_SIZE);
	fifo_io_init(&uart_ops_out_fifo[3].fifo_info, uart_ops_out_fifo[3].pBuff , UART_OPS_OUT_BUF_FOUR_SIZE);


	//init timer
	uart_ops_clr_req_send_alarm();
	uart_ops_clr_enter_tx_alarm();
	uart_ops_clr_tx_alarm();
	uart_ops_clr_ack_alarm();
	uart_ops_clr_rx_alarm();
}

/***********************************************************************
*  Name         : uart_ops_init_var
*  Description : init all the spi var
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void uart_ops_init_var(void)
{
	uart_ops_init_global_var();
	uart_ops_init_struct_var();
}

/***********************************************************************
*  Name         : uart_ops_crq_retry
*  Description : retry to set the crq to navi
*  Parameter  : None
*  Returns      : None
***********************************************************************/
void uart_ops_crq_retry(void)
{
	uart_ops_end_send();
	uart_ops_set_restart_crq();
}

/***********************************************************************
*  Name         : uart_ops_trig_tx
*  Description : to trigger the tx interrupt.
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void uart_ops_trig_tx(void)
{
	if(uart_ops_is_tx_idle())
	{
		uart_ops_start_send();
		uart_ops_set_req_send_alarm();
		uart_ops_tx_info.req_send_sync_byte = TRUE;
	}
}

/***********************************************************************
*  Name         : uart_ops_has_frame_rev
*  Description : to confirm if has frame received.
*  Parameter  : None
*  Returns      : BOOL
***********************************************************************/
static BOOL uart_ops_has_frame_rev(void)
{
	return (uart_ops_rec_frame_counter != uart_ops_rec_frame_handled_counter);
}

/***********************************************************************
*  Name         : uart_ops_has_frame_send
*  Description : to confirm if has frame to be send.
*  Parameter  : None
*  Returns      : BOOL
***********************************************************************/
static BOOL uart_ops_has_frame_send(void)
{
	return (uart_ops_send_frame_counter != uart_ops_send_frame_handled_counter);
}

/***********************************************************************
*  Name         : uart_ops_send_out_frame
*  Description : get frame from output fifo,and request sendding
*  Parameter  : None
*  Returns      : BOOL
***********************************************************************/
static BOOL uart_ops_send_out_frame(void)
{
	U8 ret_val = FALSE;
	U8 cs,i;

	if(FALSE == uart_ops_out_frame.bValid)/**uart_ops_out_frame send completely **/
	{
		if( !fifo_is_empty(&uart_ops_out_fifo[0].fifo_info) )
		{
			/**get frame to uart_ops_out_frame**/
			ret_val = fifo_output_frame(&uart_ops_out_fifo[0].fifo_info, uart_ops_out_frame.buff,&uart_ops_out_frame.length);
		}
		else if( !fifo_is_empty(&uart_ops_out_fifo[1].fifo_info) )
		{
			/**get frame to uart_ops_out_frame**/
			ret_val = fifo_output_frame(&uart_ops_out_fifo[1].fifo_info, uart_ops_out_frame.buff,&uart_ops_out_frame.length);
		}
		else if( !fifo_is_empty(&uart_ops_out_fifo[2].fifo_info) )
		{
			/**get frame to uart_ops_out_frame**/
			ret_val = fifo_output_frame(&uart_ops_out_fifo[2].fifo_info, uart_ops_out_frame.buff,&uart_ops_out_frame.length);
		}
		else if( !fifo_is_empty(&uart_ops_out_fifo[3].fifo_info) )
		{
			/**get frame to uart_ops_out_frame**/
			ret_val = fifo_output_frame(&uart_ops_out_fifo[3].fifo_info, uart_ops_out_frame.buff,&uart_ops_out_frame.length);
		}

		if( TRUE == ret_val )
		{
			cs = ++uart_ops_tx_info.frame_id;//cs  =ID^LENGTH^DATA1^DATA2^....^DATAn
			cs^=uart_ops_out_frame.length;
			for(i=0; i<uart_ops_out_frame.length ; i++)
			{
				cs^=uart_ops_out_frame.buff[i];
			}
			uart_ops_out_frame.buff[uart_ops_out_frame.length] = cs;
			uart_ops_out_frame.bValid = TRUE;
			ret_val = TRUE;
		}
	}
	return ret_val;
}

/***********************************************************************
*  Name         : uart_ops_set_rx_alarm
*  Description : set rx alarm
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void uart_ops_set_rx_alarm(void)
{
	uart_ops_rx_timer.timer_counter = OPS_UART_RX_TIMEOUT_MAX;
}

/***********************************************************************
*  Name         : uart_ops_clr_rx_alarm
*  Description : clr rx alarm
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void uart_ops_clr_rx_alarm(void)
{
	uart_ops_rx_timer.timer_counter = 0;
	uart_ops_rx_timer.timeout_counter = 0;
}

/***********************************************************************
*  Name         : uart_ops_set_tx_alarm
*  Description : start tx alarm
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void uart_ops_set_tx_alarm(void)
{
	uart_ops_tx_timer.timer_counter = OPS_UART_TX_TIMEOUT_MAX;
}
/***********************************************************************
*  Name         : uart_ops_clr_tx_alarm
*  Description : clear tx alarm
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void uart_ops_clr_tx_alarm(void)
{
	uart_ops_tx_timer.timer_counter = 0;
	uart_ops_tx_timer.timeout_counter = 0;
}

/***********************************************************************
*  Name         : uart_ops_set_req_send_alarm
*  Description :
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void uart_ops_set_req_send_alarm(void)
{
	/**uart don't need synchronism mechanism,so set the value to 0.**/
	uart_ops_req_send_timer.timer_counter = 0;//OPS_UART_REQ_SEND_TIMEOUT_MAX;
}
/***********************************************************************
*  Name         : uart_ops_clr_req_send_alarm
*  Description :
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void uart_ops_clr_req_send_alarm(void)
{
	uart_ops_req_send_timer.timer_counter = 0;
	uart_ops_req_send_timer.timeout_counter = 0;
}

/***********************************************************************
*  Name         : uart_ops_set_ack_alarm
*  Description :
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void uart_ops_set_ack_alarm(void)
{
	uart_ops_ack_timer.timer_counter = OPS_UART_ACK_TIMEOUT_MAX;
}

/***********************************************************************
*  Name         : uart_ops_clr_ack_alarm
*  Description :
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void uart_ops_clr_ack_alarm(void)
{
	uart_ops_ack_timer.timer_counter = 0;
	uart_ops_ack_timer.timeout_counter = 0;
}


/***********************************************************************
*  Name         : uart_ops_set_enter_tx_alarm
*  Description :
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void uart_ops_set_enter_tx_alarm(void)
{
	uart_ops_enter_tx_timer.timer_counter = OPS_UART_ENTER_TX_TIMEOUT_MAX;
}

/***********************************************************************
*  Name         : uart_ops_clr_enter_tx_alarm
*  Description :
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void uart_ops_clr_enter_tx_alarm(void)
{
	uart_ops_enter_tx_timer.timer_counter = 0;
	uart_ops_enter_tx_timer.timeout_counter = 0;
}

/***********************************************************************
*  Name         : uart_ops_clr_all_send_alarm
*  Description :
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void uart_ops_clr_all_send_alarm(void)
{
	uart_ops_clr_req_send_alarm();
	uart_ops_clr_enter_tx_alarm();
	uart_ops_clr_tx_alarm();
}

/***********************************************************************
*  Name         : uart_ops_send_ack
*  Description :
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void  uart_ops_send_ack(U8 frame_id)
{
	uart_ops_tx_info.ack_id = frame_id;
	uart_ops_tx_info.req_send_ack = TRUE;
	uart_ops_trig_tx();
}

/***********************************************************************
*  Name         : uart_ops_set_restart_crq
*  Description :
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void uart_ops_set_restart_crq(void)
{
	uart_ops_tx_info.req_restart_crq = TRUE;
}

/***********************************************************************
*  Name         : uart_ops_clr_restart_crq
*  Description :
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void uart_ops_clr_restart_crq(void)
{
	uart_ops_tx_info.req_restart_crq = FALSE;
}
/***********************************************************************
*  Name         : uart_ops_req_restart_crq
*  Description :
*  Parameter  : None
*  Returns      : BOOL
***********************************************************************/
static BOOL uart_ops_req_restart_crq(void)
{
	return (uart_ops_tx_info.req_restart_crq);
}

/***********************************************************************
*  Name         : uart_ops_start_send
*  Description :
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void uart_ops_start_send(void)
{
	if((USART_SR(OPS_DEVICE.reg_addr) & USART_SR_TXE) == USART_SR_TXE)
	{
		uart_ops_tx_info.status = OPS_ST_START;
		uart_ops_tx_info.req_send_sync_byte = FALSE;
		uart_ops_req_tx_timeout_counter = 0;
		uart_ops_clr_req_send_alarm();
		uart_ops_set_enter_tx_alarm();
		opsTransmit(0x00);/*not use oxff,because when receive frame,ops start with 0xff*/
	}
}
/***********************************************************************
*  Name         : uart_ops_end_send
*  Description :
*  Parameter  : None
*  Returns      : None
***********************************************************************/
static void uart_ops_end_send(void)
{
	uart_ops_clr_all_send_alarm();
}

/**************** END OF FILE *****************************************/
