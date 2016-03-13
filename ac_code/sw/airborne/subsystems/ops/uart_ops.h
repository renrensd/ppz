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
#ifndef __UART_OPS_H_
#define __UART_OPS_H_

#include "../../modules/system/types.h"
#include "../../modules/system/fifo_if.h"
/**** Definition of constants ****/


/**** Definition of types ****/

enum
{
    OPS_ST_IDLE,
    OPS_ST_START, /*Only for tx*/
    OPS_ST_A5,
    OPS_ST_5A,
    OPS_ST_ID,
    OPS_ST_LEN,
    OPS_ST_TYPE,  /*indicate is ack frame or data frame.*/
    OPS_ST_ACK_CS,
    OPS_ST_DATA,
    OPS_ST_DATA_CS,
    OPS_ST_END,/*Only for tx*/
} UART_OPS_PROCESS_STATUS;

enum
{
    OPS_IS_SENDING_NONE,
    OPS_IS_SENDING_ACK, 
    OPS_IS_SENDING_FRAME,
} UART_OPS_SEND_TYPE;

enum
{
    OPS_ACK_WAIT_NONE,
    OPS_ACK_WAITING, 
    OPS_ACK_WAIT_TIMEOUT,
} UART_OPS_ACK_WAIT_STATE;

typedef struct 
{
    FIFO_TYPE fifo_info;
    U8  *pBuff;   
} UART_OPS_FIFO_TYPE;

typedef struct 
{
	U8 status;/** rx status **/
	U8 cs;/** cs  =ID^LENGTH^DATA1^DATA2^....^DATAn **/
	U8 last_frame_id;/** received successfully at last time **/
	U8 frame_id;/** received currentlly **/
	U8 frame_len;/** data frame len,  not including the len byte **/ 
	U8 byte_index; 
	U8 frame_type;
} UART_OPS_RX_INFO_TYPE;

typedef struct 
{
    BOOL req_send_sync_byte;
    BOOL req_send_ack;
    BOOL req_send_frame;
    BOOL req_restart_crq;
    U8 send_type;/*is sending ack or frame or nothing?*/
    U8 ack_wait_state;
    U8 status;/** tx status **/
    U8 cs; /** cs  =ID^LENGTH^DATA1^DATA2^....^DATAn **/
    U8 ack_id;/** id that just received and need to sent it's ack **/
    U8 frame_id;/** id value that will be sent in a new frame**/
    U8 frame_len; /** data frame len,  not including the len byte **/ 
    U8 byte_index;	
} UART_OPS_TX_INFO_TYPE;

typedef struct 
{
    U16 timer_counter;/** Timer = polling period*timer_counter **/
    U16 timeout_counter;   
} UART_OPS_TIMER_TYPE;

/**** Definition of macros ****/

#define OPS_UART_MSG_PRIO_NUM 	4	/*ops only define 4 priority.*/

#define OPS_UART_SEND_FRAME_INTERVAL_MIN 5/**5*2ms**/
#define OPS_UART_ENTER_TX_TIMEOUT_MAX 50/**50*2ms**/
#define OPS_UART_REQ_SEND_TIMEOUT_MAX 0/**uart don't need synchronism mechanism,so set the value to 0.**/
#define OPS_UART_TX_TIMEOUT_MAX 10/**10*2ms**/
#define OPS_UART_ACK_TIMEOUT_MAX 100/**100*2ms=200ms**/
#define OPS_UART_RX_TIMEOUT_MAX 50/**50*2ms**/

#define OPS_REQ_SEND_TIMEOUT_COUNTER_MAX 5
#define OPS_TX_TIMEOUT_COUNTER_MAX 5
#define OPS_ACK_TIMEOUT_COUNTER_MAX 5
#define OPS_RX_TIMEOUT_COUNTER_MAX 5
#define OPS_FRAME_LOST_COUNTER_MAX 5

/**** Declaration of constants ****/

#define UART_OPS_INP_BUF_ONE_SIZE 256     /* Input buffer size */
#define UART_OPS_INP_BUF_TWO_SIZE 512     /* Input buffer size */
#define UART_OPS_INP_BUF_THREE_SIZE 512     /* Input buffer size */
#define UART_OPS_INP_BUF_FOUR_SIZE 256     /* Input buffer size */

#define UART_OPS_OUT_BUF_ONE_SIZE 256     /* Input buffer size */
#define UART_OPS_OUT_BUF_TWO_SIZE 512     /* Input buffer size */
#define UART_OPS_OUT_BUF_THREE_SIZE 512     /* Input buffer size */
#define UART_OPS_OUT_BUF_FOUR_SIZE 256     /* Input buffer size */

/**** Declaration of variables ****/


/**** Declaration of functions ****/
static BOOL uart_ops_read_frame(U8 *frame,U8 *length);
static void uart_ops_handle_frame(U8 const *frame);
static void uart_ops_config(void);
static void uart_ops_init_var(void);
static void uart_ops_crq_retry(void);
static BOOL uart_ops_is_tx_idle(void);
static void uart_ops_trig_tx(void);
static BOOL uart_ops_send_out_frame(void);
static BOOL uart_ops_has_frame_rev(void);
static BOOL uart_ops_has_frame_send(void);

static void uart_ops_set_rx_alarm(void);
static void uart_ops_clr_rx_alarm(void);
static void uart_ops_set_tx_alarm(void);
static void uart_ops_clr_tx_alarm(void);
static void uart_ops_set_req_send_alarm(void);
static void uart_ops_clr_req_send_alarm(void);
static void uart_ops_set_ack_alarm(void);
static void uart_ops_clr_ack_alarm(void);
static void uart_ops_set_enter_tx_alarm(void);
;static void uart_ops_clr_enter_tx_alarm(void);
static void  uart_ops_send_ack(U8 frame_id);
static void uart_ops_set_restart_crq(void);
static void uart_ops_clr_restart_crq(void);
static BOOL uart_ops_req_restart_crq(void);
static void uart_ops_start_send(void);
static void uart_ops_end_send(void);

#endif /*__UART_OPS_H_*/

/****************************** END OF FILE ***************************/
