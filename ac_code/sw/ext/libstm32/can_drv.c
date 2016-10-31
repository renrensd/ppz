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

/*---Public include files---------------------------------------------*/



/*---Private include files--------------------------------------------*/
#include <stm32f4xx.h>
#include "stm32f4xx_conf.h"
#include "can_drv.h"
#include "std.h"
#include "subsystems/datalink/can_transport.h"

/*===VARIABLES========================================================*/
static CanTxMsg TxDrvMessage;
static CanRxMsg RxDrvMessage;
static bool_t can_drv_initialized = FALSE;
//can_rx_callback_tt can_drv_rx_callback;

/*---Global-----------------------------------------------------------*/
/***********************************************************************
* FUNCTION    : can_drv_nvic_config
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void can_drv_nvic_config(void) 
{
 	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	
}

/***********************************************************************
* FUNCTION    : can_drv_init
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void can_drv_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	
	/* Enable GPIO clock */
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  	/* Connect CAN pins to AF9 */
  	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
  	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1); 
  
  	/* Configure CAN RX and TX pins */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  	GPIO_Init(GPIOD, &GPIO_InitStructure);

	can_drv_nvic_config();

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	CAN_DeInit(CAN1);
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_11tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_Prescaler = 6;	//42MHz/(6*(1+11+2))=500Kbps, samplepoint = 12/14 = 87.5%
	CAN_Init(CAN1,&CAN_InitStructure);

	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0;
	CAN_FilterInitStructure.CAN_FilterIdLow  = 0;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh  = 0;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow   = 0;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

	/* Transmit Structure preparation */
  	TxDrvMessage.RTR = CAN_RTR_DATA;
  	TxDrvMessage.IDE = CAN_ID_STD;

	CAN_ITConfig(CAN1,CAN_IT_FMP0 | CAN_IT_TME, ENABLE);

	can_drv_initialized = TRUE;
}

/***********************************************************************
* FUNCTION    : can_drv_transmit
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
int can_drv_transmit(uint32_t id, const uint8_t *buf, uint8_t len)
{
	uint8_t i;
	uint8_t TransmitMailbox = 0; 

  	if (!can_drv_initialized) 
  	{
    	return -2;
  	}

  	if (len > 8) 
	{
    	return -1;
  	}

	TxDrvMessage.StdId = id;
	TxDrvMessage.DLC = len;
	for(i=0; i<len; i++)
	{
		TxDrvMessage.Data[i] = buf[i];
	}

	TransmitMailbox = CAN_Transmit(CAN1,&TxDrvMessage);	
}

/***********************************************************************
* FUNCTION    : can1_rx0_isr
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void can1_rx0_isr(void)
{
	CAN_Receive(CAN1, CAN_FIFO0, &RxDrvMessage);
	#ifdef BBOX_OPTION
  	can_drv_rx_callback(RxDrvMessage.StdId, RxDrvMessage.Data, RxDrvMessage.DLC);
	#endif
	CAN_FIFORelease(CAN1,CAN_FIFO0);
}

/***********************************************************************
* FUNCTION    : can1_tx_isr
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void can1_tx_isr(void)
{
	CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
	#ifdef BBOX_OPTION
	can_drv_tx_callback();
	#endif
}

/**************** END OF FILE *****************************************/

