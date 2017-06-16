/*
 * Copyright (C) 2010  Gautier Hattenberger, 2013 Tobias M眉nch
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "modules/sonar/sonar_i2c_mb1242.h"
#include "generated/airframe.h"
//#include "mcu_periph/adc.h"
#include "subsystems/abi.h"
//#include "subsystems/abi_sender_ids.h"
#ifdef SITL
#include "state.h"
#endif
#include"modules/sonar/agl_dist.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
//#include "mcu_periph/gpio.h"
//#include "arch/lpc21/efs/inc/plibc.h"
#include "std.h"

struct sonar_data sonar_mb1242;


/* Private function prototypes -----------------------------------------------*/
void sonar_i2c_start(void);
void sonar_i2c_stop(void);
void sonar_i2c_start_of_all(void);
void sonar_i2c_create(void);
void sonar_i2c_wait_clk(void);
unsigned char sonar_i2c_read_byte(unsigned char ack);
unsigned char sonar_i2c_send_byte(unsigned char dat);
unsigned char sonar_i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t length, uint8_t* buf);
uint8_t sonar_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t length, uint8_t* buf);
uint8_t sonar_i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t byte);
uint8_t sonar_read_byte(uint8_t addr, uint8_t reg);
void MB1242_red(uint8_t ADDR);
void MB1242_start_sensor(uint8_t ADDR);
void MB1242_change_address(uint8_t ADDR, uint8_t NEWADDR);
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

/***********************************************************************
*  Name        : sonar_i2c_init
*  Description :
*  Parameter   :
*  Returns     :
***********************************************************************/
void sonar_init(void)
{
	gpio_enable_clock(I2C_SONAR_GPIO_SDA_PORT);
	gpio_enable_clock(I2C_SONAR_GPIO_SCL_PORT);
	gpio_mode_setup(I2C_SONAR_GPIO_SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, I2C_SONAR_GPIO_SDA);
	gpio_mode_setup(I2C_SONAR_GPIO_SCL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, I2C_SONAR_GPIO_SCL);
	gpio_set_output_options(I2C_SONAR_GPIO_SDA_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, I2C_SONAR_GPIO_SDA);
	gpio_set_output_options(I2C_SONAR_GPIO_SCL_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, I2C_SONAR_GPIO_SCL);
	sonar_i2c_create();
	MB1242_start_sensor(MB1242_ADDR);
}

/***********************************************************************
*  Name        : sonar_i2c_start
*  Description : sends an i2c start condition on the bus
*  Parameter   : none
*  Returns     : none
***********************************************************************/
void sonar_i2c_start(void)
{
	SONAR_SDA_SET_INPUT();
	SONAR_SCL_SET_INPUT();

	i2c_delay();

	SONAR_SDA_CLR_VAL();
	SONAR_SDA_SET_OUTPUT();

	i2c_delay();

	SONAR_SCL_CLR_VAL();
	SONAR_SCL_SET_OUTPUT();
	i2c_delay();
}

/***********************************************************************
*  Name        : sonar_i2c_stop
*  Description : sends an i2c stop condition on the bus
*  Parameter   : none
*  Returns     : none
***********************************************************************/
void sonar_i2c_stop(void)
{
	SONAR_SCL_CLR_VAL();
	SONAR_SCL_SET_OUTPUT();
	i2c_delay();

	SONAR_SDA_CLR_VAL();
	SONAR_SDA_SET_OUTPUT();
	i2c_delay();

	SONAR_SCL_SET_VAL();
	SONAR_SCL_SET_INPUT();
	i2c_delay();

	SONAR_SDA_SET_VAL();
	SONAR_SDA_SET_INPUT();
	i2c_delay();
}

void sonar_i2c_start_of_all(void)
{
	sonar_i2c_start();
	sonar_i2c_start();
	sonar_i2c_start();
	sonar_i2c_start();
	sonar_i2c_start();
	sonar_i2c_start();
	sonar_i2c_start();
	sonar_i2c_start();
	sonar_i2c_start();
	sonar_i2c_stop();
}

/***********************************************************************
*  Name        : sonar_i2c_wait_clk
*  Description : none
*  Parameter   : none
*  Returns     : none
***********************************************************************/
void sonar_i2c_create(void)
{
	sonar_i2c_start_of_all();
}

/***********************************************************************
*  Name        : sonar_i2c_wait_clk
*  Description : none
*  Parameter   : none
*  Returns     : none
***********************************************************************/
void sonar_i2c_wait_clk(void)
{
	unsigned int i;
	i = 0;
	while ((!SONAR_SCL_GET_VAL()) && (++i < 200) );
}

/***********************************************************************
*  Name        : sonar_i2c_read_byte
*  Description : receives one byte of data from an I2C slave device.
		       when the device recognizes that it is being addressed,it will
			   acknowledge by pulling SDA low in the ninth SCL(ACK) cycle.
*  Parameter   : 0-generates nack;	1-generates ack.
*  Returns     : data read
***********************************************************************/
unsigned char sonar_i2c_read_byte(unsigned char ack)
{
	unsigned char readmask = 0;
	unsigned char count=8;
	SONAR_SDA_SET_INPUT();
	while (count--)
	{
		SONAR_SCL_SET_OUTPUT();/* low */
		i2c_delay();
		i2c_delay();
		readmask <<= 1;
		SONAR_SCL_SET_INPUT();/* high */
		sonar_i2c_wait_clk();
		i2c_delay();
		i2c_delay();
		if (SONAR_SDA_GET_VAL())
		{
			readmask+=1;
		}
	};

	if(SONAR_SDA_GET_VAL())
	{
		SONAR_SDA_SET_VAL();
	}
	else
	{
		SONAR_SDA_CLR_VAL();
	}
	SONAR_SDA_SET_OUTPUT();

	SONAR_SCL_SET_OUTPUT();/* low */
	if (ack)
	{
		SONAR_SDA_CLR_VAL();
	}
	else
	{
		SONAR_SDA_SET_VAL();
	}
	i2c_delay();
	i2c_delay();
	SONAR_SCL_SET_INPUT();/* high */
	sonar_i2c_wait_clk();
	i2c_delay();
	i2c_delay();
	SONAR_SCL_SET_OUTPUT();/* low */
	i2c_delay();
	i2c_delay();
	return readmask;
}

/***********************************************************************
*  Name        : sonar_i2c_send_byte
*  Description : none
*  Parameter   : none
*  Returns     : none
***********************************************************************/
unsigned char sonar_i2c_send_byte(unsigned char dat)
{
	unsigned char sendmask = 0x80;
	while (sendmask)
	{
		if (dat & sendmask)
		{
			SONAR_SDA_SET_INPUT();
		}
		else
		{
			SONAR_SDA_SET_OUTPUT();
		}
		i2c_delay();
		i2c_delay();
		SONAR_SCL_SET_INPUT();/* high */
		sonar_i2c_wait_clk();
		i2c_delay();
		i2c_delay();
		sendmask >>= 1;
		SONAR_SCL_SET_OUTPUT();/* low */
		i2c_delay();
		i2c_delay();
	};

	SONAR_SDA_SET_INPUT();/* high */
	i2c_delay();
	i2c_delay();
	SONAR_SCL_SET_INPUT();/* high */
	sonar_i2c_wait_clk();
	i2c_delay();
	i2c_delay();
	sendmask = SONAR_SDA_GET_VAL();
	SONAR_SCL_SET_OUTPUT();/* low */
	i2c_delay();
	i2c_delay();
	return sendmask;
	//return ( bm_i2c_wait_ack() );
}

/***********************************************************************
*  Name        : sonar_i2c_read_reg
*  Description :
*  Parameter   :
*  Returns     : 1 -> OK; 0 -> FAIL
***********************************************************************/
unsigned char sonar_i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t length, uint8_t* buf)
{
	sonar_i2c_start();
	sonar_i2c_send_byte((addr<<1) | 0x00);/* addr write */
	sonar_i2c_send_byte(reg);
	sonar_i2c_stop();

	sonar_i2c_start();
	sonar_i2c_send_byte((addr<<1) | 0x01);/* addr read */
	while(length)
	{
		if(length == 1)
		{
			*buf = sonar_i2c_read_byte(NOACK);
		}
		else
		{
			*buf++ = sonar_i2c_read_byte(ACK);
		}
		length--;
	}
	sonar_i2c_stop();

	return 1;
}

/***********************************************************************
*  Name        : sonar_i2c_read_reg
*  Description :
*  Parameter   :
*  Returns     : 1 -> OK; 0 -> FAIL
***********************************************************************/
uint8_t sonar_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t length, uint8_t* buf)
{
	sonar_i2c_start();
	sonar_i2c_send_byte((addr<<1) | 0x00);
	sonar_i2c_send_byte(reg);

	while(length--)
	{
		sonar_i2c_send_byte(*buf++);
	}
	sonar_i2c_stop();

	return 1;
}

/***********************************************************************
*  Name        : sonar_i2c_read_reg
*  Description :
*  Parameter   :
*  Returns     : 1 -> OK; 0 -> FAIL
***********************************************************************/
uint8_t sonar_i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t byte)
{
	sonar_i2c_start();
	sonar_i2c_send_byte((addr<<1) | 0x00);
	sonar_i2c_send_byte(reg);
	sonar_i2c_send_byte(byte);
	sonar_i2c_stop();

	return 1;
}

/***********************************************************************
*  Name        : sonar_read_byte
*  Description :
*  Parameter   :
*  Returns     :
***********************************************************************/
uint8_t sonar_read_byte(uint8_t addr, uint8_t reg)
{
	uint8_t res;
	sonar_i2c_start();
	sonar_i2c_send_byte((addr<<1) | 0x00);
	sonar_i2c_send_byte(reg);
	sonar_i2c_stop();

	sonar_i2c_start();
	sonar_i2c_send_byte((addr<<1) | 0x01);
	res= sonar_i2c_read_byte(NOACK);
	sonar_i2c_stop();
	return res;
}


/*********************************************
  Name：MB1242_red
  Description ：MB1242 read distance
  Parameter ： sonar ADDR
  Returns ：
*********************************************/

void MB1242_red(uint8_t ADDR)
{
	uint8_t buf[20];

	sonar_i2c_start();
	sonar_i2c_send_byte(ADDR | 0X01);
	buf[0]= sonar_i2c_read_byte(ACK);
	buf[1]= sonar_i2c_read_byte(NOACK);
	sonar_i2c_stop();

	sonar_mb1242.distance_cm=(buf[0]<<8) | buf[1];
}

/*********************************************
  Name：MB1242_start_sensor(u8 ADDR)
  Description ：MB1242 run
  Parameter ：sonar ADDR
  Returns ：
*********************************************/

void MB1242_start_sensor(uint8_t ADDR)
{
	sonar_i2c_start();
	sonar_i2c_send_byte(ADDR & 0Xfe);
	sonar_i2c_send_byte(0x51);
	sonar_i2c_stop();
}

/*********************************************
  Name：MB1242_change_address(u8 ADDR, u8 NEWADDR)
  Description：MB1242地址修改
  Parameter ：ADDR 要修改模块的地址
              NEWADDR 新地址
  Returns :

The sensor will only accept even address values. If an odd numbered address is
sent the sensor will be set to the next lowest even number. If the sensor is told
to change to one of the invalid addresses below the sensor will ignore this command
and stay at its current address.
Invalid Address Values: 0X00, 0X50, 0XA4, 0XAA
*********************************************/

void MB1242_change_address(uint8_t ADDR, uint8_t NEWADDR)
{
	sonar_i2c_start();
	sonar_i2c_send_byte( ADDR & 0xfe);
	sonar_i2c_send_byte(0xAA);
	sonar_i2c_send_byte(0xA5);
	sonar_i2c_send_byte(NEWADDR);
	sonar_i2c_stop();
}


void sonar_i2c_read(void)
{
	MB1242_red(MB1242_ADDR);
	if( sonar_mb1242.distance_cm >780 ) return;
	sonar_mb1242.distance_m=(float)sonar_mb1242.distance_cm/100.0;

	// Send ABI message
	AbiSendMsgAGL(AGL_SONAR_ADC_ID, sonar_mb1242.distance_m);

	MB1242_start_sensor(MB1242_ADDR);
#if PERIODIC_TELEMETRY
	RunOnceEvery(10,
	{
		xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
		DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &sonar_mb1242.distance_cm, &agl_dist_value_filtered);
	});
#endif
}
