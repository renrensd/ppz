/*
 * Copyright (C) 2010  Gautier Hattenberger
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

/** @file sonar_adc.h
 *  @brief simple driver to deal with one sonar sensor on ADC
 */

#ifndef _SONAR_mb1242_H_
#define _SONAR_mb1242_H_

#include "std.h"
#include "mcu_periph/link_device.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>

#include "mcu_periph/gpio.h"
#include "mcu_periph/sys_time.h"


/**** Definition of constants ****/

#define MB1242_ADDR   		0xe0

#define SONAR_SDA_SET_INPUT()			{gpio_setup_input(I2C_SONAR_GPIO_SDA_PORT, I2C_SONAR_GPIO_SDA);}
#define SONAR_SDA_SET_OUTPUT()		    {gpio_setup_output(I2C_SONAR_GPIO_SDA_PORT, I2C_SONAR_GPIO_SDA);}

#define SONAR_SCL_SET_INPUT()			{gpio_setup_input(I2C_SONAR_GPIO_SCL_PORT, I2C_SONAR_GPIO_SCL);}
#define SONAR_SCL_SET_OUTPUT()		    {gpio_setup_output(I2C_SONAR_GPIO_SCL_PORT, I2C_SONAR_GPIO_SCL);}

#define SONAR_SDA_SET_VAL()			{gpio_set(I2C_SONAR_GPIO_SDA_PORT, I2C_SONAR_GPIO_SDA);}
#define SONAR_SDA_CLR_VAL()			{gpio_clear(I2C_SONAR_GPIO_SDA_PORT, I2C_SONAR_GPIO_SDA);}

#define SONAR_SCL_SET_VAL()			{gpio_set(I2C_SONAR_GPIO_SCL_PORT, I2C_SONAR_GPIO_SCL);}
#define SONAR_SCL_CLR_VAL()			{gpio_clear(I2C_SONAR_GPIO_SCL_PORT, I2C_SONAR_GPIO_SCL);}


#define SONAR_SDA_GET_VAL()			gpio_get(I2C_SONAR_GPIO_SDA_PORT, I2C_SONAR_GPIO_SDA)
#define SONAR_SCL_GET_VAL()			gpio_get(I2C_SONAR_GPIO_SCL_PORT, I2C_SONAR_GPIO_SCL)


#define ACK             1
#define NOACK           0

#define i2c_delay()	{delay_us(1);}

struct sonar_data
{
	uint16_t distance_cm;          ///< Raw value
	float distance_m;         ///< Distance measured in meters
};

extern struct sonar_data sonar_mb1242;

extern void sonar_init(void);
extern void sonar_i2c_read(void);

#endif
