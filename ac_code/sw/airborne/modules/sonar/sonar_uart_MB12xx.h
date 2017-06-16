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

#ifndef SONAR_ADC_H
#define SONAR_ADC_H

#include "std.h"
#include "mcu_periph/link_device.h"

#ifndef SONAR_PAYLOAD_LEN
#define SONAR_PAYLOAD_LEN 64
#endif

struct sonar_data
{
	uint16_t distance_cm;          ///< Raw value
	float distance_m;         ///< Distance measured in meters
	volatile bool_t msg_received;           ///< message received flag
	uint8_t payload[SONAR_PAYLOAD_LEN]; ///< payload buffer
	uint8_t status;
	uint8_t ovrn, error;
	uint8_t payload_idx;
};

extern struct sonar_data Sonar_MB12;

//extern void sonar_adc_init(void)
extern void sonar_uart_read(void);

#endif
