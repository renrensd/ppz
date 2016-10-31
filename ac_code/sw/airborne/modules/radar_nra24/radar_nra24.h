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
 * You should have received a copy of the GNU General Public Licen
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file sonar_adc.h
 *  @brief simple driver to deal with one sonar sensor on ADC
 */

#ifndef RADAR_NRA24_H
#define RADAR_NRA24_H

#include "std.h"
#include "mcu_periph/link_device.h"

#ifndef RADAR_NRA24_PAYLOAD_LEN
#define RADAR_NRA24_PAYLOAD_LEN 8
#endif

#define  RADAR24_SPEED_R   10     //dilution of speed
#define  RADAR24_fILTER_H  0.05  //dilution of filter
#define  RADAR24_LENGTH 0.02     //tracing step length
#define  RADAR24_G 0.25
#define  CUT_OFF_PV 15
#define  NUM_FIL 8


#define RADAR_SMOOTH_MA 0
#define RADAR_BUTTERWORTH 1
#define RADAR_DIFF 2
#define CHANGE_MODE 0


struct RADAR_NRA24
{
  volatile bool_t msg_received;           ///< message received flag
  uint8_t payload[RADAR_NRA24_PAYLOAD_LEN]; ///< payload buffer
  uint8_t status;
  uint8_t ovrn, error;
  uint8_t payload_idx;
};

struct RADAR_NRA24_DATA
{
  uint16_t dis;//distance mm.
  float dis_m;
};

extern struct RADAR_NRA24  radar_nra24;
extern struct RADAR_NRA24_DATA radar_nra24_data;

extern float RADAR24_DILUT_SPEED_R;     //dilution of speed
extern float  RADAR24_DILUT_fILTER_H;    //dilution of filter
extern float  RADAR24_STER_LENGTH;     //tracing step length
extern float Radar24_G;
extern float CUT_OFF;
extern unsigned char FILTER_NUM;

extern void Init_radar24(void);

 void  radar_nra24_get_data(void);

#endif

