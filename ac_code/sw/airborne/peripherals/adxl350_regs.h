/*
 * Copyright (C) 2010-2013 The Paparazzi Team
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
 */

/**
 * @file peripherals/adxl350_regs.h
 *
 * Register and address definitions for ADXL350 accelerometer.
 */

#ifndef ADXL350_REGS_H
#define ADXL350_REGS_H

/** default I2C address */
#define ADXL350_ADDR            0xA6
#define ADXL350_ADDR_ALT        0x3A

/* Registers */
#define ADXL350_REG_BW_RATE     0x2C
#define ADXL350_REG_POWER_CTL   0x2D
#define ADXL350_REG_INT_ENABLE  0x2E
#define ADXL350_REG_DATA_FORMAT 0x31
#define ADXL350_REG_DATA_X0     0x32
#define ADXL350_REG_DATA_X1     0x33
#define ADXL350_REG_DATA_Y0     0x34
#define ADXL350_REG_DATA_Y1     0x35
#define ADXL350_REG_DATA_Z0     0x36
#define ADXL350_REG_DATA_Z1     0x37

/**
 * Selectable data rates in ADXL350_REG_BW_RATE
 * bandwith is always half of data rate
 */
enum Adxl350Rates
{
	ADXL350_RATE_25HZ   = 0x08,
	ADXL350_RATE_50HZ   = 0x09,
	ADXL350_RATE_100HZ  = 0x0A,
	ADXL350_RATE_200HZ  = 0x0B,
	ADXL350_RATE_400HZ  = 0x0C,
	ADXL350_RATE_800HZ  = 0x0D,
	ADXL350_RATE_1600HZ = 0x0E,
	ADXL350_RATE_3200HZ = 0x0F
};

/**
 * Selectable range in ADXL350_REG_DATA_FORMAT
 */
enum Adxl350Ranges
{
	ADXL350_RANGE_1G  = 0x00,
	ADXL350_RANGE_2G  = 0x01,
	ADXL350_RANGE_4G  = 0x02,
	ADXL350_RANGE_8G = 0x03
};

/* data format bits */
#define ADXL350_INT_INVERT  0x20
#define ADXL350_FULL_RES    0x08
#define ADXL350_JUSTIFY_MSB 0x04


#endif /* ADXL350_REGS_H */
