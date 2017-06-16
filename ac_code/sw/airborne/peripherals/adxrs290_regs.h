/*
 * Copyright (C)2014 Federico Ruiz Ugalde
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
 * @file peripherals/adxrs290_regs.h
 *
 * ST ADXRS290 2-axis gyroscope register definitions.
 */

#ifndef ADXRS290_REGS_H
#define ADXRS290_REGS_H

/* Registers */
#define ADXRS290_REG_ADI_ID   	0x00
#define ADXRS290_REG_MEMS_ID  	0x01
#define ADXRS290_REG_DEV_ID  	0x02
#define ADXRS290_REG_REV_ID  	0x03
#define ADXRS290_REG_SN0  		0x04
#define ADXRS290_REG_SN1 		0x05
#define ADXRS290_REG_SN2    	0x06
#define ADXRS290_REG_SN3    	0x07
#define ADXRS290_REG_OUT_X_L    0x08
#define ADXRS290_REG_OUT_X_H    0x09
#define ADXRS290_REG_OUT_Y_L    0x0A
#define ADXRS290_REG_OUT_Y_H    0x0B
#define ADXRS290_REG_TEMP_L 	0x0C
#define ADXRS290_REG_TEMP_H    	0x0D
#define ADXRS290_REG_POWER_CTL  0x10
#define ADXRS290_REG_FILTER    	0x11
#define ADXRS290_REG_DATA_READY 0x12

/** ADXRS290 device identifier contained in ADXRS290_REG_DEV_ID */
#define ADXRS290_DEV_ID_VAL       0x92
#define ADXRS290_DATA_READY_VAL   0x01

/** Digital Low Pass Filter Options */
enum Adxrs290DLPF
{
	ADXRS290_DLPF_480HZ = 0x0,//default
	ADXRS290_DLPF_320HZ = 0x1,
	ADXRS290_DLPF_160HZ = 0x2,
	ADXRS290_DLPF_80HZ = 0x3,
	ADXRS290_DLPF_56_6HZ = 0x4,
	ADXRS290_DLPF_40HZ = 0x5,
	ADXRS290_DLPF_28_3HZ = 0x6,
	ADXRS290_DLPF_20HZ = 0x7,
};

/** Digital High Pass Filter Options */
enum Adxrs290DHPF
{
	ADXRS290_DHPF_0 = 0x0, //all pass default
	ADXRS290_DHPF_0_011, //0.011Hz
	ADXRS290_DHPF_0_022,
	ADXRS290_DHPF_0_044,
	ADXRS290_DHPF_0_087,
	ADXRS290_DHPF_0_175,
	ADXRS290_DHPF_0_350,
	ADXRS290_DHPF_0_700,
	ADXRS290_DHPF_1_400,
	ADXRS290_DHPF_2_800,
	ADXRS290_DHPF_11_30,
};


enum ADXRS290_TSM
{
	ADXRS290_TSM_DISABLE = 0x0,
	ADXRS290_TSM_ENABLE = 0x1,
};

enum ADXRS290_PWR
{
	ADXRS290_PWR_STANDDY = 0x0,
	ADXRS290_PWR_MEAS = 0x1,
};

#endif /* ADXRS290_REGS_H */
