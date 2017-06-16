/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   	   *
*				  All Rights Reserved.          					   *
*   Department 	: R&D SW      									       *
*   AUTHOR	   	:            										   *
************************************************************************
* Object        : peripherals/adxrs453_reg.h
* Module        : peripherals
* Instance      :
* Description   : ADI ADXRS453 1-axis gyroscope register definitions.
*-----------------------------------------------------------------------
* Version		:
* Date			:
* Author		:
***********************************************************************/
/*-History--------------------------------------------------------------
* Version       Date    Name    Changes and comments
*
*=====================================================================*/
#ifndef __ADXRS453_REGS_H_
#define __ADXRS453_REGS_H_

/** ADXRS453 device identifier contained in ADXRS453_REG_DEV_ID */
#define ADXRS453_DEV_ID_VAL       0x52
#define ADXRS453_DATA_READY_VAL   0x01

/* The MSB for the spi commands */
#define ADXRS453_SENSOR_DATA	0x20
#define ADXRS453_WRITE_DATA		0x40
#define ADXRS453_READ_DATA		0x80

/* Memory register map */
#define ADXRS453_RATE1			0x00	// Rate Registers
#define ADXRS453_TEMP1			0x02	// Temperature Registers
#define ADXRS453_LOCST1			0x04	// Low CST Memory Registers
#define ADXRS453_HICST1			0x06	// High CST Memory Registers
#define ADXRS453_QUAD1			0x08	// Quad Memory Registers
#define ADXRS453_FAULT1			0x0A	// Fault Registers
#define ADXRS453_PID1			0x0C	// Part ID Register 1
#define ADXRS453_SNH			0x0E	// Serial Number Registers, 4 bytes
#define ADXRS453_SNL			0x10

/* Check bits */
#define ADXRS453_P				0x01	// Parity bit
#define ADXRS453_CHK			0x02
#define ADXRS453_CST			0x04
#define ADXRS453_PWR			0x08
#define ADXRS453_POR			0x10
#define ADXRS453_NVM			0x20
#define ADXRS453_Q				0x40
#define ADXRS453_PLL			0x80
#define ADXRS453_UV				0x100
#define ADXRS453_OV				0x200
#define ADXRS453_AMP			0x400
#define ADXRS453_FAIL			0x800

#define ADXRS453_WRERR_MASK		(0x7 << 29)
#define ADXRS453_GET_ST(a)		((a >> 26) & 0x3)  // Status bits

#endif /* __ADXRS453_REGS_H_ */

/****************************** END OF FILE ***************************/

