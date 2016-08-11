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
#ifndef __ATOMIC_TUNER_DEF_H_
#define __ATOMIC_TUNER_DEF_H_


/* I2C address*/
#define ATOMIC_I2C_WRITE_ADDRESS		0xC0
#define ATOMIC_I2C_READ_ADDRESS			0xC1

/* write address*/
#define ATOMIC_TUNER0_ADDR				0x00
#define ATOMIC_TUNER1_ADDR				0x01
#define ATOMIC_TUNER2_ADDR				0x02

/*read address*/
typedef enum
{
    ATOMIC_STATUS_ADDR,
    ATOMIC_LEVEL_ADDR,
    ATOMIC_USN_WAM_ADDR,
    ATOMIC_IFCOUNTER_ADDR,
    ATOMIC_ID_ADDR,
    ATOMIC_MAX_READ_ADDR,
}ATOMIC_READ_REGISTERS;


#endif //Multi-inclusion prohibition
