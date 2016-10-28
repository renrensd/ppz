/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				       *
*				  All Rights Reserved.          					   *
*   Department : R&D SW      									       *
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

/**** System include files ****/
#include <string.h>
#include <stddef.h>

/*---Public include files---------------------------------------------*/
#include "modules/system/timer_if.h"
#include "modules/system/timer_class.h"
#include "modules/system/timer_def.h"
#include "mcu_periph/sys_time.h"

/*---Private include files--------------------------------------------*/
#include "fm25v.h"

#define FRAM_MACROS
#include "fram_data.h"
#include "fram.h"
#include "fram_if.h"

#include "fram_class.h"
#include "fram_def.h"
#include "fram_class.h"
#include "fram_def.h"
#include "fram_class.h"
#include "fram_def.h"
#include "fram_class.h"
#include "fram_def.h"
#include "fram_class.h"
#include "fram_def.h"
#include "fram_class.h"
#include "fram_def.h"
#include "fram_class.h"
#include "fram_def.h"
#include "fram_class.h"
#include "fram_def.h"
#include "fram_class.h"
#include "fram_def.h"

/*===VARIABLES========================================================*/
struct FRAM_INFO fram;

/*---Global-----------------------------------------------------------*/
const uint8_t fram_init_flags[4] =
{0x55,0xAA,0x5A,0xA5};//need init the whole fram 

/*---Private----------------------------------------------------------*/

/*===FUNCTIONS========================================================*/


/*---Global-----------------------------------------------------------*/
/*****************************************************************************
*  Name        : adxrs453_spi_init
*  Description : 
*  Parameter   : void  
*  Returns     : None
*****************************************************************************/
void fram_init(void)
{
	fm25v_init(&fram.fm25v, &(FRAM_SPI_DEV), FRAM_SPI_SLAVE_IDX, &fram_spi_cb);
}

/*****************************************************************************
*  Name        : fram_spi_cb
*  Description : 
*  Parameter   : void  
*  Returns     : None
*****************************************************************************/
static void fram_spi_cb(void)
{
	fm25v_spi_cb(&fram.fm25v);
}

/*****************************************************************************
*  Name        : fram_id_write
*  Description : 
*  Parameter   : void  
*  Returns     : None
*****************************************************************************/
void fram_id_write(uint8_t id, uint8_t *write_buffer)
{
	uint16_t addr = fram_get_address(id, 0);
	uint16_t length = object_quantity[id] * object_size[id];
	
	fm25v_write(&fram.fm25v, addr, write_buffer , length);
}

/*****************************************************************************
*  Name        : fram_write
*  Description : write whole id data.
*  Parameter   : void  
*  Returns     : None
*****************************************************************************/
void fram_write(uint8_t id, uint16_t item, uint8_t *write_buffer)
{
	uint16_t addr = fram_get_address(id, item);
	uint16_t length = object_size[id];
	
	fm25v_write(&fram.fm25v, addr, write_buffer , length);
}

/*****************************************************************************
*  Name        : fram_read
*  Description : 
*  Parameter   : void  
*  Returns     : None
*****************************************************************************/
void fram_read(uint8_t id, uint16_t item, uint8_t *read_buffer)
{
    uint16_t addr = fram_get_address(id, item);
    uint16_t length = object_size[id];
    fm25v_read(&fram.fm25v, addr, read_buffer, length);
}

/*****************************************************************************
*  Name        : fram_id_read
*  Description : read whole id data.
*  Parameter   : void
*  Returns     : None
*****************************************************************************/
void fram_id_read(uint8_t id, uint8_t *read_buffer)
{
    uint16_t addr = fram_get_address(id, 0);
    uint16_t length = object_quantity[id] * object_size[id];
    fm25v_read(&fram.fm25v, addr, read_buffer, length);
}

/*****************************************************************************
*  Name        : fram_ac_param_read
*  Description : read ac param data.
*  Parameter   : void
*  Returns     : None
*****************************************************************************/
uint8_t fram_ac_param_read(uint8_t *read_buffer, uint16_t len)
{
    uint16_t addr = FRAM_AC_PARAM_ADDRESS;

	if(len >= FM25V_BUF_SIZE)
	{
		return 0;
	}
    fm25v_read(&fram.fm25v, addr, read_buffer, len);
	return 1;
}

/*****************************************************************************
*  Name        : fram_ac_param_write
*  Description : write ac param data.
*  Parameter   : void
*  Returns     : None
*****************************************************************************/
uint8_t fram_ac_param_write(uint8_t *write_buffer, uint16_t len)
{
    uint16_t addr = FRAM_AC_PARAM_ADDRESS;

	if(len >= FM25V_BUF_SIZE)
	{
		return 0;
	}
    fm25v_write(&fram.fm25v, addr, write_buffer, len);
	return 1;
}

/*******************************************************************************
**  Name      	: fram_get_address
**  Description : This function get the fram address where the information should be sent.
**  Parameter   : id,
**                item
**  Returns     : uint16_t
*******************************************************************************/
static uint16_t fram_get_address (uint8_t id, uint16_t item)
{
	return (object_base[id] + item * object_size[id]);
}

/**************** END OF FILE *****************************************/
