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

const uint8_t cl_swdl_mask_array[] =
{
   0xAA,0x55,0xA5,0x5A,
   0xAA,0xAA,0xAA,0xAA,
   0x55,0x55,0x55,0x55,
   0xFF,0xFF,0xFF,0xFF,
};

/*---Private----------------------------------------------------------*/

/*===FUNCTIONS========================================================*/


/*---Global-----------------------------------------------------------*/
/*****************************************************************************
*  Name        : fram_init
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
*  Returns     : 0-Success, other-Fail 
*****************************************************************************/
uint8_t fram_id_write(uint8_t id, uint8_t *write_buffer)
{
	uint16_t addr = fram_get_address(id, 0);
	uint16_t length = object_quantity[id] * object_size[id];
	
	return ( fm25v_write(&fram.fm25v, addr, write_buffer , length) );
}

/*****************************************************************************
*  Name        : fram_write
*  Description : write whole id data.
*  Parameter   : void  
*  Returns     : 0-Success, other-Fail 
*****************************************************************************/
uint8_t fram_write(uint8_t id, uint16_t item, uint8_t *write_buffer)
{
	uint16_t addr = fram_get_address(id, item);
	uint16_t length = object_size[id];
	
	return ( fm25v_write(&fram.fm25v, addr, write_buffer , length) );
}

/*****************************************************************************
*  Name        : fram_read
*  Description : 
*  Parameter   : void  
*  Returns     : 0-Success, other-Fail 
*****************************************************************************/
uint8_t fram_read(uint8_t id, uint16_t item, uint8_t *read_buffer)
{
    uint16_t addr = fram_get_address(id, item);
    uint16_t length = object_size[id];
    return ( fm25v_read(&fram.fm25v, addr, read_buffer, length) );
}

/*****************************************************************************
*  Name        : fram_id_read
*  Description : read whole id data.
*  Parameter   : void
*  Returns     : 0-Success, other-Fail 
*****************************************************************************/
uint8_t fram_id_read(uint8_t id, uint8_t *read_buffer)
{
    uint16_t addr = fram_get_address(id, 0);
    uint16_t length = object_quantity[id] * object_size[id];
    return ( fm25v_read(&fram.fm25v, addr, read_buffer, length) );
}

/*****************************************************************************
*  Name        : fram_ac_param_read
*  Description : read ac param data.
*  Parameter   : void
*  Returns     : 0-Success, other-Fail 
*****************************************************************************/
uint8_t fram_ac_param_read(uint8_t *read_buffer, uint16_t len)
{
    uint16_t addr = FRAM_AC_PARAM_ADDRESS;

    return ( fm25v_read(&fram.fm25v, addr, read_buffer, len) );
}

/*****************************************************************************
*  Name        : fram_ac_param_write
*  Description : write ac param data.
*  Parameter   : void
*  Returns     : 0-Success, other-Fail 
*****************************************************************************/
uint8_t fram_ac_param_write(uint8_t *write_buffer, uint16_t len)
{
    uint16_t addr = FRAM_AC_PARAM_ADDRESS;

   return ( fm25v_write(&fram.fm25v, addr, write_buffer, len) );
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

#ifdef UPGRADE_OPTION
/***********************************************************************
*  Name        : fram_write_swdl_mask
*  Description :          
*  Parameter   : none
*  Returns     : 0-Success, other-Fail 
***********************************************************************/
uint8_t fram_write_swdl_mask(void)
{
    uint8_t buf[16]=
    {
       0xAA,0x55,0xA5,0x5A,
       0xAA,0xAA,0xAA,0xAA,
       0x55,0x55,0x55,0x55,
       0xFF,0xFF,0xFF,0xFF,
    };
    
    return ( fram_write(CL_SOFTWARE_UPDATE_FLAG, 0, buf) );
}

/***********************************************************************
*  Name        : fram_read_swdl_mask
*  Description :          
*  Parameter   : none
*  Returns     : 0-Success, other-Fail 
***********************************************************************/
uint8_t fram_read_swdl_mask (uint8_t* pBlockData)
{
    return ( fram_read(CL_SOFTWARE_UPDATE_FLAG, 0, pBlockData) );
}

/*******************************************************************************
**  FUNCTION      : fram_update_is_available                                         
**  DESCRIPTION   :          
**  PARAMETERS    : void
**  RETURN        : void                                                          
*******************************************************************************/
bool_t fram_update_is_available(void)
{
    uint8_t swdl_mask[16];
    bool_t retval = FALSE;
    uint8_t i;
    bool_t swdl_get_fram_error = FALSE;

    for(i=0; i<3; i++)
    {
        swdl_get_fram_error = FALSE;               
        if( fram_read_swdl_mask(swdl_mask) == 0)
        {
            break;
        }
        else
        {
            swdl_get_fram_error = TRUE;
            delay_us(200);                 
        }
    }
    
    if(swdl_get_fram_error == FALSE)
    {
        if(memcmp(swdl_mask,cl_swdl_mask_array,16) == 0)  
        {//these two variables are the same
            retval = TRUE;
        }              
    }

    return (retval);
}
#endif	/* UPGRADE_OPTION */
/**************** END OF FILE *****************************************/

