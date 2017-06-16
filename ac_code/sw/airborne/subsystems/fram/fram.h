/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				       *
*				  All Rights Reserved.          					   *
*   Department 	: R&D SW      									       *
*   AUTHOR	   	:            										   *
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
#ifndef _FRAM_H_
#define _FRAM_H_

/* Include address and register definition */

/**** Definition of types ****/
typedef struct
{
	uint8_t section;
	uint8_t fram_id_start;
	uint8_t fram_id_end;
} FRAM_DATA_INIT_TYPE;
/**** Definition of macros ****/


/**** Declaration of constants ****/


/**** Declaration of variables ****/


/**** Declaration of functions ****/
static void fram_spi_cb(void);
static uint16_t fram_get_address (uint8_t id, uint16_t item);
static void fram_factory_reset_init(uint16_t items);


#endif /* _FRAM_H_ */

/****************************** END OF FILE ***************************/

