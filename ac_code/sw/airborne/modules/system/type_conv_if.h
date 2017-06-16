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
#ifndef _TYPE_CONV_IF_H_
#define _TYPE_CONV_IF_H_
#include "std.h"

/**** Definition of constants ****/


/**** Definition of types ****/


/**** Definition of macros ****/


/**** Declaration of constants ****/


/**** Declaration of variables ****/


/**** Declaration of functions ****/
uint8_t type_conv_bcd_to_hex(uint8_t bcd);
uint8_t type_conv_hex_to_bcd(uint8_t hex);
uint8_t type_conv_hex_to_ascii(uint8_t hex);
uint8_t type_conv_ascii_to_hex(uint8_t ascii);
void type_conv_u16_to_u8(uint16_t u16, uint8_t* puint8_t);
uint16_t type_conv_u8_to_u16(uint8_t *puint8_t);
void byte_to_hex_string(uint8_t * dst, uint8_t data);
uint8_t byte_to_string(uint8_t * dst, uint8_t data);
extern uint8_t byte_to_string2(uint8_t * dst, uint8_t data);

#endif /*_TYPE_CONV_IF_H_*/

/****************************** END OF FILE ***************************/

