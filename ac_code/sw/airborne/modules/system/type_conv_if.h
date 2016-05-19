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

#include "..\..\CONFIG\INC\TYPES.H" 
/**** Definition of constants ****/


/**** Definition of types ****/ 


/**** Definition of macros ****/


/**** Declaration of constants ****/


/**** Declaration of variables ****/


/**** Declaration of functions ****/ 
U8 type_conv_bcd_to_hex(U8 bcd);
U8 type_conv_hex_to_bcd(U8 hex);
U8 type_conv_hex_to_ascii(U8 hex);
U8 type_conv_ascii_to_hex(U8 ascii);
void type_conv_u16_to_u8(U16 u16, U8* pU8);
U16 type_conv_u8_to_u16(U8 *pU8);
void byte_to_hex_string(U8 * dst, U8 data);
U8 byte_to_string(U8 * dst, U8 data);

#endif /*_TYPE_CONV_IF_H_*/

/****************************** END OF FILE ***************************/

