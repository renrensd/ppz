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
#ifndef _TOOLS_IF_H_
#define _TOOLS_IF_H_

/**** Definition of constants ****/


/**** Definition of types ****/ 
typedef struct
{
    U32 bit[8];
} RAM_BIT_BAND_TYPE;


/**** Definition of macros ****/


/**** Declaration of constants ****/


/**** Declaration of variables ****/


/**** Declaration of functions ****/
extern RAM_BIT_BAND_TYPE* get_ram_bitband( U8 *a );


#endif /*_TOOLS_IF_H_*/

/****************************** END OF FILE ***************************/

