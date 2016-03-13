/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : R&D SW      									   *
*   AUTHOR	   : 										   *
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
#ifndef _FIFO_H_
#define _FIFO_H_ 
#include "types.h" 

/**** Definition of constants ****/


/**** Definition of types ****/ 

/**** Definition of macros ****/


/**** Declaration of constants ****/


/**** Declaration of variables ****/


/**** Declaration of functions ****/
static BOOL fifo_is_full(FIFO_TYPE *p_this  );
static BOOL fifo_input_byte( FIFO_TYPE *p_this , unsigned char byte );
static BOOL fifo_output_byte(FIFO_TYPE *p_this, U8 *pdata);
static U16 fifo_current_length(FIFO_TYPE *p_this  );
static U16 fifo_have_enough_space(FIFO_TYPE *p_this , U8 nBytes);


#endif /*_FIFO_IF_H_*/

/****************************** END OF FILE ***************************/
