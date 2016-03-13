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
#ifndef _FIFO_U16_IF_H_
#define _FIFO_U16_IF_H_ 

/**** Definition of constants ****/


/**** Definition of types ****/ 
typedef struct 
{
    U16  *mpFIFOBuffer;
    U16  mFIFOHead;   
    U16  mFIFOTail;  
    U16  mFIFOSize;
} FIFO_U16_TYPE;

/**** Definition of macros ****/


/**** Declaration of constants ****/


/**** Declaration of variables ****/


/**** Declaration of functions ****/
extern void fifo_init_u16(FIFO_U16_TYPE *this, U16 *pBuf, U16 nSize);
extern BOOL fifo_input_u16(FIFO_U16_TYPE *this, U16 input_data);
extern BOOL fifo_output_u16(FIFO_U16_TYPE *this, U16 *pdata);
extern BOOL fifo_u16_is_empty(FIFO_U16_TYPE *this);

#endif /*_FIFO_U16_IF_H_*/

/****************************** END OF FILE ***************************/

