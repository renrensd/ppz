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
#ifndef _FIFO_IF_H_
#define _FIFO_IF_H_
#include "types.h"

/**** Definition of constants ****/


/**** Definition of types ****/
typedef struct
{
	U8  *mpFIFOBuffer;
	U16  mFIFOHead;
	U16  mFIFOTail;
	U16  mFIFOSize;
} FIFO_TYPE;

/**** Definition of macros ****/


/**** Declaration of constants ****/


/**** Declaration of variables ****/


/**** Declaration of functions ****/
extern void fifo_io_init( FIFO_TYPE *p_this, U8 *pBuf, U16 nSize);
extern BOOL fifo_input_frame( FIFO_TYPE *p_this , U8 *pFrame, U8 length);
extern BOOL fifo_output_frame( FIFO_TYPE *p_this, U8 *pFrame, U8 *length);
extern BOOL fifo_is_empty(FIFO_TYPE *p_this  );
#endif /*_FIFO_IF_H_*/

/****************************** END OF FILE ***************************/
