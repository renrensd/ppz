/***********************************************************************
*   Copyright (C) Huizhou Desay SV Automotive Co., Ltd.				   *
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
#ifndef _TYPES_H_
#define _TYPES_H_

/**** Definition of constants ****/


/**** Definition of types ****/
#ifndef TRUE
#define TRUE         1
#endif

#ifndef FALSE
#define FALSE        0
#endif

typedef signed char     S8;
typedef unsigned char   U8;
typedef short           S16;
typedef unsigned short  U16;
typedef int             S32;
typedef unsigned int    U32;
typedef long long       S64;
typedef unsigned long long U64;
typedef unsigned char   BIT;
typedef unsigned char   BOOL;

typedef void (*OP_FUNC) (void*);
typedef void (*TIMER_FUNC) (U16 param);
typedef void (*VOID_FUNC) (void);


/**** Definition of macros ****/

#endif /*_TYPES_H_*/

/****************************** END OF FILE ***************************/

