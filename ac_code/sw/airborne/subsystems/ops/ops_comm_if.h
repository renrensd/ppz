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
#ifndef _OPS_COMM_IF_H_
#define _OPS_COMM_IF_H_ 
#include "../../modules/system/types.h"

/**** Definition of constants ****/

/**** Definition of macros ****/

/**** Declaration of constants ****/


/**** Declaration of variables ****/

/**** Declaration of functions ****/
extern void ops_comm_create(void);
extern BOOL ops_comm_send_frame(U8 type, U16 id, U8 nArgs, U8 const *pArg);
extern void ops_comm_send_polling(void);
extern void ops_comm_read_polling(void);
extern void ops_comm_down(void);

#endif /*_OPS_COMM_IF_H_*/
/****************************** END OF FILE ***************************/

