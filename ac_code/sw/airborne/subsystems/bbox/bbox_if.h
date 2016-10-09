/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : R&D SW    									   *
*   AUTHOR	   :             										   *
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
#ifndef _BBOX_IF_H_
#define _BBOX_IF_H_ 

/**** Definition of constants ****/


/**** Definition of types ****/ 

/**** Definition of macros ****/
struct BBOX_INFO
{
	bool_t con_flag;	//connect flag,TRUE:connected
	
	
};

/**** Declaration of constants ****/


/**** Declaration of variables ****/
extern struct BBOX_INFO bbox_info;

/**** Declaration of functions ****/
extern void bbox_task(void);
extern void bbox_init(void);

#endif /*_BBOX_IF_H_*/
/****************************** END OF FILE ***************************/

