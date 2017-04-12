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
#define MAX_BBOX_SV_VERSION_LEN  30

/**** Definition of types ****/ 

/**** Definition of macros ****/
struct BBOX_INFO
{
	bool_t con_flag;	//connect flag,TRUE:connected
	bool_t first_con;
	bool_t start_log;
	bool_t gps_time;
	bool_t status;
	bool_t sv_update;
	uint8_t sv_len;
	uint8_t version[MAX_BBOX_SV_VERSION_LEN];
	
};

/**** Declaration of constants ****/


/**** Declaration of variables ****/
extern struct BBOX_INFO bbox_info;

/**** Declaration of functions ****/
extern void bbox_task(void);
extern void bbox_init(void);

#endif /*_BBOX_IF_H_*/
/****************************** END OF FILE ***************************/

