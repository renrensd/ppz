/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				       *
*				  All Rights Reserved.          					   *
*   Department : R&D SW      									       *
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
#ifndef _ENG_APP_H_
#define _ENG_APP_H_ 

/**** Definition of constants ****/

/**** Definition of types ****/ 



/**** Definition of macros ****/

#define ROM_OF_ONE_BLOCK_SIZE 0x1000
#define ROM_OF_BLOCK_NUM	 0x100
#define ROM_START_ADDRESS	 0x08000000
#define ROM_TOTAL_SIZE		 0x100000	//ROM_OF_ONE_BLOCK_SIZE * ROM_OF_BLOCK_NUM


/**** Declaration of constants ****/


/**** Declaration of variables ****/
extern uint8_t debug_sn_code;

/**** Declaration of functions ****/
extern void eng_app_set_debug_sn_code(uint8_t sn);
extern bool_t eng_app_check_debug_sn(void);

#endif /*_ENG_APP_H_*/

/****************************** END OF FILE ***************************/

