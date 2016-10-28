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

/**** Definition of constants ****/


/**** Definition of types ****/ 


/**** Definition of macros ****/
/*				class				        quantity	        size		 previous class*/
FRAM_START
    CLASS    (CL_SOFTWARE_VERSION,	 		1,             SW_VERSION_LONGTH,			cl_software_version_array) 
 	CLASS    (CL_SOFTWARE_UPDATE_FLAG,	 	3,                  16,			cl_software_update_flag_array)
 	CLASS    (CL_FRAM_INIT_FLAG,	 	    1,                   4,			cl_eeprom_init_flag_array)
FRAM_END



