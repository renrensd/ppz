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
 	CLASS    (CL_SOFTWARE_UPDATE_FLAG,	 	1,                  16,			cl_software_update_flag_array)
    CLASS    (CL_SOFTWARE_VERSION,	 		1,             SW_VERSION_LONGTH,			cl_software_version_array) 
 	CLASS    (CL_FRAM_INIT_FLAG,	 	    1,                   4,			cl_fram_init_flag_array)
	CLASS  	 (CL_PRODUCT_SERIES_NUMBER,	 	1,                  12,			cl_ac_serial_number_array)
	CLASS	 (CL_12NC_SERIES_NUMBER,		1,                   5,			cl_no_init_data_array)
	CLASS	 (CL_MANUFACTURE_DATE,			1,                   4,			cl_no_init_data_array)
	CLASS	 (CL_HARDWARE_VERSION,			1,                   1,			cl_ac_hw_version_array)
	CLASS	 (CL_FRAM_RESERVE1,				1,                   1,			cl_no_init_data_array)
	CLASS	 (CL_FRAM_RESERVE2,				1,                   1,			cl_no_init_data_array)

FRAM_END



