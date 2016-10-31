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

/*---Public include files---------------------------------------------*/
#include "std.h"

/*---Private include files--------------------------------------------*/
#include "fram_data.h"

/*===VARIABLES========================================================*/
/*---Global-----------------------------------------------------------*/

/*---Private----------------------------------------------------------*/


/*===CONST========================================================*/
/* project number */
/*this array hasn't put into the eeprom,just used in the DIA module,*/
const uint8_t project_number_array[] =
{
    "EF01_01_" 
};

/* software version */
const uint8_t cl_software_version_array[SW_VERSION_LONGTH+1] =
{
    "31.01_161030_T  "  //for C1
};

/* update flag flag *///CL_SOFTWARE_UPDATE_FLAG
const uint8_t cl_software_update_flag_array[] =
{
    0xFF,0xFF,0xFF,0xFF,  //normal run
    0xFF,0xFF,0xFF,0xFF,  //normal run
    0xFF,0xFF,0xFF,0xFF,  //normal run
    0xFF,0xFF,0xFF,0xFF,  //normal run
//  0xAA,0x55,0xA5,0x5A   //need update
};

const uint8_t cl_fram_init_flag_array[] = 
{
    //0x55,0xAA,0x5A,0xA5//need init the whole eeprom
    0xFF,0xFF,0xFF,0xFF
};

const uint8_t cl_test_array[] = 
{
    10,20,30,40,50,60,70,80,90,100
};
/*===FUNCTIONS========================================================*/

/*---Global-----------------------------------------------------------*/

/*---Private----------------------------------------------------------*/





