/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   	   *
*				  All Rights Reserved.          					   *
*   Department : R&D SW   									           *
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

*=====================================================================*/

/**** System include files ****/
#include "std.h"
#include "modules/system/type_conv_if.h"
#include "modules/system/tools_if.h"

/*---Public include files---------------------------------------------*/
#include "modules/system/timer_if.h"
#include "modules/system/timer_class.h"
#include "modules/system/timer_def.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/fram/fram_if.h"
#include "subsystems/fram/fram_data.h"

#include "subsystems/fram/fram_class.h"
#include "subsystems/fram/fram_def.h"

/*---Private include files--------------------------------------------*/
#include "eng_app.h"
#include "eng_app_if.h"

/*===VARIABLES========================================================*/

/*---Global-----------------------------------------------------------*/

 
/*---Private----------------------------------------------------------*/
static uint8_t  eol_state;
static uint8_t  ac_version[SIZE_OF_AC_VERSION + SIZE_OF_AC_PROJECT_NAME];
static HW_VERSION  ac_hardware_version;
static MANUFACTURE_INFO  manufacture_info;
static uint32_t  checksum_result;
static uint8_t *checksum_ptr;
static uint16_t counter_rom_segment; 
static uint8_t eng_con_flag = 0;

//the following only used before first connect completed.

/*===FUNCTIONS========================================================*/

/*---Global-----------------------------------------------------------*/
void eng_task(void)
{ 
	tm_stimulate(TIMER_TASK_ENG);

	if(eng_con_flag == 1)	//write
	{
		eng_con_flag = 0;
    	fram_write(CL_PRODUCT_SERIES_NUMBER, 0, cl_ac_serial_number_array);
		fram_write(CL_HARDWARE_VERSION, 0, cl_ac_hw_version_array);
	}
	else if(eng_con_flag == 2)	//read
	{
		eng_con_flag = 0;
		eng_get_ac_identification(ENG_GET_AC_VERSION);
    	eng_get_ac_identification(ENG_GET_AC_HW_VERSION);
		eng_get_ac_identification(ENG_GET_MANUFACTURE_INFO);
		eng_get_ac_identification(ENG_GET_AC_CHECKSUM);
	}
}

/***********************************************************************
* FUNCTION    : eng_init
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void eng_init(void)
{
	//eng_get_ac_identification(ENG_GET_AC_CHECKSUM);
	//eng_get_ac_identification(ENG_GET_AC_VERSION);
}

/***********************************************************************
* FUNCTION    : eng_get_ac_checksum
* DESCRIPTION : Return ac checksum after it is requeted correctly.  
* INPUTS      : Message Pointer
* RETURN      : uint16_t
***********************************************************************/
uint16_t eng_get_ac_checksum(void)
{
	return 	checksum_result;
}

/***********************************************************************
* FUNCTION    : eng_get_ac_version
* DESCRIPTION : Return ac version after it is requeted correctly.  
* INPUTS      : void
* RETURN      : uint8_t*
***********************************************************************/
uint8_t* eng_get_ac_version(void)
{
	return 	&ac_version[0];
}

/***********************************************************************
* FUNCTION    : eng_get_ac_project_name
* DESCRIPTION : Return ac project name.  
* INPUTS      : void
* RETURN      : uint8_t*
***********************************************************************/
uint8_t* eng_get_ac_project_name(void)
{
	return 	(uint8_t*)&project_number_array[0];
}

/***********************************************************************
* FUNCTION    : eng_get_ac_hardware_version
* DESCRIPTION : Return ac hardware version after it is requeted correctly.  
* INPUTS      : void
* RETURN      : uint8_t
***********************************************************************/
uint8_t* eng_get_ac_hardware_version(void)
{
	uint8_t i;
	uint8_t temp_eep_read;
	uint8_t *version_ptr;

	i = 0;
	version_ptr = &ac_hardware_version.string[0];
	temp_eep_read = ac_hardware_version.eep_data;

	version_ptr[i++] = 'H';
	version_ptr[i++] = 'W';
	i += byte_to_string2(version_ptr + i, temp_eep_read);
	version_ptr[i++] = 0;
	return version_ptr;
}
	
/***********************************************************************
* FUNCTION    : eng_get_product_series_number
* DESCRIPTION : Return product_series_number after it is requeted correctly.  
* INPUTS      : void
* RETURN      : uint8_t*
***********************************************************************/
uint8_t* eng_get_product_series_number(void)
{
	return 	&manufacture_info.cl_product_series_number[0];
}

/***********************************************************************
* FUNCTION    : eng_get_12nc_series_number
* DESCRIPTION : Return 12nc_series_number after it is requeted correctly.  
* INPUTS      : void
* RETURN      : uint8_t*
***********************************************************************/
uint8_t* eng_get_12nc_series_number(void)
{
	return 	&manufacture_info.cl_12nc_series_number[0];
}

/***********************************************************************
* FUNCTION    : eng_get_manufacture_date
* DESCRIPTION : Return manufacture_date after it is requeted correctly.  
* INPUTS      : void
* RETURN      : uint8_t*
***********************************************************************/
uint8_t* eng_get_manufacture_date(void)
{
	return 	&manufacture_info.cl_manufacture_date[0];
}

/***********************************************************************
* FUNCTION    : eng_get_ac_identification
* DESCRIPTION : Handle AC identification acqucition.  
* INPUTS      : Message Pointer
* RETURN      : void
***********************************************************************/
void eng_get_ac_identification(uint8_t val)
{
	uint8_t arg[SIZE_OF_AC_VERSION + SIZE_OF_AC_PROJECT_NAME];
	uint8_t *mptr;
	uint8_t i;
	uint8_t j;
	uint8_t length;
	
	if(ENG_GET_AC_VERSION == val)
	{
		//request version
		fram_read((uint8_t)CL_SOFTWARE_VERSION,0,&ac_version[0]);
		
		mptr = eng_get_ac_project_name();
		length = get_size_of_string(mptr);
		if(length > SIZE_OF_AC_PROJECT_NAME)
		{
			length = SIZE_OF_AC_PROJECT_NAME;
		}
		for(i = 0, j = 0; j < length; i++, j++)
		{
			arg[i]	= mptr[j];
		}
		mptr = eng_get_ac_version();
		for(i = length, j = 0; j < SIZE_OF_AC_VERSION; i++, j++)
		{
			arg[i]	= mptr[j];
		}

		for(i=0; i < length+SIZE_OF_AC_VERSION; i++)
		{
			ac_version[i] = arg[i];
		}
		
	}
	else if(ENG_GET_AC_CHECKSUM == val)
	{
		//request checksum
		counter_rom_segment = 0;
		tm_create_timer((uint8_t)TIMER_GET_CAL_AC_CHECKSUM,12 MSECONDS,(uint8_t)TIMER_PERIODIC,0);		
	}
	else if(ENG_GET_AC_HW_VERSION == val)
	{
		//request hardware version
		fram_read((uint8_t)CL_HARDWARE_VERSION,0,&ac_hardware_version.eep_data);

		mptr = eng_get_ac_hardware_version();
		for(i=0; i < SIZE_OF_HW_VERSION; i++)
		{
			ac_hardware_version.string[i] = mptr[i];
		}
	}
	else if(ENG_GET_MANUFACTURE_INFO == val)
	{
		//request product_series_number
		fram_read((uint8_t)CL_PRODUCT_SERIES_NUMBER,0,&manufacture_info.cl_product_series_number[0]); 
	}
}

/***********************************************************************
* FUNCTION    : eng_ac_checksum_calculation
* DESCRIPTION : Calculate flash bin file checksum.  
* INPUTS      : uint16_t param
* RETURN      : void
***********************************************************************/
void eng_ac_checksum_calculation(void)
{
	uint32_t i;
      
	if(counter_rom_segment >= ROM_OF_BLOCK_NUM)
	{
		counter_rom_segment = 0;
		tm_kill_timer((uint8_t)TIMER_GET_CAL_AC_CHECKSUM);		  
		
		return;
	}
	
    if(counter_rom_segment == 0)
    {  
		checksum_ptr = (uint8_t *)ROM_START_ADDRESS;
	  	checksum_result = 0;
    }

    for(i = ROM_OF_ONE_BLOCK_SIZE; i > 0; i--)
    {  
		checksum_result += *checksum_ptr;
		checksum_ptr++;
    }
    counter_rom_segment++;
}

/**************** END OF FILE *****************************************/
