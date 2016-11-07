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
#ifndef _ENG_APP_IF_H_
#define _ENG_APP_IF_H_ 

/**** Definition of constants ****/


/**** Definition of types ****/ 


/**** Definition of macros ****/
#define SIZE_OF_AC_VERSION		16
#define SIZE_OF_AC_PROJECT_NAME	16
#define SIZE_OF_HW_VERSION		11
#define SIZE_OF_BBOX_VERSION	16

#define SIZE_OF_PRODUCT_SERIES_NUMBER	12
#define SIZE_OF_12NC_SERIES_NUMBER		5
#define SIZE_OF_MANUFACTURE_DATE		4

enum
{
	ENG_GET_AC_VERSION,
	ENG_GET_AC_CHECKSUM,
	ENG_GET_AC_HW_VERSION,
	ENG_GET_MANUFACTURE_INFO,
};
/**** Declaration of constants ****/

/**** Definition of types ****/ 
typedef struct
{
	uint8_t eep_data;
	uint8_t string[SIZE_OF_HW_VERSION];
}HW_VERSION;

typedef struct
{
	uint8_t cl_product_series_number[SIZE_OF_PRODUCT_SERIES_NUMBER];
	uint8_t cl_12nc_series_number[SIZE_OF_12NC_SERIES_NUMBER];
	uint8_t cl_manufacture_date[SIZE_OF_MANUFACTURE_DATE];
}MANUFACTURE_INFO;

/**** Declaration of variables ****/
extern const uint8_t project_number_array[SIZE_OF_AC_PROJECT_NAME];


/**** Declaration of functions ****/
extern void eng_task(void);
extern void eng_init(void);
extern uint16_t eng_get_ac_checksum(void);
extern uint8_t* eng_get_ac_version(void);
extern uint8_t* eng_get_ac_project_name(void);
extern uint8_t* eng_get_ac_hardware_version(void);
extern uint8_t* eng_get_product_series_number(void);
extern uint8_t* eng_get_12nc_series_number(void);
extern uint8_t* eng_get_manufacture_date(void);
extern void eng_get_ac_identification(uint8_t val);


#endif /*_ENG_APP_IF_H_*/

/****************************** END OF FILE ***************************/

