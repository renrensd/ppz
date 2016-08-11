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

/**** System include files ****/
#include "..\..\CONFIG\INC\CONFIG.H"
#include "..\..\CONFIG\INC\TYPES.H"   

/*---Public include files---------------------------------------------*/
#include "..\..\COMM\INC\IIC_BUS_IF.H"
#include "..\..\EEPROM\INC\EEPROM_CLASS.H"  
#include "..\..\EEPROM\INC\EEPROM_DEF.h"
#include "..\..\EEPROM\INC\EEPROM_IF.h"
#include "..\..\SYSTEM\INC\TIMER_IF.H"

/*---Private include files--------------------------------------------*/
#include "..\INC\REC_TYPES.H"   
#include "..\INC\REC_DATA_IF.H"

#include "..\INC\ATOMIC_TUNER_DEF.H"   
#include "..\INC\ATOMIC_TUNER_IF.H"   
#include "..\INC\ATOMIC_TUNER.H"   

#include "..\INC\tuner_state_def_if.h"
#include "..\INC\tuner_state_def.h"
#include "..\INC\tuner_state_def_if.h"
#include "..\INC\tuner_state_def.h"



/*===VARIABLES========================================================*/

/*---Global-----------------------------------------------------------*/
ATOMIC_WRITE_REGISTERS_STRUCT at_write_regs;
U8 at_tuner_fm_search_lvl;
U8 at_tuner_am_search_lvl;
 
/*---Private----------------------------------------------------------*/
static U8 at_tuner_state;

static const REC_NON_LINEAR_TBL_TYPE rec_fm_rssi_tbl[REC_BAND_MAX][2] = 
{
    {
        {
        REC_FM_RSSI_LOW_MIN,
        REC_FM_RSSI_LOW_MAX,
        rec_fm_rssi_tbl_low
        },
        {
        REC_FM_RSSI_HIGH_MIN,
        REC_FM_RSSI_HIGH_MAX,
        rec_fm_rssi_tbl_high
        }
    },
    
    {
        {
        REC_AM_RSSI_LOW_MIN,
        REC_AM_RSSI_LOW_MAX,
        rec_am_rssi_tbl_low
        },
        {
        REC_AM_RSSI_HIGH_MIN,
        REC_AM_RSSI_HIGH_MAX,
        rec_am_rssi_tbl_high
        }
    },
};

/* step: 3.18ADC ~ 1dB, mutiply 100 for integer calculation*/
static const REC_LINEAR_SEG_TYPE at_rssi_linear_seg[REC_BAND_MAX] =
{
    {20,     40,     100,   163,    318},
    {14,     34,     95,    156,    318},
};

/*===FUNCTIONS========================================================*/

/*---Global-----------------------------------------------------------*/
/*****************************************************************************
*  Name        : at_tuner_create
*  Description : use to init tuner variables
*  Parameter   : void  
*  Returns     : None
*****************************************************************************/
void at_tuner_create(void)
{
    #ifdef TUNER_PARAM_MENU
    at_param_init( CL_ATOMIC_PARAMETERS_FM, at_eep_fm_data, ATOMIC_EEP_FM_MAX, cl_atomic_parameters_fm_array );
    at_param_init( CL_ATOMIC_PARAMETERS_AM, at_eep_am_data, ATOMIC_EEP_AM_MAX, cl_atomic_parameters_am_array );
    #endif    
}

/**************** END OF FILE *****************************************/
