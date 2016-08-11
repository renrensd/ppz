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
#ifndef _ATOMIC_TUNER_H_
#define _ATOMIC_TUNER_H_ 

/**** Definition of constants ****/
typedef enum
{
    //  5
    AT_TUNER_GET_QUALITY_STEP_1 = REC_OP_STATE_MAX,
    AT_TUNER_GET_QUALITY_STEP_2,
    AT_TUNER_GET_QUALITY_STEP_3,
    AT_TUNER_GET_QUALITY_STEP_4,
    AT_TUNER_GET_QUALITY_STEP_5,

    //  10
    AT_TUNER_GET_QUALITY_STEP_6,
    AT_TUNER_GET_QUALITY_STEP_7,
    AT_TUNER_GET_QUALITY_STEP_8,
}AT_TUNER_GET_QUALITY_STEP_TYPES;

#define REC_FM_RSSI_LOW_MIN         0
#define REC_FM_RSSI_LOW_MAX         19
#define REC_FM_RSSI_LOW_VAL_SIZE    (REC_FM_RSSI_LOW_MAX-REC_FM_RSSI_LOW_MIN+1)


/**** Definition of types ****/ 

typedef struct
{
    U8 status;
    U8 level;
    U8 usn_wam;
    U8 ifcounter;
    U8 device_id;
}ATOMIC_READ_REGISTERS_STRUCT;

typedef struct
{
    U8 mode_sa;
    U8 tuner0;
    U8 tuner1;
    U8 tuner2;
    U8 radio;
    U8 softmute0;
    U8 softmute1;
    U8 softmute2;
    U8 highcut0;
    U8 highcut1;
    U8 highcut2;
    U8 stereo0;
    U8 stereo1;
    U8 stereo2;
    U8 bandwidth;
    U8 level_align;
    U8 am_lna;
}ATOMIC_WRITE_REGISTERS_STRUCT;

typedef struct
{
    U8 min;
    U8 max;
    const U8* tbl;
}REC_NON_LINEAR_TBL_TYPE;

typedef struct
{
    U8 min_id;
    U8 max_id;
    U8 min_val;
    U8 max_val;    
    U16 step;
}REC_LINEAR_SEG_TYPE;

/**** Definition of macros ****/


/**** Declaration of constants ****/


/**** Declaration of variables ****/


/**** Declaration of functions ****/
static BOOL at_tuner_read(U8* buf, U8 size);

static BOOL at_tuner_write(U8* buf, U8 size);

static BOOL at_tuner_read_status(void);

static void at_band_regs_init(U8 band);

#endif /*_ATOMIC_TUNER_H_*/

/****************************** END OF FILE ***************************/

