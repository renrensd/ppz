/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : RN R&D SW1      									   *
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
#ifndef _MONITORING_HEIGHT_H_
#define _MONITORING_HEIGHT_H_ 

#include "std.h"
#include "math/pprz_algebra_int.h"
/**** Definition of constants ****/


/**** Definition of types ****/ 
struct Height_Monitor {
   uint32_t sonar_ground_counter;   //inspect counter
   uint32_t baro_ground_counter;
   
   bool_t sonar_ground_check;  //set TRUE to stop check
   bool_t baro_ground_check;
   
   float sonar_aver;    //inspect average
   float baro_aver;

   uint16_t sonar_interval_counter;  //interval out counter
   uint16_t baro_interval_counter;
   uint32_t sonar_update_counter;    //use for check update frequence
   uint32_t baro_update_counter;
   uint8_t  sonar_error_data_counter;
   //uint8_t  baro_error_data_counter;
   uint8_t  sonar_fix_data_counter;
   uint8_t  baro_fix_data_counter;
   //bit1:error_data;  bit2:fix_data  bit3:frequence  bit4:noise  bit5:range
   //x_code once set,need reset by checker,sensors inspect can not change it.
   uint8_t  sonar_code;    
   uint8_t  baro_code;
   uint8_t  sonar_status;  //0:normal  1:dead_distance   2:over_distance   3:error
   bool_t   baro_status;   //1:fail  0:ok

};


extern struct Height_Monitor h_moni;


/**** Definition of macros ****/

/**** Declaration of constants ****/

/**** Declaration of variables ****/


/**** Declaration of functions ****/
extern void height_moni_init(void);
extern uint8_t height_ground_check(void);
extern void height_flight_check(void);
extern void height_ground_reset(void);
extern void height_frequence_check(void);

#endif /*_MONITORING_HEIGHT_H_ */

/****************************** END OF FILE ***************************/



