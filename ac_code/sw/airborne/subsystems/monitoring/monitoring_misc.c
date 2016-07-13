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

/**** System include files ****/
#include "subsystems/monitoring/monitoring_misc.h"
#include "subsystems/monitoring/monitoring.h"
#include "firmwares/rotorcraft/nav_flight.h"
//#include "subsystems/electrical.h"
//#include "modules/energy/bat_manager.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "subsystems/gps.h"
#include "subsystems/rc_nav/rc_nav_xbee.h"
#include "subsystems/mission/gcs_nav_xbee.h"
#include "subsystems/ops/ops_app_if.h"
#include "subsystems/ops/ops_app.h"
#include "subsystems/mission/task_process.h"
#include "state.h"


/*===VARIABLES========================================================*/
#define GROUND_LIMIT_ELECTRICITY  2000   //unit=mAh
#define GROUND_LIMIT_CURRENT  5000    //unit=mA
#define GROUND_LIMIT_TEMPER   0     //

#define FLIGHT_LIMIT_CURRENT  500000    //unit=mA
#define HOME_ELEC_RATIO   20

#define MAX_GROUND_ATT  0.175  //about 10deg
#define MAX_GROUND_RATE  0.02   //about 1deg/s
#define MAX_GROUND_INS_S  1.0  //0.8
#define MAX_GROUND_INS_A  0.5
#define MAX_GROUND_INS_Z  0.6

#define MAX_FLIGHT_TIME 510   //13min

#define LESS_RES_CAP 5  //5%

static uint8_t ahrs_ground_check(void);
static uint8_t ins_ground_check(void);



/*===FUNCTIONS========================================================*/

/***********************************************************************
* FUNCTIONS   : initial functions
* DESCRIPTION : 
* INPUTS      : 
* RETURN      : none
***********************************************************************/
void misc_moni_init(void)
{
	rc_lost = TRUE;
}

/***********************************************************************
* FUNCTIONS   : battery ground check
* DESCRIPTION :  only on ground check once
* INPUTS      : 
* RETURN      : none
***********************************************************************/
uint8_t battery_ground_check(void)
{
	//first to sure battery manager is ok
	uint8_t check_code=0;
	#if 0
	if( electrical.energy <GROUND_LIMIT_ELECTRICITY)
		check_code |=0x01;
	if( electrical.current >GROUND_LIMIT_CURRENT )
		check_code |=0x02;
    #endif
    return check_code;
}

/***********************************************************************
* FUNCTIONS   : battery flight check
* DESCRIPTION : ept_ms could by recover info close
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void battery_flight_check(void)
{	
	bool_t flag_trigger = 0;
#if 0
	bool_t flag_trigger=0;
	if( electrical.bat_low || (sqrt(distance2_to_takeoff)*HOME_ELEC_RATIO) >electrical.energy  )
	{   //hover 10s,back home
	    flag_trigger=1;   //record error trigger
	    set_except_mission(BAT_LOW,TRUE,FALSE, TRUE,5, TRUE,FALSE,2);	
		//need give special alter to RC and GCS
                  #if TEST_MSG
		bat_flight=1;
		#endif
	}
#else
	if( autopilot_flight_time >MAX_FLIGHT_TIME)
	{   //hover 10s,back home
	    flag_trigger = 1;   //record error trigger
	    set_except_mission(BAT_LOW,TRUE,FALSE, FALSE,0, TRUE,FALSE,2);	
		//need give special alter to RC and GCS
                  #if TEST_MSG
		bat_flight=1;
		#endif
	}
#endif
	
        /*
	if( electrical.bat_critical )
	{   //only alert
	    flag_trigger=1;   //record error trigger
	    set_except_mission(BAT_CRITICAL,TRUE,FALSE, FALSE,0, FALSE,TRUE,3);	
		#if TEST_MSG
		bat_flight=2;
		#endif
	}
	*/
   /*
	if( electrical.current >FLIGHT_LIMIT_CURRENT )
	{   //only alert
	    flag_trigger=1;   //record error trigger
	    set_except_mission(BAT_OTHER,TRUE,FALSE, FALSE,0, FALSE,FALSE,3);	
		#if TEST_MSG
		bat_flight=3;
		#endif
	}
   */
    if(!flag_trigger)
    {
		//recover
		em[BAT_LOW].active =0;
		em[BAT_LOW].finished =0;
		em[BAT_CRITICAL].active =0;
		em[BAT_CRITICAL].finished =0;
		em[BAT_OTHER].active=0;
		em[BAT_OTHER].finished=0;
    }
	
}

/***********************************************************************
* FUNCTIONS   : gps flight check
* DESCRIPTION : ept_ms could close by gps recover
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void gps_flight_check(void) 
{  
    if( GpsFixValid() )
	{
		if( gps.stable )
        {   //could be recovered
			em[GPS_ACC].active =0;
			em[GPS_ACC].finished =0;
			em[GPS_LOST].active =0;
			em[GPS_LOST].finished =0;
			#if TEST_MSG
			gps_flight=0;
			#endif
		}
		else
		{   
			if(gps.pacc <2000)  
			{
				em[GPS_ACC].alert_grade = 2;
			}
		    else  
			{
				em[GPS_ACC].alert_grade = 3;
		    }
			set_except_mission(GPS_ACC,TRUE,FALSE, TRUE,0xFF, FALSE,FALSE,2);	
			#if TEST_MSG
			gps_flight=1;
			#endif
		}
    }
	else
	{
		set_except_mission(GPS_LOST,TRUE,FALSE, TRUE,0xFF, FALSE,FALSE,3);	
		#if TEST_MSG
		gps_flight=2;
		#endif
	}
	
}

/***********************************************************************
* FUNCTIONS   : ops flight check
* DESCRIPTION : ept_ms could close by ops recover
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void ops_flight_check(void) 
{
	if(ops_info.con_flag == OPS_NOT_CONNECT)  //ops lost,maybe spray is open
	{
		set_except_mission(OPS_LOST,TRUE,FALSE, TRUE,0xFF, FALSE,FALSE,2);	
	}
	else
	{
		em[OPS_LOST].active = FALSE;
		em[OPS_LOST].finished = FALSE;
	}
/*	
	if(ops_info.res_cap <LESS_RES_CAP )  //no pesticide and open spray
	{
		set_except_mission(OPS_EMPTY,TRUE,FALSE, TRUE,0xFF, FALSE,FALSE,1);	
	}
	else
	{   
		em[OPS_EMPTY].active = FALSE;
		em[OPS_EMPTY].finished = FALSE;
	}
*/
}

/***********************************************************************
* FUNCTIONS   : rc flight check
* DESCRIPTION : ept_ms could close by rc recover
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void rc_communication_flight_check(void)
{
	if(rc_lost && flight_mode == nav_rc_mode )
	{
		set_except_mission(RC_COM_LOST,TRUE,FALSE, TRUE,0xFF, FALSE,FALSE,3);	
	}
	else
	{
		em[RC_COM_LOST].active = FALSE;
		em[RC_COM_LOST].finished = FALSE;
	}	
}


/***********************************************************************
* FUNCTIONS   : gcs flight check
* DESCRIPTION : ept_ms could close by gcs recover
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void gcs_communication_flight_check(void)
{
	if(gcs_lost && flight_mode == nav_gcs_mode )
	{
		set_except_mission(GCS_COM_LOST,TRUE,FALSE, TRUE,0xFF, FALSE,FALSE,3);	
	}
	else
	{
		em[GCS_COM_LOST].active = FALSE;
		em[GCS_COM_LOST].finished = FALSE;
	}
	
}

void lift_flight_check(void)
{
}

void task_running_check(void)
{
	if(task_error_state==TASK_NORMAL)
	{
		em[TASK_NO].active = FALSE;
		em[TASK_PARSE].active = FALSE;
		em[TASK_BREAK].active = FALSE;
	}
	else if(task_error_state==TASK_PARSE_ERROR)
	{
		set_except_mission(TASK_PARSE,TRUE,FALSE, TRUE,0xFF, FALSE,FALSE,2);	
	}
	else if(task_error_state==TASK_RUN_OVER)
	{
		set_except_mission(TASK_NO,TRUE,FALSE, TRUE,0xFF, FALSE,FALSE,1);	
	}
	else if(task_error_state==TASK_INTERRUPT)
	{
		set_except_mission(TASK_BREAK,TRUE,FALSE, FALSE,0, FALSE,FALSE,0);	
	}
}

void mode_convert_check(void)
{
	if(mode_convert_a2m)
	{
		set_except_mission(MODE_CONVERT_A2M,TRUE,FALSE, FALSE,0, FALSE,FALSE,0);	
	}
	else
	{
		em[MODE_CONVERT_A2M].active = FALSE;
	}
}

/***********************************************************************
* FUNCTIONS   : autopilot ground check
* DESCRIPTION : part of ground check,after other modules pass needed check 
                the information fuse
* INPUTS      : none
* RETURN      : none
***********************************************************************/
uint8_t autopilot_ground_check(void)
{
	//uint8_t check_code=0;
	if( !ahrs_ground_check() ) return 0;
	if( !ins_ground_check() ) return 0;
	else return 1;	
}

static uint8_t ahrs_ground_check(void)
{
	if( !stateIsAttitudeValid() )  return 0;
	if( fabs(stateGetNedToBodyEulers_f()->phi) >MAX_GROUND_ATT)  return 0;
	if( fabs(stateGetNedToBodyEulers_f()->theta) >MAX_GROUND_ATT)  return 0;
	if( fabs(stateGetBodyRates_f()->p) >MAX_GROUND_RATE)  return 0;
	if( fabs(stateGetBodyRates_f()->q) >MAX_GROUND_RATE)  return 0;
	if( fabs(stateGetBodyRates_f()->r) >MAX_GROUND_RATE)  return 0;
	
	else return 1;
}

static uint8_t ins_ground_check(void)
{
	if (!state.ned_initialized_i) return 0;
 	if( fabs(stateGetSpeedEnu_f()->x) >MAX_GROUND_INS_S)  return 0;
	if( fabs(stateGetSpeedEnu_f()->y) >MAX_GROUND_INS_S)  return 0;
	if( fabs(stateGetSpeedEnu_f()->z) >MAX_GROUND_INS_S)  return 0;
	if( fabs(stateGetAccelNed_f()->x) >MAX_GROUND_INS_A)  return 0;
	if( fabs(stateGetAccelNed_f()->y) >MAX_GROUND_INS_A)  return 0;
	if( fabs(stateGetAccelNed_f()->z) >MAX_GROUND_INS_A)  return 0;
	if( fabs(stateGetPositionEnu_f()->z) >MAX_GROUND_INS_Z)  return 0;

	else return 1;
}


/**************** END OF FILE *****************************************/


