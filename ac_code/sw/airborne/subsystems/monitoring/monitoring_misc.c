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
#include "subsystems/electrical.h"
#include "subsystems/gps.h"
#include "subsystems/rc_nav/rc_nav_xbee.h"
#include "subsystems/ops/ops_app_if.h"
#include "subsystems/ops/ops_app.h"

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
	rc_lost=TRUE;
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
	#if 0
	bool_t flag_trigger=0;
	if( electrical.bat_low || (sqrt(distance2_to_home)*HOME_ELEC_RATIO) >electrical.energy  )
	{   //hover 10s,back home
	    flag_trigger=1;   //record error trigger
	    set_except_misssion(BAT_LOW,TRUE,FALSE, TRUE,10, TRUE,FALSE,2);	
        #if TEST_MSG
		bat_flight=1;
		#endif
	}

	if( electrical.bat_critical )
	{   //only alert
	    flag_trigger=1;   //record error trigger
	    set_except_misssion(BAT_CRITICAL,TRUE,FALSE, FALSE,0, FALSE,FALSE,3);	
		#if TEST_MSG
		bat_flight=2;
		#endif
	}

	if( electrical.current >FLIGHT_LIMIT_CURRENT )
	{   //only alert
	    flag_trigger=1;   //record error trigger
	    set_except_misssion(BAT_OTHER,TRUE,FALSE, FALSE,0, FALSE,FALSE,3);	
		#if TEST_MSG
		bat_flight=3;
		#endif
	}

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
	#endif
	
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
        {   //recover
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
			if(gps.pacc <2000)  em[GPS_ACC].alert_grade =2;
		    else  em[GPS_ACC].alert_grade =3;
			set_except_misssion(GPS_ACC,TRUE,FALSE, TRUE,0xFF, FALSE,FALSE,2);	
			#if TEST_MSG
			gps_flight=1;
			#endif
		}
    }
	else
	{
		set_except_misssion(GPS_LOST,TRUE,FALSE, TRUE,0xFF, FALSE,FALSE,3);	
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
		set_except_misssion(OPS_LOST,TRUE,FALSE, TRUE,10, TRUE,FALSE,2);	
	}
	else if(ops_info.res_cap <LESS_RES_CAP )  //no pesticide and open spray
	{
		set_except_misssion(OPS_EMPTY,TRUE,FALSE, TRUE,0xFF, FALSE,FALSE,1);	
	}
	else
	{   //normal.maybe recover
		em[OPS_EMPTY].active=FALSE;
		em[OPS_EMPTY].finished=FALSE;
		em[OPS_LOST].active=FALSE;
		em[OPS_LOST].finished=FALSE;
	}
}

/***********************************************************************
* FUNCTIONS   : rc flight check
* DESCRIPTION : ept_ms could close by rc recover
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void rc_flight_check(void)
{
	if(rc_lost)
	{
		set_except_misssion(RC_KEY_LOST,TRUE,FALSE, TRUE,10, TRUE,FALSE,3);	
	}
	else if(0) //sign,can not do current
	{
		set_except_misssion(RC_KEY_SIGNAL,TRUE,FALSE, TRUE,0xFF, FALSE,FALSE,2);	
	}
	else
	{
		em[RC_KEY_LOST].active=FALSE;
		em[RC_KEY_LOST].finished=FALSE;
		em[RC_KEY_SIGNAL].active=FALSE;	
		em[RC_KEY_SIGNAL].finished=FALSE;
	}
	
}

void lift_flight_check(void)
{
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
/*
  float phi=fabs(stateGetNedToBodyEulers_f()->phi);
  if( phi <MAX_GROUND_ATT)  
  {;}
  else
  {return 0;}
  
  float theta=fabs(stateGetNedToBodyEulers_f()->theta);
  if( theta <MAX_GROUND_ATT)  
  {;}
  else
  {return 0;}
  
  float p=fabs(stateGetBodyRates_f()->p);
  if( p <MAX_GROUND_RATE)  
  {;}
  else
  {return 0;}

  float q=fabs(stateGetBodyRates_f()->q);
  if( q <MAX_GROUND_RATE)  
  {;}
  else
  {return 0;}

  float r=fabs(stateGetBodyRates_f()->r);
  if( r <MAX_GROUND_RATE)  
  {;}
  else
  {return 0;}
  
  return 1;
  */
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
/*
  float sx=fabs(stateGetSpeedEnu_f()->x);
  if( sx <MAX_GROUND_INS_S)  
  {;}
  else
  {return 0;}
  
  float sy=fabs(stateGetSpeedEnu_f()->y);
  if( sy <MAX_GROUND_INS_S)  
  {;}
  else
  {return 0;}
  
  float sz=fabs(stateGetSpeedEnu_f()->z);
  if( sz <MAX_GROUND_INS_S)  
  {;}
  else
  {return 0;}

  float ax=fabs(stateGetAccelNed_f()->x);
  if( ax <MAX_GROUND_INS_A)  
  {;}
  else
  {return 0;}

  float ay=fabs(stateGetAccelNed_f()->y);
  if( ay <MAX_GROUND_INS_A)  
  {;}
  else
  {return 0;}

  float az=fabs(stateGetAccelNed_f()->z);
  if( az <MAX_GROUND_INS_A)  
  {;}
  else
  {return 0;}

  float pz=fabs(stateGetPositionEnu_f()->z);
  if( pz <MAX_GROUND_INS_Z)  
  {;}
  else
  {return 0;}
  
  return 1;
  */
}


/**************** END OF FILE *****************************************/


