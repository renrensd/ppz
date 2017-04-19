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
//#include "modules/energy/bat_manager.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "subsystems/gps.h"
#include "subsystems/rc_nav/rc_nav_xbee.h"
#include "subsystems/mission/gcs_nav_xbee.h"
#include "subsystems/ops/ops_app_if.h"
#include "subsystems/ops/ops_app.h"
#include "subsystems/mission/task_process.h"

#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/stabilization.h"

#include "state.h"

#include "subsystems/ahrs/ahrs_float_mlkf.h"
#include "subsystems/ins/ins_int.h"
#include "subsystems/fram/fram_if.h"
#include "modules/gps/gps2_ublox.h"
#include "modules/mag_cali/mag_cali.h"


/*===VARIABLES========================================================*/
#define GROUND_LIMIT_ELECTRICITY  2000   //unit=mAh
#define GROUND_LIMIT_CURRENT  5000    //unit=mA
#define GROUND_LIMIT_TEMPER   0     //

#define FLIGHT_LIMIT_CURRENT  500000    //unit=mA
#define HOME_ELEC_RATIO   20

#define MAX_GROUND_ATT  0.175  //about 10deg
#define MAX_GROUND_RATE  0.02   //about 1deg/s
#define MAX_GROUND_INS_S  0.8
#define MAX_GROUND_INS_A  0.5
#define MAX_GROUND_INS_Z  0.6

#define MAX_FLIGHT_TIME 780   //13min
#define BAT_LIMIT_VOL  432   //unit:0.1v
#define BAT_LIMIT_CAP  20  //unit:percent

#define LESS_RES_CAP 3  //5%

static uint8_t ahrs_ground_check(void);
static uint8_t ins_ground_check(void);
static bool_t yaw_command_monitor(void);
static bool_t thrust_command_monitor(void);
static bool_t lift_lost_detect(void);



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

uint8_t board_ground_check(void)
{
	if(fram_error.data_wrong || fram_error.read_data_fail || fram_error.write_data_fail)
	{
		return 1;  //signed fram wrong
	}
	else
	{
		return 0;
	}
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
	static uint8_t bat_counter = 0;

	if( (electrical.vsupply)&&(ops_info.con_flag==OPS_CONNECTED)&&((ops_info.sys_error&0x01)==0) )
	{
		//if( electrical.bat_low || (sqrt(distance2_to_takeoff)*HOME_ELEC_RATIO) >electrical.energy  )
		if( (electrical.vsupply < BAT_LIMIT_VOL) || (electrical.remain_percent < BAT_LIMIT_CAP) )
		{   
			bat_counter++;
			if(bat_counter > 3)
			{
				/*hover 10s,back home*/
			    flag_trigger = 1;   //record error trigger
			    set_except_mission(BAT_LOW,TRUE,FALSE, TRUE,10, TRUE,FALSE,2);	
				//need give special alter to RC and GCS
		        #if TEST_MSG
				bat_flight=1;
				#endif
				bat_counter = 3;
			}
		}
		else
		{
			bat_counter = 0;
		}
	}
	else
	{
		if( autopilot_flight_time >MAX_FLIGHT_TIME)
		{   //hover 10s,back home
		    flag_trigger = 1;   //record error trigger
		    set_except_mission(BAT_LOW,TRUE,FALSE, FALSE,0, TRUE,FALSE,2);	
			//need give special alter to RC and GCS
	        #if TEST_MSG
			bat_flight=1;
			#endif
		}
	}
    if(!flag_trigger)
    {
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
	static uint16_t count = 0;
	static float diff_sum = 0;
	static bool_t diff_err = FALSE;

	if(GpsFixValid())
	{
		if (ins_int_is_rtk_pos_xy_valid() && ins_int_is_rtk_pos_z_valid())
		{   //could be recovered
			em[GPS_ACC].active = 0;
			em[GPS_ACC].finished = 0;
			em[GPS_LOST].active = 0;
			em[GPS_LOST].finished = 0;
#if TEST_MSG
			gps_flight = 0;
#endif
		}
		else
		{
			if (gps.pacc < 6000)
			{
				em[GPS_ACC].alert_grade = 2;
			}
			else
			{
				em[GPS_ACC].alert_grade = 3;
			}
			set_except_mission(GPS_ACC, TRUE, FALSE, TRUE, 0xFF, FALSE, FALSE, 2);
#if TEST_MSG
			gps_flight = 1;
#endif
		}
	}
	else
	{
		set_except_mission(GPS_LOST, TRUE, FALSE, TRUE, 0xFF, FALSE, FALSE, 3);
#if TEST_MSG
		gps_flight = 2;
#endif
	}

#ifdef USE_GPS_HEADING
	if(ahrs_mlkf_is_rtk_heading_valid())
	{
		if(!diff_err)
		{
			em[GPS_HEADING].active = 0;
			em[GPS_HEADING].finished = 0;
			force_use_heading_redundency(FALSE);
		}

		if ( mag_cali.cali_ok && ground_check_pass )
		{
			diff_sum += fabsf(ahrs_mlkf.diff_heading);
			if (++count >= (10))	// 2s
			{
				diff_sum /= (float) (10);
				if (diff_sum > 20)
				{
					diff_err = TRUE;
					set_except_mission(GPS_HEADING, TRUE, FALSE, TRUE, 0xFF, FALSE, FALSE, 2);
					force_use_heading_redundency(TRUE);
				}
				else
				{
					diff_err = FALSE;
				}
				count = 0;
				diff_sum = 0;
			}
		}
	}
	else
	{
		set_except_mission(GPS_HEADING,TRUE,FALSE, TRUE,0xFF, FALSE,FALSE,2);
	}
#endif

#ifdef USE_GPS2_UBLOX
	if(gps2.p_stable)
	{
		em[GPS_UBLOX_FAIL].active = 0;
		em[GPS_UBLOX_FAIL].finished = 0;
	}
	else
	{
		set_except_mission(GPS_UBLOX_FAIL,TRUE,FALSE, FALSE,0, FALSE,FALSE,1);
	}
#endif
}

/***********************************************************************
* FUNCTIONS   : ops_ground_check
* DESCRIPTION : check ops communication
* INPUTS      : none
* RETURN      : none
***********************************************************************/
bool_t ops_ground_check(void)
{
	if(ops_info.con_flag == OPS_NOT_CONNECT)
	{
		return FALSE;
	}
	else
	{
		return TRUE;
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
	if( (ops_info.con_flag == OPS_NOT_CONNECT)
		||(ops_info.sys_error&0x01) )//ops lost,maybe spray is open,bat_info no update
	{
		set_except_mission(OPS_LOST,TRUE,FALSE, TRUE,0xFF, FALSE,FALSE,2);	
	}
	else
	{
		em[OPS_LOST].active = FALSE;
		em[OPS_LOST].finished = FALSE;
	}
	
	if(ops_info.res_cap <LESS_RES_CAP )  //no pesticide and open spray
	{
		set_except_mission(OPS_EMPTY,TRUE,FALSE, TRUE,0xFF, FALSE,FALSE,1);	
	}
	else
	{   
		if(ops_info.sys_error>>1)        //see spray protocol
		{
			set_except_mission(OPS_BLOCKED,TRUE,FALSE, TRUE,0xFF, FALSE,FALSE,2);			
		}
		else
		{
			em[OPS_BLOCKED].active = FALSE;
			em[OPS_BLOCKED].finished = FALSE;
		}
	}
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


#define MAX_ERROR_Z_THRUST  8000
#define MAX_ERROR_YAW_COMMAND 8000

#define MAX_ERROR_ATT (1.05)  //60deg

void lift_flight_check(void)
{
	/*ac lower setpoint height >1m and thrust cmd >MAX_ERROR_Z_THRUST*/
	if( thrust_command_monitor() )  
	{
		set_except_mission(LIFT_POWER,TRUE,FALSE, TRUE,0XFF, FALSE,FALSE,3);	//land direct
	}

    /*yaw command overrun continual 3s*/
	if( yaw_command_monitor() )
	{
		set_except_mission(LIFT_POWER,TRUE,FALSE, TRUE,0xFF, FALSE,FALSE,3);	//land direct
	}

	/*keep 1s att > 60deg, lock motors direct !!!*/
	if( lift_lost_detect() )
	{
		//;  //AP_MODE will set kill,lock motors
	}
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
	if( !stateIsLocalCoordinateValid()) return 0;
 	if( fabs(stateGetSpeedEnu_f()->x) >MAX_GROUND_INS_S)  return 0;
	if( fabs(stateGetSpeedEnu_f()->y) >MAX_GROUND_INS_S)  return 0;
	if( fabs(stateGetSpeedEnu_f()->z) >MAX_GROUND_INS_S)  return 0;
	if( fabs(stateGetAccelNed_f()->x) >MAX_GROUND_INS_A)  return 0;
	if( fabs(stateGetAccelNed_f()->y) >MAX_GROUND_INS_A)  return 0;
	if( fabs(stateGetAccelNed_f()->z) >MAX_GROUND_INS_A)  return 0;
	if( fabs(stateGetPositionEnu_f()->z) >MAX_GROUND_INS_Z)  return 0;

	else return 1;
}

static bool_t lift_lost_detect(void)
{
	static uint8_t counter = 0;
	if( autopilot_in_flight)
	{
		if( fabs(stateGetNedToBodyEulers_f()->phi)>MAX_ERROR_ATT || fabs(stateGetNedToBodyEulers_f()->theta)>MAX_ERROR_ATT )
		{
			counter++;
			if(counter > MONITORING_FREQUENCY)
			{
				counter = MONITORING_FREQUENCY;
				return TRUE;
			}
		}
		else 
		{
			counter = 0;
		}
	}
	else
	{
		counter = 0;
	}
	return FALSE;
}


static bool_t yaw_command_monitor(void)
{
	static uint16_t counter = 0;
	if( abs(stabilization_cmd[COMMAND_YAW]) > MAX_ERROR_YAW_COMMAND )
	{
		counter++;
		if(counter > MONITORING_FREQUENCY * 5)  //continual 5s
		{
			counter = MONITORING_FREQUENCY * 5;
			return TRUE;
		}
	}
	else
	{
		counter = 0;
	}
	return FALSE;
}

static bool_t thrust_command_monitor(void)
{
	static uint16_t counter1 = 0, counter2 = 0;
	bool_t err1 = 0, err2 = 0;

	if (guidance_v_get_thrust_error_1())
	{
		counter1++;
		if (counter1 > (MONITORING_FREQUENCY * 2))  //continual 2s
		{
			counter1 = MONITORING_FREQUENCY * 2;
			err1 = TRUE;
		}
	}
	else
	{
		counter1 = 0;
	}

	if (guidance_v_get_thrust_error_2())
	{
		counter2++;
		if (counter2 > (MONITORING_FREQUENCY * 2))  //continual 2s
		{
			counter2 = MONITORING_FREQUENCY * 2;
			err2 = TRUE;
		}
	}
	else
	{
		counter2 = 0;
	}

	if (err1 || err2)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}
/**************** END OF FILE *****************************************/


