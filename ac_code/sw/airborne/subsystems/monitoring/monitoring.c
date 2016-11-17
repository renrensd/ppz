/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : R&D SW      									   *
*   AUTHOR	   :             										   *
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

#include "firmwares/rotorcraft/autopilot.h"
#include "subsystems/gps.h"

#include "subsystems/monitoring/monitoring.h"   
#include "subsystems/monitoring/monitoring_imu.h"  
#include "subsystems/monitoring/monitoring_height.h"
#include "subsystems/monitoring/monitoring_misc.h"
#include "subsystems/datalink/downlink.h"
#include "firmwares/rotorcraft/nav_flight.h"
#include "subsystems/rc_nav/rc_nav_xbee.h"
#include "subsystems/mission/gcs_nav_xbee.h"
#include "subsystems/ins/ins_int.h"

#include "led.h"

#ifdef USE_GPS_NMEA
 #include "subsystems/gps/gps_nmea.h"
#endif

/*===VARIABLES========================================================*/
/*ground check step*/

/*forward leds indicate AC status*/
#define GREED_LED 2
#define RED_LED 3
enum Ground_Check_Step
{
	BATTERY_CHECK = 0,
	BAORD_CHECK,
	IMU_CHECK,
	HEIGHT_CHECK,
	GPS_CHECK,
	CALIBRATION_CHECK,
	AUTOPILOT_CHECK,
	OPS_CHECK,
	RC_CONNECT
} ground_check_step;  //use to sign step in ground check;


/*ground check step result*/
#define CHECKING 0
#define PASS_C 1
#define FAILED 2

/*ground check result*/
#define PASS 0
#define BATTERY_FAIL 1
#define BAORD_FAIL 2
#define IMU_FAIL 3
#define HEIGHT_FAIL 4
#define GPS_WAIT 5
#define CALIBRATION_FAIL 6
#define AUTOPILOT_FAIL 7
#define OPS_FAIL 8
#define RC_FAIL 9

/*monitoring state*/
#define GROUND_MONITORING 0
#define FLIGHT_MONITORING 1
uint8_t monitor_cmd;  
uint8_t rc_alert_grade;

static bool_t monitoring_state;  //monitoring state
static bool_t run_monitoring_flag; 
bool_t ground_check_pass;  //global var use to sign ground monitoring result

uint16_t monitoring_fail_code;
uint32_t em_code;   /*one bit express one emergency in EPT_MS_NB sequence*/

struct except_mission em[EPT_MS_NB];

bool_t rc_cmd_interrupt;
bool_t gcs_cmd_interrupt;
bool_t mode_convert_a2m;


#if TEST_MSG
uint8_t fs_imu;
uint8_t fre_imu;
uint8_t fix_imu;
uint8_t g_range_imu;
uint8_t g_noise_imu;
uint8_t mag_emi_counter;
uint8_t mag_emi;
uint8_t sonar_error_data;
uint8_t height_fix;
uint8_t height_noise;
uint8_t height_fre;
uint8_t baro_flight_range;
uint8_t sonar_bound;
uint8_t baro_status;
uint8_t bat_flight;
uint8_t gps_flight;
#endif


/*static functions declare*/
static void except_mission_manage(void);
static void except_mission_update(void);
static inline void alert_grade_update(void);
static inline uint8_t check_hover_ms(void);
static inline uint8_t check_home_ms(void);
static inline uint8_t check_land_ms(void);

static void monitoring_task(void);

/*---Global-----------------------------------------------------------*/
void monitoring_init(void)
{
	ground_monitoring_init();
	//flight_monitoring_init();
	imu_moni_init();
	height_moni_init();
	misc_moni_init();
	ground_check_pass = FALSE;
	run_monitoring_flag = TRUE;
	monitoring_fail_code = PASS;
	em_code = 0;
	rc_cmd_interrupt = FALSE;
	gcs_cmd_interrupt = FALSE;
	mode_convert_a2m = FALSE;
	rc_alert_grade = 0;
	monitor_cmd = CM_NONE;	
}

int8_t monitoring_reset_emer(void)
{
	if(!autopilot_in_flight)
	{
		for(uint8_t i = 0; i<EPT_MS_NB; i++)
		{
			em[i].active = FALSE;
		}
		return 1;
	}
	else
	{
		return 0;
	}
}

void monitoring_periodic(void)
{
	monitoring_task();
	if(run_monitoring_flag)
	{
		if(monitoring_state == GROUND_MONITORING) {  //want to return ground_monitoring(),need reset sensor ground check flag
		   ground_monitoring();
		   LED_TOGGLE(RED_LED);  //red led toggle, sign running ground monitoring
		}
		else 
		{
			flight_monitoring();  //once ground_monitoring() finished,it will run.
			LED_ON(GREED_LED);
		}
	}
	else
	{
		LED_ON(RED_LED); //red led on, sign ground monitoring fail
	}
	
  #if PERIODIC_TELEMETRY
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
    DOWNLINK_SEND_MONITORING(DefaultChannel, DefaultDevice, &ground_check_step, &monitoring_fail_code);
	#if TEST_MSG
	DOWNLINK_SEND_MONI_MSG(DefaultChannel, DefaultDevice, 
							&fs_imu,
							&fre_imu,
							&fix_imu,
							&g_range_imu,
							&g_noise_imu,
							&mag_emi_counter,
							&mag_emi,
							&sonar_error_data,
							&height_fix,
							&height_noise,
							&height_fre,
							&baro_flight_range,
							&sonar_bound,
							&baro_status,
							&bat_flight,
							&gps_flight,
							&monitor_cmd,
							&rc_alert_grade,
							&ins_int.update_on_agl,
							&ins_int.baro_valid,
							&em_code );
	#endif
  #endif
}

static void monitoring_task(void)
{
	RunOnceEvery( MONITORING_FREQUENCY/2, imu_frequence_check() );     //need periodic =2hz
	//RunOnceEvery( MONITORING_FREQUENCY, height_frequence_check() );    //need periodic =1hz
	RunOnceEvery( MONITORING_FREQUENCY, except_mission_update() );     //need periodic =1hz
	RunOnceEvery( MONITORING_FREQUENCY/2, rc_lost_check() );           //need periodic =2hz
	RunOnceEvery( MONITORING_FREQUENCY/2, gcs_lost_check() );           //need periodic =2hz
	except_mission_manage();	
}

void ground_monitoring_init(void)
{
	monitoring_state = GROUND_MONITORING;
	ground_check_step = 0;
}
/***********************************************************************
* FUNCTION    : ground_monitoring
* DESCRIPTION : systems will do self_inspection once on_ground
*               self_inspection only one time until finished flight
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void ground_monitoring(void)
{
	static uint32_t time_record;
	uint8_t check_state;
	if( get_sys_time_msec() <5000 ) return;   
    switch(ground_check_step) {
		case BATTERY_CHECK:      
			if( battery_ground_check()==0 )   //battery manger module no run may go pass
			{
				ground_check_step++;  //next step
				time_record=get_sys_time_msec();
			}
			else  
			{
				monitoring_fail_code = BATTERY_FAIL;
			}
			break;
			
		case BAORD_CHECK:
			ground_check_step++;     //direct next step
			time_record=get_sys_time_msec();
			break;
			
		case IMU_CHECK:	
			//ground_check_step++;  //next step
			//break;
			
			check_state=imu_ground_check();
			
			if(check_state==FAILED) 
			{
				monitoring_fail_code = IMU_FAIL;
			}
			else if(check_state==PASS_C) 
			{
				ground_check_step++;  //next step
				time_record=get_sys_time_msec();
			}
			//else:check_state=CHECKING  running
			if( (get_sys_time_msec()-time_record) >20000)  
			{
				monitoring_fail_code = IMU_FAIL;
			}
			break;
			
		case HEIGHT_CHECK:
		  #if 0
			check_state=height_ground_check();
			
			if(check_state==FAILED)
			{
				monitoring_fail_code=HEIGHT_FAIL;
			}
			else if(check_state==PASS_C) 
			{
				ground_check_step++;  //next step
				time_record=get_sys_time_msec();
			}
			//else ---running
			if( (get_sys_time_msec()-time_record) >20000)
			{
				monitoring_fail_code=HEIGHT_FAIL;
			}
		  #else
		   ins_int.baro_valid = TRUE;
		   ground_check_step++;  //next step
		  #endif
			break;
			
		case GPS_CHECK:
			if( GpsFixValid() 
			   && gps.p_stable 
			  #ifdef USE_GPS_HEADING
			   && gps.h_stable
			  #endif 
			   && (gps.num_sv>15)   //default use zhonghaida RTK
			                           ) 
			{
				ground_check_step++;  //next step
			}
			break;
		case CALIBRATION_CHECK:
			time_record=get_sys_time_msec();
			ground_check_step++;  //next step
			break;
			
		case AUTOPILOT_CHECK:
			if( autopilot_ground_check() )
			{
				ground_check_step++;  //next step
			}
			else
			{
				if( (get_sys_time_msec()-time_record) >10000)
				{
					monitoring_fail_code=AUTOPILOT_FAIL;
				}
			}
			break;
			
		case OPS_CHECK:
			ground_check_step++;  //next step
			break;
		case RC_CONNECT:	
			if( !rc_lost || !gcs_lost )
			{
				//ground_check_step = 0;        //reset step
				monitoring_state = FLIGHT_MONITORING;    //turn to flight monitoring
				ground_check_pass = TRUE;			
			}
			break;
			
		default:
			break;		
    } 
	
	if( PASS != monitoring_fail_code)
	{
		run_monitoring_flag = FALSE;    //ground check fail,stop running monitoring
		//ground_check_step = 0;   //reset step
		monitoring_state = FLIGHT_MONITORING;
		ground_check_pass = FALSE;
	}
}

void flight_monitoring_init(void)
{
	
}
/***********************************************************************
* FUNCTION    : flight_monitoring
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void flight_monitoring(void)  //TODOM:need conside each step periodic
{	
	RunOnceEvery( MONITORING_FREQUENCY*2, battery_flight_check() );
	//RunOnceEvery( MONITORING_FREQUENCY*2, board_flight_check() );
	RunOnceEvery( MONITORING_FREQUENCY, imu_flight_check() );
	//RunOnceEvery( MONITORING_FREQUENCY, height_flight_check() );
	RunOnceEvery( MONITORING_FREQUENCY, gps_flight_check() );
	RunOnceEvery( MONITORING_FREQUENCY*2, ops_flight_check() );
	RunOnceEvery( MONITORING_FREQUENCY, rc_communication_flight_check() );
	RunOnceEvery( MONITORING_FREQUENCY, gcs_communication_flight_check() );
	RunOnceEvery( MONITORING_FREQUENCY/2, lift_flight_check() );
	RunOnceEvery( MONITORING_FREQUENCY/2, task_running_check() );   
	RunOnceEvery( MONITORING_FREQUENCY/2, mode_convert_check() );   
	if(0)  //land,turn to ground monitoring
	{
		imu_ground_reset();
		height_ground_reset();
		monitoring_state = GROUND_MONITORING;
		ground_check_step = 0; 
	}
}


static void except_mission_manage(void)
{ 	
	if( flight_mode==nav_rc_mode && rc_cmd_interrupt ) 
	{   // set all active ms finished
		for(uint8_t i=0; i<EPT_MS_NB; i++)
		{
			if(em[i].active)  em[i].finished=TRUE;   
		}
		//current motion set none
		monitor_cmd = CM_NONE;
		rc_cmd_interrupt = FALSE;  //reset 
		gcs_cmd_interrupt = FALSE;
		return;
	}
	else if( flight_mode==nav_gcs_mode && gcs_cmd_interrupt ) 
	{   // set all active ms finished
		for(uint8_t i=0; i<EPT_MS_NB; i++)
		{
			if(em[i].active)  em[i].finished=TRUE;   
		}
		//current motion set none
		monitor_cmd = CM_NONE;
		rc_cmd_interrupt = FALSE;  //reset 
		gcs_cmd_interrupt = FALSE;
		return;
	}
	
	else
	{ 
		if(monitor_cmd == CM_LAND)
		{
			return;   //land can not interrupt
		}
		if(check_land_ms())
		{
			monitor_cmd = CM_LAND;
			return;
		}
		if(check_home_ms())
		{
			monitor_cmd = CM_HOME;
			return;
		}
		if(check_hover_ms())
		{
			monitor_cmd = CM_HOVER;
			return;
		}
		monitor_cmd = CM_NONE;   //if no ept,set monitor_cmd=CM_NONE
	}	
}

static inline uint8_t check_land_ms(void)
{	
	for(uint8_t i=0; i<EPT_MS_NB; i++)
	{
		if(em[i].active && !(em[i].finished))
		{ 
			if( !em[i].hover.hover && !em[i].home && em[i].land )
			return TRUE;
		}			
	}
	return FALSE;	
}

static inline uint8_t check_home_ms(void)
{	
	for(uint8_t i=0; i<EPT_MS_NB; i++)
	{
		if(em[i].active && !(em[i].finished))
		{ 
			if(!em[i].hover.hover && em[i].home)
			return TRUE;
		}			
	}
	return FALSE;	
}

static inline uint8_t check_hover_ms(void)
{	
	for(uint8_t i=0; i<EPT_MS_NB; i++)
	{
		if(em[i].active && !(em[i].finished))
		{ 
			if(em[i].hover.hover)
			return TRUE;
		}			
	}
	return FALSE;	
}

static void except_mission_update(void)
{
	for(uint8_t i=0; i<EPT_MS_NB; i++)
	{
		if(em[i].active && !(em[i].finished))   //ms is active and not finished 
		{
			if(em[i].hover.hover)
			{ 
				if(em[i].hover.keep_time && em[i].hover.keep_time != 0xFF)  
				{
					em[i].hover.keep_time--;
				}
				if(!em[i].hover.keep_time)  
				{
					em[i].hover.hover = FALSE;
				}
			}
			if(!(em[i].hover.hover) && !(em[i].home) && !(em[i].land))
			{   
				em[i].finished = TRUE;   
			}			
		}
	}
	alert_grade_update();
}

static inline void alert_grade_update(void)
{
	rc_alert_grade = 0;  //reset grade to 0
	em_code = 0;  //set default 0,use active to set 1
	for(uint8_t i=0; i<EPT_MS_NB; i++)
	{
		if(em[i].active)
		{ 
			if(em[i].alert_grade >rc_alert_grade)  
			{
				rc_alert_grade=em[i].alert_grade;
			}
			em_code = em_code|(1<<i);
		}			
	}

	if(em_code)
	{
		LED_ON(RED_LED);  
	}
}

uint8_t data_fix_check(int32_t data, int32_t last_data, uint8_t *counter, uint8_t max_counter)
{
	if( data==last_data)  (*counter)++; 
	else *counter=0;  
    if(*counter> max_counter) 
	{
		(*counter)--;   //avoid overflow
		return TRUE;
    }
	else 
	{
		return FALSE;
	}
}

void set_except_mission(    uint8_t em_nb,
	                     bool_t em_active,
	                     bool_t em_finished,
	                     bool_t em_hover,
	                     uint8_t em_keep_time,
	                     bool_t em_home,
	                     bool_t em_land,
	                     uint8_t em_alert_grade)
{
	if(em[em_nb].active || em[em_nb].finished)  return; 
	em[em_nb].active=em_active;
	em[em_nb].hover.hover=em_hover;
	em[em_nb].hover.keep_time=em_keep_time;
	em[em_nb].home=em_home;
	em[em_nb].land=em_land;
	em[em_nb].alert_grade =em_alert_grade;
	em[em_nb].finished=em_finished;
}

/**************** END OF FILE *****************************************/

