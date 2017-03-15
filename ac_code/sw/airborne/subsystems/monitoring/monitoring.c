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

#ifdef USE_GPS2_UBLOX
 #include "modules/gps/gps2_ublox.h"
#endif

/*===VARIABLES========================================================*/
/*ground check step*/

#define MONITOR_TASK_FREQUENCE 10
/*forward leds indicate AC status*/
#define GREED_LED 3
#define RED_LED 2


/*ground check step result*/
#define CHECKING 0
#define PASS_C 1
#define FAILED 2

/*ground check result*/
#define PASS 0
/*
#define BATTERY_FAIL 1
#define BOARD_FAIL 2
#define IMU_FAIL 3
#define HEIGHT_FAIL 4
#define OPS_FAIL 5
#define GPS_WAIT 6
#define CALIBRATION_FAIL 7
#define AUTOPILOT_FAIL 8
#define RC_FAIL 9
*/
enum Battery_Check
{
	BATTERY_RUNING,
	BATTERY_LOW_POWER,
	BATTERY_FAIL_GET_INFO
};
enum Board_Check
{
	BOARD_RUNING,
	BOARD_TEMPER_ERROR,
	BOARD_FRAM_ERROR
};
enum Imu_Check
{
	IMU_RUNNING,
	IMU_ACC_FREQUNCE_ERROR,
	IMU_ACC_UPDATE_ERROR,
	IMU_ACC_STATIC_OFFSET,
	IMU_ACC_NOISE_ERROR,
	IMU_GYRO_FREQUNCE_ERROR,
	IMU_GYRO_UPDATE_ERROR,
	IMU_GYRO_STATIC_OFFSET,
	IMU_GYRO_NOISE_ERROR,
	IMU_MAG_FREQUNCE_ERROR,
	IMU_MAG_UPDATE_ERROR,
	IMU_MAG_STATIC_OFFSET,
	IMU_MAG_NOISE_ERROR
};

enum Baro_Check
{
	BARO_RUNNING,
	BARO_FREQUNCE_ERROR,
	BARO_UPDATE_ERROR,
	BARO_STATIC_OFFSET,
	BARO_NOISE_ERROR
};

enum Ops_Check
{
	OPS_RUNNING,
	OPS_NO_LINK,
	OPS_ERROR
};

enum Gps_Check
{
	GPS_RUNNING,
	GPS_HW_ERROR,
	GPS_WAITING_FIX,
	GPS_SINGLE_STATUS,
	GPS_FLOAT_STATUS,
	GPS_WAITING_HEADING
};

enum Ground_Check_Step ground_check_step;  //use to sign step in ground check;
/*monitoring state*/
#define GROUND_MONITORING 0
#define FLIGHT_MONITORING 1
uint8_t monitor_cmd;
uint8_t em_alert_grade;

static bool_t monitoring_state;  //monitoring state
static bool_t run_monitoring_flag;
bool_t ground_check_pass;  //global var use to sign ground monitoring result

uint16_t monitoring_fail_code;
uint32_t em_code;   /*one bit express one emergency in EPT_MS_NB sequence*/

struct except_mission em[EPT_MS_NB];

bool_t rc_cmd_interrupt;
bool_t gcs_cmd_interrupt;
bool_t mode_convert_a2m;

uint8_t mdebug_att_flag = FALSE;
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
static void monitoring_led_update(void);
static void monitoring_task(void);
static inline void monitoring_msg_handle(void);

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
	em_alert_grade = 0;
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

static void monitoring_led_update(void)
{
	if(run_monitoring_flag)
	{
		if(monitoring_state == GROUND_MONITORING)
		{
		   LED_ON(GREED_LED);
		   LED_TOGGLE(RED_LED);  //red led toggle, sign running ground monitoring
		}
		else
		{
			if(em_code)
			{
				LED_OFF(GREED_LED);
				LED_TOGGLE(RED_LED);
			}
			else
			{
				LED_OFF(RED_LED);
				LED_ON(GREED_LED);
			}
		}
	}
	else
	{
		LED_OFF(GREED_LED);
		LED_ON(RED_LED); //red led on, sign ground monitoring fail
	}
}

static inline void monitoring_msg_handle(void)
{
	static uint8_t fre_counter = 0;
	fre_counter++;
	fre_counter%=5;

	#if USE_MANU_DEBUG
	if(mdebug_att_flag)
	{
		struct FloatEulers temp_att = *stateGetNedToBodyEulers_f();
		DOWNLINK_SEND_ATTITUDE(MdebugChannel, MdebugDevice, &temp_att.theta, &temp_att.phi, &temp_att.psi);
	}
	#endif
	if(fre_counter == 1)   //run 2hz
	{
		monitoring_led_update();

		#if USE_MANU_DEBUG
		DOWNLINK_SEND_MONITORING(MdebugChannel, MdebugDevice, &ground_check_step, &monitoring_fail_code);
		#endif
	    #ifdef PERIODIC_TELEMETRY
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
							&em_alert_grade,
							&em_code );
		#endif
    	#endif
	}
}

void monitoring_periodic(void)
{
	monitoring_task();

	if(run_monitoring_flag)
	{
		if(monitoring_state == GROUND_MONITORING)
		{  //want to return ground_monitoring(),need reset sensor ground check flag
		    ground_monitoring();
		}
		else
		{
			flight_monitoring();  //once ground_monitoring() finished,it will run.
		}
	}

	monitoring_msg_handle();
}

static void monitoring_task(void)  //10hz
{
	RunOnceEvery( MONITORING_FREQUENCY/2, imu_frequence_check() );     //need periodic =2hz
	RunOnceEvery( MONITORING_FREQUENCY, height_frequence_check() );    //need periodic =1hz
	RunOnceEvery( MONITORING_FREQUENCY, except_mission_update() );     //need periodic =1hz
	RunOnceEvery( MONITORING_FREQUENCY/5, rc_lost_check() );           //need periodic =5hz
	RunOnceEvery( MONITORING_FREQUENCY/2, gcs_lost_check() );           //need periodic =2hz
	alert_grade_update();
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

	if (get_sys_time_msec() < 5000)
		return;

	switch (ground_check_step)
	{
	case BATTERY_CHECK:
		if (0 == battery_ground_check())   //battery manger module no run may go pass
		{
			ground_check_step++;  //next step
			time_record = get_sys_time_msec();
		}
		else
		{
			monitoring_fail_code = BATTERY_FAIL_GET_INFO;
		}
		break;

	case BOARD_CHECK:
		if (!board_ground_check())
		{
			ground_check_step++;     //direct next step
			time_record = get_sys_time_msec();
		}
		else
		{
			monitoring_fail_code = BOARD_FRAM_ERROR;
		}
		break;

	case IMU_CHECK:
		check_state = imu_ground_check();

		if (check_state == FAILED)
		{
			monitoring_fail_code = imu_ground_check_code();
		}
		else if (check_state == PASS_C)
		{
			ground_check_step++;  //next step
			time_record = get_sys_time_msec();
		}

		if ((get_sys_time_msec() - time_record) > 20000)
		{
			monitoring_fail_code = imu_ground_check_code();
		}
		break;

	case HEIGHT_CHECK:
		check_state = height_ground_check();

		if (check_state == FAILED)
		{
			monitoring_fail_code = height_ground_check_code();
		}
		else if (check_state == PASS_C)
		{
			ground_check_step++;  //next step
			time_record = get_sys_time_msec();
		}
		if ((get_sys_time_msec() - time_record) > 20000)
		{
			monitoring_fail_code = BARO_UPDATE_ERROR;
		}
		break;

	case OPS_CHECK:
		if (ops_ground_check())
		{
			ground_check_step++;  //next step
			time_record = get_sys_time_msec();
		}
		else
		{
			monitoring_fail_code = OPS_NO_LINK;
		}
		break;

	case GPS_CHECK:
		if ( GpsFixValid() && gps.p_stable
#ifdef USE_GPS_HEADING
				&& gps.h_stable && (gps.num_sv > 15)   //default use zhonghaida RTK
#endif
#ifdef USE_GPS2_UBLOX
				//&& gps2.p_stable
#endif
				)
		{
			monitoring_fail_code = PASS;
			ground_check_step++;  //next step
		}
		else if (gps_nmea.pos_type < 16)
		{
			monitoring_fail_code = GPS_WAITING_FIX;
		}
		else if (gps_nmea.pos_type == 16)
		{
			monitoring_fail_code = GPS_SINGLE_STATUS;
		}
		else if (gps_nmea.pos_type < 49)
		{
			monitoring_fail_code = GPS_FLOAT_STATUS;
		}
		else
		{
			monitoring_fail_code = GPS_WAITING_HEADING;
		}

		if ((!gps.alive) && ((get_sys_time_msec() - time_record) > 40000))
		{
			monitoring_fail_code = GPS_HW_ERROR;  //no gps msg received
		}
		break;

	case CALIBRATION_CHECK:
		time_record = get_sys_time_msec();
		ground_check_step++;  //next step
		break;

	case AUTOPILOT_CHECK:
		if (autopilot_ground_check())
		{
			ground_check_step++;  //next step
		}
		else
		{
			if ((get_sys_time_msec() - time_record) > 10000)
			{
				monitoring_fail_code = 1; //fail
			}
		}
		break;

	case RC_CONNECT:
		if (!rc_lost || !gcs_lost)
		{
			//ground_check_step = 0;        //reset step
			monitoring_state = FLIGHT_MONITORING;    //turn to flight monitoring
			ground_check_pass = TRUE;
		}
		break;

	default:
		break;
	}

	if ( PASS != monitoring_fail_code)
	{
		if ((ground_check_step != GPS_CHECK)
				|| ((ground_check_step == GPS_CHECK) && (monitoring_fail_code == GPS_HW_ERROR)))
		{
			run_monitoring_flag = FALSE;    //ground check fail,stop running monitoring

			monitoring_state = FLIGHT_MONITORING;
			ground_check_pass = FALSE;
		}
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
	RunOnceEvery( MONITORING_FREQUENCY,   battery_flight_check() );
	//RunOnceEvery( MONITORING_FREQUENCY*2, board_flight_check() );
	RunOnceEvery( MONITORING_FREQUENCY/5, imu_flight_check() );
	RunOnceEvery( MONITORING_FREQUENCY/2, height_flight_check() );
	RunOnceEvery( MONITORING_FREQUENCY/5, gps_flight_check() );
	RunOnceEvery( MONITORING_FREQUENCY,   ops_flight_check() );
	RunOnceEvery( MONITORING_FREQUENCY/5, rc_communication_flight_check() );
	RunOnceEvery( MONITORING_FREQUENCY/2, gcs_communication_flight_check() );
	lift_flight_check();
	task_running_check();
	mode_convert_check();
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
}

static inline void alert_grade_update(void)
{
	em_alert_grade = 0;  //reset grade to 0
	em_code = 0;  //set default 0,use active to set 1
	for(uint8_t i=0; i<EPT_MS_NB; i++)
	{
		if(em[i].active)
		{
			if(em[i].alert_grade > em_alert_grade)
			{
				em_alert_grade = em[i].alert_grade;
			}
			em_code = em_code|(1<<i);
		}
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

void set_except_mission( uint8_t em_nb,
	                     bool_t em_active,
	                     bool_t em_finished,
	                     bool_t em_hover,
	                     uint8_t em_keep_time,
	                     bool_t em_home,
	                     bool_t em_land,
	                     uint8_t alert_grade)
{
	if(em[em_nb].active || em[em_nb].finished)  return;
	em[em_nb].active = em_active;
	em[em_nb].hover.hover = em_hover;
	em[em_nb].hover.keep_time = em_keep_time;
	em[em_nb].home = em_home;
	em[em_nb].land = em_land;
	em[em_nb].alert_grade = alert_grade;
	em[em_nb].finished = em_finished;
}

/**************** END OF FILE *****************************************/

