/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : RN R&D SW2      									   *
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
#ifndef _MONITORING_H_
#define _MONITORING_H_

#include "std.h"
#include "math.h"
#include "math/pprz_algebra_int.h"

#define TEST_MSG 1

//below is emergency type
#define BAT_LOW 0
#define BAT_CRITICAL 1
#define BAT_OTHER 2
#define IMU_MOMENTARY 3
#define GPS_HEADING 4
#define IMU_CRITICAL  5
#define HEIGHT_SONAR 6
#define HEIGHT_BARO 7
#define HEIGHT_BOTH 8
#define BOARD_TEMP 9
#define BOARD_OUT 10
#define GCS_COM_LOST 11
#define RC_COM_LOST 12
#define GPS_ACC  13
#define GPS_LOST  14
#define OPS_EMPTY  15
#define OPS_LOST  16 
#define LIFT_POWER  17
#define TASK_NO 18
#define TASK_PARSE 19
#define TASK_BREAK 20
#define MODE_CONVERT_A2M 21
#define OPS_BLOCKED 22
#define GPS_UBLOX_FAIL  23


#define EPT_MS_NB 24

/*use for monitor_cmd*/
#define CM_NONE 0
#define CM_HOVER 1
#define CM_HOME 2
#define CM_LAND 3   //direct land in current position
#define CM_LOCK 4   //lock motors in flight,not use now ,dangerous

struct hover_ms
{
	bool_t hover;
	bool_t height_limit;  //can break by rc_cmd_interrupt
	uint8_t keep_time;
};

struct except_mission
{
	struct hover_ms hover;
	bool_t active;     //once active,alert continual until set false
	bool_t finished;   //true meas em is break by user
	bool_t home;
	bool_t land;
	//bool_t lock;
	uint8_t alert_grade;  //0--->3:none  low  middle  high
    //int8_t error_code;
};

enum Ground_Check_Step
{
	BATTERY_CHECK = 0,
	BOARD_CHECK,
	IMU_CHECK,
	HEIGHT_CHECK,
	OPS_CHECK,
	UBLOX_CHECK,
	RTK_CHECK,
	CALIBRATION_CHECK,
	AUTOPILOT_CHECK,
	RC_CONNECT,
	BBOX_CHECK
};
extern enum Ground_Check_Step ground_check_step;  //use to sign step in ground check;
extern uint16_t monitoring_fail_code;

#define CHECK_INTERVAL(_x, _sta, _deta)  ((_x)>((_sta)-(_deta)) && (_x)<((_sta)+(_deta)))

extern struct except_mission em[EPT_MS_NB];
extern uint32_t em_code;
extern bool_t rc_cmd_interrupt;
extern bool_t gcs_cmd_interrupt;
extern bool_t mode_convert_a2m;
extern uint8_t em_alert_grade;
extern uint8_t monitor_cmd;
extern bool_t ground_check_pass;

extern uint8_t mdebug_att_flag;
#if TEST_MSG
extern uint8_t fs_imu;
extern uint8_t fre_imu;
extern uint8_t fix_imu;
extern uint8_t g_range_imu;
extern uint8_t g_noise_imu;
extern uint8_t mag_emi_counter;
extern uint8_t mag_emi;
extern uint8_t sonar_error_data;
extern uint8_t height_fix;
extern uint8_t height_noise;
extern uint8_t height_fre;
extern uint8_t baro_flight_range;
extern uint8_t sonar_bound;
extern uint8_t baro_status;
extern uint8_t bat_flight;
extern uint8_t gps_flight;
#endif

extern void monitoring_init(void);
extern void monitoring_periodic(void);
extern uint8_t data_fix_check(int32_t data, int32_t last_data, uint8_t *counter, uint8_t max_counter);
extern void set_except_mission(uint8_t em_nb,bool_t em_active,bool_t em_finished,bool_t em_hover,uint8_t em_keep_time,bool_t em_home,bool_t em_land,uint8_t alert_grade);
extern void ground_monitoring_init(void);
extern void flight_monitoring_init(void);
extern void ground_monitoring(void);
extern void flight_monitoring(void);
extern int8_t monitoring_reset_emer(void);
extern void set_mdebug_att_flag(uint8_t value);

#endif /*_MONITORING_H_*/

/****************************** END OF FILE ***************************/

