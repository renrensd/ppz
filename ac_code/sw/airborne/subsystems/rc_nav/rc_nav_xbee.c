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

#include "subsystems/rc_nav/rc_nav_xbee.h"
#include "subsystems/rc_nav/rc_nav_drift.h"
#include "state.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/nav_flight.h"
#include "math.h"
#include "subsystems/datalink/datalink.h"

#ifdef OPS_OPTION
#include "subsystems/ops/ops_app_if.h"
#include "subsystems/ops/ops_msg_if.h"   
#endif
#include "subsystems/monitoring/monitoring.h" 

#include "subsystems/datalink/downlink.h"

#if DATALINK == XBEE
#include "subsystems/datalink/xbee.h"
#endif	/* DATALINK==XBEE */

#if DATALINK == TRANSPTA
#include "subsystems/datalink/transpta.h"
#endif	/* TRANSPTA */

#include "subsystems/mission/task_spray_misc.h"
#ifdef CALIBRATION_OPTION
#include "firmwares/rotorcraft/autopilot.h"
#include "calibration.h"
#endif	/* CALIBRATION_OPTION */
#include "firmwares/rotorcraft/guidance/guidance_h.h"

#define JOYSTICK_UPDATE_FREQUENCE  (10)

//orientation
#define ORIENT_FOR  1
#define ORIENT_BACK -1
#define ORIENT_NONE 0

//rc_motion_cmd
#define K_FORWARD    0x25
#define K_BACKWARD   0x29
#define K_RIGHT      0x45
#define K_LEFT       0x49
#define K_UP         0x15
#define K_DOWN       0x19
#define K_HOLD       0x10
#define K_CW         0x85
#define K_CCW        0x89
#define K_HOVER      0

#define RC_MAX_COUNT  10   //lost_time_out =10/(5hz)=2s

struct rc_set rc_set_info;   //rc mode information
struct rc_motion rc_motion_info;

uint8_t rc_motion_cmd;
uint8_t rc_set_cmd;
uint8_t rc_count;   //rc lost counter
bool_t rc_lost;

int32_t rc_motion32;

static uint8_t rc_motion_cmd_verify(uint8_t cmd);
static void rc_motion_signal_update(void);
static void rc_motion_speed_update(void);
static void rc_setpoint_parse(void);
static void rc_motion_reset(void);

void rc_nav_init(void)
{
	rc_motion_info_init();
	rc_set_info_init();
}

void rc_periodic_task(void)  //run 16hz
{
	if (flight_mode == nav_rc_mode)
	{
		rc_motion_signal_update();
		rc_motion_speed_update();
		rc_setpoint_parse();
	}
	else
	{
		rc_motion_reset();
	}
#if 0//PERIODIC_TELEMETRY
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	DOWNLINK_SEND_JOYSTICK_DEBUG(DefaultChannel, DefaultDevice,
			&rc_motion32,
			&rc_motion_info.joystick_info.current_stick_ratio[0],
			&rc_motion_info.joystick_info.current_stick_ratio[1],
			&rc_motion_info.joystick_info.current_stick_ratio[2],
			&rc_motion_info.joystick_info.current_stick_ratio[3],
			&rc_motion_info.joystick_info.smooth_stick_ratio[0],
			&rc_motion_info.joystick_info.smooth_stick_ratio[1],
			&rc_motion_info.joystick_info.smooth_stick_ratio[2],
			&rc_motion_info.joystick_info.smooth_stick_ratio[3],
			&rc_motion_info.joystick_info.percent_rssi,
			&rc_motion_info.speed_fb,
			&rc_motion_info.speed_rl,
			&rc_motion_info.speed_v,
			&rc_motion_info.rotation_rate );
#endif
}

static void rc_motion_reset(void)
{
	for (uint8_t i = 0; i < 4; i++)
	{
		rc_motion_info.joystick_info.smooth_stick_ratio[i] = 0;
		rc_motion_info.joystick_info.current_stick_ratio[i] = 0;
	}
}

/***********************************************************************
 * FUNCTION    : rc_motion_signal_update
 * DESCRIPTION : use left shift uint16 signal counter to caculate signal quality
 * INPUTS      : none
 * RETURN      : none
 ***********************************************************************/
static void rc_motion_signal_update(void)  //run 16hz
{
	rc_motion_info.joystick_info.signal_counter = rc_motion_info.joystick_info.signal_counter << 1;
	if (rc_motion_info.joystick_info.received_flag)
	{
		rc_motion_info.joystick_info.signal_counter++;
		rc_motion_info.joystick_info.received_flag = FALSE;   //reset received flag
	}
	uint8_t number_flag = 0;
	for (uint8_t n = 0; n < 16; n++)
	{
		if (rc_motion_info.joystick_info.signal_counter & (1 << n))
		{
			number_flag++;
		}
	}
	if (number_flag > JOYSTICK_UPDATE_FREQUENCE)
	{
		number_flag = JOYSTICK_UPDATE_FREQUENCE;
	}
	rc_motion_info.joystick_info.percent_rssi = (float) number_flag / JOYSTICK_UPDATE_FREQUENCE;
}

/***********************************************************************
 * FUNCTION    : rc_motion_speed_update
 * DESCRIPTION : use signal quality limit update flight speed
 * INPUTS      : none
 * RETURN      : none
 ***********************************************************************/
#define JOYSTICK_RATIO (1.0/125.0)
#define JOYSTICK_RL_SPEED_RATIO (0.5)   //rl_speed = JOYSTICK_RL_SPEED_RATIO*fb_speed
static void rc_motion_speed_update(void)
{
	rc_motion_info.max_speed_fb = ac_config_info.max_flight_speed;
	rc_motion_info.max_speed_rl = JOYSTICK_RL_SPEED_RATIO * rc_motion_info.max_speed_fb;

	static float speed_limit;
	float current_speed_limit;
	if (rc_motion_info.joystick_info.percent_rssi > 0.7f)
	{
		current_speed_limit = 1.0f;
	}
	else if (rc_motion_info.joystick_info.percent_rssi > 0.5f)
	{
		current_speed_limit = rc_motion_info.joystick_info.percent_rssi + 0.2f;
	}
	else if (rc_motion_info.joystick_info.percent_rssi > 0.1f)
	{
		current_speed_limit = rc_motion_info.joystick_info.percent_rssi;
	}
	else
	{
		current_speed_limit = 0.0f;
	}
	speed_limit += (current_speed_limit - speed_limit) * 0.2f;

	rc_motion_info.speed_fb = (float) rc_motion_info.joystick_info.smooth_stick_ratio[0] * JOYSTICK_RATIO * speed_limit
			* rc_motion_info.max_speed_fb;
	rc_motion_info.speed_rl = (float) rc_motion_info.joystick_info.smooth_stick_ratio[1] * JOYSTICK_RATIO * speed_limit
			* rc_motion_info.max_speed_rl;
	rc_motion_info.speed_v = (float) rc_motion_info.joystick_info.smooth_stick_ratio[2] * JOYSTICK_RATIO * speed_limit
			* rc_motion_info.max_speed_v;

	rc_motion_info.rotation_rate = (float) rc_motion_info.joystick_info.smooth_stick_ratio[3] * JOYSTICK_RATIO
			* speed_limit * rc_motion_info.max_rotation_rate;

}

/***********************************************************************
 * FUNCTION    : rc_setpoint_parse
 * DESCRIPTION : convert body setpoint speed to ned setpoint speed
 * INPUTS      : none
 * RETURN      : none
 ***********************************************************************/
static void rc_setpoint_parse(void)
{
	guidance_h_set_vrc_vel_sp_body(rc_motion_info.speed_fb, rc_motion_info.speed_rl);

	guidance_h.vrc_heading_rate_sp = rc_motion_info.rotation_rate;

	NavVerticalClimbMode(rc_motion_info.speed_v);
}

/***********************************************************************
 * FUNCTION    : rc_setpoint_speed_parse
 * DESCRIPTION : convert body speed to earth speed,using heading
 * INPUTS      : body speed from rc
 * RETURN      : none
 ***********************************************************************/
void rc_setpoint_speed_parse(float speed_fb, float speed_rl)
{
	guidance_h_set_vrc_vel_sp_body(speed_fb, speed_rl);
	horizontal_mode = HORIZONTAL_MODE_RC;    //request to set mode
}

/***********************************************************************
 * FUNCTION    : rc_motion_cmd_verify
 * DESCRIPTION : check take_off cmd and the condition for rc_motion_cmd
 * INPUTS      : rc_motion_cmd
 * RETURN      : error_code,pass is 0
 ***********************************************************************/
static uint8_t rc_motion_cmd_verify(uint8_t cmd)
{
#ifdef DEBUG_RC
	/*on ground, execute stop spray in any flight mode, for debug or test*/
	if( cmd==K_HOLD && !autopilot_in_flight)
	{
#ifdef OPS_OPTION
		ops_stop_spraying();
#endif
	}
#endif

	uint8_t error_code = 0;
	if (flight_mode != nav_rc_mode)
	{  //error,request in manual mode
		return error_code = 10;
	}

	/*in ground ,up_key give take_off cmd*/
	if (cmd == K_UP && !rc_set_info.locked && !autopilot_in_flight && ground_check_pass && rc_set_info.vtol == LOCKED)
	{
		rc_set_info.vtol = TAKE_OFF; /*generate take off cmd*/
		return error_code = 1;    //not 0, for key up will set climb cmd
	}

	/*on ground,execute stop spray*/
	if (cmd == K_HOLD && !autopilot_in_flight)
	{
#ifdef OPS_OPTION
		ops_stop_spraying();
#endif
	}

	if (!autopilot_in_flight)
	{    //error,ac need take off first
		return error_code = 11;
	}

	if (rc_set_info.home || rc_set_info.vtol == LOCKED)
	{    //error,refuse action while aircraft in home mode
		return error_code = 12;
	}

	if (rc_set_info.vtol == TAKE_OFF || rc_set_info.vtol == LAND)
	{    //error,aircraft is taking off/landing, can not interrupt
		return error_code = 13;
	}

	return error_code;  //all pass
}

/***********************************************************************
 * FUNCTION    : rc_motion_orientation_parse
 * DESCRIPTION : according to hover_cmd,set the for/backward orientation
 * INPUTS      : current cmd, pointer of speed_grade
 * RETURN      : void
 ***********************************************************************/
static void rc_motion_orientation_parse(uint8_t cmd, int8_t *speed_grade)
{
	switch (cmd)
	{
	case K_HOVER:
		rc_motion_info.orientation_flag = 0;
		*speed_grade = 0;
		break;
	case K_FORWARD:
		if (!rc_motion_info.orientation_flag)
		{
			rc_motion_info.orientation_fb = ORIENT_FOR;
			rc_motion_info.orientation_flag = 1;
		}
		break;
	case K_BACKWARD:
		if (!rc_motion_info.orientation_flag)
		{
			rc_motion_info.orientation_fb = ORIENT_BACK;
			rc_motion_info.orientation_flag = 1;
		}
		break;
	default:
		break;
	}
}

/***********************************************************************
 * FUNCTION    : rc_motion_cmd_execution
 * DESCRIPTION :
 * INPUTS      : current cmd
 * RETURN      : unuseful
 ***********************************************************************/
#ifdef VIRTUAL_JOYSTICK
uint8_t rc_motion_cmd_execution(uint32_t cmd)
{
	rc_motion32 = cmd;
	rc_motion_info.joystick_info.received_flag = TRUE;   //get motion sign
	for (uint8_t i = 0; i < 4; i++)
	{
		rc_motion_info.joystick_info.current_stick_ratio[i] = (cmd >> (i * 8)) & 0xff;
	}
	for (uint8_t j = 0; j < 4; j++)
	{
		rc_motion_info.joystick_info.smooth_stick_ratio[j] = (rc_motion_info.joystick_info.smooth_stick_ratio[j]
				+ rc_motion_info.joystick_info.current_stick_ratio[j]) / 2;
	}

	return 0;
}
#else
uint8_t rc_motion_cmd_execution( uint8_t cmd )
{
	static int8_t fb_grade = 0;
	uint8_t error_code = 0;
	rc_motion_cmd = cmd;

	/*check error before motion execution*/
	error_code = rc_motion_cmd_verify(rc_motion_cmd);
	if(error_code)
	{   //something error,return
		return error_code;
	}

	/*parse for/backward orientation*/
	rc_motion_orientation_parse(rc_motion_cmd, &fb_grade);

	switch(rc_motion_cmd)
	{
		case K_HOVER:
		{
			RC_HOVER();     //do hover
			break;
		}

		case K_UP:
		RC_Climb(V_DEFAULT_GRADE);
		break;
		case K_DOWN:
		RC_Decline(V_DEFAULT_GRADE);
		break;

		case K_HOLD:
		if(nav_climb)
		{
			//RC_Z_HOLD();  replace with stop climb speed
			RC_Z_STOP();
		}
		else
		{
#ifdef OPS_OPTION    //back home,need stop spraying
			ops_stop_spraying();
#endif
		}
		break;

		case K_FORWARD:
		if( 0==(fb_grade+1) )
		{   //avoid reverse orientation
			break;
		}
		fb_grade++;
		BoundAbs(fb_grade,5);   //avoid out of limit grade
		rc_motion_info.speed_fb = ( RC_FB_SPEED(fb_grade) );
		break;
		case K_BACKWARD:
		if( 0==(fb_grade-1) )
		{   //avoid reverse orientation
			break;
		}
		fb_grade--;
		BoundAbs(fb_grade,5);   //avoid out of limit grade
		rc_motion_info.speed_fb = ( RC_FB_SPEED(fb_grade) );
		break;

		case K_RIGHT:
		{
			if(rc_motion_info.speed_fb!=0.0)
			{
				rc_speed_rl_drift(1);
			}
			else
			{
				rc_motion_info.speed_rl= RL_SPEED;
			}
			break;
		}
		case K_LEFT:
		{
			if(rc_motion_info.speed_fb!=0.0)
			{
				rc_speed_rl_drift(-1);
			}
			else
			{
				rc_motion_info.speed_rl= -RL_SPEED;
			}
			break;
		}

		case K_CW:
		if(rc_motion_info.speed_fb!=0.0)
		{
			rc_rate_rotation_drift(1);
		}
		else
		{
			rc_motion_info.rotation_rate= RC_MAX_ROTATION_RATE;
			rc_turn_rate=RATE_BFP_OF_REAL( rc_motion_info.rotation_rate );
		}
		break;
		case K_CCW:
		if(rc_motion_info.speed_fb!=0.0)
		{
			rc_rate_rotation_drift(-1);
		}
		else
		{
			rc_motion_info.rotation_rate= -RC_MAX_ROTATION_RATE;
			rc_turn_rate=RATE_BFP_OF_REAL( rc_motion_info.rotation_rate );
		}
		break;

		default: return error_code=7; //motion_type wrong
	}

	//execution set guidance_h_speed_sp
	rc_setpoint_speed_parse(rc_motion_info.speed_fb,rc_motion_info.speed_rl);

	//use for monitoring process
	rc_cmd_interrupt=TRUE;

	return error_code=0;//process success
}
#endif

/***********************************************************************
 * FUNCTION    : rc_set_cmd_parse
 * DESCRIPTION : leave take_off/mainpower/mode unused
 * INPUTS      : current cmd
 * RETURN      : unuseful
 ***********************************************************************/
uint8_t rc_set_cmd_parse(uint8_t cmd)
{
	bool_t enter = FALSE;
	rc_set_cmd = cmd;

	switch (rc_set_cmd)
	{
#if 1
	case RC_SET_AUTO:
		if ( AP_MODE_NAV == autopilot_mode)
		{
			if (autopilot_in_flight)
			{
				break; /*if aircraft in flight, refuse change mode from manual to auto*/
			}

			flight_mode_enter(nav_gcs_mode);
			Flag_AC_Flight_Ready = FALSE;
		}
		break;

	case RC_SET_MANUAL:
		if(autopilot_rc == FALSE)
		{
			enter = TRUE;
		}
		else
		{
			if(AP_MODE_NAV == autopilot_mode)
			{
				enter = TRUE;
			}
		}

		if (enter)
		{
			if (flight_state == taking_off || flight_state == landing)
			{
				break;
			}
			if (flight_mode == nav_gcs_mode && autopilot_in_flight)
			{
				mode_convert_a2m = TRUE;
				spray_break_and_continual();
			}

			flight_mode_enter(nav_rc_mode);
			Flag_AC_Flight_Ready = TRUE;
		}
		break;
#endif

#ifdef VIRTUAL_JOYSTICK
	case RC_OPEN_SPRAY:
		if (flight_mode == nav_rc_mode)
		{
#ifdef OPS_OPTION
			ops_start_spraying();  //make sure ops is opened
#endif
		}
		break;
	case RC_CLOSE_SPRAY:
		if (flight_mode == nav_rc_mode)
		{
#ifdef OPS_OPTION
			ops_stop_spraying();  //make sure ops is opened
#endif
		}
		break;

#else
		case RC_ADD_SPRAY:
#ifndef DEBUG_RC
		if(flight_mode==nav_rc_mode)
#endif
		{
			if( get_spray_switch_state() ) /*only ops is opened grade add*/
			{
				rc_set_info.spray_grade++;
				Bound(rc_set_info.spray_grade, 0, 5);
				rc_update_ops_config_param(rc_set_info.spray_grade);
			}
#ifdef OPS_OPTION
			ops_start_spraying();  //make sure ops is opened
#endif
		}
		break;
		case RC_REDUCE_SPRAY:
#ifndef DEBUG_RC
		if(flight_mode==nav_rc_mode)
#endif
		{
			if(rc_set_info.spray_grade)
			{
				if( get_spray_switch_state() ) /*only ops is opened grade add*/
				{
					rc_set_info.spray_grade--;
					//Bound(rc_set_info.spray_grade, 0, 5);
					rc_update_ops_config_param(rc_set_info.spray_grade);
				}
#ifdef OPS_OPTION
				if(!rc_set_info.spray_grade)
				{
					ops_stop_spraying();  //if set 0 grade,stop spray
				}
				else
				{
					ops_start_spraying();
				}
#endif
			}
		}
		break;
#endif

	case RC_TAKE_OFF:
		if (flight_mode == nav_rc_mode && !autopilot_in_flight && !rc_set_info.locked && rc_set_info.vtol == LOCKED
				&& ground_check_pass)
		{
			rc_set_info.vtol = TAKE_OFF; /*generate take off cmd*/
		}
		break;

	case RC_LAND:
		if (flight_mode == nav_rc_mode && autopilot_in_flight)
		{
			rc_set_info.vtol = LAND;

			//use for monitoring process
			rc_cmd_interrupt = TRUE;
		}
		break;

	case RC_HOME:
		if (flight_mode == nav_rc_mode && autopilot_in_flight && flight_state == cruising)
		{
			rc_set_info.home = TRUE;
#ifdef OPS_OPTION
			ops_stop_spraying(); //back home,make sure spray stop
#endif
			//use for monitoring process
			rc_cmd_interrupt = TRUE;
		}
		break;

	case RC_STOP_HOME:
		if (flight_mode == nav_rc_mode && autopilot_in_flight && flight_state == home)
		{
			rc_set_info.home = FALSE;
			/*use for monitoring process*/
			rc_cmd_interrupt = TRUE;
		}
		break;

		/*LOCK set cmd not request in nav_rc_mode*/
	case RC_LOCK:
		if (rc_set_info.vtol == LOCKED)   //request ac is not in flight cmd
		{
			rc_set_info.locked = TRUE;
		}
		break;
	case RC_UNLOCK:
		if (rc_set_info.vtol == LOCKED)   //request ac is not in flight cmd
		{
			rc_set_info.locked = FALSE;
		}
		break;
#ifdef CALIBRATION_OPTION
		case RC_MAG_CALIBRATION_BEGIN:
		if(kill_throttle)
		{
			cali_mag_begin();
		}
		break;
		case RC_MAG_CALIBRATION_END:
		if(kill_throttle)
		{
			cali_mag_end();
		}
		break;
#endif /* CALIBRATION_OPTION */

	default:
		break;
	}
	return TRUE;
}

void rc_set_info_reset(void)
{
	rc_set_info.home = FALSE;
#ifdef OPS_OPTION
	ops_stop_spraying();   //TODOM
#endif
}

void rc_set_info_init(void)
{
	rc_set_info.mode_state = nav_gcs_mode;
	rc_set_info.vtol = LOCKED;
	rc_set_info.home = FALSE;
	rc_set_info.spray_grade = 0;
#ifdef OPS_OPTION
	ops_stop_spraying();
#endif
	rc_set_info.locked = TRUE;
}

void rc_motion_info_init(void)
{
	rc_motion_info.speed_fb = 0.0;
	rc_motion_info.speed_rl = 0.0;
	rc_motion_info.rotation_rate = 0.0;
	rc_motion_info.orientation_fb = ORIENT_NONE;
	rc_motion_info.orientation_flag = 0;
	rc_motion_info.max_speed_v = RC_MAX_V_SPEED;
	rc_motion_info.max_rotation_rate = RC_MAX_ROTATION_RATE;
}

/** use rc heartbeat msg check communication
 */
void rc_lost_check(void)
{
	if (!rc_lost)
	{
		rc_count++;
	}
	if (rc_count > RC_MAX_COUNT)
	{
		rc_lost = TRUE;
		rc_count = 0;
	}
}

void rc_set_connect(void)
{
	rc_count = 0;  //reset rc_count,use for check rc_lost
	rc_lost = FALSE;
}

void rc_set_rc_type(void)
{
	rc_type = REAL_RC;
}
