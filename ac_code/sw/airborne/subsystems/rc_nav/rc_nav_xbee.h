/**
 * @file subsystem/rc_nav_xbee.h
 *
 * rc_nav functions.
 */

#ifndef RC_NAV_XBEE_H
#define RC_NAV_XBEE_H

#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"

//#define DEBUG_RC
//#define VIRTUAL_JOYSTICK

//rc_vtol_cmd
#define LOCKED 0
#define TAKE_OFF 1
#define LAND 2
#define CRUISE 3

struct Joystick_Motion
{
	bool_t received_flag;
	uint16_t sign_counter;
	int8_t current_stick_ratio[4];
	int8_t smooth_stick_ratio[4];
	float percent_rssi;  //0--1.0
};


struct rc_motion
{
	bool_t orientation_flag;
	int8_t orientation_fb;
	float speed_fb;
	float max_speed_fb;
	float speed_rl;
	float max_speed_rl;
	float rotation_rate;
	float max_rotation_rate;
	float speed_v;	
	float max_speed_v;
	struct Joystick_Motion joystick_info;
};

struct rc_set
{
	uint8_t mode_state;
	uint8_t vtol;   //total nav_rc_mode ac state
	bool_t  home;
	uint8_t spray_grade;
	bool_t  m_power;	
	bool_t  locked; //ac motors locked status
};

enum manual_mission
{
	MAN_HOVER=1,
    MAN_MOVE,
    MAN_TAKE_OFF,
    MAN_LAND,
    MAN_GROUND
};

/*  RC equiment functions package
 */
//virtical control functions
#define RC_V_Rate 0.5
#define RC_V_Max_Speed  1.5
#define V_DEFAULT_GRADE 1

#define RC_Climb(_grade)  { \
    float climb = RC_V_Rate*_grade; \
    if (climb > RC_V_Max_Speed) climb=RC_V_Max_Speed; \
    NavVerticalClimbMode(climb); \
  }
	
#define RC_Decline(_grade)  { \
    float climb = -RC_V_Rate*_grade; \
    if (climb < -RC_V_Max_Speed) climb = -RC_V_Max_Speed; \
    NavVerticalClimbMode(climb); \
  }

#define RC_Z_HOLD()  { \
	nav_climb =0;  \
    float alt = GetPosZ_u(); \
    NavVerticalAltitudeMode(alt,0); \
  }

#define RC_Z_STOP()  {\
	NavVerticalClimbMode(0); \
  }

//horizontal control functions
#define FB_Accel 1.0
#define FB_Max_Speed 5.0
#define RL_DRIFT_SPEED 0.4
#define RL_SPEED 1.0

#define RAD_OF_DEG(d) (d*M_PI/180.)
#define ROTATION_RATE RAD_OF_DEG(40)   //unit =rad/s
#define ROTATION_DRIFT_RATE 0.2

#define RC_FB_SPEED(_grade)  ({ \
   float speed_f=FB_Accel*_grade; \
   BoundAbs(speed_f,FB_Max_Speed);  \
   speed_f; \
 })


//when turn into RC_MODE,change to RC_HOVER once,importance!!!
//guidance_h_hover_enter();  add to RC_MODE enter,need modify
//only adding MODE_RC could use sp_speed
/*
 need conside the use for more exact!!!
*/
#define RC_HOVER()  do{ \
   rc_motion_info.speed_fb=0.0; \
   rc_motion_info.speed_rl=0.0; \
   guidance_h.sp.speed.x =0.0;  \
   guidance_h.sp.speed.y =0.0;  \
   rc_motion_info.rotation_rate=0.0; \
   rc_turn_rate=0; \
   horizontal_mode = HORIZONTAL_MODE_RC;  \
   RC_Z_HOLD(); \
 }while(0) 
//guidance_h_nav_rc_enter();

extern struct rc_set rc_set_info;
extern struct rc_motion rc_motion_info;
extern uint8_t rc_motion_cmd;
extern uint8_t rc_set_cmd;
extern uint8_t rc_count;
extern bool_t  rc_lost;

#ifdef VIRTUAL_JOYSTICK
extern uint8_t rc_motion_cmd_execution( uint32_t cmd );
#else
extern uint8_t rc_motion_cmd_execution(uint8_t cmd);
#endif
//extern uint8_t rc_set_response_pack(void);
extern uint8_t rc_set_cmd_parse(uint8_t cmd);
extern void rc_setpoint_speed_parse(float speed_fb,float speed_rl);
extern void rc_set_info_reset(void);
extern void rc_lost_check(void);
extern void rc_set_connect(void);
extern void rc_set_info_init(void);
extern void rc_motion_info_init(void);
extern void rc_nav_init(void);
extern void rc_set_rc_type(void);
    
#endif /* END OF RC_NAV_XBEE_H */
