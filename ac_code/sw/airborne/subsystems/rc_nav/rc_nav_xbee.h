/**
 * @file subsystem/rc_nav_xbee.h
 *
 * rc_nav functions.
 */

#ifndef RC_NAV_XBEE_H
#define RC_NAV_XBEE_H

#include "firmwares/rotorcraft/navigation.h"

//rc_vtol_cmd
#define LOCKED 0
#define TAKE_OFF 1
#define LAND 2
#define CRUISE 3

enum motion_type
{   
	hover   =0,
	virtical=1,    //0b:0001
	fb_ward =2,    //0b:0010
	rl_ward =4,    //0b:0100
	rotation=8     //0b:1000
};

struct rc_set
{
	uint8_t mode_state;
	uint8_t vtol;
	bool_t home;
	bool_t spray;
	bool_t m_power;	
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
#define RC_V_Rate 0.3
#define RC_V_Max_Speed  1.5

#define RC_Climb(_keep_time)  { \
    float climb = RC_V_Rate*_keep_time; \
    if (climb > RC_V_Max_Speed) climb=RC_V_Max_Speed; \
    NavVerticalClimbMode(climb); \
  }
	
#define RC_Decline(_keep_time)  { \
    float climb = -RC_V_Rate*_keep_time; \
    if (climb < -RC_V_Max_Speed) climb = -RC_V_Max_Speed; \
    NavVerticalClimbMode(climb); \
  }

#define RC_Z_HOLD()  { \
    float alt = GetPosZ_u(); \
    NavVerticalAltitudeMode(alt,0); \
  }

//horizontal control functions
#define RC_H_Rate 0.6
#define RC_H_Max_Speed 3
#define RC_Turn_Accel 0.5  //rad/s2,not deg/s2
#define RC_Turn_Rate 1.5

#define RC_FORWARD(_keep_time)  ({ \
   float speed_f=RC_H_Rate*_keep_time; \
   if(speed_f>RC_H_Max_Speed) speed_f=RC_H_Max_Speed; \
   speed_f; \
 })

#define RC_BACKWARD(_keep_time)  ({ \
   float speed_b=-RC_H_Rate*_keep_time; \
   if(speed_b<-RC_H_Max_Speed) speed_b=-RC_H_Max_Speed; \
   speed_b; \
 })

#define RC_RIGHT(_keep_time)  ({ \
   float speed_r=RC_H_Rate*_keep_time; \
   if(speed_r>RC_H_Max_Speed) speed_r=RC_H_Max_Speed; \
   speed_r; \
 })


#define RC_LEFT(_keep_time)  ({ \
   float speed_l=-RC_H_Rate*_keep_time; \
   if(speed_l<-RC_H_Max_Speed) speed_l=-RC_H_Max_Speed; \
   speed_l; \
 })

#define RC_TURN_RIGHT(_keep_time)  ({ \
   float rc_rotate_rate=RC_Turn_Accel * _keep_time; \
   if (rc_rotate_rate > RC_Turn_Rate)  rc_rotate_rate=RC_Turn_Rate; \
   rc_rotate_rate; \
 })


#define RC_TURN_LEFT(_keep_time)  ({ \
   float rc_rotate_rate=-RC_Turn_Accel * _keep_time; \
   if (rc_rotate_rate < -RC_Turn_Rate)  rc_rotate_rate=-RC_Turn_Rate; \
   rc_rotate_rate; \
 })

//when turn into RC_MODE,change to RC_HOVER once,importance!!!
//guidance_h_hover_enter();  add to RC_MODE enter,need modify
//only adding MODE_RC could use sp_speed
/*
#define RC_HOVER()  do{ \
   horizontal_mode = HORIZONTAL_MODE_RC; \
   fb_speed=0.0; \
   rl_speed=0.0; \
   rc_turn_rate=0; \
   RC_Z_HOLD(); \
 }while(0) 
*/
#define RC_HOVER()  do{ \
   fb_speed=0.0; \
   rl_speed=0.0; \
   rc_turn_rate=0; \
   RC_Z_HOLD(); \
   guidance_h_nav_rc_enter(); \
 }while(0) 

extern struct rc_set rc_set_info;
extern uint8_t last_response; 
extern uint8_t rc_motion_cmd;
extern uint8_t rc_set_cmd;
extern uint8_t rc_count;
extern bool_t  rc_lost;
extern float fb_speed;  //globle var,use save rc_cmd speed
extern float rl_speed;

extern uint8_t rc_motion_cmd_execution(uint8_t cmd);
extern uint8_t rc_set_response_pack(void);
extern uint8_t rc_set_cmd_pasre(uint8_t cmd);
extern void rc_set_info_reset(void);
extern void rc_lost_check(void);
    
#endif /* END OF RC_NAV_XBEE_H */
