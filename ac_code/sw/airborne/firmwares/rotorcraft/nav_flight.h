/**
 * @file firmwares/rotorcraft/flight_nav.h
 *
 * Rotorcraft flight_nav functions.
 */

#ifndef FLIGHT_NAV_H
#define FLIGHT_NAV_H

#include "firmwares/rotorcraft/navigation.h"

//globle var flight_mode
#define nav_kill_mode 0
#define nav_rc_mode 1
#define nav_gcs_mode 2

enum Flight_State
{
    preparing=0,
	ready=1,
	taking_off=2,
	cruising=3,
	home=4,     //direct toward flight back
	landing=5
	//locked=5  landing->ready
};

struct config_info
{                            //communication data unit
	float spray_height;     //unit=m
	float spray_wide;       //unit=m
	uint8_t concentration;  //unit=ml/m2
	float spray_speed;      //recommnad, unit=m/s
	float max_flight_speed;            //unit=m/s
	float max_flight_height;    //unit=m
};

extern uint8_t flight_mode;
extern enum Flight_State  flight_state;
extern struct EnuCoor_i wp_home_tk;
extern struct EnuCoor_i wp_ms_break;
extern struct EnuCoor_i wp_start;
extern uint8_t mission_state;   //only for debug
extern struct config_info ac_config_info;

extern void nav_flight_init(void);
extern bool_t nav_flight(void);
extern uint32_t get_flight_status(void);
extern void flight_mode_enter(uint8_t new_mode);
extern uint8_t flight_demo(void);


#endif /* END OF RC_NAV_H */
