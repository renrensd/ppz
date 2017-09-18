/**
 * @file firmwares/rotorcraft/flight_nav.h
 *
 * Rotorcraft flight_nav functions.
 */

#ifndef NAV_FLIGHT_H
#define NAV_FLIGHT_H

#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/mission/task_process.h"

//globle var for flight_mode
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

enum Rc_Type
{
	REAL_RC = 0,
	VIRTUAL_RC
};

enum Spray_Convert_Type
{
	CIRCLE_CONVERT = 0,
	WAYPOINT_FORWARD,
	WAYPOINT_P2P
};

struct config_info
{
	enum Spray_Convert_Type spray_convert_type;
	uint8_t atomization_grade;      //grade 1/2/3...
	uint16_t spray_concentration;   //unit=ml/m2
	float spray_height;             //unit=m
	float spray_wide;               //unit=m
	float spray_speed;              //recommnad, unit=m/s
	float max_flight_speed;         //unit=m/s
	float max_flight_height;        //unit=m
	uint8_t rocker_remote_status;	// 1:enable
	uint8_t force_redun_status;			//1:enable
	uint8_t landfrom_track;					//1:enable
	uint8_t landfrom_mode;					// 1=terrace 2=sloping
	uint8_t radar_oa_en;								//1:enable
};

extern uint8_t flight_mode;
extern bool_t is_force_use_all_redundency;
extern uint16_t flight_status;
extern enum Flight_State  flight_state;
extern enum Rc_Type rc_type;
extern struct EnuCoor_i wp_take_off;
extern Gcs_State task_state;;   //only for debug
extern struct config_info ac_config_info;
extern float distance2_to_takeoff;

extern void nav_flight_init(void);
extern void nav_flight(void);
extern uint16_t get_flight_status(void);
extern void flight_mode_enter(uint8_t new_mode);
extern uint8_t force_use_all_redundency_and_vrc(uint8_t enable);
extern void force_use_heading_redundency(bool_t enable);

#endif /* END OF RC_NAV_H */
