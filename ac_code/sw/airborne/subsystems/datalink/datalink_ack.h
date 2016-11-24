
/** \file subsystems/datalink/datalink_parse.c
 *  \brief parse common status parameter,called by datalink.c
 *
 */
#ifndef DATALINK_ACK_H
#define DATALINK_ACK_H
#include "std.h"

/*
enum set_cmd
{ 
 <!--8bits:abcd efgh                                            -->
 <!--flight_mode:        set_auto  =0x10;    set_manual =0x01   -->
 <!--spray control:      stop_spray=0x20;    open_spray =0x02   -->
 <!--VTOL  control:      take_off  =0x30;    land       =0x03   -->
 <!--home_mode:          home      =0x40;    stop_home  =0x04   -->
 <!--mainpower control:  mainpower_on=0x50;  mainpower_off=0x05 -->
    set_manual =1,    //0x01,
    open_spray =2,    //0x02,
    land       =3,    //0x03,
    home_stop  =4,    //0x04,
    mainpower_off=5,   //0x05
    set_auto   =16,   //0x10,
    stop_spray =32,   //0x20,
    take_off   =48,   //0x30,    
    home       =64,   //0x40,    
    mainpower_on =80, //0x50,    	 
};
*/
//set_config ID
#define CONFIG_ALL             0
#define SPRAY_HEIGHT           1    //unit=cm
#define SPRAY_WIDE             2    //unit=cm
#define SPRAY_CONCENTRATION    3    //unit=ml/m2
#define SPRAY_SPEED            4    //unit=cm/s
#define MAX_FLIGHT_SPEED       5    //unit=cm/s
#define MAX_FLIGHT_HEIGHT      6    //unit=cm
#define ATOMIZATION_GRADE      7    //atomization_grade

enum engine_type
{
	Electricity=1,
	Fuel       =2,
	Hybrid     =3
};

enum Set_Command
{
	GCS_CMD = 1,
	REQUEST_AC_INFO = 2,
	DELETE_ALL = 3,
	OPS_SELFCLEAN
};

/* task_ack_type: add = 0, update = 1, delete = 2, get = 3; */
enum Task_Ack_Type
{
	TASK_ADD = 0,
	TASK_UPDATE = 1,
	TASK_DELETE =2,
	TASK_GET =3
};

//extern uint8_t rc_set_cmd_pasre(uint8_t cmd);
extern void send_heart_beat_A2R_msg(void);
extern void send_heart_beat_A2VR_msg(void);
extern void DlSetConfig(uint8_t id, int8_t *pt_value ,uint8_t length);
extern void send_aircraft_info_state(void);
extern uint8_t DlSetCommand(uint8_t id, uint8_t pt_value);

#endif /* DATALINK_H */
