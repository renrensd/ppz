
/** \file subsystems/datalink/datalink_ack.c
 *  \brief parse common status parameter,called by datalink.c
 *
 */
#include "subsystems/datalink/datalink_ack.h"
#include "uplink_ac.h"
#include "subsystems/datalink/downlink.h"
#include "firmwares/rotorcraft/nav_flight.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/rc_nav/rc_nav_xbee.h"
#include "subsystems/mission/task_manage.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "subsystems/ops/ops_app_if.h"
#include "subsystems/ops/ops_msg_if.h"
#include "subsystems/eng/eng_app_if.h"
#include "subsystems/monitoring/monitoring.h"

/*
<field name="sys_time" type="uint16" unit="s"/>
<field name="ac_state" type="uint8" values="ground|flight"/>
<field name="flight_mode"       type="uint8" values="kill|manual|auto"/>
<field name="battery_remain"  type="uint8"  unit="percent"/>
<field name="pesticides_remain" type="uint8" unit="percent"/>
<field name="pesticides_threshold"   type="uint8"/>
<field name="home_state"        type="uint8" values="stop_home|home"/>
<field name="lock_state"        type="uint8" values="unlocked|locked"/>
<field name="ac_ready"          type="uint8" values="unready|ready"/>
<field name="error_code"        type="uint8" />
<field name="alert_grade"       type="uint8" values="ok|low|medium|high"/>
*/

void send_heart_beat_A2R_msg(void)
{   
	uint16_t system_time = sys_time.nb_sec;
	uint8_t ac_state = (uint8_t)autopilot_in_flight;
	uint8_t battery_remain = 85;      //unit=percent, need update from battery module
	uint8_t pesticides_remain = (uint8_t)(ops_info.res_cap&0xFF);   //unit=percent,need updata from spray working module
	uint8_t ac_ready = (uint8_t)ground_check_pass;
	uint8_t error_code = 0;
	uint8_t spray_flag;
	if(get_spray_switch_state())
	{
		spray_flag = 1;
	}
	else
	{
		spray_flag = 0;
	}
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_RC);
	DOWNLINK_SEND_HEART_BEAT_AC_RC_STATE(SecondChannel, SecondDevice,
		                                 &system_time, 
		                                 &ac_state,
		                                 &flight_mode,
		                                 &battery_remain,
		                                 &pesticides_remain,
		                                 &rc_set_info.spray_grade,
		                                 &rc_set_info.home,
		                                 &rc_set_info.locked,
		                                 &ac_ready,
		                                 &error_code,
		                                 &rc_alert_grade,
		                                 &spray_flag);
}

void send_heart_beat_A2VR_msg(void)
{   
	uint16_t system_time = sys_time.nb_sec;
	uint8_t ac_state = (uint8_t)autopilot_in_flight;
	uint8_t battery_remain = 85;      //unit=percent, need update from battery module
	uint8_t pesticides_remain = (uint8_t)(ops_info.res_cap&0xFF);   //unit=percent,need updata from spray working module
	uint8_t ac_ready = (uint8_t)ground_check_pass;
	uint8_t error_code = 0;
	uint8_t spray_flag;
	if(get_spray_switch_state())
	{
		spray_flag = 1;
	}
	else
	{
		spray_flag = 0;
	}
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_GCS);
	DOWNLINK_SEND_HEART_BEAT_AC_RC_STATE(SecondChannel, SecondDevice,
		                                 &system_time, 
		                                 &ac_state,
		                                 &flight_mode,
		                                 &battery_remain,
		                                 &pesticides_remain,
		                                 &rc_set_info.spray_grade,
		                                 &rc_set_info.home,
		                                 &rc_set_info.locked,
		                                 &ac_ready,
		                                 &error_code,
		                                 &rc_alert_grade,
		                                 &spray_flag);
}


void DlSetConfig(uint8_t id, int8_t *pt_value ,uint8_t length)
{   
	switch(id)
	{
		case CONFIG_ALL:
			if(length==12)
			{
				uint8_t i = 0;
				ac_config_info.spray_height = ((float)((uint16_t)(*((uint8_t*)pt_value+i)|*((uint8_t*)pt_value+i+1)<<8)))/100.0;
				i+=2;
				ac_config_info.spray_wide = ((float)((uint16_t)(*((uint8_t*)pt_value+i)|*((uint8_t*)pt_value+i+1)<<8)))/100.0;
				i+=2;
				ac_config_info.spray_concentration = (uint16_t)(*((uint8_t*)pt_value+i)|*((uint8_t*)pt_value+i+1)<<8);
				i+=2;
				ac_config_info.spray_speed = ((float)((uint16_t)(*((uint8_t*)pt_value+i)|*((uint8_t*)pt_value+i+1)<<8)))/100.0;
				i+=2;
				ac_config_info.max_flight_speed = ((float)((uint16_t)(*((uint8_t*)pt_value+i)|*((uint8_t*)pt_value+i+1)<<8)))/100.0;
				i+=2;
				ac_config_info.max_flight_height = ((float)((uint16_t)(*((uint8_t*)pt_value+i)|*((uint8_t*)pt_value+i+1)<<8)))/100.0;
				i+=2;
				ac_config_info.atomization_grade = (uint8_t)(*((uint8_t*)pt_value+i));

				ops_update_config_param(ac_config_info.spray_concentration, PARAM_FLOW_DENSITY);
			}
			break;
			
		case SPRAY_HEIGHT:
			ac_config_info.spray_height = (float)((int16_t)(*((uint8_t*)pt_value)|*((uint8_t*)pt_value+1)<<8))/100.0;
			break;
		case SPRAY_WIDE:
			ac_config_info.spray_wide = (float)((int16_t)(*((uint8_t*)pt_value)|*((uint8_t*)pt_value+1)<<8))/100.0;
			break;
		case SPRAY_CONCENTRATION:
			ac_config_info.spray_concentration = (uint8_t)(*((uint8_t*)pt_value));
			
			ops_update_config_param(ac_config_info.spray_concentration, PARAM_FLOW_DENSITY);
			break;
		case SPRAY_SPEED:
			ac_config_info.spray_speed = (float)((int16_t)(*((uint8_t*)pt_value)|*((uint8_t*)pt_value+1)<<8))/100.0;
		    break;
		case MAX_FLIGHT_SPEED:
			ac_config_info.max_flight_speed = (float)((int16_t)(*((uint8_t*)pt_value)|*((uint8_t*)pt_value+1)<<8))/100.0;
			break;
		case MAX_FLIGHT_HEIGHT:
			ac_config_info.max_flight_height = (float)((int16_t)(*((uint8_t*)pt_value)|*((uint8_t*)pt_value+1)<<8))/100.0;
			break;
		case ATOMIZATION_GRADE:
			ac_config_info.atomization_grade = (uint8_t)(*((uint8_t*)pt_value));
			break;
			
		default: break;
	}
	#if PERIODIC_TELEMETRY
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	DOWNLINK_SEND_AC_CONFIG_INFO(DefaultChannel, DefaultDevice,
		                         &ac_config_info.spray_height,
		                         &ac_config_info.spray_wide,
		                         &ac_config_info.spray_concentration,
		                         &ac_config_info.spray_speed,
		                         &ac_config_info.max_flight_speed,
		                         &ac_config_info.max_flight_height);
	#endif
}

void send_aircraft_info_state(void)
{
/*  
    <field name="engine_type" type="uint8"/>
    <field name="max_voyage" type="uint16" unit="m"/>
    <field name="battery_capacity" type="int32" unit="mAh"/>
    <field name="pesticides_capacity" type="int16" unit="0.1L"/>
    <field name="spray_wide"   type="uint16" unit="cm"/>
    <field name="spray_height" type="uint16" unit="cm"/>
    <field name="max_flight_height"   type="uint16" unit="cm"/> 
    <field name="spray_concentration"   type="uint8" unit="ml/m2"/>
    <field name="max_flight_speed"   type="uint16" unit="cm/s"/>
    <field name="spray_flight_speed"   type="uint16" unit="cm/s"/>
    <field name="sn_and_sv"   type="char[30]"/>    
 */ 
    //all need relative to the information
	enum engine_type ac_engine_type = Electricity;
	uint16_t max_vayage = 500;
	int32_t  battery_capacity = 16000;   //get from electrical
	int16_t  pesticides_capacity = 50;   //get from electrical
	uint16_t spray_wide = (uint16_t)(ac_config_info.spray_wide*100); 
	uint16_t spray_height = (uint16_t)(ac_config_info.spray_height*100);
	uint16_t max_flight_height= (uint16_t)(ac_config_info.max_flight_height*100);
	uint16_t  spray_concentration = ac_config_info.spray_concentration;
	uint8_t  atomization_grade = ac_config_info.atomization_grade;  //need add
	uint16_t max_flight_speed = (uint16_t)(ac_config_info.max_flight_speed*100.0);
	uint16_t spray_flight_speed = (uint16_t)(ac_config_info.spray_speed*100.0);
	char     ac_sn[10]="EFA117";	 //fix info
	char     ac_sv[25]="";
	char     ops_sv[25]="";
	char     bbox_sv[25]="";
	
	char*    buf = eng_get_ac_version(); 
	for(uint8_t i=0; i<25; i++)
	{
		ac_sv[i] = *(buf+i);
	}
	buf = &(ops_info.ops_sv[0]);
	for(uint8_t j=0; j<25; j++)
	{
		ops_sv[j] = *(buf+j);
	}
	
	xbee_tx_header(XBEE_ACK,XBEE_ADDR_GCS);
	DOWNLINK_SEND_AIRCRAFT_INFO_STATE(SecondChannel, SecondDevice, 
		                              &ac_engine_type, 
		                              &max_vayage, 
		                              &battery_capacity, 
		                              &pesticides_capacity, 
		                              &spray_wide, 
		                              &spray_height, 
		                              &max_flight_height, 
		                              &spray_concentration, 
		                              &atomization_grade,
		                              &max_flight_speed, 
		                              &spray_flight_speed, 
		                              &ac_sn[0],
		                              &ac_sv[0],
		                              &ops_sv[0],
		                              &bbox_sv[0]);
}

uint8_t DlSetCommand(uint8_t id, uint8_t pt_value)
{
	uint8_t response = 0;
	enum Set_Command command_gcs = (enum Set_Command)(id);
	switch(command_gcs)
	{
		case GCS_CMD:
			response = parse_gcs_cmd(pt_value);
			break;
			
		case REQUEST_AC_INFO:
			send_aircraft_info_state();
			break;
			
		case DELETE_ALL:
			response = command_delete_all_task();
			break;

		case OPS_SELFCLEAN:
			if(pt_value)
			{
				if(1)//!autopilot_in_flight)
				{
					ops_msg_start_selfclean();
				}
				else
				{
					response = 1;
				}
			}
			else
			{
				ops_msg_stop_selfclean();
			}
			break;
			
		default:  
			response = 1;
			break;
	}
	
	return response;
}

