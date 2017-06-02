
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
#if USE_MANU_DEBUG
#include "modules/acc_cali/acc_cali.h"
#include "subsystems/actuators/motor_mixing.h"
#endif

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
		                                 &em_alert_grade,
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
		                                 &em_alert_grade,
		                                 &spray_flag);
}


void DlSetConfig(uint8_t id, int8_t *pt_value ,uint8_t length)
{   
	switch(id)
	{
		case CONFIG_ALL:
			if(length==13)
			{
				uint8_t i = 0;
				ac_config_info.spray_height = ((float)((uint16_t)(*((uint8_t*)pt_value+i)|*((uint8_t*)pt_value+i+1)<<8)))/100.0;

				i+=2;
				ac_config_info.spray_wide = ((float)((uint16_t)(*((uint8_t*)pt_value+i)|*((uint8_t*)pt_value+i+1)<<8)))/100.0;
				ops_set_config_param((uint16_t)(ac_config_info.spray_wide*100), PARAM_SPRAY_WIDE);

				i+=2;
				ac_config_info.spray_concentration = (uint16_t)(*((uint8_t*)pt_value+i)|*((uint8_t*)pt_value+i+1)<<8);
				ops_set_config_param(ac_config_info.spray_concentration, PARAM_FLOW_DENSITY);
				
				i+=2;
				ac_config_info.spray_speed = ((float)((uint16_t)(*((uint8_t*)pt_value+i)|*((uint8_t*)pt_value+i+1)<<8)))/100.0;

				i+=2;
				ac_config_info.max_flight_speed = ((float)((uint16_t)(*((uint8_t*)pt_value+i)|*((uint8_t*)pt_value+i+1)<<8)))/100.0;

				i+=2;
				ac_config_info.max_flight_height = ((float)((uint16_t)(*((uint8_t*)pt_value+i)|*((uint8_t*)pt_value+i+1)<<8)))/100.0;

				i+=2;
				ac_config_info.atomization_grade = (uint8_t)(*((uint8_t*)pt_value+i));
				ops_set_config_param(ac_config_info.atomization_grade, PARAM_SPRAY_ATOM);
				ops_update_config_param();
			}
			break;
			
		case SPRAY_HEIGHT:
			ac_config_info.spray_height = (float)((int16_t)(*((uint8_t*)pt_value)|*((uint8_t*)pt_value+1)<<8))/100.0;
			break;
		case SPRAY_WIDE:
			ac_config_info.spray_wide = (float)((int16_t)(*((uint8_t*)pt_value)|*((uint8_t*)pt_value+1)<<8))/100.0;
			ops_set_config_param((uint16_t)(ac_config_info.spray_wide*100), PARAM_SPRAY_WIDE);
			ops_update_config_param();
			break;
		case SPRAY_CONCENTRATION:
			ac_config_info.spray_concentration = (uint8_t)(*((uint8_t*)pt_value));
			ops_set_config_param(ac_config_info.spray_concentration, PARAM_FLOW_DENSITY);
			ops_update_config_param();
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
			ops_set_config_param(ac_config_info.atomization_grade, PARAM_SPRAY_ATOM);
			ops_update_config_param();
			break;
		case JOYSTICK_ENABLE:
			if(!autopilot_in_flight)
			{
				ac_config_info.rocker_remote_status=(uint8_t)(*((uint8_t*)pt_value)); //add by lg
				check_joystick_enable(ac_config_info.rocker_remote_status);
			}
			break;
		case U_BLOX_ENABLE:
			ac_config_info.force_redun_status = (uint8_t)(*((uint8_t*)pt_value));
			ac_config_info.force_redun_status = force_use_all_redundency_and_vrc(ac_config_info.force_redun_status);
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

void send_gcs_components_info(void)
{
	static bool_t update_flag = FALSE;
	if(eng_components_info.ops_sv.sv_update
	   &&!update_flag
	   #ifdef BBOX_OPTION
	   &&eng_components_info.bbox_sv.sv_update
	   #endif
	                                          )
	{
		send_aircraft_info_state();
		update_flag = TRUE;
	}
}

void send_aircraft_info_state(void)
{
	enum engine_type ac_engine_type = Electricity;
	uint16_t max_vayage = 500;
	int32_t  battery_capacity = 16000;   //get from electrical
	int16_t  pesticides_capacity = 50;   //get from electrical
	uint16_t spray_wide = (uint16_t)(ac_config_info.spray_wide*100); 
	uint16_t spray_height = (uint16_t)(ac_config_info.spray_height*100);
	uint16_t max_flight_height= (uint16_t)(ac_config_info.max_flight_height*100);
	uint16_t spray_concentration = ac_config_info.spray_concentration;
	uint8_t  atomization_grade = ac_config_info.atomization_grade;  //need add
	uint16_t max_flight_speed = (uint16_t)(ac_config_info.max_flight_speed*100.0);
	uint16_t spray_flight_speed = (uint16_t)(ac_config_info.spray_speed*100.0);
	uint8_t rocker_remote_status=ac_config_info.rocker_remote_status;
	uint8_t misc_status =((uint8_t)(ac_config_info.force_redun_status<<1)+(ac_config_info.rocker_remote_status));
	char     ac_sn[12]="";
	char     ac_sv[25]="";
	char     ops_sv[25]="";
	char     bbox_sv[25]="";
	uint8_t *temp_pt = eng_get_product_series_number();
	memcpy(ac_sn, temp_pt, 12);
	
	if(eng_components_info.ac_sv.sv_update)
	{
		memcpy(ac_sv, &eng_components_info.ac_sv.version[0], eng_components_info.ac_sv.sv_len);
	}
	if(eng_components_info.ops_sv.sv_update)
	{
		memcpy(ops_sv, &eng_components_info.ops_sv.version[0], eng_components_info.ops_sv.sv_len);
	}
	#ifdef BBOX_OPTION
	if(eng_components_info.bbox_sv.sv_update)
	{
		memcpy(bbox_sv, &eng_components_info.bbox_sv.version[0], eng_components_info.bbox_sv.sv_len);
	}
	#endif
	
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
		                              &bbox_sv[0],
		                              &misc_status);
}

uint8_t DlSetGcsCommand(uint8_t id, uint8_t pt_value)
{
	uint8_t response = 0;
	enum Set_GCS_Command command_gcs = (enum Set_GCS_Command)(id);
	switch(command_gcs)
	{
		case GCS_CMD:
			response = parse_gcs_cmd(pt_value);
			break;
			
		case REQUEST_AC_INFO:
			send_aircraft_info_state();
			break;
			
		case DELETE_ALL_TASK:
			response = command_delete_all_task();
			task_init();
			break;

		case OPS_SELFCLEAN:
			if(pt_value)
			{
				if(!autopilot_in_flight)
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

		case OPS_SPRAY_CONTROL:
			if(pt_value)
			{
				ops_msg_direct_open_spray();
			}
			else
			{
				ops_msg_direct_stop_spray();
			}
			break;

		case OPS_CHANNEL_CONTROL:
			ops_set_config_param(pt_value, PARAM_SPRAY_CHANNEL);
			ops_update_config_param();
			
		default:  
			response = 1;
			break;
	}
	
	return response;
}

#if USE_MANU_DEBUG
bool_t DlSetMCCommand(uint8_t id, uint8_t pt_value)
{
	bool_t response = TRUE;
	switch(id)
	{
		case MC_ATTITUDE_TEST:
			set_mdebug_att_flag(pt_value);
			break;
			
		case MC_CALIBRATE_ACC:
			set_acc_cali_enable(pt_value);
			break;
			
		case MC_CALIBRATE_ECS:
			set_esc_calibration(pt_value);
			break;

		case MC_TEST_MOTORS:
			set_particular_motor_run(pt_value);
			
		default:  
			response = FALSE;
			break;
	}
	return response;
}
#endif


