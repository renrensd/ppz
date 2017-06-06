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

#include "subsystems/mission/task_process.h"
#include "subsystems/mission/task_spray_misc.h"

#include "subsystems/ops/ops_msg_if.h"
#include "subsystems/ops/ops_app_if.h" 
#include "firmwares/rotorcraft/nav_flight.h"

#include "subsystems/datalink/downlink.h"

#include "firmwares/rotorcraft/guidance/guidance_h_ref.h"
#include "uplink_ac.h"

#include "subsystems/monitoring/monitoring.h"   

#include "modules/system/timer_if.h"
#include "modules/system/timer_class.h"
#include "modules/system/timer_def.h"

#define FLIGHT_PATH  1
#define SPRAY_PATH  2


enum Gcs_Task_Cmd gcs_task_cmd;
enum Gcs_Task_Cmd last_task_cmd;
enum Gcs_Task_Cmd save_task_cmd;
Task_Error task_error_state;

bool_t from_wp_useful;
bool_t hover_flag;
bool_t manual_pause_flag;
bool_t spray_switch_flag;     /*usefor sign spray_line start and end*/
static bool_t spray_caculate_flag;
static bool_t height_align;    //flag of height check for next line
static bool_t heading_align;   //flag of heading check for next line

float sprayed_lines_distance = 0.0f;
float current_sprayed_distance = 0.0f;

/*interrupt scene var*/
//from_wp and next_wp are request can't be changed before recover
struct EnuCoor_i current_wp_scene;    /*use to save pos for exceptional interrupt, no use now*/
struct EnuCoor_i interrupt_wp_scene;    /*use to save pos of ac exceptional interrupt hover */

struct EnuCoor_i home_wp_enu;
struct EnuCoor_i reland_wp_enu;

enum Task_Action AC_action;

void task_process_init(void);
bool_t achieve_next_wp(void);
bool_t get_start_line(void);
struct EnuCoor_i save_task_scene(void);
bool_t gcs_hover_enter(void);
void get_shortest_reland_wp(void);
bool_t run_normal_task(void);
void spray_work_run(void);
bool_t task_wp_empty_handle(void);
void set_auto_stop_brake(void);

static inline bool_t task_nav_update_target(struct EnuCoor_i enu_wp);
static inline bool_t task_nav_hover(struct EnuCoor_i hover_wp);
static inline bool_t task_nav_wp(struct EnuCoor_i first_wp);
static inline bool_t task_nav_pre_path(struct EnuCoor_i p_start_wp, struct EnuCoor_i p_end_wp, uint8_t flight_type);
static inline bool_t task_nav_path(struct EnuCoor_i p_start_wp, struct EnuCoor_i p_end_wp);
static float set_path_flight_info(uint8_t type);

void send_current_task_state(uint8_t wp_state);
void send_current_task(uint8_t wp_state);


void avoid_obstacle_init()
{
	oa_data.spray_boundary_valid = FALSE;
	oa_data.obstacles_valid = FALSE;
	oa_data.home_valid = FALSE;
	oa_data.spray_boundary_vertices_num = 0;
	oa_data.obstacles_vertices_num = 0;
	oa_data.obstacles_num = 0;
}

void task_init(void)
{
	task_manage_init();
	task_process_init();
	task_spray_misc_init();
	avoid_obstacle_init();
}

void task_process_init(void)
{
	gcs_task_cmd = GCS_CMD_NONE;
	last_task_cmd = GCS_CMD_NONE;
	task_error_state = TASK_NORMAL;
	from_wp_useful = FALSE;
	hover_flag = FALSE;
	manual_pause_flag = FALSE;
	spray_switch_flag = FALSE;
	spray_caculate_flag = FALSE;
	height_align = FALSE;
	from_wp.wp_en.z = 0;  /*z not useful*/
	next_wp.wp_en.z = 0;
	release_stop_brake();
	
}

/***********************************************************************
* FUNCTION    : achieve_next_wp
* DESCRIPTION : get waypoint info to from/next wp
* INPUTS      : none
* RETURN      : TRUE or FALSE
***********************************************************************/
bool_t achieve_next_wp(void)
{
	if(nb_pending_wp < 1) 
	{
		return FALSE;
	}
	
	/*waypoint info move from next to from*/
	VECT2_COPY(from_wp.wp_en, next_wp.wp_en);
	from_wp.action = next_wp.action;
	from_wp.wp_id = next_wp.wp_id;
	struct EnuCoor_i distance_wp;
	

	VECT2_DIFF(distance_wp,from_wp.wp_en,task_wp[0].wp_en);
	/*get waypoint info from task_wp*/
	VECT2_COPY(next_wp.wp_en, task_wp[0].wp_en);
	next_wp.action = task_wp[0].action;
	next_wp.wp_id = task_wp[0].wp_id;
	for(uint8_t i=0; i<nb_pending_wp; i++)
	{
		TASK_WP_LEFT_SHIFT(i+1, 1);
	}
	nb_pending_wp--;
	
	/*remove forepart waypoint*/



	#ifdef USE_PLANED_OA
    if( planed_oa.test_on )
	{
	     planed_oa.wp_move_done_flag = TRUE;
    }
    #endif
	
	
	return TRUE;
}

/***********************************************************************
* FUNCTION    : get_start_line
* DESCRIPTION : get waypoint info to from/next wp for first line
* INPUTS      : none
* RETURN      : TRUE or FALSE
***********************************************************************/
bool_t get_start_line(void)
{
	/*request nb_pending_wp at least 2*/
	if( nb_pending_wp >= 1 )
	{
		achieve_next_wp();
		VECT2_COPY(from_wp.wp_en, wp_home);
		from_wp.action = FLIGHT_LINE;
		from_wp_useful = TRUE;
		return TRUE;
	}
	return FALSE;
}

/***********************************************************************
* FUNCTION    : auto_task_ready_check
* DESCRIPTION : check task if ready
* INPUTS      : none
* RETURN      : TRUE or FALSE
***********************************************************************/
bool_t auto_task_ready_check(void)
{
	if( FALSE==wp_home_useful 
		|| 2>nb_pending_wp 
		|| SPRAY_CONVERT==task_wp[0].action
		|| TERMINATION==task_wp[0].action   )
	{
		return FALSE;
	}
	if(GCS_CMD_START!=gcs_task_cmd) //"|| autopilot_in_flight" checkding is done in calling
	{
		return FALSE;
	}
	return TRUE;
}

/***********************************************************************
* FUNCTION    : gcs_task_run
* DESCRIPTION : run gcs task
* INPUTS      : none
* RETURN      : Gcs_State, running state
***********************************************************************/
Gcs_State gcs_task_run(void)
{
	Gcs_State gcs_state = GCS_RUN_NORMAL;
	switch(gcs_task_cmd)
	{
		case GCS_CMD_NONE:
			gcs_state = GCS_RUN_NONE;
			break;
			
		case GCS_CMD_START:
			if(!from_wp_useful)
			{
				ops_info.sum_sprayed_distance = 0; //reset at start
				if( !get_start_line() )
				{
					//waypoints is less than 2,error generate
					gcs_state = GCS_RUN_ERROR;
					task_error_state = TASK_RUN_OVER;
				}
			}
			if( run_normal_task() )
			{
				gcs_state = GCS_RUN_LANDING;
			}
			break;
			
		case GCS_CMD_PAUSE:
			gcs_state = GCS_RUN_PAUSE;
			if( last_task_cmd != gcs_task_cmd)
			{
				save_task_cmd = last_task_cmd;
			}
			gcs_hover_enter();   //for stop spray
			set_auto_stop_brake();		
			
			break;
			
		case GCS_CMD_CONTI:
            gcs_task_cmd = save_task_cmd;

            #ifdef USE_PLANED_OA
			/*if( planed_oa.test_on && (oa_wp_search_state > 2) )
			{
			    oa_error_force_recover();
			}*/
            #endif

			release_stop_brake();
			
			break;
			
		case GCS_CMD_BHOME:
			gcs_state = GCS_RUN_HOME;
			AC_action = TERMINATION;
			if( gcs_hover_enter() )
			{
				set_stop_brake(SMOOTH_BRAKE);
				spray_break_and_continual();
				VECT2_COPY(home_wp_enu, wp_home);

                #ifdef USE_PLANED_OA
				if( planed_oa.test_on )
				{
				    planed_oa_data_reset();
				}
                #endif
			}
			else
			{
				if(task_nav_pre_path(interrupt_wp_scene, home_wp_enu, FLIGHT_PATH))
				{
					#ifdef USE_PLANED_OA
					if( planed_oa.test_on )
					{
						planed_oa.back_home_ready = TRUE;

						if( planed_oa.oa_home_flag )
						{
							planed_oa.wp_move_done_flag = TRUE;
							planed_oa.oa_home_flag = FALSE;
						}
					}
                    #endif

					release_stop_brake();
					if( !task_nav_path(interrupt_wp_scene, home_wp_enu) ) 
					{
						/*no more task_wp to run, do land motion*/
						task_nav_hover(home_wp_enu);
						gcs_state = GCS_RUN_LANDING;
						
                        #ifdef USE_PLANED_OA
						if( planed_oa.test_on )
						{
						    planed_oa.back_home_ready = FALSE;
						}
						#endif
					}
				}
			}
			break;
			
		case GCS_CMD_RELAND:
			gcs_state = GCS_RUN_RELAND;
			if( gcs_hover_enter() )
			{
				set_stop_brake(SMOOTH_BRAKE);
				spray_break_and_continual();
				get_shortest_reland_wp();
			}
			else
			{   
				if(task_nav_pre_path(interrupt_wp_scene, reland_wp_enu, FLIGHT_PATH))
				{
					release_stop_brake();
					if( !task_nav_path(interrupt_wp_scene, reland_wp_enu) ) 
					{
						/*no more task_wp to run, do land motion*/
						task_nav_hover(reland_wp_enu);
						gcs_state = GCS_RUN_LANDING;
					}
				}
			}
			break;
			
		case GCS_CMD_DLAND:
			if( gcs_hover_enter() )
			{
				set_stop_brake(PAUSE_BRAKE);
				spray_break_and_continual();
			}
			else
			{
				gcs_state = GCS_RUN_LANDING;
			}
			break;

		case GCS_CMD_LOCK:
			gcs_state = GCS_RUN_LOCK;
			if(!autopilot_rc) NavKillMode();		//add by lg
			//NavKillThrottle();  //crash motion
			break;
			
		default:
			gcs_state = GCS_RUN_ERROR;
			task_error_state = TASK_PARSE_ERROR;
			break;
	}
	
	last_task_cmd = gcs_task_cmd;

	return gcs_state;
}

/***********************************************************************
* FUNCTION    : save_task_scene
* DESCRIPTION : unnormal operation, save current pos
* INPUTS      : none
* RETURN      : current pos
***********************************************************************/
struct EnuCoor_i save_task_scene(void)
{
	current_wp_scene = *stateGetPositionEnu_i();

	/*interrupt, need stop spray*/
   #ifdef OPS_OPTION
	ops_stop_spraying(); 
   #endif
	
	return current_wp_scene;
}

/***********************************************************************
* FUNCTION    : gcs_hover_enter
* DESCRIPTION : hover motion prepare and set
* INPUTS      : none
* RETURN      : TRUE or FALSE
***********************************************************************/
bool_t gcs_hover_enter(void)
{
	static uint8_t steady_counter = 0;

	if( last_task_cmd != gcs_task_cmd)
	{  
		save_task_scene();
		steady_counter = 0;
		return TRUE;
	}
	
	if( steady_counter <= 5 )
	{
		if( NavGetHoverSteady() )
		{
			steady_counter++;
		}
		if(steady_counter == 6)  //make sure hover steady, set navigation_target here
		{
			interrupt_wp_scene = *stateGetPositionEnu_i();
			task_nav_update_target(interrupt_wp_scene);
		}
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/***********************************************************************
* FUNCTION    : set_auto_stop_brake
* DESCRIPTION : give brake accel grade according to the situation 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void set_auto_stop_brake(void)
{
	if(last_task_cmd != gcs_task_cmd)
	{
		if(manual_pause_flag)
		{
			set_stop_brake(URGENT_BRAKE);
			manual_pause_flag = FALSE;
		}
		else if(em_alert_grade < 2)
		{
			set_stop_brake(SMOOTH_BRAKE);
		}
		else if(em_alert_grade == 2)
		{
			set_stop_brake(PAUSE_BRAKE);
		}
		else
		{
			set_stop_brake(URGENT_BRAKE);
		}
	}
}

/*reseve*/
void get_shortest_reland_wp(void)
{
	float distance_cur_reland[NB_RESERVE_LAND];
	struct EnuCoor_i wp_reland;
	
	for(uint8_t i=0; i<nb_pending_reland; i++)
	{
		VECT2_COPY(wp_reland, wp_reserve_land[i]);
		distance_cur_reland[i] = get_dist2_to_point( &wp_reland );
	}
	
	VECT2_COPY(reland_wp_enu, wp_reserve_land[0]);
	for(uint8_t j=1; j<nb_pending_reland; j++)
	{		
		if( distance_cur_reland[j] < distance_cur_reland[j-1] )
		{
			VECT2_COPY(reland_wp_enu, wp_reserve_land[j]);
		}
	}
}

/***********************************************************************
* FUNCTION    : run_normal_task
* DESCRIPTION : execute task according to from_wp.action
* INPUTS      : none
* RETURN      : TRUE or FALSE
***********************************************************************/
bool_t run_normal_task(void)
{
	uint8_t wp_state = 2; /*default:run over from_wp state*/
	spray_work_run();
	AC_action = from_wp.action;
	switch(from_wp.action)
	{
		case FLIGHT_LINE:
		{
			if(task_nav_pre_path(from_wp.wp_en, next_wp.wp_en, FLIGHT_PATH))
			{
				spray_switch_flag = FALSE;
				if( !task_nav_path(from_wp.wp_en, next_wp.wp_en) ) 
				{
					if(SPRAY_LINE==next_wp.action) /*if next action is spray line,make sure start point is exact*/
					{
						task_nav_hover(next_wp.wp_en);
					}
					if(stateGetHorizontalSpeedNorm_f() < 0.3) /*make sure hover motion setted*/
					{
						if( !achieve_next_wp() )  
						{
							task_wp_empty_handle();
						}
						else
						{
							wp_state = 1;  /*interrupt reaching next_wp msg to gcs*/
						}
					}
				}
			}
			break;
		}
			
		case SPRAY_LINE:
		{
			if(task_nav_pre_path(from_wp.wp_en, next_wp.wp_en, SPRAY_PATH))
			{
				spray_switch_flag = TRUE;  //AC aligned in spray_line,begin spray
				if( !task_nav_path(from_wp.wp_en, next_wp.wp_en) ) 
				{	
					if(spray_switch_flag)
					{
						current_sprayed_distance = 0.0f;  //reset
						sprayed_lines_distance += sqrt(get_dist2_p2p(from_wp.wp_en, next_wp.wp_en));
					}
					spray_switch_flag = FALSE;  //AC end of spray_line, stop spray
					
					if( !achieve_next_wp() )  
					{
						task_wp_empty_handle();
					}
					else
					{
						wp_state = 1;
						spray_caculate_flag = FALSE;   /*reset spray caculate flag for two continual spray line*/
					}
				}
				else
				{
					current_sprayed_distance = sqrt(get_dist2_to_point(&(from_wp.wp_en)));   //caculate sprayed distance of curent line
					ops_info.sum_sprayed_distance = current_sprayed_distance + sprayed_lines_distance;
				}
			}
			break;
		}
			
		case SPRAY_CONVERT:
		{
			if( ac_config_info.spray_convert_type == CIRCLE_CONVERT )
			{
				if(spray_convert_info.useful)
				{
					/*if finish convert,get next_wp*/
					if( nav_spray_convert(spray_convert_info.center, 
											 spray_convert_info.radius, 
											 spray_convert_info.heading_sp) )
					{
						/*no more task_wp to run*/
						if( !achieve_next_wp() )  
						{
							task_wp_empty_handle();
						}
						else
						{
							spray_convert_info.useful = FALSE;  /*reset caculate*/
							wp_state = 1;
						}					
					}
				}
				else
				{
					//convert fail
					task_error_state = TASK_PARSE_ERROR;
				}
			}
			else  //ac_config_info.spray_convert_type==WAYPOINT_FORWARD/P2P
			{
				//task_nav_wp(struct EnuCoor_i first_wp)
				/*if finish convert,get next_wp*/
				if( !task_nav_path(from_wp.wp_en, next_wp.wp_en) )
				{
					/*no more task_wp to run*/
					if( !achieve_next_wp() )  
					{
						task_wp_empty_handle();
					}
					else
					{
						wp_state = 1;
					}					
				}
			}
			break;
		}
			
		case HOVERING:
		{
			//how to continual?
			task_nav_hover(from_wp.wp_en);
			break;
		}
			
		case TERMINATION:
		{
			if( hover_flag )  
			{
				spray_switch_flag = FALSE;
				task_nav_hover(from_wp.wp_en);
				if(stateGetHorizontalSpeedNorm_f() < 0.3) /*make sure hover motion setted*/
				{
					hover_flag = FALSE;
					VECT2_COPY(home_wp_enu, wp_home);	
				}
			}
			else
			{
				if(task_nav_pre_path(from_wp.wp_en, home_wp_enu, FLIGHT_PATH))
				{
					if( !task_nav_path(from_wp.wp_en, home_wp_enu) ) 
					{
						wp_state = 1;
						task_nav_hover(home_wp_enu);
						return TRUE;  /*task is finished, next to do land*/
					}
				}
			}
			break;
		}
			
		default:
			break;				
	}
	
	send_current_task(wp_state);
	
	return FALSE;  /*task is running*/
}

/***********************************************************************
* FUNCTION    : spray_work_run
* DESCRIPTION : spray work control
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void spray_work_run(void)
{
	/*spray switch control*/
	/*if( ((SPRAY_LINE==from_wp.action)||(SPRAY_CONVERT== from_wp.action))
	    && !get_spray_switch_state() 
	    && get_nav_route_mediacy() 
	    && spray_switch_flag==TRUE ) */
	if( !get_spray_switch_state() 
	    && spray_switch_flag==TRUE )
	{
	   #ifdef OPS_OPTION
		ops_start_spraying(); 
	   #endif 	
	}
	/*else if( (SPRAY_LINE!=from_wp.action || !get_nav_route_mediacy())
		      && get_spray_switch_state() )*/
	/*else if( (!get_nav_route_mediacy())
		      || (get_spray_switch_state() 
		      &&spray_switch_flag == FALSE))*/
	else if( (get_spray_switch_state() 
		      &&spray_switch_flag == FALSE))
	{
	   #ifdef OPS_OPTION
		ops_stop_spraying(); 
	   #endif 	
	}

	/*convert info pre_caculate*/
	if(ac_config_info.spray_convert_type == CIRCLE_CONVERT)
	{
		if( SPRAY_LINE==from_wp.action && FALSE==spray_caculate_flag)
		{
			uint8_t spray_convert_state = spray_convert_caculate();
			switch(spray_convert_state)
			{
				case SPRAY_CONVERT_SUCCESS:
					spray_convert_info.useful = TRUE;
					spray_caculate_flag = TRUE;
					break;
					
				case SPRAY_CONVERT_CONTINUAL:
					spray_convert_info.useful = FALSE;
					break;
					
				case SPRAY_CONVERT_FAIL:
					spray_convert_info.useful = FALSE;
					spray_caculate_flag = TRUE;
					break;
				default:
					break;
			}		
			#if 0//PERIODIC_TELEMETRY
		    xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	        DOWNLINK_SEND_SPRAY_CONVERT_INFO(DefaultChannel, DefaultDevice, 
				                &spray_convert_info.center.x,
				                &spray_convert_info.center.y,
				                &spray_convert_info.radius,
				                &spray_convert_info.heading_sp,
				                &spray_convert_info.useful   );
			#endif
		}
		else if( SPRAY_LINE != from_wp.action && TRUE==spray_caculate_flag)
		{
			spray_caculate_flag = FALSE;   /*reset spray caculate flag for next spray convert*/
		}
	}
	
}

/***********************************************************************
* FUNCTION    : task_wp_empty_handle
* DESCRIPTION : no task handle
* INPUTS      : none
* RETURN      : 
***********************************************************************/
bool_t task_wp_empty_handle(void)
{
	/*task break or finish*/
	if( HOVERING==next_wp.action || TERMINATION==next_wp.action )
	{
		/*waypoint info move from next to from*/
		VECT2_COPY(from_wp.wp_en, next_wp.wp_en);
		from_wp.action = next_wp.action;
		from_wp.wp_id = next_wp.wp_id;
		hover_flag = TRUE;
		send_current_task(1);
		return TRUE;
	}
	else
	{
		//no more task_wp exeption generate
		task_error_state = TASK_RUN_OVER;
		return FALSE;
	}
}

/***********************************************************************
* FUNCTION    : set_path_flight_info
* DESCRIPTION : according to the action get flight height and speed
* INPUTS      : waypoint action
* RETURN      : flight height
***********************************************************************/
static float set_path_flight_info(uint8_t type)
{
  switch(type)
  {
  		case FLIGHT_PATH:
			nav_set_flight_speed(ac_config_info.max_flight_speed);
			return ac_config_info.max_flight_height;
			
		case SPRAY_PATH:
			nav_set_flight_speed(ac_config_info.spray_speed);
			return ac_config_info.spray_height;
			
		default:
			return ac_config_info.max_flight_height;
  }
}

/***********************************************************************
* FUNCTION    : task_nav_update_target
* DESCRIPTION : set current posion to navigation target
* INPUTS      : 
* RETURN      : 
***********************************************************************/
static inline bool_t task_nav_update_target(struct EnuCoor_i enu_wp)
{
   VECT3_COPY(navigation_target, enu_wp);
   return FALSE;
}

/** Navigation function hover flight
*/
static inline bool_t task_nav_hover(struct EnuCoor_i hover_wp)
{
   horizontal_mode = HORIZONTAL_MODE_WAYPOINT;

   VECT3_COPY(navigation_target, hover_wp);
   NavVerticalAltitudeMode(ac_config_info.max_flight_height, 0.);
   
   return FALSE;
}

/** Navigation function to a single waypoint
*/
static inline bool_t task_nav_wp(struct EnuCoor_i first_wp)
{
  struct EnuCoor_i target_wp = first_wp;

  if (nav_approaching_target(&target_wp, NULL, 0.5)) 
  {
    return FALSE;
  }
  
  //Go to Target Waypoint
  horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
  VECT3_COPY(navigation_target, target_wp);

  
  return TRUE;
}


/** Navigation function along a segment
*/
static inline bool_t task_nav_pre_path(struct EnuCoor_i p_start_wp, struct EnuCoor_i p_end_wp, uint8_t flight_type)
{
  static float flight_height_last = 0.0f;  //use reord last time flight height,once changed it will adjust the height before forword flight

  float flight_height = set_path_flight_info(flight_type); 
  NavVerticalAltitudeMode(flight_height, 0.);  //set flight height setpoint
  
  if(flight_height_last != flight_height)
  {
  	  height_align = FALSE;
  }
  flight_height_last = flight_height;

  if( !height_align )
  {
  	  if( nav_check_height() )
  	  {
	  	  height_align = TRUE;
  	  }
  }

  if( ac_config_info.spray_convert_type == WAYPOINT_P2P
  	   && flight_type == SPRAY_PATH                      )
  {
  	nav_set_heading_parallel_line(&p_start_wp, &p_end_wp);
  }
  else
  {
  	nav_set_heading_forward_line(&p_start_wp, &p_end_wp); 
  }
  
  heading_align = nav_check_heading();
  if( height_align && heading_align )
  {
  	return TRUE;
  }
  return FALSE;
}

/** path run,set horizontal_mode to ROUTE
*/
static inline bool_t task_nav_path(struct EnuCoor_i p_start_wp, struct EnuCoor_i p_end_wp)
{

#ifdef USE_PLANED_OA
	if( planed_oa.test_on )
	{
		bool_t oa_task_flag = oa_task_nav_path( p_start_wp, p_end_wp );
		return oa_task_flag;
	}
	else
	{
		if( nav_approaching_from(&p_end_wp, &p_start_wp, 0) )
		{
			return FALSE;
		}
	}
#else
	//Check proximity and wait for 'duration' seconds in proximity circle if desired 
	if( nav_approaching_from(&p_end_wp, &p_start_wp, 0) ) 
	{
		return FALSE;
	}
#endif

  horizontal_mode = HORIZONTAL_MODE_ROUTE;
  nav_route(&p_start_wp, &p_end_wp);
  //NavVerticalAltitudeMode(flight_height, 0.);  
  return TRUE;
  
}

/** for debug
*/
void send_task_info_pc(void) 
{
	#if PERIODIC_TELEMETRY
	static uint8_t i = 0;
	if( nb_pending_wp==0 )
	{
		return;
	}
	i++;
	if(i >= nb_pending_wp) 
	{
		i=0;
	}
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	DOWNLINK_SEND_TASK_INFO(DefaultChannel, DefaultDevice,
	   	                              &nb_pending_wp,
	   	                              &task_wp[i].wp_id,
	   	                              &task_wp[i].action,
	   	                              &task_wp[i].wp_en.x,
	   	                              &task_wp[i].wp_en.y);

#if 0	
    static uint8_t j = 0;    
    static uint8_t k = 2;
	if(k!=2)
	{
		k=2;
		j++;
	}
	else
	{
		k=1;
	}
	if(j >= nb_pending_wp) 
	{
		j=0;
	}
	uint8_t nb_unexecuted_wp = 1;
	uint8_t wp_state = j%2+1;
	uint16_t system_time = sys_time.nb_sec;
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_GCS);
	DOWNLINK_SEND_CURRENT_TASK_STATE(SecondChannel, SecondDevice,
	   	                              &system_time,
	   	                              &task_wp[j].wp_id,
	   	                              &task_wp[j].action,	
	   	                              &k,
	   	                              &nb_unexecuted_wp);
#endif
   #endif
}

/***********************************************************************
* FUNCTION    : send_current_task_state
* DESCRIPTION : periodic send task state to gcs
* INPUTS      : state express in line or out of line
* RETURN      : 
***********************************************************************/
void send_current_task_state(uint8_t wp_state) 
{
	uint16_t system_time = sys_time.nb_sec;
	uint8_t nb_unexecuted_wp;
	if(TASK_RUN_OVER==task_error_state)
	{
		nb_unexecuted_wp = 0;
	}
	else
	{
		nb_unexecuted_wp = nb_pending_wp + 1;
	}
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_GCS);
	DOWNLINK_SEND_CURRENT_TASK_STATE(SecondChannel, SecondDevice,
	   	                              &system_time,
	   	                              &from_wp.wp_id,
	   	                              &from_wp.action,
	   	                              &wp_state,
	   	                              &nb_unexecuted_wp);
}

void send_current_task(uint8_t wp_state)
{
	if(1 == wp_state)/*reach from_wp,interrupt send*/
	{
		send_current_task_state(wp_state);
	}
	else  /*send 2s periodic*/
	{
		RunOnceEvery(64, send_current_task_state(wp_state));  
		//RunOnceEvery(50, send_task_info_pc());
	}
	#if 0//PERIODIC_TELEMETRY
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	DOWNLINK_SEND_DEBUG_TASK(DefaultChannel, DefaultDevice,
	   	                              &heading_align,
	   	                              &height_align        );
	#endif
}





/****************************** END OF FILE ***************************/
