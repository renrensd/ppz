
/** @file subsystems/mission/mission_manage.h
 *  @brief mission messages parser for mission interface,and saving the mission information
 */


#ifndef MISSION_MANAGE_H
#define MISSION_MANAGE_H

#include "std.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_int.h"
#include "subsystems/datalink/datalink_ack.h"   //get ms_info struct
#include "mcu_periph/sys_time.h"

#define MS_REPORT_FRE 1
enum MissionType {
  //in message_new.xml
  //mission_type  0:flight_path; 1:spray_normal; 2.spray_insert; 3:normal_home_land;4:hover; 5:emergency_home_land; 6:land_reserve; 7:land_force
  ms_path=0,
  ms_spray_normal=1,
  ms_spray_insert=2,
  ms_home=3,
  ms_hover=4,
  ms_emer_home=5,
  ms_land_reserve=6,
  ms_land_force=7,
  
  ms_avoidance=10,
  ms_circle=11
};

enum mission_cmd {  
  Add=0,        
  Updata=1,       
  Delete=2,  
  Get=3,
  Clear
};

//pointer,use save all mission waypoints
union pt_wp {
  struct EnuCoor_i *wp_i;
  struct EnuCoor_f *wp_f;
}; 

union ms_wp {  //save enu coord data,union
  struct EnuCoor_i wp_i;  
  struct EnuCoor_f wp_f;
}; 

struct _mission_wp {
  union {
    struct EnuCoor_f wp_f;
    struct EnuCoor_i wp_i;
  } wp;
};

struct _mission_circle {
  union {
    struct EnuCoor_f center_f;
    struct EnuCoor_i center_i;
  } center;

  float radius;
};

struct _mission_path {
  struct EnuCoor_i  *path_p;  //mission pointer
  uint8_t path_idx;           //present flight line sequence
  uint8_t nb_wp;              //present wp_path number
  uint8_t nb_rsland; 
};

//add for survey mission
//#define MISSION_SURVEY_NB 6
struct _mission_survey {
  struct EnuCoor_i   *survey_p; //mission pointer
  uint8_t survey_idx;           //present survey line sequence
  uint8_t nb_survey;
  uint8_t nb_rsland; 
  bool_t  survey_insert;
};

enum element_status
{
	standby=0, 
	running=1,
	success_done=2,
	fail_done=3,
	pause=4,
	stop=5
};

struct _mission_element {
  enum MissionType type;
  union {
    struct _mission_wp mission_wp;
    struct _mission_circle mission_circle;
    //struct _mission_segment mission_segment;
    struct _mission_path mission_path;
	struct _mission_survey mission_survey;
  } element;
  //mission_status: 
  enum element_status status;
  float duration; ///< time to spend in the element (<= 0 to disable)
  bool_t element_exist;
};

/** Max number of elements in the tasks' list
 *  can be redefined
 */
#ifndef MISSION_ELEMENT_NB
#define MISSION_ELEMENT_NB 20
#endif

struct _mission {
  struct  _mission_element elements[MISSION_ELEMENT_NB];  //normal idx begin from 1, 0 use for reland
  float   element_time;   ///< time in second spend in the current element,caculate in mission_run()
  uint8_t add_idx;        ///< last index
  uint8_t current_idx;    ///< current mission element index
};

extern struct _mission mission;

/** Init mission structure
*/
extern void mission_init(void);

/** Get current mission element
 * @return return a pointer to the next mission element or NULL if no more elements
 */
extern struct _mission_element *get_mission(void);

/** Get the ENU component of LLA mission point
 * This function is firmware specific.
 * @param point pointer to the output ENU point (float)
 * @param lla pointer to the input LLA coordinates (int)
 * @return TRUE if conversion is succesful, FALSE otherwise
 */
extern bool_t mission_point_of_lla(struct EnuCoor_f *point, struct LlaCoor_f *lla);

/** Run mission
 *
 * This function should be implemented into a dedicated file since
 * navigation functions are different for different firmwares
 * Currently, this function should be called from the flight plan
 *
 * @return return TRUE when the mission is running, FALSE when it is finished
 */
extern uint8_t mission_run(void);

/** Report mission status
 *
 * Send mission status over datalink
 */
//extern void mission_status_report(void);

/** Parsing functions called when a mission message is received
*/
extern bool_t mission_clear_all(void);
extern int8_t mission_add_parse(struct mission_info ms_info);
extern int8_t mission_add_type_parse(struct mission_info ms_info,uint8_t ms_id,uint8_t len_data);
extern int8_t mission_update_parse(struct mission_info ms_info);
extern int8_t mission_get(uint8_t mission_id, uint8_t wp_type);
extern int8_t mission_delete(uint8_t mission_id);	
extern uint8_t get_mission_executable(void);
extern void send_current_mission(void);

#endif // MISSION_COMMON_H

