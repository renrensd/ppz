/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				       *
*				  All Rights Reserved.          					   *
*   Department 	: R&D SW      									       *
*   AUTHOR	   	:            										   *
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
#ifndef _FRAM_IF_H_
#define _FRAM_IF_H_
#include "fm25v.h"

/**** Definition of constants ****/
extern const uint8_t fram_init_flags[4];

/**** Definition of types ****/ 
struct FRAM_INFO
{
	struct FM25V_SPI fm25v;                  /* The fram chip */
	
};

struct FRAM_ERROR_INFO
{
	bool_t read_data_fail;
	bool_t write_data_fail;
	bool_t data_wrong;
};
extern struct FRAM_ERROR_INFO fram_error;

enum FRAM_DATA_INIT_SECTION_TYPE
{
	FRAM_DATA_INIT_SECTION_ONE,

	/*Add some section here*/
	FRAM_DATA_INIT_SECTION_MAX
};

/**** Definition of macros ****/
#define FRAM_AC_PARAM_ADDRESS	0x2000


/**** Declaration of functions ****/
extern void fram_init(void);
extern void fram_write_test(uint8_t val);
extern void fram_read_test(uint8_t val);

extern uint8_t fram_id_write(uint8_t id, uint8_t *write_buffer);
extern uint8_t fram_write(uint8_t id, uint16_t item, uint8_t *write_buffer);
extern uint8_t fram_read (uint8_t id, uint16_t item, uint8_t *read_buffer);
extern uint8_t fram_id_read (uint8_t id, uint8_t *read_buffer);
extern uint8_t fram_ac_param_read(uint8_t *read_buffer, uint16_t len);
extern uint8_t fram_ac_param_write(uint8_t *write_buffer, uint16_t len);
extern uint8_t fram_mag_cali_data_read(uint8_t *read_buffer);
extern uint8_t fram_mag_cali_data_write(uint8_t *write_buffer);
extern uint8_t fram_mag_cali_default_data_write(void);
extern uint8_t fram_acc_cali_data_read(uint8_t *write_buffer);
extern uint8_t fram_acc_cali_data_write(uint8_t *write_buffer);
extern uint8_t fram_sn_data_write(uint8_t *write_buffer);
extern uint8_t fram_sn_data_read(uint8_t *write_buffer);

#ifdef UPGRADE_OPTION
extern uint8_t fram_write_swdl_mask(void);
extern uint8_t fram_erase_swdl_mask (void);
extern uint8_t fram_read_swdl_mask (uint8_t* pBlockData);
extern bool_t fram_update_is_available(void);
#endif	/* UPGRADE_OPTION */
extern void fram_init_all_data(void);

extern void fram_mag_cali_get(void);
extern void fram_acc_cali_get(void);

#endif //_FRAM_IF_H_

