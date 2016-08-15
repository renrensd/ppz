/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : R&D SW      									   *
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
#ifndef _BAT_MANAGER_H_
#define _BAT_MANAGER_H_ 
#include BOARD_CONFIG

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>

#include "mcu_periph/gpio.h"
#include "mcu_periph/sys_time.h"

#include "std.h"
#include "mcu_periph/i2c.h"
#include "math/pprz_algebra_int.h"


/**** Definition of constants ****/
#define BQ76940_OPTION

#define BQ76920_ADDR   		0x30	/* 8bit write address for i2c. */
#define BQ76930_ADDR   		0x10	/* 8bit write address for i2c. */
#define BQ76940_ADDR   		0x10	/* 8bit write address for i2c. */
#define BQ34Z100_ADDR    	0xAA	/* 8bit write address for i2c. */

#define BAT_INFO_WDG_CNT	240	/* 1s. */		
/** config status states */
enum BQ769XCONFIG
{
	BQ76920_CONFIG,
	BQ76930_CONFIG,
	BQ76940_CONFIG
};

#ifdef BQ76920_OPTION
enum BQ7692CONFSTAUS
{
  	BQ7692_CONF_UNINIT,
  	BQ7692_CONF_READ_GAIN1,
  	BQ7692_CONF_READ_GAIN1_OK,
  	BQ7692_CONF_READ_GAIN2,
  	BQ7692_CONF_READ_GAIN2_OK,
  	BQ7692_CONF_READ_OFFSET,
  	BQ7692_CONF_READ_OFFSET_OK,
  	BQ7692_CONF_REG1,
  	BQ7692_CONF_REG2,
  	BQ7692_CONF_REG3,
  	BQ7692_CONF_REG4,
  	BQ7692_CONF_REG5,
  	BQ7692_CONF_REG6,
  	BQ7692_CONF_REG7,
  	BQ7692_CONF_REG8,
  	
  	BQ7692_CONF_DONE
};

enum BQ7692READSTAUS
{
  	BQ7692_READ_UNINIT,
	BQ7692_READ_GAIN1,
	BQ7692_READ_GAIN1_OK,
	BQ7692_READ_GAIN2,
	BQ7692_READ_GAIN2_OK,
	BQ7692_READ_OFFSET,
	BQ7692_READ_OFFSET_OK,
  	BQ7692_READ_REG1,
  	BQ7692_READ_REG1_OK,
  	BQ7692_READ_REG2,
  	BQ7692_READ_REG2_OK,
  	BQ7692_READ_REG3,
  	BQ7692_READ_REG3_OK,
  	BQ7692_READ_REG4,
  	BQ7692_READ_REG4_OK,
  	BQ7692_READ_REG5,
  	BQ7692_READ_REG5_OK,
  	BQ7692_READ_REG6,
  	BQ7692_READ_REG6_OK,
  	BQ7692_READ_REG7,
  	BQ7692_READ_REG7_OK,
  	BQ7692_READ_REG8,
  	BQ7692_READ_REG8_OK,
  	
  	BQ7692_READ_DONE
};
#endif	/* BQ76920_OPTION */

#ifdef BQ76930_OPTION
enum BQ7693CONFSTAUS
{
  	BQ7693_CONF_UNINIT,
  	BQ7693_CONF_READ_GAIN1,
  	BQ7693_CONF_READ_GAIN1_OK,
  	BQ7693_CONF_READ_GAIN2,
  	BQ7693_CONF_READ_GAIN2_OK,
  	BQ7693_CONF_READ_OFFSET,
  	BQ7693_CONF_READ_OFFSET_OK,
  	BQ7693_CONF_REG1,
  	BQ7693_CONF_REG2,
  	BQ7693_CONF_REG3,
  	BQ7693_CONF_REG4,
  	BQ7693_CONF_REG5,
  	BQ7693_CONF_REG6,
  	BQ7693_CONF_REG7,
  	BQ7693_CONF_REG8,
  	
  	BQ7693_CONF_DONE
};

enum BQ7693READSTAUS
{
  	BQ7693_READ_UNINIT,
	BQ7693_READ_GAIN1,
	BQ7693_READ_GAIN1_OK,
	BQ7693_READ_GAIN2,
	BQ7693_READ_GAIN2_OK,
	BQ7693_READ_OFFSET,
	BQ7693_READ_OFFSET_OK,
  	BQ7693_READ_REG1,
  	BQ7693_READ_REG1_OK,
  	BQ7693_READ_REG2,
  	BQ7693_READ_REG2_OK,
  	BQ7693_READ_REG3,
  	BQ7693_READ_REG3_OK,
  	BQ7693_READ_REG4,
  	BQ7693_READ_REG4_OK,
  	BQ7693_READ_REG5,
  	BQ7693_READ_REG5_OK,
  	BQ7693_READ_REG6,
  	BQ7693_READ_REG6_OK,
  	BQ7693_READ_REG7,
  	BQ7693_READ_REG7_OK,
  	BQ7693_READ_REG8,
  	BQ7693_READ_REG8_OK,
	BQ7693_READ_REG9,
	BQ7693_READ_REG9_OK,
	BQ7693_READ_REG10,
	BQ7693_READ_REG10_OK,
  	BQ7693_READ_REG11,
  	BQ7693_READ_REG11_OK,
	BQ7693_READ_REG12,
	BQ7693_READ_REG12_OK,
	BQ7693_READ_REG13,
	BQ7693_READ_REG13_OK,
  	BQ7693_READ_REG14,
  	BQ7693_READ_REG14_OK,
  	
  	BQ7693_READ_DONE
};
#endif	/* BQ76920_OPTION */

#ifdef BQ76940_OPTION
enum BQ7694CONFSTAUS
{
  	BQ7694_CONF_UNINIT,
  	BQ7694_CONF_READ_GAIN1,
  	BQ7694_CONF_READ_GAIN1_OK,
  	BQ7694_CONF_READ_GAIN2,
  	BQ7694_CONF_READ_GAIN2_OK,
  	BQ7694_CONF_READ_OFFSET,
  	BQ7694_CONF_READ_OFFSET_OK,
  	BQ7694_CONF_REG1,
  	BQ7694_CONF_REG2,
  	BQ7694_CONF_REG3,
  	BQ7694_CONF_REG4,
  	BQ7694_CONF_REG5,
  	BQ7694_CONF_REG6,
  	BQ7694_CONF_REG7,
  	BQ7694_CONF_REG8,
  	
  	BQ7694_CONF_DONE
};

enum BQ7694READSTAUS
{
  	BQ7694_READ_UNINIT,
	BQ7694_READ_GAIN1,
	BQ7694_READ_GAIN1_OK,
	BQ7694_READ_GAIN2,
	BQ7694_READ_GAIN2_OK,
	BQ7694_READ_OFFSET,
	BQ7694_READ_OFFSET_OK,
  	BQ7694_READ_REG1,
  	BQ7694_READ_REG1_OK,
  	BQ7694_READ_REG2,
  	BQ7694_READ_REG2_OK,
  	BQ7694_READ_REG3,
  	BQ7694_READ_REG3_OK,
  	BQ7694_READ_REG4,
  	BQ7694_READ_REG4_OK,
  	BQ7694_READ_REG5,
  	BQ7694_READ_REG5_OK,
  	BQ7694_READ_REG6,
  	BQ7694_READ_REG6_OK,
  	BQ7694_READ_REG7,
  	BQ7694_READ_REG7_OK,
  	BQ7694_READ_REG8,
  	BQ7694_READ_REG8_OK,
	BQ7694_READ_REG9,
	BQ7694_READ_REG9_OK,
	BQ7694_READ_REG10,
	BQ7694_READ_REG10_OK,
  	BQ7694_READ_REG11,
  	BQ7694_READ_REG11_OK,
	BQ7694_READ_REG12,
	BQ7694_READ_REG12_OK,
	BQ7694_READ_REG13,
	BQ7694_READ_REG13_OK,
  	BQ7694_READ_REG14,
  	BQ7694_READ_REG14_OK,
  	BQ7694_READ_REG15,
  	BQ7694_READ_REG15_OK,
  	BQ7694_READ_REG16,
  	BQ7694_READ_REG16_OK,
  	BQ7694_READ_REG17,
  	BQ7694_READ_REG17_OK,
  	BQ7694_READ_REG18,
  	BQ7694_READ_REG18_OK,
	BQ7694_READ_REG19,
	BQ7694_READ_REG19_OK,
	BQ7694_READ_REG20,
	BQ7694_READ_REG20_OK,
  	BQ7694_READ_REG21,
  	BQ7694_READ_REG21_OK,
	BQ7694_READ_REG22,
	BQ7694_READ_REG22_OK,
	BQ7694_READ_REG23,
	BQ7694_READ_REG23_OK,
  	BQ7694_READ_REG24,
  	BQ7694_READ_REG24_OK,
  	BQ7694_READ_REG25,
  	BQ7694_READ_REG25_OK,
  	BQ7694_READ_REG26,
  	BQ7694_READ_REG26_OK,
  	
  	BQ7694_READ_DONE
};
#endif	/* BQ76940_OPTION */

enum BQ34ZCONFSTAUS
{
  	BQ34Z_CONF_UNINIT,
  	BQ34Z_CONF_REG1,
  	BQ34Z_CONF_REG2,
  	BQ34Z_CONF_DONE
};

enum BQ34ZREADSTAUS
{
  	BQ34Z_READ_UNINIT,
  	BQ34Z_READ_REG1,
  	BQ34Z_READ_REG1_OK,
  	BQ34Z_READ_REG2,
  	BQ34Z_READ_REG2_OK,
  	BQ34Z_READ_DONE
};

/**** Declaration of variables ****/
#ifdef BQ76920_OPTION
struct BQ76920
{
	struct i2c_periph *i2c_p;
  	struct i2c_transaction i2c_trans;
  	bool_t initialized;                 ///< config done flag
  	enum BQ7692CONFSTAUS init_status; ///< init status
  	enum BQ7692READSTAUS read_status;
  	volatile bool_t data_available;     ///< data ready flag
  	uint8_t buf[20];
	uint8_t gain1;
	uint8_t gain2;
	uint8_t offset;
};
#endif	/* BQ76920_OPTION */

#ifdef BQ76930_OPTION
struct BQ76930
{
	struct i2c_periph *i2c_p;
  	struct i2c_transaction i2c_trans;
  	bool_t initialized;                 ///< config done flag
  	enum BQ7693CONFSTAUS init_status; ///< init status
  	enum BQ7693READSTAUS read_status;
  	volatile bool_t data_available;     ///< data ready flag
  	uint8_t buf[20];
	uint8_t gain1;
	uint8_t gain2;
	uint8_t offset;
};
#endif	/* BQ76930_OPTION */

#ifdef BQ76940_OPTION
struct BQ76940
{
	struct i2c_periph *i2c_p;
  	struct i2c_transaction i2c_trans;
  	bool_t initialized;                 ///< config done flag
  	enum BQ7694CONFSTAUS init_status; ///< init status
  	enum BQ7694READSTAUS read_status;
  	volatile bool_t data_available;     ///< data ready flag
  	uint8_t buf[40];
	uint8_t gain1;
	uint8_t gain2;
	uint8_t offset;
};
#endif	/* BQ76940_OPTION */


struct BQ34Z100
{
	struct i2c_periph *i2c_p;
  	struct i2c_transaction i2c_trans;
  	bool_t initialized;                 ///< config done flag
  	enum BQ34ZCONFSTAUS init_status; ///< init status
    enum BQ34ZREADSTAUS read_status; ///< init status
  	volatile bool_t data_available;     ///< data ready flag
  	uint8_t buf[20];
};

struct BAT_INFO
{
	uint8_t ov_trip_2;
	uint8_t uv_trip_2;
    uint8_t ov_trip_3;
	uint8_t uv_trip_3;
	uint8_t ov_trip_4;
	uint8_t uv_trip_4;
	uint16_t gain_2;
	uint16_t offset_2;
	uint16_t gain_3;
	uint16_t offset_3;
	uint16_t gain_4;
	uint16_t offset_4;
	uint16_t vc3[4];
	uint16_t vc6[7];
	uint16_t vc12[13];
	uint16_t vcc3;
	uint16_t vcc6;
	uint16_t vcc12;
	uint16_t percent;
	uint16_t remaining;
	uint16_t fullcharge;
	uint16_t volt;
	int32_t curr;
	int32_t avrcurr;
	float temp;
	uint16_t ov;
	uint16_t uv;
	uint16_t wdg_cnt;
	
	struct BQ34Z100 bq34z;
#ifdef BQ76930_OPTION
    struct BQ76930 bq7693;
#endif
#ifdef BQ76920_OPTION
	struct BQ76920 bq7692;
#endif
#ifdef BQ76940_OPTION
	struct BQ76940 bq7694;
#endif
};

extern struct BAT_INFO bat_info;


/**** Declaration of functions ****/
#ifdef BQ76920_OPTION
void bq7692_start_configure(struct BQ76920 *bq7692);
void bq7692_read(struct BQ76920 *bq7692);
#endif	/* BQ76920_OPTION */

#ifdef BQ76930_OPTION
void bq7693_start_configure(struct BQ76930 *bq7693);
void bq7693_read(struct BQ76930 *bq7693);
#endif	/* BQ76930_OPTION */

#ifdef BQ76940_OPTION
void bq7694_start_configure(struct BQ76940 *bq7694);
void bq7694_read(struct BQ76940 *bq7694);
#endif	/* BQ76940_OPTION */

void bq34z_start_configure(struct BQ34Z100*bq34z);
void bq34z_read(struct BQ34Z100 *bq34z);

extern void bm_init(void);
extern void bat_periodic(void);
extern void bat_event(void);

/// convenience function: read or start configuration if not already initialized
#ifdef BQ76920_OPTION
static inline void bq7692_periodic(struct BQ76920 *bq7692)
{
  	if(bq7692->initialized) 
	{
    	bq7692_read(bq7692);
  	} 
	else 
	{
    	bq7692_start_configure(bq7692);
  	}
}
#endif	/* BQ76920_OPTION */

#ifdef BQ76930_OPTION
static inline void bq7693_periodic(struct BQ76930 *bq7693)
{
	if(bq7693->initialized) 
	{
    	bq7693_read(bq7693);
  	} 
	else 
	{
    	bq7693_start_configure(bq7693);
  	}
  	
}
#endif	/* BQ76930_OPTION */

#ifdef BQ76940_OPTION
static inline void bq7694_periodic(struct BQ76940 *bq7694)
{
  	if(bq7694->initialized) 
	{
    	bq7694_read(bq7694);
  	} 
	else 
	{
    	bq7694_start_configure(bq7694);
  	}
}
#endif	/* BQ76940_OPTION */

static inline void bq34z_periodic(struct BQ34Z100 *bq34z)
{
  	if(bq34z->initialized) 
	{
    	bq34z_read(bq34z);
  	} 
	else 
	{
    	bq34z_start_configure(bq34z);
  	}
}

#endif /*_BAT_MANAGER_H_*/

/****************************** END OF FILE ***************************/

