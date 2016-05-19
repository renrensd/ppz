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
#include "bat_manager.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/sys_time.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define BM_I2C_DEV i2c1
#ifndef BQ769X_STARTUP_DELAY
#define BQ769X_STARTUP_DELAY 1.0
#endif

/* Private variables ---------------------------------------------------------*/
struct BAT_INFO bat_info;
struct BAT_INFO bat_info_f[3];

uint8_t bat_clear_cnt;
uint8_t ga1=0;
uint8_t ga2=0;
uint8_t off=0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void bq7692_init(struct BQ76920 *bq7692, struct i2c_periph *i2c_p, uint8_t addr);
static void bq7693_init(struct BQ76930 *bq7693, struct i2c_periph *i2c_p, uint8_t addr);
static void bq7694_init(struct BQ76940 *bq7694, struct i2c_periph *i2c_p, uint8_t addr);
static void bq34z_init(struct BQ34Z100 *bq34z, struct i2c_periph *i2c_p, uint8_t addr);
static void bq7692_send_config(struct BQ76920 *bq7692);
static void bq7692_read_reg(struct BQ76920 *bq7692, uint8_t reg, uint8_t len);
static void bq7693_send_config(struct BQ76930 *bq7693);
static void bq7693_read_reg(struct BQ76930 *bq7693, uint8_t reg, uint8_t len);
static void bq7694_send_config(struct BQ76940 *bq7694);
static void bq7694_read_reg(struct BQ76940 *bq7694, uint8_t reg, uint8_t len);
static void bq34z_read_reg(struct BQ34Z100 *bq34z, uint8_t reg, uint8_t len);
static void bq7692_event(struct BQ76920 *bq7692);
static void bq7693_event(struct BQ76930 *bq7693);
static void bq7694_event(struct BQ76940 *bq7694);
static void bq34z_event(struct BQ34Z100 *bq34z);
static uint8_t cal_crc(uint8_t *ptr, uint16_t len);
static void bq7692_write_reg_crc(struct BQ76920 *bq7692, uint8_t reg,uint8_t data);
static void bq7693_write_reg_crc(struct BQ76930 *bq7693, uint8_t reg,uint8_t data);
static void bq7694_write_reg_crc(struct BQ76940 *bq7694, uint8_t reg,uint8_t data);
static void OUV_config(uint8_t model);
static void bq34z_write_reg(struct BQ34Z100 *bq34z, uint8_t reg,uint8_t data);

/* Public functions ----------------------------------------------------------*/

/***********************************************************************
*  Name        : bm_init
*  Description :
*  Parameter   : 
*  Returns     : 
***********************************************************************/
void bm_init(void)////test vo   change
{
	bat_info.ov = 4300;//max
	bat_info.uv = 3000;//min
	//bq7692_init(&bat_info.bq7692, &(BM_I2C_DEV), BQ76920_ADDR);
	//bq7693_init(&bat_info.bq7693, &(BM_I2C_DEV), BQ76930_ADDR);
	bq7694_init(&bat_info.bq7694, &(BM_I2C_DEV), BQ76940_ADDR);
	bq34z_init(&bat_info.bq34z, &(BM_I2C_DEV), BQ34Z100_ADDR);
}
/***********************************************************************
*  Name        : bq7692_init
*  Description : Initialize bq7692 struct and set default config options.
*  Parameter   : bq7692 -  bq7692 struct
				 i2c_p - I2C periperal to use
				 addr - I2C address of bq7692
*  Returns     : 
***********************************************************************/
static void bq7692_init(struct BQ76920 *bq7692, struct i2c_periph *i2c_p, uint8_t addr)
{
  	/* set i2c_peripheral */
  	bq7692->i2c_p = i2c_p;
  	/* set i2c address */
  	bq7692->i2c_trans.slave_addr = addr;
  	bq7692->i2c_trans.status = I2CTransDone;
  	/* set default config options */
  	bq7692->initialized = FALSE;
  	bq7692->init_status = BQ7692_CONF_UNINIT;
	bq7692->read_status = BQ7692_READ_UNINIT;
}

/***********************************************************************
*  Name        : bq7693_init
*  Description : Initialize bq7693 struct and set default config options.
*  Parameter   : bq7693 -  bq7693 struct
				 i2c_p - I2C periperal to use
				 addr - I2C address of bq7693
*  Returns     : 
***********************************************************************/
static void bq7693_init(struct BQ76930 *bq7693, struct i2c_periph *i2c_p, uint8_t addr)
{
  	/* set i2c_peripheral */
  	bq7693->i2c_p = i2c_p;
  	/* set i2c address */
  	bq7693->i2c_trans.slave_addr = addr;
  	bq7693->i2c_trans.status = I2CTransDone;
  	/* set default config options */
  	bq7693->initialized = FALSE;
  	bq7693->init_status = BQ7693_CONF_UNINIT;
	bq7693->read_status = BQ7693_READ_UNINIT;
}

/***********************************************************************
*  Name        : bq7694_init
*  Description : Initialize bq7694 struct and set default config options.
*  Parameter   : bq7694 -  bq7694 struct
				 i2c_p - I2C periperal to use
				 addr - I2C address of bq7694
*  Returns     : 
***********************************************************************/
static void bq7694_init(struct BQ76940 *bq7694, struct i2c_periph *i2c_p, uint8_t addr)
{
  	/* set i2c_peripheral */
  	bq7694->i2c_p = i2c_p;
  	/* set i2c address */
  	bq7694->i2c_trans.slave_addr = addr;
  	bq7694->i2c_trans.status = I2CTransDone;
  	/* set default config options */
  	bq7694->initialized = FALSE;
  	bq7694->init_status = BQ7694_CONF_UNINIT;
	bq7694->read_status = BQ7694_READ_UNINIT;
}


/***********************************************************************
*  Name        : bq34z_init
*  Description : Initialize bq34z_init struct and set default config options.
*  Parameter   : bq34z_init -  bq34z_init struct
				 i2c_p - I2C periperal to use
				 addr - I2C address of bq34z_init
*  Returns     : 
***********************************************************************/
static void bq34z_init(struct BQ34Z100 *bq34z, struct i2c_periph *i2c_p, uint8_t addr)
{
  	/* set i2c_peripheral */
  	bq34z->i2c_p = i2c_p;
  	/* set i2c address */
  	bq34z->i2c_trans.slave_addr = addr;
  	bq34z->i2c_trans.status = I2CTransDone;
  	/* set default config options */
  	bq34z->initialized = FALSE;
  	bq34z->init_status = BQ34Z_CONF_UNINIT;
	bq34z->read_status = BQ34Z_READ_REG1;
}

/// Configuration function called once before normal use
static void bq7692_send_config(struct BQ76920 *bq7692)
{
  	switch (bq7692->init_status) 
  	{
    	case BQ7692_CONF_READ_GAIN1:
		bq7692_read_reg(bq7692, 0x50, 1);
      		bq7692->init_status++;
      		break;
    	case BQ7692_CONF_READ_GAIN2:
      		bq7692_read_reg(bq7692, 0x59, 1);
      		bq7692->init_status++;
      		break;
    	case BQ7692_CONF_READ_OFFSET:
      		bq7692_read_reg(bq7692, 0x51, 1);
      		bq7692->init_status++;
      		break;
		case BQ7692_CONF_REG1:
      		bq7692_write_reg_crc(bq7692, 0x04, 0x18);//enable adc and exteral ptc detect.
      		bq7692->init_status++;
      		break;
		case BQ7692_CONF_REG2:
      		bq7692_write_reg_crc(bq7692, 0x06, 0x9e);//set short current threshold and delay protect.
      		bq7692->init_status++;
      		break;
		case BQ7692_CONF_REG3:
      		bq7692_write_reg_crc(bq7692, 0x07, 0x5f);//set overcurrent threshold and delay protect.
      		bq7692->init_status++;
      		break;
		case BQ7692_CONF_REG4:
      		bq7692_write_reg_crc(bq7692, 0x08, 0x50);//set bat_info.ov and bat_info.uv delay protect time.
      		bq7692->init_status++;
      		break;
		case BQ7692_CONF_REG5:
      		bq7692_write_reg_crc(bq7692, 0x09, bat_info.ov_trip_2);//set bat_info.ov protect threshold.
      		bq7692->init_status++;
      		break;
		case BQ7692_CONF_REG6:
      		bq7692_write_reg_crc(bq7692, 0x0a, bat_info.uv_trip_2);//set bat_info.uv protect threshold.
      		bq7692->init_status++;
      		break;	
		case BQ7692_CONF_REG7:
      		bq7692_write_reg_crc(bq7692, 0x00, 0x00);//clear alarms.
      		bq7692->init_status++;
      		break;	
		case BQ7692_CONF_REG8:
      		bq7692_write_reg_crc(bq7692, 0x05, 0x03);//enable charge and discharge pin.
      		bq7692->init_status++;
      		break;
    	case BQ7692_CONF_DONE:
      		bq7692->initialized = TRUE;
			bq7692->read_status = BQ7692_READ_GAIN1;
      		bq7692->i2c_trans.status = I2CTransDone;
      		break;
    	default:
      		break;
  	}
}

static void bq7693_send_config(struct BQ76930 *bq7693)
{
  	switch (bq7693->init_status) 
  	{
    	         case BQ7693_CONF_READ_GAIN1:
		bq7693_read_reg(bq7693, 0x50, 1);
      		bq7693->init_status++;
      		break;
    		case BQ7693_CONF_READ_GAIN2:
      		bq7693_read_reg(bq7693, 0x59, 1);
      		bq7693->init_status++;
      		break;
    		case BQ7693_CONF_READ_OFFSET:
      		bq7693_read_reg(bq7693, 0x51, 1);
      		bq7693->init_status++;
      		break;
		case BQ7693_CONF_REG1:
      		bq7693_write_reg_crc(bq7693, 0x04, 0x18);//enable adc and exteral ptc detect.
      		bq7693->init_status++;
      		break;
		case BQ7693_CONF_REG2:
      		bq7693_write_reg_crc(bq7693, 0x06, 0x9e);//set short current threshold and delay protect.
      		bq7693->init_status++;
      		break;
		case BQ7693_CONF_REG3:
      		bq7693_write_reg_crc(bq7693, 0x07, 0x5f);//set overcurrent threshold and delay protect.
      		bq7693->init_status++;
      		break;
		case BQ7693_CONF_REG4:
      		bq7693_write_reg_crc(bq7693, 0x08, 0x50);//set bat_info.ov and bat_info.uv delay protect time.
      		bq7693->init_status++;
      		break;
		case BQ7693_CONF_REG5:
      		bq7693_write_reg_crc(bq7693, 0x09, bat_info.ov_trip_3);//set bat_info.ov protect threshold.
      		bq7693->init_status++;
      		break;
		case BQ7693_CONF_REG6:
      		bq7693_write_reg_crc(bq7693, 0x0a, bat_info.uv_trip_3);//set bat_info.uv protect threshold.
      		bq7693->init_status++;
      		break;	
		case BQ7693_CONF_REG7:
      		bq7693_write_reg_crc(bq7693, 0x00, 0x00);//clear alarms.
      		bq7693->init_status++;
      		break;	
		case BQ7693_CONF_REG8:
      		bq7693_write_reg_crc(bq7693, 0x05, 0x03);//enable charge and discharge pin.
      		bq7693->init_status++;
      		break;
    	case BQ7693_CONF_DONE:
      		bq7693->initialized = TRUE;
		bq7693->read_status = BQ7693_READ_GAIN1;
      		bq7693->i2c_trans.status = I2CTransDone;
      		break;
    	default:
      		break;
  	}
}

static void bq7694_send_config(struct BQ76940 *bq7694)
{
  	switch (bq7694->init_status) 
  	{
    	case BQ7694_CONF_READ_GAIN1:
		bq7694_read_reg(bq7694, 0x50, 1);
      		bq7694->init_status++;
      		break;
    	case BQ7694_CONF_READ_GAIN2:
      		bq7694_read_reg(bq7694, 0x59, 1);
      		bq7694->init_status++;
      		break;
    	case BQ7694_CONF_READ_OFFSET:
      		bq7694_read_reg(bq7694, 0x51, 1);
      		bq7694->init_status++;
      		break;
		case BQ7694_CONF_REG1:
      		bq7694_write_reg_crc(bq7694, 0x04, 0x18);//enable adc and exteral ptc detect.
      		bq7694->init_status++;
      		break;
		case BQ7694_CONF_REG2:
      		bq7694_write_reg_crc(bq7694, 0x06, 0x9e);//set short current threshold and delay protect.
      		bq7694->init_status++;
      		break;
		case BQ7694_CONF_REG3:
      		bq7694_write_reg_crc(bq7694, 0x07, 0x5f);//set overcurrent threshold and delay protect.
      		bq7694->init_status++;
      		break;
		case BQ7694_CONF_REG4:
      		bq7694_write_reg_crc(bq7694, 0x08, 0x50);//set bat_info.ov and bat_info.uv delay protect time.
      		bq7694->init_status++;
      		break;
		case BQ7694_CONF_REG5:
      		bq7694_write_reg_crc(bq7694, 0x09, bat_info.ov_trip_4);//set bat_info.ov protect threshold.
      		bq7694->init_status++;
      		break;
		case BQ7694_CONF_REG6:
      		bq7694_write_reg_crc(bq7694, 0x0a, bat_info.uv_trip_4);//set bat_info.uv protect threshold.
      		bq7694->init_status++;
      		break;	
		case BQ7694_CONF_REG7:
      		bq7694_write_reg_crc(bq7694, 0x00, 0x00);//clear alarms.
      		bq7694->init_status++;
      		break;	
		case BQ7694_CONF_REG8:
      		bq7694_write_reg_crc(bq7694, 0x05, 0x03);//enable charge and discharge pin.
      		bq7694->init_status++;
      		break;
    	case BQ7694_CONF_DONE:
      		bq7694->initialized = TRUE;
			bq7694->read_status = BQ7694_READ_GAIN1;
      		bq7694->i2c_trans.status = I2CTransDone;
      		break;
    	default:
      		break;
  	}
}


/// Configuration function called once before normal use
static void bq34z_send_config(struct BQ34Z100 *bq34z)
{
  	switch (bq34z->init_status) 
  	{
		case BQ34Z_CONF_REG1:
      		bq34z_write_reg(bq34z, 0x00, 0x41);//enable adc and exteral ptc detect.
      		bq34z->init_status++;
      		break;
		case BQ34Z_CONF_REG2:
      		bq34z_write_reg(bq34z, 0x01, 0x00);//enable adc and exteral ptc detect.
      		bq34z->init_status++;
      		break;
    	case BQ34Z_CONF_DONE:
      		bq34z->initialized = TRUE;
      		bq34z->i2c_trans.status = I2CTransDone;
      		break;
    	default:
      		break;
  	}
}


// Configure
void bq7692_start_configure(struct BQ76920 *bq7692)
{
  	// wait before starting the configuration
  	// doing to early may void the mode configuration
  	if (bq7692->init_status == BQ7692_CONF_UNINIT && get_sys_time_float() > BQ769X_STARTUP_DELAY) 
  	{
    	bq7692->init_status++;
    	if (bq7692->i2c_trans.status == I2CTransSuccess || bq7692->i2c_trans.status == I2CTransDone) 
		{
      		bq7692_send_config(bq7692);
    	}
  	}
}

void bq7693_start_configure(struct BQ76930 *bq7693)
{
  	// wait before starting the configuration
  	// doing to early may void the mode configuration
 
  	if (bq7693->init_status == BQ7693_CONF_UNINIT && get_sys_time_float() > BQ769X_STARTUP_DELAY) 
  	{
    	bq7693->init_status++;
    	if (bq7693->i2c_trans.status == I2CTransSuccess || bq7693->i2c_trans.status == I2CTransDone) 
	{
      		bq7693_send_config(bq7693);
    	}
  	}
}

void bq7694_start_configure(struct BQ76940 *bq7694)
{
  	// wait before starting the configuration
  	// doing to early may void the mode configuration
  	if (bq7694->init_status == BQ7694_CONF_UNINIT && get_sys_time_float() > BQ769X_STARTUP_DELAY) 
  	{
    	bq7694->init_status++;
    	if (bq7694->i2c_trans.status == I2CTransSuccess || bq7694->i2c_trans.status == I2CTransDone) 
		{
      		bq7694_send_config(bq7694);
    	}
  	}
}

// Configure
void bq34z_start_configure(struct BQ34Z100*bq34z)
{
  	// wait before starting the configuration
  	// doing to early may void the mode configuration
  	if (bq34z->init_status == BQ34Z_CONF_UNINIT) 
  	{
    	bq34z->init_status++;
    	if (bq34z->i2c_trans.status == I2CTransSuccess || bq34z->i2c_trans.status == I2CTransDone) 
		{
      		bq34z_send_config(bq34z);
    	}
  	}
}

static void bq7692_read_reg(struct BQ76920 *bq7692, uint8_t reg, uint8_t len)
{
  	if (bq7692->initialized && bq7692->i2c_trans.status == I2CTransDone) 
  	{
    	bq7692->i2c_trans.buf[0] = reg;
    	bq7692->i2c_trans.type = I2CTransTxRx;
    	bq7692->i2c_trans.len_r = len;
    	bq7692->i2c_trans.len_w = 1;
    	i2c_submit(bq7692->i2c_p, &(bq7692->i2c_trans));
  	}
}

static void bq7693_read_reg(struct BQ76930 *bq7693, uint8_t reg, uint8_t len)
{
  	if (bq7693->initialized && bq7693->i2c_trans.status == I2CTransDone) 
  	{
    	bq7693->i2c_trans.buf[0] = reg;
    	bq7693->i2c_trans.type = I2CTransTxRx;
    	bq7693->i2c_trans.len_r = len;
    	bq7693->i2c_trans.len_w = 1;
    	i2c_submit(bq7693->i2c_p, &(bq7693->i2c_trans));
  	}
}

static void bq7694_read_reg(struct BQ76940 *bq7694, uint8_t reg, uint8_t len)
{
  	if (bq7694->initialized && bq7694->i2c_trans.status == I2CTransDone) 
  	{
    	bq7694->i2c_trans.buf[0] = reg;
    	bq7694->i2c_trans.type = I2CTransTxRx;
    	bq7694->i2c_trans.len_r = len;
    	bq7694->i2c_trans.len_w = 1;
    	i2c_submit(bq7694->i2c_p, &(bq7694->i2c_trans));
  	}
}

static void bq34z_read_reg(struct BQ34Z100 *bq34z, uint8_t reg, uint8_t len)
{
  	if (bq34z->initialized && bq34z->i2c_trans.status == I2CTransDone) 
  	{
    	bq34z->i2c_trans.buf[0] = reg;
    	bq34z->i2c_trans.type = I2CTransTxRx;
    	bq34z->i2c_trans.len_r = len;
    	bq34z->i2c_trans.len_w = 1;
    	i2c_submit(bq34z->i2c_p, &(bq34z->i2c_trans));
  	}
}

static void bq7692_event(struct BQ76920 *bq7692)
{
  	if (bq7692->initialized) 
	{
    	if (bq7692->i2c_trans.status == I2CTransFailed) 
		{
      		bq7692->i2c_trans.status = I2CTransDone;
    	} 
		else if (bq7692->i2c_trans.status == I2CTransSuccess) 
		{
			if(bq7692->read_status == BQ7692_READ_GAIN1_OK)
			{
				bq7692->gain1 = bq7692->i2c_trans.buf[0];
			}
			else if(bq7692->read_status == BQ7692_READ_GAIN2_OK)
			{
				bq7692->gain2 = bq7692->i2c_trans.buf[0];
			}
			else if(bq7692->read_status == BQ7692_READ_OFFSET_OK)
			{
				bq7692->offset = bq7692->i2c_trans.buf[0];
				bat_info.offset_2= bq7692->offset;
				bat_info.gain_2=365 + (uint16_t)(((bq7692->gain1&0x0c) << 1) | ((bq7692->gain2&0xe0)>>5));
				
				if(bat_info.offset_2>=128)
				{
					bat_info.offset_2-=256;
				}
			}
			else if( (bq7692->read_status > BQ7692_READ_OFFSET_OK) 
				     && (bq7692->read_status%2 == 0) )
			{
				bq7692->buf[(bq7692->read_status-8) / 2] = bq7692->i2c_trans.buf[0];
				if(bq7692->read_status == BQ7692_READ_REG8_OK)
				{
					bq7692->read_status = BQ7692_READ_UNINIT;
			       	bq7692->data_available = TRUE;
					bat_info.vc3[1]=(int)(((bq7692->buf[0]&0x3f)<<8)|bq7692->buf[1])*bat_info.gain_2/1000+bat_info.offset_2;
				    bat_info.vc3[2]=(int)(((bq7692->buf[2]&0x3f)<<8)|bq7692->buf[3])*bat_info.gain_2/1000+bat_info.offset_2;
				    bat_info.vc3[3]=(int)(((bq7692->buf[4]&0x3f)<<8)|bq7692->buf[5])*bat_info.gain_2/1000+bat_info.offset_2;
				    bat_info.vcc3=(int)((bq7692->buf[6]<<8)|bq7692->buf[7])*bat_info.gain_2*4/1000+3*bat_info.offset_2; 

					DOWNLINK_SEND_ENERGY_BQ7692(DefaultChannel, DefaultDevice,&bat_info.vcc3,&bat_info.vc3[1],&bat_info.vc3[2],
		                    &bat_info.vc3[3]);
				}
			}
			
			bq7692->i2c_trans.status = I2CTransDone;
			bq7692->read_status++;
    	}
  	} 
	else if (bq7692->init_status != BQ7692_CONF_UNINIT) 
	{ // Configuring but not yet initialized
    	if (bq7692->i2c_trans.status == I2CTransSuccess || bq7692->i2c_trans.status == I2CTransDone) 
		{
			if (bq7692->init_status == BQ7692_CONF_READ_GAIN1_OK) 
			{
	          	bat_info.bq7692.gain1 = (uint16_t)(bq7692->i2c_trans.buf[0] & 0x0c) << 1;
	            bq7692->init_status++;
				bq7692->i2c_trans.status = I2CTransDone;
				bq7692_send_config(bq7692);
	        }
			else if (bq7692->init_status == BQ7692_CONF_READ_GAIN2_OK) 
			{
				bat_info.bq7692.gain2 = (bq7692->i2c_trans.buf[0] & 0xe0) >> 5;
				bat_info.gain_2 = (uint16_t)(bat_info.bq7693.gain2 | bat_info.bq7693.gain1) + 365;
	           		 bq7692->init_status++;
				bq7692->i2c_trans.status = I2CTransDone;
				bq7692_send_config(bq7692);
	        }
			else if (bq7692->init_status == BQ7692_CONF_READ_OFFSET_OK) 
			{
	          	bat_info.offset_2 = bq7692->i2c_trans.buf[0];
				if(bat_info.offset_2 >= 128)
				{
					bat_info.offset_2 -= 256;
				}

				OUV_config(BQ76920_CONFIG);
	           		bq7692->init_status++;
				bq7692->i2c_trans.status = I2CTransDone;
				bq7692_send_config(bq7692);
	        }
			else
			{
				bq7692->i2c_trans.status = I2CTransDone;
				bq7692_send_config(bq7692);
			}
    	}
    	if (bq7692->i2c_trans.status == I2CTransFailed) 
		{
	      	bq7692->init_status--;
	     	bq7692->i2c_trans.status = I2CTransDone;
	      	bq7692_send_config(bq7692); // Retry config (TODO max retry)
    	}
  	}
}

static void bq7693_event(struct BQ76930 *bq7693)
{
  	if (bq7693->initialized) 
	{
    	if (bq7693->i2c_trans.status == I2CTransFailed) 
		{
      		bq7693->i2c_trans.status = I2CTransDone;
    	} 
		else if (bq7693->i2c_trans.status == I2CTransSuccess) 
		{
			if(bq7693->read_status == BQ7693_READ_GAIN1_OK)
			{
				bq7693->gain1 = bq7693->i2c_trans.buf[0];
			}
			else if(bq7693->read_status == BQ7693_READ_GAIN2_OK)
			{
				bq7693->gain2 = bq7693->i2c_trans.buf[0];
			}
			else if(bq7693->read_status == BQ7693_READ_OFFSET_OK)
			{
				bq7693->offset = bq7693->i2c_trans.buf[0];
				bat_info.offset_3= bq7693->offset;
				bat_info.gain_3=365 + (uint16_t)(((bq7693->gain1&0x0c) << 1) |((bq7693->gain2&0xe0)>>5));
				
				if(bat_info.offset_3>=128)
				{
					bat_info.offset_3-=256;
				}
			}
			else if( (bq7693->read_status > BQ7693_READ_OFFSET_OK) 
				     && (bq7693->read_status%2 == 0) )
			{
				bq7693->buf[(bq7693->read_status-8) / 2] = bq7693->i2c_trans.buf[0];
				if(bq7693->read_status == BQ7693_READ_REG14_OK)
				{
					bq7693->read_status = BQ7693_READ_UNINIT;
			       		bq7693->data_available = TRUE;
					bat_info.vc6[1]=(int)(((bq7693->buf[0]&0x3f)<<8)|bq7693->buf[1])*bat_info.gain_3/1000+bat_info.offset_3;
				    	bat_info.vc6[2]=(int)(((bq7693->buf[2]&0x3f)<<8)|bq7693->buf[3])*bat_info.gain_3/1000+bat_info.offset_3;
				    	bat_info.vc6[3]=(int)(((bq7693->buf[4]&0x3f)<<8)|bq7693->buf[5])*bat_info.gain_3/1000+bat_info.offset_3;
				    	bat_info.vc6[4]=(int)(((bq7693->buf[6]&0x3f)<<8)|bq7693->buf[7])*bat_info.gain_3/1000+bat_info.offset_3;
				    	bat_info.vc6[5]=(int)(((bq7693->buf[8]&0x3f)<<8)|bq7693->buf[9])*bat_info.gain_3/1000+bat_info.offset_3;
				    	bat_info.vc6[6]=(int)(((bq7693->buf[10]&0x3f)<<8)|bq7693->buf[11])*bat_info.gain_3/1000+bat_info.offset_3;
				    	bat_info.vcc6=(int)((bq7693->buf[12]<<8)|bq7693->buf[13])*bat_info.gain_3*4/1000+6*bat_info.offset_3; 

					DOWNLINK_SEND_ENERGY_BQ7693(DefaultChannel, DefaultDevice,&bat_info.vcc6,&bat_info.vc6[1],&bat_info.vc6[2],
		                    &bat_info.vc6[3],&bat_info.vc6[4],&bat_info.vc6[5],&bat_info.vc6[6]);
				}
			}
			
			bq7693->i2c_trans.status = I2CTransDone;
			bq7693->read_status++;
    	}
  	} 
	else if (bq7693->init_status != BQ7693_CONF_UNINIT) 
	{ // Configuring but not yet initialized
    	if (bq7693->i2c_trans.status == I2CTransSuccess || bq7693->i2c_trans.status == I2CTransDone) 
		{
			if (bq7693->init_status == BQ7693_CONF_READ_GAIN1_OK) 
			{
	          	bat_info.bq7693.gain1 = (uint16_t)(bq7693->i2c_trans.buf[0] & 0x0c) << 1;
	            bq7693->init_status++;
				bq7693->i2c_trans.status = I2CTransDone;
				bq7693_send_config(bq7693);
	        }
			else if (bq7693->init_status == BQ7693_CONF_READ_GAIN2_OK) 
			{
				bat_info.bq7693.gain2 = (bq7693->i2c_trans.buf[0] & 0xe0) >> 5;
				bat_info.gain_3= (uint16_t)(bat_info.bq7693.gain2 | bat_info.bq7693.gain1) + 365;
	            bq7693->init_status++;
				bq7693->i2c_trans.status = I2CTransDone;
				bq7693_send_config(bq7693);
	        }
			else if (bq7693->init_status == BQ7693_CONF_READ_OFFSET_OK) 
			{
	          	bat_info.offset_3= bq7693->i2c_trans.buf[0];
				if(bat_info.offset_3>= 128)
				{
					bat_info.offset_3-= 256;
				}

				OUV_config(BQ76930_CONFIG);
	          		bq7693->init_status++;
				bq7693->i2c_trans.status = I2CTransDone;
				bq7693_send_config(bq7693);
	        }
			else
			{
				bq7693->i2c_trans.status = I2CTransDone;
				bq7693_send_config(bq7693);
			}
    	}
    	if (bq7693->i2c_trans.status == I2CTransFailed) 
		{
	      	bq7693->init_status--;
	     	bq7693->i2c_trans.status = I2CTransDone;
	      	bq7693_send_config(bq7693); // Retry config (TODO max retry)
    	}
  	}
}

static void bq7694_event(struct BQ76940 *bq7694)
{
  	if (bq7694->initialized) 
	{
    	if (bq7694->i2c_trans.status == I2CTransFailed) 
		{
      		bq7694->i2c_trans.status = I2CTransDone;
    	} 
		else if (bq7694->i2c_trans.status == I2CTransSuccess) 
		{
			if(bq7694->read_status == BQ7694_READ_GAIN1_OK)
			{
				bq7694->gain1 = bq7694->i2c_trans.buf[0];
			}
			else if(bq7694->read_status == BQ7694_READ_GAIN2_OK)
			{
				bq7694->gain2 = bq7694->i2c_trans.buf[0];
			}
			else if(bq7694->read_status == BQ7694_READ_OFFSET_OK)
			{
				bq7694->offset = bq7694->i2c_trans.buf[0];
				bat_info.offset_4= bq7694->offset;
				bat_info.gain_4=365 + (uint16_t)(((bq7694->gain1&0x0c) << 1) |((bq7694->gain2&0xe0)>>5));
				
				if(bat_info.offset_4>=128)
				{
					bat_info.offset_4-=256;
				}
			}
			else if( (bq7694->read_status > BQ7694_READ_OFFSET_OK) 
				     && (bq7694->read_status%2 == 0) )
			{
				bq7694->buf[(bq7694->read_status-8) / 2] = bq7694->i2c_trans.buf[0];
				if(bq7694->read_status == BQ7694_READ_REG26_OK)
				{
					bq7694->read_status = BQ7694_READ_UNINIT;
			       		bq7694->data_available = TRUE;
					bat_info.vc12[1]=(int)(((bq7694->buf[0]&0x3f)<<8)|bq7694->buf[1])*bat_info.gain_4/1000+bat_info.offset_4;
				    	bat_info.vc12[2]=(int)(((bq7694->buf[2]&0x3f)<<8)|bq7694->buf[3])*bat_info.gain_4/1000+bat_info.offset_4;
				    	bat_info.vc12[3]=(int)(((bq7694->buf[4]&0x3f)<<8)|bq7694->buf[5])*bat_info.gain_4/1000+bat_info.offset_4;
				    	bat_info.vc12[4]=(int)(((bq7694->buf[6]&0x3f)<<8)|bq7694->buf[7])*bat_info.gain_4/1000+bat_info.offset_4;
				    	bat_info.vc12[5]=(int)(((bq7694->buf[8]&0x3f)<<8)|bq7694->buf[9])*bat_info.gain_4/1000+bat_info.offset_4;
				    	bat_info.vc12[6]=(int)(((bq7694->buf[10]&0x3f)<<8)|bq7694->buf[11])*bat_info.gain_4/1000+bat_info.offset_4;
					bat_info.vc12[7]=(int)(((bq7694->buf[12]&0x3f)<<8)|bq7694->buf[13])*bat_info.gain_4/1000+bat_info.offset_4;
					bat_info.vc12[8]=(int)(((bq7694->buf[14]&0x3f)<<8)|bq7694->buf[15])*bat_info.gain_4/1000+bat_info.offset_4;
					bat_info.vc12[9]=(int)(((bq7694->buf[16]&0x3f)<<8)|bq7694->buf[17])*bat_info.gain_4/1000+bat_info.offset_4;
					bat_info.vc12[10]=(int)(((bq7694->buf[18]&0x3f)<<8)|bq7694->buf[19])*bat_info.gain_4/1000+bat_info.offset_4;
					bat_info.vc12[11]=(int)(((bq7694->buf[20]&0x3f)<<8)|bq7694->buf[21])*bat_info.gain_4/1000+bat_info.offset_4;
					bat_info.vc12[12]=(int)(((bq7694->buf[22]&0x3f)<<8)|bq7694->buf[23])*bat_info.gain_4/1000+bat_info.offset_4;
					bat_info.vcc12=(int)((bq7694->buf[24]<<8)|bq7694->buf[25])*bat_info.gain_4*4/1000+12*bat_info.offset_4; 

					DOWNLINK_SEND_ENERGY_BQ7694(DefaultChannel, DefaultDevice,&bat_info.vcc12,&bat_info.vc12[1],&bat_info.vc12[2],
		                    &bat_info.vc12[3],&bat_info.vc12[4],&bat_info.vc12[5],&bat_info.vc12[6],&bat_info.vc12[7],&bat_info.vc12[8],
		                    &bat_info.vc12[9],&bat_info.vc12[10],&bat_info.vc12[11],&bat_info.vc12[12]);
				}
			}
			
			bq7694->i2c_trans.status = I2CTransDone;
			bq7694->read_status++;
    	}
  	} 
	else if (bq7694->init_status != BQ7694_CONF_UNINIT) 
	{ // Configuring but not yet initialized
    	if (bq7694->i2c_trans.status == I2CTransSuccess || bq7694->i2c_trans.status == I2CTransDone) 
		{
			if (bq7694->init_status == BQ7694_CONF_READ_GAIN1_OK) 
			{
	          	bat_info.bq7694.gain1 = (uint16_t)(bq7694->i2c_trans.buf[0] & 0x0c) << 1;
	            bq7694->init_status++;
				bq7694->i2c_trans.status = I2CTransDone;
				bq7694_send_config(bq7694);
	        }
			else if (bq7694->init_status == BQ7694_CONF_READ_GAIN2_OK) 
			{
				bat_info.bq7694.gain2 = (bq7694->i2c_trans.buf[0] & 0xe0) >> 5;
				bat_info.gain_4= (uint16_t)(bat_info.bq7694.gain2 | bat_info.bq7694.gain1) + 365;
	            bq7694->init_status++;
				bq7694->i2c_trans.status = I2CTransDone;
				bq7694_send_config(bq7694);
	        }
			else if (bq7694->init_status == BQ7694_CONF_READ_OFFSET_OK) 
			{
	          	bat_info.offset_4= bq7694->i2c_trans.buf[0];
				if(bat_info.offset_4>= 128)
				{
					bat_info.offset_4-= 256;
				}

				OUV_config(BQ76940_CONFIG);
	            bq7694->init_status++;
				bq7694->i2c_trans.status = I2CTransDone;
				bq7694_send_config(bq7694);
	        }
			else
			{
				bq7694->i2c_trans.status = I2CTransDone;
				bq7694_send_config(bq7694);
			}
    	}
    	if (bq7694->i2c_trans.status == I2CTransFailed) 
		{
	      	bq7694->init_status--;
	     	bq7694->i2c_trans.status = I2CTransDone;
	      	bq7694_send_config(bq7694); // Retry config (TODO max retry)
    	}
  	}
}


static void bq34z_event(struct BQ34Z100 *bq34z)
{
  	if (bq34z->initialized) 
	{
    	if (bq34z->i2c_trans.status == I2CTransFailed) 
		{
      		bq34z->i2c_trans.status = I2CTransDone;
    	} 
		else if (bq34z->i2c_trans.status == I2CTransSuccess) 
		{
			if(bq34z->read_status == BQ34Z_READ_REG1_OK)
			{
				bat_info.percent = bq34z->i2c_trans.buf[2];
				bat_info.remaining= (bq34z->i2c_trans.buf[5] << 8) | bq34z->i2c_trans.buf[4];
				bat_info.fullcharge= (bq34z->i2c_trans.buf[7] << 8) | bq34z->i2c_trans.buf[6];
				bat_info.volt= (bq34z->i2c_trans.buf[9] << 8) | bq34z->i2c_trans.buf[8];
				bat_info.temp= ((float)((bq34z->i2c_trans.buf[13] << 8) | bq34z->i2c_trans.buf[12])/10)-273.15;

				if((bq34z->i2c_trans.buf[11]>>7) & 0x01)
				{   
					bq34z->i2c_trans.buf[10] = ~bq34z->i2c_trans.buf[10];
					bq34z->i2c_trans.buf[11] = ~bq34z->i2c_trans.buf[11];
					bat_info_f[0].avrcurr = -1- ((bq34z->i2c_trans.buf[11] << 8) | bq34z->i2c_trans.buf[10]);
				}
				else 
				{
					  bat_info_f[0].avrcurr= (bq34z->i2c_trans.buf[11] << 8) | bq34z->i2c_trans.buf[10];
				}
				bat_info.avrcurr= bat_info_f[0].avrcurr;
				
				bq34z->i2c_trans.status = I2CTransDone;
				bq34z->read_status++;
			}
			else if(bq34z->read_status == BQ34Z_READ_REG2_OK)
			{
				if((bq34z->i2c_trans.buf[1]>>7)&0x01)
				{
					bq34z->i2c_trans.buf[0] = ~bq34z->i2c_trans.buf[0];
					bq34z->i2c_trans.buf[1] = ~bq34z->i2c_trans.buf[1];
					bat_info_f[0].curr = -1- ((bq34z->i2c_trans.buf[1] << 8) | bq34z->i2c_trans.buf[0]);
				}
				else 
				{
					bat_info_f[0].curr = (bq34z->i2c_trans.buf[1] << 8) | bq34z->i2c_trans.buf[0];
				}
				bat_info.curr = bat_info_f[0].curr;
				bq34z->read_status = BQ34Z_READ_UNINIT;

				bq34z->i2c_trans.status = I2CTransDone;
			    bq34z->read_status++;

				DOWNLINK_SEND_ENERGY_BQ34Z(DefaultChannel, DefaultDevice,&bat_info.percent,&bat_info.remaining,&bat_info.fullcharge,
							&bat_info.volt,&bat_info.curr,&bat_info.avrcurr,&bat_info.temp);
			}
    	}
  	} 
	else if (bq34z->init_status != BQ34Z_CONF_UNINIT) 
	{ // Configuring but not yet initialized
    	if (bq34z->i2c_trans.status == I2CTransSuccess || bq34z->i2c_trans.status == I2CTransDone) 
		{
      		bq34z->i2c_trans.status = I2CTransDone;
			bq34z_send_config(bq34z);
    	}
  	}
}


void bat_periodic(void)
{
	//gpio_set(TEST_GPIO_PIN1);
	//bq7692_periodic(&bat_info.bq7692);
	//bq7693_periodic(&bat_info.bq7693);
	bq7694_periodic(&bat_info.bq7694);
	bq34z_periodic(&bat_info.bq34z);
	//clear_data();
	//clear_faults();
	//gpio_clear(TEST_GPIO_PIN1);
}

void bat_event(void)
{
         //bq7692_event(&bat_info.bq7692);
	//bq7693_event(&bat_info.bq7693);
	bq7694_event(&bat_info.bq7694);
	bq34z_event(&bat_info.bq34z);
}

static uint8_t cal_crc(uint8_t *ptr, uint16_t len)
 {
    unsigned char i;
    unsigned char crc=0;
    while(len-- != 0)         
	{
        for(i=0x80; i!=0; i/=2)  
		{
            if((crc&0x80) != 0)  
            {
				crc *= 2; 
				crc ^= 0x07;
			} 
            else 
            {
                crc *= 2;
            }
						
            if((*ptr&i) != 0) 
            {
                crc ^= 0x07; 
            }
        }
        ptr++;
    }
   	return(crc);
}

static void bq7692_write_reg_crc(struct BQ76920 *bq7692, uint8_t reg,uint8_t data)
{
	uint8_t buf[3];
	uint8_t crc = 0;
	
    buf[0] = BQ76920_ADDR;
    buf[1] = reg;
    buf[2] = data;
    crc = cal_crc(&buf[0], 3);	

	bq7692->i2c_trans.type = I2CTransTx;
 	bq7692->i2c_trans.buf[0] = reg;
  	bq7692->i2c_trans.buf[1] = data;
	bq7692->i2c_trans.buf[2] = crc;
 	bq7692->i2c_trans.len_r = 0;
  	bq7692->i2c_trans.len_w = 3;
  	i2c_submit(bq7692->i2c_p, &(bq7692->i2c_trans));
}

static void bq7693_write_reg_crc(struct BQ76930 *bq7693, uint8_t reg,uint8_t data)
{
	uint8_t buf[3];
	uint8_t crc = 0;
	
    buf[0] = BQ76930_ADDR;
    buf[1] = reg;
    buf[2] = data;
    crc = cal_crc(&buf[0], 3);	

	bq7693->i2c_trans.type = I2CTransTx;
 	bq7693->i2c_trans.buf[0] = reg;
  	bq7693->i2c_trans.buf[1] = data;
	bq7693->i2c_trans.buf[2] = crc;
 	bq7693->i2c_trans.len_r = 0;
  	bq7693->i2c_trans.len_w = 3;
  	i2c_submit(bq7693->i2c_p, &(bq7693->i2c_trans));
}

static void bq7694_write_reg_crc(struct BQ76940 *bq7694, uint8_t reg,uint8_t data)
{
	uint8_t buf[3];
	uint8_t crc = 0;
	
    buf[0] = BQ76940_ADDR;
    buf[1] = reg;
    buf[2] = data;
    crc = cal_crc(&buf[0], 3);	

	bq7694->i2c_trans.type = I2CTransTx;
 	bq7694->i2c_trans.buf[0] = reg;
  	bq7694->i2c_trans.buf[1] = data;
	bq7694->i2c_trans.buf[2] = crc;
 	bq7694->i2c_trans.len_r = 0;
  	bq7694->i2c_trans.len_w = 3;
  	i2c_submit(bq7694->i2c_p, &(bq7694->i2c_trans));
}

static void bq34z_write_reg(struct BQ34Z100 *bq34z, uint8_t reg,uint8_t data)
{
	uint8_t buf[3];
	
    buf[0] = BQ34Z100_ADDR;
    buf[1] = reg;
    buf[2] = data;

	bq34z->i2c_trans.type = I2CTransTx;
 	bq34z->i2c_trans.buf[0] = reg;
  	bq34z->i2c_trans.buf[1] = data;
 	bq34z->i2c_trans.len_r = 0;
  	bq34z->i2c_trans.len_w = 2;
  	i2c_submit(bq34z->i2c_p, &(bq34z->i2c_trans));
}

static void OUV_config(uint8_t model)
{
	uint16_t ov_trip_full = 0;
	uint16_t uv_trip_full = 0;
         if(model == BQ76920_CONFIG)
         {
    		ov_trip_full = ((bat_info.ov - bat_info.offset_2)*1000) / bat_info.gain_2;
		uv_trip_full = ((bat_info.uv - bat_info.offset_2)*1000) / bat_info.gain_2;
		bat_info.ov_trip_2= (ov_trip_full & 0x0fff) >> 4;
		bat_info.uv_trip_2= (uv_trip_full & 0x0fff) >> 4;
         }
	else if(model == BQ76930_CONFIG)
	{
		ov_trip_full = ((bat_info.ov - bat_info.offset_3)*1000) / bat_info.gain_3;
		uv_trip_full = ((bat_info.uv - bat_info.offset_3)*1000) / bat_info.gain_3;
		bat_info.ov_trip_3= (ov_trip_full & 0x0fff) >> 4;
		bat_info.uv_trip_3= (uv_trip_full & 0x0fff) >> 4;
	}
	else if(model == BQ76940_CONFIG)
	{
		ov_trip_full = ((bat_info.ov - bat_info.offset_4)*1000) / bat_info.gain_4;
		uv_trip_full = ((bat_info.uv - bat_info.offset_4)*1000) / bat_info.gain_4;
		bat_info.ov_trip_4= (ov_trip_full & 0x0fff) >> 4;
		bat_info.uv_trip_4= (uv_trip_full & 0x0fff) >> 4;
	}
}

#if 0
//BQ76930 alarm clear
void clear_faults(void)
{
	 bat_clear_cnt++;
	 if(bat_clear_cnt==4)
	 {
		bq769x_write_reg_crc(BQ76930_ADDR, 0x00, 0x00);//clear alarms.
		bq769x_write_reg_crc(BQ76930_ADDR, 0x05, 0x03); //enable charge and discharge pin.
		bat_clear_cnt=0;
	 }
}

void pop_order(uint8_t *buf, uint8_t len)
{
	uint8_t i, j;
	uint8_t temp;

	for(i=0; i<len-1; i++)
	{
		for(j=i+1; j<len; j++)
		{
			if(buf[i]>buf[j])
			{
				temp=buf[i];
				buf[i]=buf[j];
				buf[j]=temp;
			}
		}
	}
}
#endif

void bq7692_read(struct BQ76920 *bq7692)
{
  	switch (bq7692->read_status) 
  	{
		case BQ7692_READ_GAIN1:
			bq7692_read_reg(bq7692, 0x50, 1);
      		bq7692->read_status++;
      		break;
		case BQ7692_READ_GAIN2:
			bq7692_read_reg(bq7692, 0x59, 1);
      		bq7692->read_status++;
      		break;
		case BQ7692_READ_OFFSET:
			bq7692_read_reg(bq7692, 0x51, 1);
      		bq7692->read_status++;
      		break;
		case BQ7692_READ_REG1:
			bq7692_read_reg(bq7692, 0x0c, 1);
      		bq7692->read_status++;
      		break;
    	case BQ7692_READ_REG2:
      		bq7692_read_reg(bq7692, 0x0d, 1);
      		bq7692->read_status++;
      		break;
    	case BQ7692_READ_REG3:
      		bq7692_read_reg(bq7692, 0x0e, 1);
      		bq7692->read_status++;
      		break;
    	case BQ7692_READ_REG4:
      		bq7692_read_reg(bq7692, 0x0f, 1);
      		bq7692->read_status++;
      		break;
    	case BQ7692_READ_REG5:
			bq7692_read_reg(bq7692, 0x14, 1);
      		bq7692->read_status++;
      		break;
    	case BQ7692_READ_REG6:
      		bq7692_read_reg(bq7692, 0x15, 1);
      		bq7692->read_status++;
      		break;
    	case BQ7692_READ_REG7:
      		bq7692_read_reg(bq7692, 0x2a, 1);
      		bq7692->read_status++;
      		break;
    	case BQ7692_READ_REG8:
      		bq7692_read_reg(bq7692, 0x2b, 1);
      		bq7692->read_status++;
      		break;
    	default:
      		break;
  	}
}

void bq7693_read(struct BQ76930 *bq7693)
{
  	switch (bq7693->read_status) 
  	{
		case BQ7693_READ_GAIN1:
			bq7693_read_reg(bq7693, 0x50, 1);
      		bq7693->read_status++;
      		break;
		case BQ7693_READ_GAIN2:
			bq7693_read_reg(bq7693, 0x59, 1);
      		bq7693->read_status++;
      		break;
		case BQ7693_READ_OFFSET:
			bq7693_read_reg(bq7693, 0x51, 1);
      		bq7693->read_status++;
      		break;
		case BQ7693_READ_REG1:
			bq7693_read_reg(bq7693, 0x0c, 1);
      		bq7693->read_status++;
      		break;
    	case BQ7693_READ_REG2:
      		bq7693_read_reg(bq7693, 0x0d, 1);
      		bq7693->read_status++;
      		break;
    	case BQ7693_READ_REG3:
      		bq7693_read_reg(bq7693, 0x0e, 1);
      		bq7693->read_status++;
      		break;
    	case BQ7693_READ_REG4:
      		bq7693_read_reg(bq7693, 0x0f, 1);
      		bq7693->read_status++;
      		break;
    	case BQ7693_READ_REG5:
			bq7693_read_reg(bq7693, 0x14, 1);
      		bq7693->read_status++;
      		break;
    	case BQ7693_READ_REG6:
      		bq7693_read_reg(bq7693, 0x15, 1);
      		bq7693->read_status++;
      		break;
    	case BQ7693_READ_REG7:
      		bq7693_read_reg(bq7693, 0x16, 1);
      		bq7693->read_status++;
      		break;
    	case BQ7693_READ_REG8:
      		bq7693_read_reg(bq7693, 0x17, 1);
      		bq7693->read_status++;
      		break;
    	case BQ7693_READ_REG9:
			bq7693_read_reg(bq7693, 0x18, 1);
      		bq7693->read_status++;
      		break;
    	case BQ7693_READ_REG10:
      		bq7693_read_reg(bq7693, 0x19, 1);
      		bq7693->read_status++;
      		break;
    	case BQ7693_READ_REG11:
      		bq7693_read_reg(bq7693, 0x1e, 1);
      		bq7693->read_status++;
      		break;
    	case BQ7693_READ_REG12:
      		bq7693_read_reg(bq7693, 0x1f, 1);
      		bq7693->read_status++;
      		break;
    	case BQ7693_READ_REG13:
			bq7693_read_reg(bq7693, 0x2a, 1);
      		bq7693->read_status++;
      		break;
    	case BQ7693_READ_REG14:
      		bq7693_read_reg(bq7693, 0x2b, 1);
      		bq7693->read_status++;
      		break;
    	default:
      		break;
  	}
}

void bq7694_read(struct BQ76940 *bq7694)
{
  	switch (bq7694->read_status) 
  	{
		case BQ7694_READ_GAIN1:
			bq7694_read_reg(bq7694, 0x50, 1);
      		bq7694->read_status++;
      		break;
		case BQ7694_READ_GAIN2:
			bq7694_read_reg(bq7694, 0x59, 1);
      		bq7694->read_status++;
      		break;
		case BQ7694_READ_OFFSET:
			bq7694_read_reg(bq7694, 0x51, 1);
      		bq7694->read_status++;
      		break;
		case BQ7694_READ_REG1:
			bq7694_read_reg(bq7694, 0x0c, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG2:
      		bq7694_read_reg(bq7694, 0x0d, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG3:
      		bq7694_read_reg(bq7694, 0x0e, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG4:
      		bq7694_read_reg(bq7694, 0x0f, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG5:
			bq7694_read_reg(bq7694, 0x10, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG6:
      		bq7694_read_reg(bq7694, 0x11, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG7:
      		bq7694_read_reg(bq7694, 0x14, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG8:
      		bq7694_read_reg(bq7694, 0x15, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG9:
			bq7694_read_reg(bq7694, 0x16, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG10:
      		bq7694_read_reg(bq7694, 0x17, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG11:
      		bq7694_read_reg(bq7694, 0x18, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG12:
      		bq7694_read_reg(bq7694, 0x19, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG13:
			bq7694_read_reg(bq7694, 0x1a, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG14:
      		bq7694_read_reg(bq7694, 0x1b, 1);
      		bq7694->read_status++;
      		break;
		case BQ7694_READ_REG15:
			bq7694_read_reg(bq7694, 0x1e, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG16:
      		bq7694_read_reg(bq7694, 0x1f, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG17:
      		bq7694_read_reg(bq7694, 0x20, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG18:
      		bq7694_read_reg(bq7694, 0x21, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG19:
			bq7694_read_reg(bq7694, 0x22, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG20:
      		bq7694_read_reg(bq7694, 0x23, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG21:
      		bq7694_read_reg(bq7694, 0x24, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG22:
      		bq7694_read_reg(bq7694, 0x25, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG23:
			bq7694_read_reg(bq7694, 0x28, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG24:
      		bq7694_read_reg(bq7694, 0x29, 1);
      		bq7694->read_status++;
      		break;
		case BQ7694_READ_REG25:
			bq7694_read_reg(bq7694, 0x2a, 1);
      		bq7694->read_status++;
      		break;
    	case BQ7694_READ_REG26:
      		bq7694_read_reg(bq7694, 0x2b, 1);
      		bq7694->read_status++;
      		break;
    	default:
      		break;
  	}
}


void bq34z_read(struct BQ34Z100 *bq34z)
{
  	switch (bq34z->read_status) 
  	{
		case BQ34Z_READ_REG1:
			bq34z_read_reg(bq34z, 0x00, 14);
      			bq34z->read_status++;
      			break;
		case BQ34Z_READ_REG2:
			bq34z_read_reg(bq34z, 0x10, 2);
      			bq34z->read_status++;
      			break;
		
    	default:
      		break;
  	}
}

#if 0
static void clear_data(void)
{
	bat_info.vc[1]=0;
	bat_info.vc[2]=0;
	bat_info.vc[3]=0;
	bat_info.vc[4]=0;
	bat_info.vc[5]=0;
	bat_info.vc[6]=0;
	bat_info.vcc=0; 
	bat_info.percent=0;
	bat_info.remaining=0;
	bat_info.fullcharge=0;
	bat_info.volt=0;
    bat_info.curr=0;
	bat_info.avrcurr=0;
	bat_info.temp=0;
}
#endif


/**************** END OF FILE *****************************************/
