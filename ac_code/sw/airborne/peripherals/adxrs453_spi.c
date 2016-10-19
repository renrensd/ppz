/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				       *
*				  All Rights Reserved.          					   *
*   Department : R&D SW      									       *
*   AUTHOR	   :            										   *
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

/**** System include files ****/
  

/*---Public include files---------------------------------------------*/
#include <libopencm3/stm32/gpio.h>
#include "modules/system/timer_if.h"
#include "modules/system/timer_class.h"
#include "modules/system/timer_def.h"
#include "mcu_periph/sys_time.h"

/*---Private include files--------------------------------------------*/
#include "peripherals/adxrs453_spi.h"



/*===VARIABLES========================================================*/

/*---Global-----------------------------------------------------------*/

 
/*---Private----------------------------------------------------------*/

/*===FUNCTIONS========================================================*/
static unsigned char adxrs453_parity_bit(uint32_t data);
static void adxrs_build_read_seq(struct Adxrs453_Spi *adxrs, uint8_t reg);
static void  adxrs453_build_write_seq(struct Adxrs453_Spi *adxrs, uint8_t reg, uint16_t val);
static void adxrs453_spi_write_reg(struct Adxrs453_Spi *adxrs, uint8_t reg, uint16_t val);
static void adxrs453_spi_send_config(struct Adxrs453_Spi *adxrs);
static void adxrs453_sensor_data_transfer(struct Adxrs453_Spi *adxrs);
static uint16_t adxrs453_reg_data_transfer(struct Adxrs453_Spi *adxrs);
static void adxrs_build_sensor_seq(struct Adxrs453_Spi *adxrs, uint8_t seq);


/*---Global-----------------------------------------------------------*/
/*****************************************************************************
*  Name        : adxrs453_spi_init
*  Description : 
*  Parameter   : void  
*  Returns     : None
*****************************************************************************/
void adxrs453_spi_init(struct Adxrs453_Spi *adxrs, struct spi_periph *spi_p, uint8_t slave_idx)
{
  /* set spi_peripheral */
  adxrs->spi_p = spi_p;

  /* configure spi transaction */
  adxrs->spi_trans.cpol = SPICpolIdleLow;
  adxrs->spi_trans.cpha = SPICphaEdge1;
  adxrs->spi_trans.dss = SPIDss8bit;
  adxrs->spi_trans.bitorder = SPIMSBFirst;
  adxrs->spi_trans.cdiv = SPIDiv32;

  adxrs->spi_trans.select = SPISelectUnselect;
  adxrs->spi_trans.slave_idx = slave_idx;
  adxrs->spi_trans.output_length = 4;
  adxrs->spi_trans.input_length = 4;
  // callback currently unused
  adxrs->spi_trans.before_cb = NULL;
  adxrs->spi_trans.after_cb = NULL;
  adxrs->spi_trans.input_buf = &(adxrs->rx_buf[0]);
  adxrs->spi_trans.output_buf = &(adxrs->tx_buf[0]);

  /* set inital status: Success or Done */
  adxrs->spi_trans.status = SPITransDone;

  adxrs->initialized = FALSE;
  adxrs->data_available = FALSE;
  adxrs->init_status = ADXRS453_CONF_UNINIT;
}

/*****************************************************************************
*  Name        : adxrs453_spi_start_configure
*  Description : 
*  Parameter   : void  
*  Returns     : None
*****************************************************************************/
void adxrs453_spi_start_configure(struct Adxrs453_Spi *adxrs)
{
	if (adxrs->init_status == ADXRS453_CONF_UNINIT) 
  	{
		adxrs->init_status++;
    	if( (adxrs->spi_trans.status == SPITransSuccess) || (adxrs->spi_trans.status == SPITransDone) )
		{
      		adxrs453_spi_send_config(adxrs);
    	}
  	}
}

/*****************************************************************************
*  Name        : adxrs453_spi_read
*  Description : 
*  Parameter   : void  
*  Returns     : None
*****************************************************************************/
void adxrs453_spi_read(struct Adxrs453_Spi *adxrs, uint8_t axis)
{
	if( adxrs->initialized && (adxrs->spi_trans.status == SPITransDone) )
  	{
		adxrs->spi_trans.output_length = 4;
		adxrs->spi_trans.input_length = 4;
		adxrs_build_sensor_seq(adxrs, axis);
		spi_submit(adxrs->spi_p, &(adxrs->spi_trans));
  	}
}

/*****************************************************************************
*  Name        : adxrs453_spi_event
*  Description : 
*  Parameter   : void  
*  Returns     : None
*****************************************************************************/
void adxrs453_spi_event(struct Adxrs453_Spi *adxrs, uint8_t axis)
{
	if (adxrs->initialized) 
	{
		if (adxrs->spi_trans.status == SPITransFailed) 
		{
			adxrs->spi_trans.status = SPITransDone;
		} 
		else if (adxrs->spi_trans.status == SPITransSuccess) 
		{
			adxrs453_sensor_data_transfer(adxrs);
			if(adxrs->sq == axis)
			{
				adxrs->data_available = TRUE;
			}
		}
		adxrs->spi_trans.status = SPITransDone;

	} 
	else if (adxrs->init_status != ADXRS453_CONF_UNINIT) 
	{ // Configuring but not yet initialized
    	switch (adxrs->spi_trans.status) 
		{
	      	case SPITransFailed:
	        	adxrs->init_status--; // Retry config (TODO max retry)
	      	case SPITransSuccess:
		        if (adxrs->init_status == ADXRS453_READ_PID1_OK) 
				{
		        	if( (uint8_t)(adxrs453_reg_data_transfer(adxrs)>>8) == ADXRS453_DEV_ID_VAL )
				  	{
		            	adxrs->init_status++;
		          	} 
				  	else 
				  	{
		            	adxrs->init_status = ADXRS453_CONF_UNINIT;
		          	}
		        }
	      	case SPITransDone:
	        	adxrs->spi_trans.status = SPITransDone;
	        	adxrs453_spi_send_config(adxrs);
	        	break;
	      	default:
	        	break;
    	}
  }
}

/*****************************************************************************
*  Name        : adxrs453_spi_periodic
*  Description : read or start configuration if not already initialized
*  Parameter   : void  
*  Returns     : None
*****************************************************************************/
void adxrs453_spi_periodic(struct Adxrs453_Spi *adxrs453, uint8_t axis)
{
	if(adxrs453->initialized)
  	{
    	adxrs453_spi_read(adxrs453, axis);
  	} 
  	else 
  	{
    	adxrs453_spi_start_configure(adxrs453);
  	}
}

/*---Private----------------------------------------------------------*/
/*****************************************************************************
*  Name        : adxrs453_parity_bit
*  Description : Sets or clears the parity bit in order to ensure that the overall 
*        		 parity of the data word is odd
*  Parameter   : void  
*  Returns     : parityBit
*****************************************************************************/
static uint8_t adxrs453_parity_bit(uint32_t data)
{
    unsigned char parityBit = 0;
    unsigned char bitIndex  = 0;
    unsigned char sum       = 0;

    for(bitIndex = 0; bitIndex < 32; bitIndex++)
    {
        sum += ((data >> bitIndex) & 0x1);
    }
    if (! (sum % 2))
    {
        parityBit |= 0x1;
    }

    return parityBit;
}

/*****************************************************************************
*  Name        : adxrs_build_read_seq
*  Description : 
*  Parameter   : void  
*  Returns     : None
*****************************************************************************/
static void adxrs_build_read_seq(struct Adxrs453_Spi *adxrs, uint8_t reg)
{
	uint8_t data[4] = {0, 0, 0, 0};
	uint32_t val = 0;

	data[0] = ADXRS453_READ_DATA | (reg >> 7);
	data[1] = reg << 1;
	val += ((uint32_t)data[0] << 24);
	val += ((uint32_t)data[1] << 16);
	data[3] = adxrs453_parity_bit(val);

	adxrs->tx_buf[0] = data[0];
	adxrs->tx_buf[1] = data[1];
	adxrs->tx_buf[2] = data[2];
	adxrs->tx_buf[3] = data[3];
}

/*****************************************************************************
*  Name        : adxrs_build_sensor_seq
*  Description : 
*  Parameter   : seq:SQ0:SQ1,  value<=0x03
*  Returns     : None
*****************************************************************************/
static void adxrs_build_sensor_seq(struct Adxrs453_Spi *adxrs, uint8_t seq)
{
	uint8_t data[4] = {0, 0, 0, 0};
	uint32_t val = 0;

	data[0] = ADXRS453_SENSOR_DATA | ((seq&0x03)<<6);	//SQ2 = 0
	data[1] = 0;
	val += ((uint32_t)data[0] << 24);
	val += ((uint32_t)data[1] << 16);
	data[3] = adxrs453_parity_bit(val);

	adxrs->tx_buf[0] = data[0];
	adxrs->tx_buf[1] = data[1];
	adxrs->tx_buf[2] = data[2];
	adxrs->tx_buf[3] = data[3];
}

static void  adxrs453_build_write_seq(struct Adxrs453_Spi *adxrs, uint8_t reg, uint16_t val)
{
	uint8_t data[4] = {0, 0, 0, 0};
	uint32_t temp = 0;
		
	data[0] = ADXRS453_WRITE_DATA | (reg >> 7);
	data[1] = (reg << 1) | (val >> 15);
	data[2] = (val & 0x7F80) >> 7;
	data[3] = (val & 0xFF) << 1;
	temp += ((uint32_t)data[0] << 24);
	temp += ((uint32_t)data[1] << 16);
	data[3] = adxrs453_parity_bit(temp);
	
	adxrs->tx_buf[0] = data[0];
	adxrs->tx_buf[1] = data[1];
	adxrs->tx_buf[2] = data[2];
	adxrs->tx_buf[3] = data[3];
}

static void adxrs453_spi_write_reg(struct Adxrs453_Spi *adxrs, uint8_t reg, uint16_t val)
{
	adxrs->spi_trans.output_length = 4;
  	adxrs->spi_trans.input_length = 0;
	adxrs453_build_write_seq(adxrs, reg, val);
  	spi_submit(adxrs->spi_p, &(adxrs->spi_trans));
}

/***********************************************************************
* FUNCTION    : adxrs453_spi_send_config
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
static void adxrs453_spi_send_config(struct Adxrs453_Spi *adxrs)
{

  switch (adxrs->init_status) 
  {
	case ADXRS453_CONF_STARTUP_STEP1:
		adxrs->spi_trans.output_length = 4;
      	adxrs->spi_trans.input_length = 4;
		adxrs->tx_buf[0] = 0x20;
		adxrs->tx_buf[1] = 0x00;
		adxrs->tx_buf[2] = 0x00;
		adxrs->tx_buf[3] = 0x03;
		spi_submit(adxrs->spi_p, &(adxrs->spi_trans));
		adxrs->timer_tick = tm_get_system_timer_tick();
		adxrs->init_status = ADXRS453_CONF_STARTUP_STEP1_WAIT;
		break;
	case ADXRS453_CONF_STARTUP_STEP1_WAIT:
		if( tm_is_time_passed(adxrs->timer_tick, ADXRS453_STARTUP_DELAY) )
		{
			adxrs->init_status = ADXRS453_CONF_STARTUP_STEP2;
		}
		break;
	case ADXRS453_CONF_STARTUP_STEP2:
		adxrs->spi_trans.output_length = 4;
      	adxrs->spi_trans.input_length = 4;
		adxrs->tx_buf[0] = 0x20;
		adxrs->tx_buf[1] = 0x00;
		adxrs->tx_buf[2] = 0x00;
		adxrs->tx_buf[3] = 0x00;
		spi_submit(adxrs->spi_p, &(adxrs->spi_trans));
		adxrs->timer_tick = tm_get_system_timer_tick();
		adxrs->init_status = ADXRS453_CONF_STARTUP_STEP2_WAIT;
		break;
	case ADXRS453_CONF_STARTUP_STEP2_WAIT:
		if( tm_is_time_passed(adxrs->timer_tick, ADXRS453_STARTUP_DELAY) )
		{
			adxrs->init_status = ADXRS453_CONF_STARTUP_STEP3;
		}
		break;
	case ADXRS453_CONF_STARTUP_STEP3:
		adxrs->spi_trans.output_length = 4;
      	adxrs->spi_trans.input_length = 4;
		adxrs->tx_buf[0] = 0x20;
		adxrs->tx_buf[1] = 0x00;
		adxrs->tx_buf[2] = 0x00;
		adxrs->tx_buf[3] = 0x00;
		spi_submit(adxrs->spi_p, &(adxrs->spi_trans));
		adxrs->timer_tick = tm_get_system_timer_tick();
		adxrs->init_status = ADXRS453_CONF_STARTUP_STEP3_WAIT;
		break;
	case ADXRS453_CONF_STARTUP_STEP3_WAIT:
		if( tm_is_time_passed(adxrs->timer_tick, ADXRS453_STARTUP_DELAY) )
		{
			adxrs->init_status = ADXRS453_CONF_STARTUP_STEP4;
		}
		break;
	case ADXRS453_CONF_STARTUP_STEP4:
		adxrs->spi_trans.output_length = 4;
      	adxrs->spi_trans.input_length = 4;
		adxrs->tx_buf[0] = 0x20;
		adxrs->tx_buf[1] = 0x00;
		adxrs->tx_buf[2] = 0x00;
		adxrs->tx_buf[3] = 0x00;
		spi_submit(adxrs->spi_p, &(adxrs->spi_trans));
		adxrs->timer_tick = tm_get_system_timer_tick();
		adxrs->init_status = ADXRS453_CONF_STARTUP_STEP4_WAIT;
		break;
	case ADXRS453_CONF_STARTUP_STEP4_WAIT:
		if( tm_is_time_passed(adxrs->timer_tick, ADXRS453_STARTUP_DELAY2) )
		{
			adxrs->init_status = ADXRS453_READ_PID1;
		}
		break;
	case ADXRS453_READ_PID1:
      	/* query device id */
      	adxrs->spi_trans.output_length = 4;
      	adxrs->spi_trans.input_length = 4;
      	adxrs_build_read_seq(adxrs, ADXRS453_PID1);
      	spi_submit(adxrs->spi_p, &(adxrs->spi_trans));
      	delay_us(15);
      	if( spi_submit(adxrs->spi_p, &(adxrs->spi_trans)) ) 
	  	{
        	adxrs->init_status++;
      	}
      	break;
    case ADXRS453_CONF_DONE:
      	adxrs->initialized = TRUE;
      	adxrs->spi_trans.status = SPITransDone;
      	break;
    default:
      	break;
  }
}

uint16_t pid1_dev_id;
/***********************************************************************
* FUNCTION    : adxrs453_reg_data_transfer
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
static uint16_t adxrs453_reg_data_transfer(struct Adxrs453_Spi *adxrs)
{
	uint16_t ret;

	ret = ((uint16_t)adxrs->rx_buf[1] << 11) |
          ((uint16_t)adxrs->rx_buf[2] << 3)  |
                    (adxrs->rx_buf[3] >> 5);
	pid1_dev_id = ret;
	return ret;
}

/***********************************************************************
* FUNCTION    : adxrs453_sensor_data_transfer
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
static void adxrs453_sensor_data_transfer(struct Adxrs453_Spi *adxrs)
{

	adxrs->data.rates.p = ((adxrs->rx_buf[0]&0x03) << 14)
						  | (adxrs->rx_buf[1] << 6)  
						  | (adxrs->rx_buf[2] >> 2);
	adxrs->sq = adxrs->rx_buf[0] >> 5;
	adxrs->status = adxrs->rx_buf[3];
	adxrs->parity = ((adxrs->rx_buf[0]>>4)&0x01) | ((adxrs->rx_buf[3]&0x01)<<1);
}

#if 0
/***************************************************************************//**
 * @brief Reads the sensor data.
 *
 * @param None.
 *
 * @return registerValue - The sensor data.
*******************************************************************************/
unsigned long ADXRS453_GetSensorData(void)
{
    unsigned char dataBuffer[4] = {0, 0, 0, 0};
    unsigned long command       = 0;
    unsigned char bitNo         = 0;
    unsigned char sum           = 0;
    unsigned long registerValue = 0;
    
    dataBuffer[0] = ADXRS453_SENSOR_DATA;
    command = ((unsigned long)dataBuffer[0] << 24) |
              ((unsigned long)dataBuffer[1] << 16) |
              ((unsigned short)dataBuffer[2] << 8) |
              dataBuffer[3];
    for(bitNo = 31; bitNo > 0; bitNo--)
    {
        sum += ((command >> bitNo) & 0x1);
    }
    if(!(sum % 2))
    {
        dataBuffer[3] |= 1;
    }
    SPI_Write(ADXRS453_SLAVE_ID, dataBuffer, 4);
    SPI_Read(ADXRS453_SLAVE_ID, dataBuffer, 4);
    registerValue = ((unsigned long)dataBuffer[0] << 24) |
                    ((unsigned long)dataBuffer[1] << 16) |
                    ((unsigned short)dataBuffer[2] << 8) |
                    dataBuffer[3];
    
    return registerValue;
}

/***************************************************************************//**
 * @brief Reads the rate data and converts it to degrees/second.
 *
 * @param None.
 *
 * @return rate - The rate value in degrees/second.
*******************************************************************************/
float ADXRS453_GetRate(void)
{
    unsigned short registerValue = 0;
    float          rate          = 0.0;
    
    registerValue = ADXRS453_GetRegisterValue(ADXRS453_REG_RATE);
   
    /*!< If data received is in positive degree range */
    if(registerValue < 0x8000)
    {
        rate = ((float)registerValue / 80);
    }
    /*!< If data received is in negative degree range */
    else
    {
        rate = (-1) * ((float)(0xFFFF - registerValue + 1) / 80.0);
    }
    
    return rate;
}

/***************************************************************************//**
 * @brief Reads temperature from ADXRS453 and converts it to degrees Celsius
 *
 * @param None.
 *
 * @return temperature.
*******************************************************************************/
unsigned char ADXRS453_GetTemperature(void)
{
	unsigned short temp = 0;
	
	temp = ADXRS453_GetRegisterValue(ADXRS453_TEMP1);
	temp >>= 6;	
	temp -= 0x31f;
	temp /= 5;
	
	return (unsigned char)temp;
}
#endif

/**************** END OF FILE *****************************************/
