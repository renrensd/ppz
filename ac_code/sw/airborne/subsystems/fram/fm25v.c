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

#include "modules/system/timer_if.h"
#include "modules/system/timer_class.h"
#include "modules/system/timer_def.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/gpio.h"
#include BOARD_CONFIG

/*---Private include files--------------------------------------------*/
#include "fm25v.h"


/*===VARIABLES========================================================*/

/*---Global-----------------------------------------------------------*/

 
/*---Private----------------------------------------------------------*/

/*===FUNCTIONS========================================================*/


/*---Global-----------------------------------------------------------*/
/*****************************************************************************
*  Name        : fm25v_init
*  Description : 
*  Parameter   : void  
*  Returns     : None
*****************************************************************************/
void fm25v_init(struct FM25V_SPI *fm, struct spi_periph *spi_p, const uint8_t slave_idx, SPICallback spi_cb)
{
	/* Set spi_peripheral and start flash address */
	fm->spi_p = spi_p;
	fm->flash_addr = 0x0;

	/* Set the spi transaction */
	fm->spi_t.cpol = SPICpolIdleLow;
	fm->spi_t.cpha = SPICphaEdge1;
	fm->spi_t.dss = SPIDss8bit;
	fm->spi_t.bitorder = SPIMSBFirst;
	fm->spi_t.cdiv = SPIDiv4;	//21MHz clk.

	fm->spi_t.input_length = 0;
	fm->spi_t.output_length = 0;
	fm->spi_t.input_buf = fm->input_buf;
	fm->spi_t.output_buf = fm->output_buf;
	fm->spi_t.slave_idx = slave_idx;
	fm->spi_t.select = SPISelectUnselect;
	fm->spi_t.status = SPITransDone;
	fm->spi_t.after_cb = spi_cb;

	/* Update the status and start with enabling writing */
	fm->status = FM25V_STATE_IDLE;
	fm->valid = FALSE;
	//fm25v_write_en(fm);
	fm25v_read_id(fm);
}

/*****************************************************************************
*  Name        : fm25v_spi_cb
*  Description : 
*  Parameter   : void  
*  Returns     : None
*****************************************************************************/
void fm25v_spi_cb(struct FM25V_SPI *fm)
{
	switch(fm->status)
	{
		case FM25V_STATE_READ_ID:
			if(fm->spi_t.status == SPITransFailed)
			{
				fm->error_cnt++;
			}
			else if( (fm->input_buf[7] == FM25V_DEV_ID_7) && (fm->input_buf[8] == FM25V_DEV_ID_8) )
			{
				fm->valid = TRUE;
			}
			
      		fm->status = FM25V_STATE_IDLE;
			fm->spi_t.input_buf = fm->input_buf;
			break;
		case FM25V_STATE_WRITE_BYTES:
			switch(fm->status_idx)
		  	{
		        // Send the address with 2 or 1 byte(s) of data
		        case 0:
			        fm->status_idx = 1;
			        fm->output_buf[0] = FM25V_WRITE;
			        fm->output_buf[1] = ((fm->flash_addr&0x3F) >> 8) & 0xFF;
			        fm->output_buf[2] = fm->flash_addr & 0xFF;
					fm->transfer_idx = 0;
					for(uint16_t i=0; i<fm->transfer_length; i++)
					{
						fm->output_buf[3+i] = fm->transfer_buf[i];
					}
			        fm->spi_t.output_length = 3 + fm->transfer_length;
			        fm->spi_t.input_length = 0;
			        spi_submit(fm->spi_p, &fm->spi_t);
			        break;
				case 1:
					if(fm->spi_t.status == SPITransFailed)
					{
						fm->error_cnt++;
					}
					
					fm->status = FM25V_STATE_IDLE;
					break;
		        
		        default:
					fm->error_cnt++;
					fm->status = FM25V_STATE_IDLE;
		          break;
	      	}
	      	break;

		case FM25V_STATE_READ_BYTES:
			if(fm->spi_t.status == SPITransFailed)
			{
				fm->error_cnt++;
			}
			fm->reading_flag = FALSE;
			// Reset the buffer pointer and goto idle
      		fm->spi_t.input_buf = fm->input_buf;
      		fm->status = FM25V_STATE_IDLE;
			break;
		default:
			break;
	}
}

/*****************************************************************************
*  Name        : fm25v_read_id
*  Description : 
*  Parameter   : void  
*  Returns     : None
*****************************************************************************/
void fm25v_read_id(struct FM25V_SPI *fm)
{
	if(fm->status != FM25V_STATE_IDLE)
	{
		return;
	}

	// Write the read id command to the chip
	fm->status = FM25V_STATE_READ_ID;
	fm->output_buf[0] = FM25V_RDID;
	fm->spi_t.output_length = 1;
	fm->spi_t.input_length = 10;
	spi_submit(fm->spi_p, &fm->spi_t);
}

/*****************************************************************************
*  Name        : fm25v_write
*  Description : 
*  Parameter   : void  
*  Returns     : None
*****************************************************************************/
void fm25v_write(struct FM25V_SPI *fm, uint16_t addr, uint8_t *buf, uint16_t len)
{
	if(fm->status != FM25V_STATE_IDLE) 
	{
		return;
	}

	if(len > FM25V_BUF_SIZE)
	{
		return;
	}

	// Set the transfer buffer
	fm->transfer_buf = buf; // Not copied so keep buffer available!
	fm->transfer_idx = 0;
	fm->transfer_length = len;
	fm->flash_addr = addr;

	// Enable writing
	fm->status = FM25V_STATE_WRITE_BYTES;
	fm->status_idx = 0;
	fm->output_buf[0] = FM25V_WREN;
	fm->spi_t.output_length = 1;
	fm->spi_t.input_length = 0;
	spi_submit(fm->spi_p, &fm->spi_t);
}

//uint16_t fm_t1,fm_t2,fm_tt;
/*****************************************************************************
*  Name        : fm25v_read
*  Description : 
*  Parameter   : void  
*  Returns     : None
*****************************************************************************/
void fm25v_read(struct FM25V_SPI *fm, uint16_t addr, uint8_t *buf, uint16_t len)
{
	uint16_t i;
	uint16_t fm_timeout = 0;


	if(fm->status != FM25V_STATE_IDLE) 
	{
		return;
	}

	if(len > FM25V_BUF_SIZE)
	{
		return;
	}

	fm->spi_t.input_buf = buf;
	fm->status = FM25V_STATE_READ_BYTES;
	fm->status_idx = 0;
	fm->output_buf[0] = FM25V_READ;
	fm->output_buf[1] = ((addr&0x3F) >> 8) & 0xFF;
	fm->output_buf[2] = addr & 0xFF;
	
	fm->spi_t.output_length = 3;
	fm->spi_t.input_length = 3 + len;
	fm->reading_flag = TRUE;
	spi_submit(fm->spi_p, &fm->spi_t);
	//gpio_set(DEBUG_GPIO);
	//fm_t1 = get_sys_time_usec();
	fm_timeout = 0;
	while( fm->reading_flag && (++fm_timeout < 60000) );
	//fm_tt = get_sys_time_usec() - fm_t1;
	//gpio_clear(DEBUG_GPIO);
}
/**************** END OF FILE *****************************************/
