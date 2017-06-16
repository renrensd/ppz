/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				       *
*				  All Rights Reserved.          					   *
*   Department 	: R&D SW      									       *
*   AUTHOR	   	:            										   *
************************************************************************
* Object        :
* Module        :
* Instance      :
* Description   : FM25V01 driver
*-----------------------------------------------------------------------
* Version:
* Date:
* Author:
***********************************************************************/
/*-History--------------------------------------------------------------
* Version       Date    Name    Changes and comments
*
*=====================================================================*/
#ifndef _FM25V_H_
#define _FM25V_H_
#include "mcu_periph/spi.h"

/* Include address and register definition */
#define FM25V_WREN		0x06
#define FM25V_WRDI		0x04
#define FM25V_RDSR		0x05
#define FM25V_WRSR		0x01
#define FM25V_READ		0x03
#define FM25V_FSTRD		0x0B
#define FM25V_WRITE		0x02
#define FM25V_SLEEP		0xB9
#define FM25V_RDID		0x9F

#define FM25V_DEV_ID_7	0xC2
#define FM25V_DEV_ID_8	0x21
#define FM25V_DEV_ID_9	0x00

#define FM25V_BUF_SIZE	1020
#define FM25V_TIMEOUT_VAL	10000


enum FM25V_STATUS
{
	FM25V_STATE_UNINIT,                  /**< The chip isn't initialized */
	FM25V_STATE_IDLE,                    /**< The chip is idle and can be used */
	FM25V_STATE_READ_ID,                 /**< The chip is busy with getting the chip ID */
	FM25V_STATE_CHIP_ERASE,              /**< The chip is busy erasing itself */
	FM25V_STATE_WRITE_BYTES,             /**< The chip is busy writing bytes */
	FM25V_STATE_READ_BYTES,              /**< The chip is busy reading bytes */
};

struct FM25V_SPI
{
	volatile enum FM25V_STATUS status;   /**< The status of the SST25VFxxxx flash chip */
	uint16_t status_idx;                       /**< The counter of substatuses */
	struct spi_periph *spi_p;                 /**< The SPI peripheral for the connection */
	struct spi_transaction spi_t;             /**< The SPI transaction used for the writing and reading of registers */
	uint8_t input_buf[FM25V_BUF_SIZE+4];      /**< The input buffer for the SPI transaction */
	uint8_t output_buf[FM25V_BUF_SIZE+4];     /**< The output buffer for the SPI transaction */
	uint16_t flash_addr;                      /**< The flash address to write at */

	uint8_t *transfer_buf;                    /**< The transfer buffer */
	uint8_t *input_temp_buf;
	uint16_t transfer_idx;                     /**< The transfer idx is used for counting input/output bytes */
	uint16_t transfer_length;                  /**< The transfer buffer length */

	bool_t valid;
	uint16_t error_cnt;
	bool_t reading_flag;						/* wait read data,1-reading data,0-read finished. */
	bool_t writing_flag;						/* wait write data,1-writing data,0-write finished. */
};
/**** Definition of macros ****/


/**** Declaration of constants ****/


/**** Declaration of variables ****/


/**** Declaration of functions ****/
extern void fm25v_spi_cb(struct FM25V_SPI *fm);
extern void fm25v_init(struct FM25V_SPI *fm, struct spi_periph *spi_p, const uint8_t slave_idx, SPICallback spi_cb);
extern uint8_t fm25v_write(struct FM25V_SPI *fm, uint16_t addr, uint8_t *buf, uint16_t len);
extern uint8_t fm25v_read(struct FM25V_SPI *fm, uint16_t addr, uint8_t *buf, uint16_t len);

#endif /* _FM25V_H_ */

/****************************** END OF FILE ***************************/

