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
#ifndef _ADXRS453_SPI_H_
#define _ADXRS453_SPI_H_

/**** System include files ****/
#include "std.h"
#include "math/pprz_algebra_int.h"

/*---Public include files---------------------------------------------*/
#include "mcu_periph/spi.h"

/*---Private include files--------------------------------------------*/
#include "peripherals/adxrs453.h"


/**** Definition of constants ****/

/**** Definition of types ****/
struct Adxrs453_Spi
{
	struct spi_periph *spi_p;
	struct spi_transaction spi_trans;
	volatile uint8_t tx_buf[4];
	volatile uint8_t rx_buf[8];	 // for adxrs453, this is a temporary  of sensor regeister data. real sensor data is in databuf.
	enum Adxrs453ConfStatus init_status; ///< init status
	bool_t initialized;                  ///< config done flag
	volatile bool_t data_available;      ///< data ready flag
	union
	{
		struct Int16Rates rates;           ///< data vector in accel coordinate system
		int16_t value[3];                 ///< data values accessible by channel index
	} data;
	struct Adxrs453Config config;
	volatile uint16_t timer_tick;	/* adxrs453 startup delay time:ms. */
	uint8_t sq;
	uint8_t cur_seq;
	uint8_t parity;
	uint8_t status;
};



/**** Definition of macros ****/


/**** Declaration of constants ****/


/**** Declaration of variables ****/


/**** Declaration of functions ****/
extern void adxrs453_spi_init(struct Adxrs453_Spi *adxrs453, struct spi_periph *spi_p, uint8_t addr);
extern void adxrs453_spi_start_configure(struct Adxrs453_Spi *adxrs453);
extern void adxrs453_spi_read(struct Adxrs453_Spi *adxrs453, uint8_t axis);
extern void adxrs453_spi_event(struct Adxrs453_Spi *adxrs453, uint8_t axis);
extern void adxrs453_spi_periodic(struct Adxrs453_Spi *adxrs453, uint8_t axis);

#endif /* _ADXRS453_SPI_H_ */

/****************************** END OF FILE ***************************/

