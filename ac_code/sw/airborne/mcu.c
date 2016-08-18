/*
 * Copyright (C) 2010-2012 The Paparazzi team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file mcu.c
 * @brief Arch independent mcu ( Micro Controller Unit ) utilities.
 */
#include <string.h>
#include "mcu.h"
#include "std.h"
#ifndef NPS_SIMU
 #include <libopencm3/cm3/scb.h>
#endif
#ifdef PERIPHERALS_AUTO_INIT
#include "mcu_periph/sys_time.h"
#ifdef USE_LED
#include "led.h"
#endif
#if defined RADIO_CONTROL
#if defined RADIO_CONTROL_LINK  || defined RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT
#include "subsystems/radio_control.h"
#endif
#endif
#if USE_UART0 || USE_UART1 || USE_UART2 || USE_UART3 || USE_UART4 || USE_UART5 || USE_UART6 || USE_UART7 || USE_UART8
#define USING_UART 1
#include "mcu_periph/uart.h"
#endif
#if USE_I2C0 || USE_I2C1 || USE_I2C2 || USE_I2C3
#define USING_I2C 1
#include "mcu_periph/i2c.h"
#endif
#if USE_ADC
#include "mcu_periph/adc.h"
#endif
#if USE_USB_SERIAL
#include "mcu_periph/usb_serial.h"
#endif
#ifdef USE_UDP
#include "mcu_periph/udp.h"
#endif
#if USE_SPI
#include "mcu_periph/spi.h"
#endif
#ifdef USE_DAC
#include "mcu_periph/dac.h"
#endif
#ifdef SYS_TIMER_OPTION
#include "modules/system/timer_if.h"
#endif

#ifdef CALIBRATION_OPTION
#include "calibration.h"
#endif

#ifdef WDG_OPTION
#include "wdg.h"
#endif

#ifdef FAULT_OPTION
#include <libopencm3/stm32/rcc.h>
#endif

#endif /* PERIPHERALS_AUTO_INIT */

#ifndef NPS_SIMU
//#include "mcu_periph/gpio.h"
#include BOARD_CONFIG


void usage_fault_handler(void);
void bus_fault_handler(void);
void mem_manage_handler(void);
void nmi_handler(void);
void hard_fault_handler(void);
#endif

#ifdef FAULT_OPTION
struct MCU_FAULT_INFO mcu_fault_info __attribute__ ((section(".bkpram"), aligned(4)));
#endif	/* FAULT_OPTION */

#ifdef WDG_OPTION
volatile uint16_t mcu_watchdog_flag __attribute__ ((section(".bkpram"), zero_init));
#endif	/* WDG_OPTION */

struct MCU_INFO mcu_info;

void WEAK board_init(void)
{
  // default board init function does nothing...
  #ifndef NPS_SIMU
	gpio_setup_output(ECS_PWM_EN_GPIO);
  	gpio_set(ECS_PWM_EN_GPIO);
	gpio_setup_output(OPS_PWR_EN_GPIO);
  	gpio_set(OPS_PWR_EN_GPIO);
	
    //gpio_setup_input_pulldown(DEBUG_GPIO);
  #endif
}

void mcu_init(void)
{
  mcu_arch_init();
  #ifdef FAULT_OPTION
  mcu_check_reset_source();
  #endif
  
  /* If we have a board specific init function, call it.
   * Otherwise it will simply call the empty weak function.
   */
  board_init();

#ifdef PERIPHERALS_AUTO_INIT
  //#ifndef RTOS_IS_CHIBIOS	// TODOM:
  sys_time_init();
  //#endif //RTOS_IS_CHIBIOS
#ifdef SYS_TIMER_OPTION 
 #ifndef NPS_SIMU    //nps not use timer
  tm_reset_create();
 #endif
#endif
#ifdef USE_LED
  led_init();
#endif
  /* for now this means using spektrum */
#if defined RADIO_CONTROL & defined RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT & defined RADIO_CONTROL_BIND_IMPL_FUNC
  RADIO_CONTROL_BIND_IMPL_FUNC();
#endif
#if USE_UART0
  uart0_init();
#endif
#if USE_UART1
  uart1_init();
#endif
#if USE_UART2
  uart2_init();
#endif
#if USE_UART3
  uart3_init();
#endif
#if USE_UART4
  uart4_init();
#endif
#if USE_UART5
  uart5_init();
#endif
#if USE_UART6
  uart6_init();
#endif
#if USE_UART7
  uart7_init();
#endif
#if USE_UART8
  uart8_init();
#endif
#if USING_UART
  uart_arch_init();
#endif
#ifdef USE_I2C0
  i2c0_init();
#endif
#ifdef USE_I2C1
  i2c1_init();
#endif
#ifdef USE_I2C2
  i2c2_init();
#endif
#ifdef USE_I2C3
  i2c3_init();
#endif
#if USE_ADC
  adc_init();
#endif
#if USE_USB_SERIAL
  VCOM_init();
#endif

#if USE_SPI
#if SPI_MASTER

#if USE_SPI0
  spi0_init();
#endif
#if USE_SPI1
  spi1_init();
#endif
#if USE_SPI2
  spi2_init();
#endif
#if USE_SPI3
  spi3_init();
#endif
  spi_init_slaves();
#endif // SPI_MASTER

#if SPI_SLAVE
#if USE_SPI0_SLAVE
  spi0_slave_init();
#endif
#if USE_SPI1_SLAVE
  spi1_slave_init();
#endif
#if USE_SPI2_SLAVE
  spi2_slave_init();
#endif
#if USE_SPI3_SLAVE
  spi3_slave_init();
#endif
#endif // SPI_SLAVE

#if SPI_SLAVE_HS
  spi_slave_hs_init();
#endif
#endif // USE_SPI

#ifdef USE_DAC
  dac_init();
#endif

#if USE_UDP0 || USE_UDP1 || USE_UDP2
  udp_arch_init();
#endif

#ifdef CALIBRATION_OPTION
  sd_fatfs_init();
  #ifdef FAULT_OPTION
  if(mcu_info.pw_is_first_on == FALSE)
  {
  	mcu_write_file_fault();
  }
  #endif	/* FAULT_OPTION */
#endif	/* CALIBRATION_OPTION */

#ifdef WDG_OPTION
	wdg_init();
	mcu_fault_info.wdg_error_cnt = 0;
	mcu_fault_info.reset_type = 0;
#endif

#else
  INFO("PERIPHERALS_AUTO_INIT not enabled! Peripherals (including sys_time) need explicit initialization.")
#endif /* PERIPHERALS_AUTO_INIT */

}

void mcu_usagefault_test(void)
{
	struct time
	{
		unsigned char hour;
		unsigned char min;
		unsigned char sec;
	};

	struct time *p = 0;
	p->hour = 0;
}

uint8_t mcu_val;
void mcu_event(void)
{	
#if USING_I2C
  i2c_event();
#endif

#if USE_USB_SERIAL
  VCOM_event();
#endif

#if 0
	if(mcu_val == 100)
	{
		mcu_val = 0;
		mcu_usagefault_test();
	}
	else if(mcu_val == 10)
	{
		mcu_val = 0;
		while(1);
	}
#endif
}

#ifdef FAULT_OPTION
void mcu_fault_info_handle(uint16_t msp_offset)
{
	//gpio_set(ECS_PWM_EN_GPIO);
	uint32_t *vp = (mcu_fault_info.msp + msp_offset);
	
	for(uint8_t i=0; i<15; i++)
	{
		mcu_fault_info.msp_data[i] = *(vp+i);
	}
	
	mcu_fault_info.hfsr= SCB_HFSR;
	mcu_fault_info.bfar= SCB_BFAR;
	mcu_fault_info.cfsr= SCB_CFSR;
	mcu_fault_info.mmfar=SCB_MMFAR;

	if(mcu_get_reset_type() != MCU_RESET_BY_WWDG)
	{
		mcu_set_reset_type(MCU_RESET_BY_SW);
	}
	scb_reset_system();
	while(1);

#if 0
	  __asm volatile
    (
    " tst lr, #4                                                \n"
    " ite eq                                                    \n"
    " mrseq r0, msp                                             \n"
    " mrsne r0, psp                                             \n"
    " ldr r1, [r0, #24]                                         \n"
    " ldr r2, handler2_address_const                            \n"
    " bx r2                                                     \n"
    " handler2_address_const: .word _prvGetRegistersFromStack    \n"
    );
	volatile uint32_t psr __attribute__((unused));
	volatile uint32_t pc __attribute__((unused));
	volatile uint32_t msp __attribute__((unused));
	volatile uint32_t r0 __attribute__((unused));
	volatile uint32_t r1 __attribute__((unused));
	volatile uint32_t r2 __attribute__((unused));
	volatile uint32_t r3 __attribute__((unused));
	volatile uint32_t r12 __attribute__((unused));
	volatile uint32_t lr __attribute__((unused));
	 
	__asm volatile ("MOV %0, pc" : "=r" (pc) );
    __asm volatile ("MOV %0, r0" : "=r" (r0) );
	__asm volatile ("MOV %0, r1" : "=r" (r1) );
	__asm volatile ("MOV %0, r2" : "=r" (r2) );
	__asm volatile ("MOV %0, r3" : "=r" (r3) );
	__asm volatile ("MOV %0, r12" : "=r" (r12) );
	__asm volatile ("MOV %0, lr" : "=r" (lr) );
	__asm volatile ("MRS %0, psr" : "=r" (psr) );
#endif	
}

/*****************************************************************************
*  Name        : mcu_check_reset_source
*  Description : check which source trigger the mcu start
*  Parameter   : None
*  Returns     : None
*****************************************************************************/
void mcu_check_reset_source(void)
{
    uint32_t rst_source;
	
	rst_source = RCC_CSR;
	mcu_info.reset_src = rst_source;
	mcu_fault_info.reset_src = rst_source;
	
	if( (rst_source & RCC_CSR_PORRSTF) == RCC_CSR_PORRSTF )
    {
		memset(&mcu_fault_info,0,sizeof(mcu_fault_info));
#ifdef WDG_OPTION
		mcu_watchdog_flag = 0;
#endif
		mcu_info.pw_is_first_on = TRUE;
	}
	else
	{
		mcu_info.pw_is_first_on = FALSE;
	}

	RCC_CSR |= RCC_CSR_RMVF;
}

/*****************************************************************************
*  Name        : mcu_set_reset_type
*  Description : 
*  Parameter   : None
*  Returns     : None
*****************************************************************************/
void mcu_set_reset_type(uint8_t type)
{
	mcu_fault_info.reset_type = type;
}

/*****************************************************************************
*  Name        : mcu_get_reset_type
*  Description : 
*  Parameter   : None
*  Returns     : None
*****************************************************************************/
uint8_t mcu_get_reset_type(void)
{
	return mcu_fault_info.reset_type;
}
#endif	/* FAULT_OPTION */

#ifndef NPS_SIMU
void usage_fault_handler(void)
{
	__asm volatile ("MRS %0, msp" : "=r" (mcu_fault_info.msp) );
	mcu_fault_info_handle(0x08);
	while(1);
}

void bus_fault_handler(void)
{
	//gpio_set(ECS_PWM_EN_GPIO);
	__asm volatile ("MRS %0, msp" : "=r" (mcu_fault_info.msp) );
	mcu_fault_info_handle(0x08);
	while(1);
}

void mem_manage_handler(void)
{
	//gpio_set(ECS_PWM_EN_GPIO);
	__asm volatile ("MRS %0, msp" : "=r" (mcu_fault_info.msp) );
	mcu_fault_info_handle(0x08);
	while(1);
}

void nmi_handler(void)
{
    //gpio_set(ECS_PWM_EN_GPIO);
    __asm volatile ("MRS %0, msp" : "=r" (mcu_fault_info.msp) );
    mcu_fault_info_handle(0x08);
	while(1);
}

void hard_fault_handler(void)
{
	//gpio_set(ECS_PWM_EN_GPIO);
	__asm volatile ("MRS %0, msp" : "=r" (mcu_fault_info.msp) );
	mcu_fault_info_handle(0x08);
	while(1);
}

#ifdef WDG_OPTION
/***********************************************************************
* FUNCTION    : nvic_wwdg_isr
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void nvic_wwdg_isr(void)
{
	uint8_t wr;
	uint8_t tr;

	WWDG_ClearFlag();
	mcu_fault_info.wdg_error_cnt++;
	wr = WWDG->CFR & 0x7F;
	tr = WWDG->CR & 0x7F;
	if(tr < wr)
	{
		if(mcu_fault_info.wdg_error_cnt < 2)
		{
			wdg_feed();
		}
		else
		{
			mcu_fault_info.wdg_error_cnt = 0;
			__asm volatile ("MRS %0, msp" : "=r" (mcu_fault_info.msp) );
			mcu_set_reset_type(MCU_RESET_BY_WWDG);
			mcu_fault_info_handle(0x08);
		}
	}	
}

/*****************************************************************************
*  Name        : mcu_set_task_wdg_flag
*  Description : 
*  Parameter   : WDG_TASK_IDS
*  Returns     : None
*****************************************************************************/
void mcu_set_task_wdg_flag(uint16_t task_id)
{
	 mcu_watchdog_flag = 1 << task_id;
}
#endif /* WDG_OPTION */

#ifdef FAULT_OPTION
/***********************************************************************
* FUNCTION    : mcu_write_file_fault
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void mcu_write_file_fault(void)
{
    sd_write_file_fault("<--- mcu_fault_info_start --->", 0, 1);
	sd_write_file_fault("mcu_reset_source", mcu_fault_info.reset_src, 0);
	sd_write_file_fault("mcu_reset_type", mcu_fault_info.reset_type, 0);
	sd_write_file_fault("mcu_reset_time", sys_time.nb_sec, 0);
#ifdef WDG_OPTION
	sd_write_file_fault("mcu_watchdog_flag", mcu_watchdog_flag, 0);
#endif
	sd_write_file_fault("mcu_fault_info.msp", mcu_fault_info.msp, 0);
	sd_write_file_fault("mcu_fault_info.hfsr", mcu_fault_info.hfsr, 0);
	sd_write_file_fault("mcu_fault_info.bfar", mcu_fault_info.bfar, 0);
	sd_write_file_fault("mcu_fault_info.cfsr", mcu_fault_info.cfsr, 0);
	sd_write_file_fault("mcu_fault_info.mmfar", mcu_fault_info.mmfar, 0);
	
	for(uint8_t i=0; i<15; i++)
	{
		sd_write_file_fault("mcu_fault_info.msp_data", mcu_fault_info.msp_data[i], 0);
	}

	sd_write_file_fault("<--- mcu_fault_info_end --->", 0, 2);
}
#endif	/* FAULT_OPTION */

#endif /* NPS_SIMU */

