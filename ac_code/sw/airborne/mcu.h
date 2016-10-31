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
 * @file mcu.h
 * @brief Arch independent mcu ( Micro Controller Unit ) utilities.
 */

#ifndef MCU_H
#define MCU_H


#include <mcu_arch.h>

/**
 * @defgroup mcu_periph MCU Peripherals
 * @{
 */
#ifdef FAULT_OPTION
struct MCU_FAULT_INFO
{
	uint32_t hfsr;
	uint32_t bfar;
	uint32_t mmfar;
	uint32_t cfsr;
	uint32_t msp;
	uint32_t msp_data[20];
	uint8_t wdg_error_cnt;
	uint8_t reset_type;
	uint32_t reset_src;
};

typedef enum
{
     MCU_RESET_BY_SW = 0x01,
     MCU_RESET_BY_PWR = 0x02,
     MCU_RESET_BY_PIN = 0x03,
     MCU_RESET_BY_WWDG =0x04,
}MCU_RESET_TYPE;

void mcu_fault_info_handle(uint16_t msp_offset);
extern void mcu_write_file_fault(void);
void mcu_check_reset_source(void);
void mcu_set_reset_type(uint8_t type);
void mcu_usagefault_test(void);
uint8_t mcu_get_reset_type(void);

struct MCU_INFO
{
	bool_t pw_is_first_on;	/* TRUE:mcu is first power on. */
	uint32_t reset_src;		/* MCU reset source,store RCC_CSR. */
};
extern struct MCU_INFO mcu_info;
#endif	/* FAULT_OPTION */

#ifdef WDG_OPTION
typedef enum
{  
	WDG_TASK_MAIN,  
	WDG_TASK_MODULES,
	WDG_TASK_FAILSAFE,
	WDG_TASK_TELEMETRY, 
    WDG_TASK_BARO,  
    WDG_TASK_OPS,   
    WDG_TASK_MONITORING, 
    WDG_EVENT_ALL, 
}WDG_TASK_IDS;

extern void mcu_set_task_wdg_flag(uint16_t task_id);

#endif	/* WDG_OPTION */


/**
 * Microcontroller peripherals initialization.
 * This function is responisble for setting up the microcontroller
 * after Reset.
 */
extern void mcu_init(void);

/**
 * MCU event functions.
 * Calls the event functions of used peripherals like i2c, uart, etc.
 */
extern void mcu_event(void);

/**
 * Optional board init function called at the end of mcu_init().
 */
extern void board_init(void);

/** @}*/

#endif /* MCU_H */


