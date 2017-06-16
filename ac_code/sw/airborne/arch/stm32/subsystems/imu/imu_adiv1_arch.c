/*
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */
#include "subsystems/imu.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

#include "subsystems/imu/imu_adiv1_arch.h"
#include "../../../../subsystems/imu/imu_adiv1.h"
#include "../../mcu_periph/gpio_arch.h"

void imu_adiv1_arch_init(void)
{
	rcc_periph_clock_enable(RCC_SYSCFG);
	//gpio_setup_input_pulldown(ADXRS_XY_EOC_PIN_PORT,ADXRS_XY_EOC_PIN);

#if 0
	exti_select_source(EXTI0, ADXRS_XY_EOC_PIN_PORT);
	exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
	exti_enable_request(EXTI0);

	exti_select_source(EXTI7, ADXRS_Z_EOC_PIN_PORT);
	exti_set_trigger(EXTI7, EXTI_TRIGGER_RISING);
	exti_enable_request(EXTI7);

	nvic_set_priority(NVIC_EXTI0_IRQ, 0x0e);
	nvic_enable_irq(NVIC_EXTI0_IRQ);

	nvic_set_priority(NVIC_EXTI9_5_IRQ, 0x0F);
	nvic_enable_irq(NVIC_EXTI9_5_IRQ);
#endif
}

void exti0_isr(void)
{
	exti_reset_request(EXTI0);
}


