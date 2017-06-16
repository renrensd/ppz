/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for STMicroelectronics STM32F4-Discovery board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_APOGEE
#define BOARD_NAME  "EFly STM32F4 Krooz sd"


/*
 * Board oscillators-related settings.
 * NOTE: LSE fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                25000000
#endif


/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   330

/*
 * MCU type as defined in the ST header file stm32f4xx.h.
 */
#define STM32F40_41xxx
//#define STM32F4XX

/*
 * IO pins assignments.
 */
#define GPIOA_UART4_TX              0 // 
#define GPIOA_UART4_RX              1 // 
#define GPIOA_ETH_RMII_MDIO         2 // 
#define GPIOA_ADC1_IN3              3 // BAT ADC
#define GPIOA_ADC1_IN4              4 // 
#define GPIOA_SPI1_SCK              5
#define GPIOA_SPI1_MISO             6
#define GPIOA_ETH_RMII_CRS_DV       7
#define GPIOA_I2C3_SCL              8 // I2C3-SCL / MCO
#define GPIOA_OTG_FS_VBUS           9 //
#define GPIOA_IO_3V3_SENS_EN        10 // 
#define GPIOA_OTG_FS_DM             11 //
#define GPIOA_OTG_FS_DP             12 //
#define GPIOA_SWDIO                 13 // SERIAL WIRE DEBUG
#define GPIOA_SWCLK                 14 // SERIAL WIRE DEBUG
#define GPIOA_TIM2_ICU_PPM          15 // 

#define GPIOB_ADC1_IN8              0 // 
#define GPIOB_ADC1_IN9              1 // 
#define GPIOB_BOOT1                 2 //
#define GPIOB_PIN3                  3 // 
#define GPIOB_PIN4                  4 // 
#define GPIOB_SPI1_MOSI             5 //
#define GPIOB_PWM4_CH1              6 // SERO_CH5
#define GPIOB_SPI1_CS0_IMU          7
#define GPIOB_I2C1_SCL              8
#define GPIOB_I2C1_SDA              9 //
#define GPIOB_SPI2_SCK              10
#define GPIOB_ETH_RMII_TX_EN        11
#define GPIOB_ETH_RMII_TXD0         12 //
#define GPIOB_ETH_RMII_TXD1         13 //
#define GPIOB_SPI2_MISO             14 //
#define GPIOB_SPI2_MOSI             15 //

#define GPIOC_ADC1_IN10             0 //
#define GPIOC_ETH_RMII_MDC          1 // 
#define GPIOC_ADC1_IN12             2 // 
#define GPIOC_ADC1_IN13             3 // 
#define GPIOC_ETH_RMII_RXD0         4 // 
#define GPIOC_ETH_RMII_RXD1         5 // 
#define GPIOC_USART6_TX             6 // jaune
#define GPIOC_USART6_RX             7 // vert
#define GPIOC_SDIO_D0               8
#define GPIOC_SDIO_D1               9
#define GPIOC_SDIO_D2               10
#define GPIOC_SDIO_D3               11
#define GPIOC_SDIO_CK               12
#define GPIOC_SPI2_CS1              13
#define GPIOC_OSC32_IN              14
#define GPIOC_OSC32_OUT             15

#define GPIOD_CAN1_RX               0
#define GPIOD_CAN1_TX               1
#define GPIOD_SDIO_CMD                2
#define GPIOD_IO_LED1               3
#define GPIOD_IO_LED2               4
#define GPIOD_USART2_TX             5
#define GPIOD_USART2_RX             6
#define GPIOD_SPI1_CS1_BARO         7
#define GPIOD_USART3_TX             8
#define GPIOD_USART3_RX             9
#define GPIOD_SPI2_CS0_FRAM         10	//FRAM CS
#define GPIOD_USART3_CTS            11
#define GPIOD_USART3_RTS            12
#define GPIOD_PWM4_CH2              13 //SERO_CH6
#define GPIOD_PWM4_CH3              14 //SERO_CH7
#define GPIOD_PWM4_CH4              15 //SERO_CH8

#define GPIOE_INT_SPI1_IMU          0
#define GPIOE_INT_SPI2_RDY          1
#define GPIOE_PIN2                  2
#define GPIOE_PIN3                  3
#define GPIOE_PIN4                  4
#define GPIOE_PIN5                  5
#define GPIOE_PIN6                  6
#define GPIOE_INT_SPI1_MAG          7
#define GPIOE_PIN8                  8
#define GPIOE_PWM1_CH1              9 //SERO_CH1
#define GPIOE_PIN10                 10
#define GPIOE_PWM1_CH2              11 //SERO_CH2
#define GPIOE_PIN12                 12
#define GPIOE_PWM1_CH3              13 //SERO_CH3
#define GPIOE_PWM1_CH4              14 //SERO_CH4
#define GPIOE_INT_RES15             15

#define GPIOH_OSC_IN                0
#define GPIOH_OSC_OUT               1
#define GPIOH_PIN2                  2
#define GPIOH_PIN3                  3
#define GPIOH_PIN4                  4
#define GPIOH_PIN5                  5
#define GPIOH_PIN6                  6
#define GPIOH_PIN7                  7
#define GPIOH_PIN8                  8
#define GPIOH_PIN9                  9
#define GPIOH_PIN10                 10
#define GPIOH_PIN11                 11
#define GPIOH_PIN12                 12
#define GPIOH_PIN13                 13
#define GPIOH_PIN14                 14
#define GPIOH_PIN15                 15


/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << ((n % 8) * 4))

/*
 * GPIOA setup:
 *
#define GPIOA_UART4_TX              0 //
#define GPIOA_UART4_RX              1 //
#define GPIOA_ETH_RMII_MDIO         2 //
#define GPIOA_ADC1_IN3              3 // BAT ADC
#define GPIOA_ADC1_IN4              4 //
#define GPIOA_SPI1_SCK              5
#define GPIOA_SPI1_MISO             6
#define GPIOA_ETH_RMII_CRS_DV       7
#define GPIOA_I2C3_SCL              8 // I2C3-SCL / MCO
#define GPIOA_OTG_FS_VBUS           9 //
#define GPIOA_IO_3V3_SENS_EN        10 //
#define GPIOA_OTG_FS_DM             11 //
#define GPIOA_OTG_FS_DP             12 //
#define GPIOA_SWDIO                 13 // SERIAL WIRE DEBUG
#define GPIOA_SWCLK                 14 // SERIAL WIRE DEBUG
#define GPIOA_PPM_IN                15 //
 */
#define VAL_GPIOA_MODER             (PIN_MODE_ALTERNATE(GPIOA_UART4_TX)       |     \
                                     PIN_MODE_ALTERNATE(GPIOA_UART4_RX)       |     \
                                     PIN_MODE_ALTERNATE(GPIOA_ETH_RMII_MDIO)  |     \
                                     PIN_MODE_ANALOG(GPIOA_ADC1_IN3)          |     \
                                     PIN_MODE_ANALOG(GPIOA_ADC1_IN4)          |     \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_SCK)       |     \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MISO)      |     \
                                     PIN_MODE_ALTERNATE(GPIOA_ETH_RMII_CRS_DV)|     \
                                     PIN_MODE_ALTERNATE(GPIOA_I2C3_SCL)       |     \
                                     PIN_MODE_INPUT(GPIOA_OTG_FS_VBUS)        |     \
                                     PIN_MODE_OUTPUT(GPIOA_IO_3V3_SENS_EN)    |     \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM)      |     \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP)      |     \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO)          |     \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK)          |     \
                                     PIN_MODE_ALTERNATE(GPIOA_TIM2_ICU_PPM))

#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_UART4_TX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART4_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ETH_RMII_MDIO) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC1_IN3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC1_IN4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_SCK) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MISO) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ETH_RMII_CRS_DV) |    \
                                     PIN_OTYPE_OPENDRAIN(GPIOA_I2C3_SCL) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_VBUS) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_IO_3V3_SENS_EN) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DP) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOA_TIM2_ICU_PPM))

#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_100M(GPIOA_UART4_TX) |     \
                                     PIN_OSPEED_100M(GPIOA_UART4_RX) |     \
                                     PIN_OSPEED_100M(GPIOA_ETH_RMII_MDIO) |     \
                                     PIN_OSPEED_100M(GPIOA_ADC1_IN3) |     \
                                     PIN_OSPEED_100M(GPIOA_ADC1_IN4) |     \
                                     PIN_OSPEED_100M(GPIOA_SPI1_SCK) |     \
                                     PIN_OSPEED_100M(GPIOA_SPI1_MISO) |    \
                                     PIN_OSPEED_100M(GPIOA_ETH_RMII_CRS_DV) |    \
                                     PIN_OSPEED_100M(GPIOA_I2C3_SCL) |     \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_VBUS) |  \
                                     PIN_OSPEED_100M(GPIOA_IO_3V3_SENS_EN) |    \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_DM) |    \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_DP) |    \
                                     PIN_OSPEED_100M(GPIOA_SWDIO) |        \
                                     PIN_OSPEED_100M(GPIOA_SWCLK) |        \
                                     PIN_OSPEED_100M(GPIOA_TIM2_ICU_PPM))

#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_UART4_TX) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_UART4_RX) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_ETH_RMII_MDIO) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_ADC1_IN3) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_ADC1_IN4) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_SCK) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_MISO) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_ETH_RMII_CRS_DV) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_I2C3_SCL) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_VBUS) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_IO_3V3_SENS_EN) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DP) |    \
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO) |          \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) |        \
                                     PIN_PUPDR_FLOATING(GPIOA_TIM2_ICU_PPM))

#define VAL_GPIOA_ODR               (PIN_ODR_LOW(GPIOA_UART4_TX) |     \
                                     PIN_ODR_LOW(GPIOA_UART4_RX) |     \
                                     PIN_ODR_LOW(GPIOA_ETH_RMII_MDIO) |     \
                                     PIN_ODR_LOW(GPIOA_ADC1_IN3) |     \
                                     PIN_ODR_LOW(GPIOA_ADC1_IN4) |     \
                                     PIN_ODR_LOW(GPIOA_SPI1_SCK) |     \
                                     PIN_ODR_LOW(GPIOA_SPI1_MISO) |    \
                                     PIN_ODR_LOW(GPIOA_ETH_RMII_CRS_DV) |    \
                                     PIN_ODR_HIGH(GPIOA_I2C3_SCL) |     \
                                     PIN_ODR_LOW(GPIOA_OTG_FS_VBUS) |  \
                                     PIN_ODR_LOW(GPIOA_IO_3V3_SENS_EN) |    \
                                     PIN_ODR_LOW(GPIOA_OTG_FS_DM) |    \
                                     PIN_ODR_LOW(GPIOA_OTG_FS_DP) |    \
                                     PIN_ODR_LOW(GPIOA_SWDIO) |        \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |        \
                                     PIN_ODR_LOW(GPIOA_TIM2_ICU_PPM))

#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_UART4_TX, 8) |   \
                                     PIN_AFIO_AF(GPIOA_UART4_RX, 8) |   \
                                     PIN_AFIO_AF(GPIOA_ETH_RMII_MDIO, 11) |   \
                                     PIN_AFIO_AF(GPIOA_SPI1_SCK, 5) |   \
                                     PIN_AFIO_AF(GPIOA_SPI1_MISO, 5) |  \
                                     PIN_AFIO_AF(GPIOA_ETH_RMII_CRS_DV, 11))

#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_I2C3_SCL, 4) |     \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10) |   \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10) |   \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0) |        \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0) |        \
                                     PIN_AFIO_AF(GPIOA_TIM2_ICU_PPM, 1))

/*
 * GPIOB setup:
 *
#define GPIOB_ADC1_IN8              0 //
#define GPIOB_ADC1_IN9              1 //
#define GPIOB_BOOT1                 2 //
#define GPIOB_PIN3                  3 //
#define GPIOB_PIN4                  4 //
#define GPIOB_SPI1_MOSI             5 //
#define GPIOB_PWM4_CH1              6 // SERO_CH5
#define GPIOB_SPI1_CS0_IMU          7
#define GPIOB_I2C1_SCL              8
#define GPIOB_I2C1_SDA              9 //
#define GPIOB_SPI2_SCK              10
#define GPIOB_ETH_RMII_TX_EN        11
#define GPIOB_ETH_RMII_TXD0         12 //
#define GPIOB_ETH_RMII_TXD1         13 //
#define GPIOB_SPI2_MISO             14 //
#define GPIOB_SPI2_MOSI             15 //
 */
#define VAL_GPIOB_MODER             (PIN_MODE_ANALOG(GPIOB_ADC1_IN8)        | \
                                     PIN_MODE_ANALOG(GPIOB_ADC1_IN9)        | \
                                     PIN_MODE_INPUT(GPIOB_BOOT1)           | \
                                     PIN_MODE_INPUT(GPIOB_PIN3)    | \
                                     PIN_MODE_INPUT(GPIOB_PIN4)    | \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI1_MOSI)    | \
                                     PIN_MODE_ALTERNATE(GPIOB_PWM4_CH1)   | \
                                     PIN_MODE_OUTPUT(GPIOB_SPI1_CS0_IMU)    | \
                                     PIN_MODE_ALTERNATE( GPIOB_I2C1_SCL)    | \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SDA)            | \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_SCK)    | \
                                     PIN_MODE_ALTERNATE(GPIOB_ETH_RMII_TX_EN)    | \
                                     PIN_MODE_ALTERNATE(GPIOB_ETH_RMII_TXD0)    | \
                                     PIN_MODE_ALTERNATE(GPIOB_ETH_RMII_TXD1)   | \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_MISO) | \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_MOSI))

#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_ADC1_IN8)        | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ADC1_IN9)        | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_BOOT1)           | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN3)    | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN4)    | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI1_MOSI)    | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PWM4_CH1)   | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI1_CS0_IMU)    | \
                                     PIN_OTYPE_OPENDRAIN( GPIOB_I2C1_SCL)    | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SDA)            | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI2_SCK)    | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ETH_RMII_TX_EN)    | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ETH_RMII_TXD0)    | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ETH_RMII_TXD1)   | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI2_MISO) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI2_MOSI))

#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_100M(GPIOB_ADC1_IN8)        | \
                                     PIN_OSPEED_100M(GPIOB_ADC1_IN9)        | \
                                     PIN_OSPEED_100M(GPIOB_BOOT1)           | \
                                     PIN_OSPEED_100M(GPIOB_PIN3)    | \
                                     PIN_OSPEED_100M(GPIOB_PIN4)    | \
                                     PIN_OSPEED_100M(GPIOB_SPI1_MOSI)    | \
                                     PIN_OSPEED_100M(GPIOB_PWM4_CH1)   | \
                                     PIN_OSPEED_100M(GPIOB_SPI1_CS0_IMU)    | \
                                     PIN_OSPEED_100M( GPIOB_I2C1_SCL)    | \
                                     PIN_OSPEED_100M(GPIOB_I2C1_SDA)            | \
                                     PIN_OSPEED_100M(GPIOB_SPI2_SCK)    | \
                                     PIN_OSPEED_100M(GPIOB_ETH_RMII_TX_EN)    | \
                                     PIN_OSPEED_100M(GPIOB_ETH_RMII_TXD0)    | \
                                     PIN_OSPEED_100M(GPIOB_ETH_RMII_TXD1)   | \
                                     PIN_OSPEED_100M(GPIOB_SPI2_MISO) | \
                                     PIN_OSPEED_100M(GPIOB_SPI2_MOSI))

#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_ADC1_IN8)        | \
                                     PIN_PUPDR_FLOATING(GPIOB_ADC1_IN9)        | \
                                     PIN_PUPDR_FLOATING(GPIOB_BOOT1)           | \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN3)    | \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN4)    | \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI1_MOSI)    | \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PWM4_CH1)   | \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI1_CS0_IMU)    | \
                                     PIN_PUPDR_FLOATING( GPIOB_I2C1_SCL)    | \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SDA)            | \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI2_SCK)    | \
                                     PIN_PUPDR_FLOATING(GPIOB_ETH_RMII_TX_EN)    | \
                                     PIN_PUPDR_FLOATING(GPIOB_ETH_RMII_TXD0)    | \
                                     PIN_PUPDR_FLOATING(GPIOB_ETH_RMII_TXD1)   | \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI2_MISO) | \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI2_MOSI))

#define VAL_GPIOB_ODR               (PIN_ODR_LOW(GPIOB_ADC1_IN8)        | \
                                     PIN_ODR_LOW(GPIOB_ADC1_IN9)        | \
                                     PIN_ODR_LOW(GPIOB_BOOT1)           | \
                                     PIN_ODR_LOW(GPIOB_PIN3)    | \
                                     PIN_ODR_LOW(GPIOB_PIN4)    | \
                                     PIN_ODR_LOW(GPIOB_SPI1_MOSI)    | \
                                     PIN_ODR_LOW(GPIOB_PWM4_CH1)   | \
                                     PIN_ODR_HIGH(GPIOB_SPI1_CS0_IMU)    | \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SCL)    | \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SDA)            | \
                                     PIN_ODR_LOW(GPIOB_SPI2_SCK)    | \
                                     PIN_ODR_LOW(GPIOB_ETH_RMII_TX_EN)    | \
                                     PIN_ODR_LOW(GPIOB_ETH_RMII_TXD0)    | \
                                     PIN_ODR_LOW(GPIOB_ETH_RMII_TXD1)   | \
                                     PIN_ODR_LOW(GPIOB_SPI2_MISO) | \
                                     PIN_ODR_LOW(GPIOB_SPI2_MOSI))

#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_SPI1_MOSI, 5) |     \
                                     PIN_AFIO_AF(GPIOB_PWM4_CH1, 2))


#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_I2C1_SDA, 4) |    \
									 PIN_AFIO_AF(GPIOB_I2C1_SCL, 4) |       \
                                     PIN_AFIO_AF(GPIOB_SPI2_SCK, 5) |       \
                                     PIN_AFIO_AF(GPIOB_ETH_RMII_TX_EN, 11) |       \
                                     PIN_AFIO_AF(GPIOB_ETH_RMII_TXD0, 11) |       \
                                     PIN_AFIO_AF(GPIOB_ETH_RMII_TXD1, 11) |          \
                                     PIN_AFIO_AF(GPIOB_SPI2_MISO, 5) |          \
                                     PIN_AFIO_AF(GPIOB_SPI2_MOSI, 5))

/*
 * GPIOC setup:
 *
#define GPIOC_ADC1_IN10             0 //
#define GPIOC_ETH_RMII_MDC          1 //
#define GPIOC_ADC1_IN12             2 //
#define GPIOC_ADC1_IN13             3 //
#define GPIOC_ETH_RMII_RXD0         4 //
#define GPIOC_ETH_RMII_RXD1         5 //
#define GPIOC_USART6_TX             6 // jaune
#define GPIOC_USART6_RX             7 // vert
#define GPIOC_SDIO_D0               8
#define GPIOC_SDIO_D1               9
#define GPIOC_SDIO_D2               10
#define GPIOC_SDIO_D3               11
#define GPIOC_SDIO_CK               12
#define GPIOC_SPI2_CS1              13
#define GPIOC_OSC32_IN              14
#define GPIOC_OSC32_OUT             15
 */
#define VAL_GPIOC_MODER             (PIN_MODE_ANALOG(GPIOC_ADC1_IN10) |          \
                                     PIN_MODE_ALTERNATE(GPIOC_ETH_RMII_MDC) |           \
                                     PIN_MODE_ANALOG(GPIOC_ADC1_IN12) |           \
                                     PIN_MODE_ANALOG(GPIOC_ADC1_IN13) |           \
                                     PIN_MODE_ALTERNATE(GPIOC_ETH_RMII_RXD0) |       \
                                     PIN_MODE_ALTERNATE(GPIOC_ETH_RMII_RXD1) |       \
                                     PIN_MODE_ALTERNATE(GPIOC_USART6_TX) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_USART6_RX) |  \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D0) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D1) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D2) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D3) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_CK) |   \
                                     PIN_MODE_OUTPUT(GPIOC_SPI2_CS1) |         \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) |      \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))

#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_ADC1_IN10) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ETH_RMII_MDC) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ADC1_IN12) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ADC1_IN13) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ETH_RMII_RXD0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ETH_RMII_RXD1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USART6_TX) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USART6_RX) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D3) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_CK) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SPI2_CS1) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))

#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_100M(GPIOC_ADC1_IN10) |       \
                                     PIN_OSPEED_100M(GPIOC_ETH_RMII_MDC) |       \
                                     PIN_OSPEED_100M(GPIOC_ADC1_IN12) |       \
                                     PIN_OSPEED_100M(GPIOC_ADC1_IN13) |       \
                                     PIN_OSPEED_100M(GPIOC_ETH_RMII_RXD0) |       \
                                     PIN_OSPEED_100M(GPIOC_ETH_RMII_RXD1) |       \
                                     PIN_OSPEED_100M(GPIOC_USART6_TX) |       \
                                     PIN_OSPEED_100M(GPIOC_USART6_RX) |       \
                                     PIN_OSPEED_100M(GPIOC_SDIO_D0) |       \
                                     PIN_OSPEED_100M(GPIOC_SDIO_D1) |       \
                                     PIN_OSPEED_100M(GPIOC_SDIO_D2) |       \
                                     PIN_OSPEED_100M(GPIOC_SDIO_D3) |      \
                                     PIN_OSPEED_100M(GPIOC_SDIO_CK) |       \
                                     PIN_OSPEED_100M(GPIOC_SPI2_CS1) |      \
                                     PIN_OSPEED_100M(GPIOC_OSC32_IN) |      \
                                     PIN_OSPEED_100M(GPIOC_OSC32_OUT))

#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_ADC1_IN10) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_ETH_RMII_MDC) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_ADC1_IN12) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_ADC1_IN13) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_ETH_RMII_RXD0) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_ETH_RMII_RXD1) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_USART6_TX) |         \
                                     PIN_PUPDR_FLOATING(GPIOC_USART6_RX) |       \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D0) |              \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D1) |              \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D2) |              \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D3) |              \
                                     PIN_PUPDR_FLOATING(GPIOC_SDIO_CK) |              \
                                     PIN_PUPDR_FLOATING(GPIOC_SPI2_CS1) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))

#define VAL_GPIOC_ODR               (PIN_ODR_LOW(GPIOC_ADC1_IN10) |       \
                                     PIN_ODR_LOW(GPIOC_ETH_RMII_MDC) |       \
                                     PIN_ODR_LOW(GPIOC_ADC1_IN12) |       \
                                     PIN_ODR_LOW(GPIOC_ADC1_IN13) |       \
                                     PIN_ODR_LOW(GPIOC_ETH_RMII_RXD0) |       \
                                     PIN_ODR_LOW(GPIOC_ETH_RMII_RXD1) |       \
                                     PIN_ODR_LOW(GPIOC_USART6_TX) |             \
                                     PIN_ODR_LOW(GPIOC_USART6_RX) |             \
                                     PIN_ODR_LOW(GPIOC_SDIO_D0) |             \
                                     PIN_ODR_LOW(GPIOC_SDIO_D1) |             \
                                     PIN_ODR_LOW(GPIOC_SDIO_D2) |             \
                                     PIN_ODR_LOW(GPIOC_SDIO_D3) |            \
                                     PIN_ODR_LOW(GPIOC_SDIO_CK) |             \
                                     PIN_ODR_HIGH(GPIOC_SPI2_CS1) |            \
                                     PIN_ODR_LOW(GPIOC_OSC32_IN) |            \
                                     PIN_ODR_LOW(GPIOC_OSC32_OUT))

#define VAL_GPIOC_AFRL             (PIN_AFIO_AF(GPIOC_ADC1_IN10, 0) |           \
                                    PIN_AFIO_AF(GPIOC_ETH_RMII_MDC, 11) |           \
                                    PIN_AFIO_AF(GPIOC_ADC1_IN12, 0) |           \
                                    PIN_AFIO_AF(GPIOC_ADC1_IN13, 0) |           \
                                    PIN_AFIO_AF(GPIOC_ETH_RMII_RXD0, 11) |       \
                                    PIN_AFIO_AF(GPIOC_ETH_RMII_RXD1, 11) |       \
                                    PIN_AFIO_AF(GPIOC_USART6_TX, 8) |      \
                                    PIN_AFIO_AF(GPIOC_USART6_RX, 8))

#define VAL_GPIOC_AFRH             (PIN_AFIO_AF(GPIOC_SDIO_D0, 12) |       \
                                    PIN_AFIO_AF(GPIOC_SDIO_D1, 12) |       \
                                    PIN_AFIO_AF(GPIOC_SDIO_D2, 12) |       \
                                    PIN_AFIO_AF(GPIOC_SDIO_D3, 12) |       \
                                    PIN_AFIO_AF(GPIOC_SDIO_CK, 12) |       \
                                    PIN_AFIO_AF(GPIOC_SPI2_CS1, 0) |           \
                                    PIN_AFIO_AF(GPIOC_OSC32_IN, 0) |       \
                                    PIN_AFIO_AF(GPIOC_OSC32_OUT, 0))

/*
 * GPIOD setup:
 *
#define GPIOD_CAN1_RX               0
#define GPIOD_CAN1_TX               1
#define GPIOD_SDIO_CMD              2
#define GPIOD_IO_LED1               3
#define GPIOD_IO_LED2               4
#define GPIOD_USART2_TX             5
#define GPIOD_USART2_RX             6
#define GPIOD_SPI1_CS1_BARO         7
#define GPIOD_USART3_TX             8
#define GPIOD_USART3_RX             9
#define GPIOD_SPI2_CS0_FRAM         10	//FRAM CS
#define GPIOD_USART3_CTS            11
#define GPIOD_USART3_RTS            12
#define GPIOD_PWM4_CH2              13 //SERO_CH6
#define GPIOD_PWM4_CH3              14 //SERO_CH7
#define GPIOD_PWM4_CH4              15 //SERO_CH8
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ALTERNATE(GPIOD_CAN1_RX) |          \
                                     PIN_MODE_ALTERNATE(GPIOD_CAN1_TX) |           \
                                     PIN_MODE_ALTERNATE(GPIOD_SDIO_CMD) |           \
                                     PIN_MODE_OUTPUT(GPIOD_IO_LED1) |           \
                                     PIN_MODE_OUTPUT(GPIOD_IO_LED2) |       \
                                     PIN_MODE_ALTERNATE(GPIOD_USART2_TX) |       \
                                     PIN_MODE_ALTERNATE(GPIOD_USART2_RX) |      \
                                     PIN_MODE_OUTPUT(GPIOD_SPI1_CS1_BARO) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_USART3_TX) |   \
                                     PIN_MODE_ALTERNATE(GPIOD_USART3_RX) |   \
                                     PIN_MODE_OUTPUT(GPIOD_SPI2_CS0_FRAM) |   \
                                     PIN_MODE_ALTERNATE(GPIOD_USART3_CTS) |   \
                                     PIN_MODE_ALTERNATE(GPIOD_USART3_RTS) |   \
                                     PIN_MODE_ALTERNATE(GPIOD_PWM4_CH2) |         \
                                     PIN_MODE_ALTERNATE(GPIOD_PWM4_CH3) |      \
                                     PIN_MODE_ALTERNATE(GPIOD_PWM4_CH4))

#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_CAN1_RX) |          \
                                     PIN_OTYPE_PUSHPULL(GPIOD_CAN1_TX) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SDIO_CMD) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOD_IO_LED1) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOD_IO_LED2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART2_TX) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART2_RX) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SPI1_CS1_BARO) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART3_TX) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART3_RX) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SPI2_CS0_FRAM) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART3_CTS) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART3_RTS) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PWM4_CH2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PWM4_CH3) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PWM4_CH4))

#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_100M(GPIOD_CAN1_RX) |          \
                                     PIN_OSPEED_100M(GPIOD_CAN1_TX) |           \
                                     PIN_OSPEED_25M(GPIOD_SDIO_CMD) |           \
                                     PIN_OSPEED_100M(GPIOD_IO_LED1) |           \
                                     PIN_OSPEED_100M(GPIOD_IO_LED2) |       \
                                     PIN_OSPEED_100M(GPIOD_USART2_TX) |       \
                                     PIN_OSPEED_100M(GPIOD_USART2_RX) |      \
                                     PIN_OSPEED_100M(GPIOD_SPI1_CS1_BARO) |  \
                                     PIN_OSPEED_100M(GPIOD_USART3_TX) |   \
                                     PIN_OSPEED_100M(GPIOD_USART3_RX) |   \
                                     PIN_OSPEED_100M(GPIOD_SPI2_CS0_FRAM) |   \
                                     PIN_OSPEED_100M(GPIOD_USART3_CTS) |   \
                                     PIN_OSPEED_100M(GPIOD_USART3_RTS) |   \
                                     PIN_OSPEED_100M(GPIOD_PWM4_CH2) |         \
                                     PIN_OSPEED_100M(GPIOD_PWM4_CH3) |      \
                                     PIN_OSPEED_100M(GPIOD_PWM4_CH4))

#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_CAN1_RX) |          \
                                     PIN_PUPDR_FLOATING(GPIOD_CAN1_TX) |           \
                                     PIN_PUPDR_PULLUP(GPIOD_SDIO_CMD) |           \
                                     PIN_PUPDR_FLOATING(GPIOD_IO_LED1) |           \
                                     PIN_PUPDR_FLOATING(GPIOD_IO_LED2) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_USART2_TX) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_USART2_RX) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_SPI1_CS1_BARO) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_USART3_TX) |   \
                                     PIN_PUPDR_FLOATING(GPIOD_USART3_RX) |   \
                                     PIN_PUPDR_FLOATING(GPIOD_SPI2_CS0_FRAM) |   \
                                     PIN_PUPDR_FLOATING(GPIOD_USART3_CTS) |   \
                                     PIN_PUPDR_FLOATING(GPIOD_USART3_RTS) |   \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PWM4_CH2) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PWM4_CH3) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PWM4_CH4))

#define VAL_GPIOD_ODR               (PIN_ODR_LOW(GPIOD_CAN1_RX) |          \
                                     PIN_ODR_LOW(GPIOD_CAN1_TX) |           \
                                     PIN_ODR_LOW(GPIOD_SDIO_CMD) |           \
                                     PIN_ODR_LOW(GPIOD_IO_LED1) |           \
                                     PIN_ODR_LOW(GPIOD_IO_LED2) |       \
                                     PIN_ODR_LOW(GPIOD_USART2_TX) |       \
                                     PIN_ODR_LOW(GPIOD_USART2_RX) |      \
                                     PIN_ODR_HIGH(GPIOD_SPI1_CS1_BARO) |  \
                                     PIN_ODR_LOW(GPIOD_USART3_TX) |   \
                                     PIN_ODR_LOW(GPIOD_USART3_RX) |   \
                                     PIN_ODR_HIGH(GPIOD_SPI2_CS0_FRAM) |   \
                                     PIN_ODR_LOW(GPIOD_USART3_CTS) |   \
                                     PIN_ODR_LOW(GPIOD_USART3_RTS) |   \
                                     PIN_ODR_LOW(GPIOD_PWM4_CH2) |         \
                                     PIN_ODR_LOW(GPIOD_PWM4_CH3) |      \
                                     PIN_ODR_LOW(GPIOD_PWM4_CH4))

#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_CAN1_RX, 9) |          \
                                     PIN_AFIO_AF(GPIOD_CAN1_TX, 9) |           \
                                     PIN_AFIO_AF(GPIOD_SDIO_CMD, 12) |           \
                                     PIN_AFIO_AF(GPIOD_IO_LED1, 0) |           \
                                     PIN_AFIO_AF(GPIOD_IO_LED2, 0) |       \
                                     PIN_AFIO_AF(GPIOD_USART2_TX, 7) |       \
                                     PIN_AFIO_AF(GPIOD_USART2_RX, 7) |      \
                                     PIN_AFIO_AF(GPIOD_SPI1_CS1_BARO, 0))

#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_USART3_TX, 7) |   \
                                     PIN_AFIO_AF(GPIOD_USART3_RX, 7) |   \
                                     PIN_AFIO_AF(GPIOD_SPI2_CS0_FRAM, 0) |   \
                                     PIN_AFIO_AF(GPIOD_USART3_CTS, 7) |   \
                                     PIN_AFIO_AF(GPIOD_USART3_RTS, 7) |   \
                                     PIN_AFIO_AF(GPIOD_PWM4_CH2, 2) |         \
                                     PIN_AFIO_AF(GPIOD_PWM4_CH3, 2) |      \
                                     PIN_AFIO_AF(GPIOD_PWM4_CH4, 2))


/*
 * Port E setup.
#define GPIOE_INT_SPI1_IMU          0
#define GPIOE_INT_SPI2_RDY          1
#define GPIOE_PIN2                  2
#define GPIOE_PIN3                  3
#define GPIOE_PIN4                  4
#define GPIOE_PIN5                  5
#define GPIOE_PIN6                  6
#define GPIOE_INT_SPI1_MAG          7
#define GPIOE_PIN8                  8
#define GPIOE_PWM1_CH1              9 //SERO_CH1
#define GPIOE_PIN10                 10
#define GPIOE_PWM1_CH2              11 //SERO_CH2
#define GPIOE_PIN12                 12
#define GPIOE_PWM1_CH3              13 //SERO_CH3
#define GPIOE_PWM1_CH4              14 //SERO_CH4
#define GPIOE_INT_RES15             15
 */
/* 0x00000000 */
#define VAL_GPIOE_MODER        (PIN_MODE_INPUT(GPIOE_INT_SPI1_IMU)  |        \
                                PIN_MODE_INPUT(GPIOE_INT_SPI2_RDY) |        \
                                PIN_MODE_INPUT(GPIOE_PIN2)         |        \
                                PIN_MODE_INPUT(GPIOE_PIN3)          |        \
                                PIN_MODE_INPUT(GPIOE_PIN4)         |        \
                                PIN_MODE_INPUT(GPIOE_PIN5)         |        \
                                PIN_MODE_INPUT(GPIOE_PIN6)          |        \
                                PIN_MODE_INPUT(GPIOE_INT_SPI1_MAG)  |        \
                                PIN_MODE_INPUT(GPIOE_PIN8)          |        \
                                PIN_MODE_ALTERNATE(GPIOE_PWM1_CH1)  |        \
                                PIN_MODE_INPUT(GPIOE_PIN10)         |        \
                                PIN_MODE_ALTERNATE(GPIOE_PWM1_CH2)  |        \
                                PIN_MODE_INPUT(GPIOE_PIN12)         |        \
                                PIN_MODE_ALTERNATE(GPIOE_PWM1_CH3)  |        \
                                PIN_MODE_ALTERNATE(GPIOE_PWM1_CH4)  |        \
                                PIN_MODE_INPUT(GPIOE_INT_RES15))
/* 0x00000000 */
#define VAL_GPIOE_OTYPER       (PIN_OTYPE_PUSHPULL(GPIOE_INT_SPI1_IMU)  |        \
                                PIN_OTYPE_PUSHPULL(GPIOE_INT_SPI2_RDY) |        \
                                PIN_OTYPE_PUSHPULL(GPIOE_PIN2)         |        \
                                PIN_OTYPE_PUSHPULL(GPIOE_PIN3)          |        \
                                PIN_OTYPE_PUSHPULL(GPIOE_PIN4)         |        \
                                PIN_OTYPE_PUSHPULL(GPIOE_PIN5)         |        \
                                PIN_OTYPE_PUSHPULL(GPIOE_PIN6)          |        \
                                PIN_OTYPE_PUSHPULL(GPIOE_INT_SPI1_MAG)  |        \
                                PIN_OTYPE_PUSHPULL(GPIOE_PIN8)          |        \
                                PIN_OTYPE_PUSHPULL(GPIOE_PWM1_CH1)  |        \
                                PIN_OTYPE_PUSHPULL(GPIOE_PIN10)         |        \
                                PIN_OTYPE_PUSHPULL(GPIOE_PWM1_CH2)  |        \
                                PIN_OTYPE_PUSHPULL(GPIOE_PIN12)         |        \
                                PIN_OTYPE_PUSHPULL(GPIOE_PWM1_CH3)  |        \
                                PIN_OTYPE_PUSHPULL(GPIOE_PWM1_CH4)  |        \
                                PIN_OTYPE_PUSHPULL(GPIOE_INT_RES15))
/* 0x00000000 */
#define VAL_GPIOE_OSPEEDR       0x00000000
/* 0x00000000 */
#define VAL_GPIOE_PUPDR        (PIN_PUPDR_PULLDOWN(GPIOE_INT_SPI1_IMU)  |        \
                                PIN_PUPDR_PULLDOWN(GPIOE_INT_SPI2_RDY) |        \
                                PIN_PUPDR_PULLDOWN(GPIOE_PIN2)         |        \
                                PIN_PUPDR_PULLDOWN(GPIOE_PIN3)          |        \
                                PIN_PUPDR_PULLDOWN(GPIOE_PIN4)         |        \
                                PIN_PUPDR_PULLDOWN(GPIOE_PIN5)         |        \
                                PIN_PUPDR_PULLDOWN(GPIOE_PIN6)          |        \
                                PIN_PUPDR_PULLDOWN(GPIOE_INT_SPI1_MAG)  |        \
                                PIN_PUPDR_PULLDOWN(GPIOE_PIN8)          |        \
                                PIN_PUPDR_PULLDOWN(GPIOE_PWM1_CH1)  |        \
                                PIN_PUPDR_PULLDOWN(GPIOE_PIN10)         |        \
                                PIN_PUPDR_PULLDOWN(GPIOE_PWM1_CH2)  |        \
                                PIN_PUPDR_PULLDOWN(GPIOE_PIN12)         |        \
                                PIN_PUPDR_PULLDOWN(GPIOE_PWM1_CH3)  |        \
                                PIN_PUPDR_PULLDOWN(GPIOE_PWM1_CH4)  |        \
                                PIN_PUPDR_PULLDOWN(GPIOE_INT_RES15))
/* 0x00000000 */
#define VAL_GPIOE_ODR           0x00000000
/* 0x00000000 */
#define VAL_GPIOE_AFRL          0x00000000
/* 0x00000000 */
#define VAL_GPIOE_AFRH         (PIN_AFIO_AF(GPIOE_PWM1_CH1, 1) |       \
                                PIN_AFIO_AF(GPIOE_PWM1_CH2, 1) |       \
                                PIN_AFIO_AF(GPIOE_PWM1_CH3, 1) |       \
                                PIN_AFIO_AF(GPIOE_PWM1_CH4, 1))


/*
 * Port F setup.
 */
#define VAL_GPIOF_MODER             0x00000000
#define VAL_GPIOF_OTYPER            0x00000000
#define VAL_GPIOF_OSPEEDR           0x00000000
#define VAL_GPIOF_PUPDR             0x00000000
#define VAL_GPIOF_ODR               0x00000000
#define VAL_GPIOF_AFRL              0x00000000
#define VAL_GPIOF_AFRH              0x00000000

/*
 * Port G setup.
 */
#define VAL_GPIOG_MODER             0x00000000
#define VAL_GPIOG_OTYPER            0x00000000
#define VAL_GPIOG_OSPEEDR           0x00000000
#define VAL_GPIOG_PUPDR             0x00000000
#define VAL_GPIOG_ODR               0x00000000
#define VAL_GPIOG_AFRL              0x00000000
#define VAL_GPIOG_AFRH              0x00000000

/*
 * Port H setup.
 */
#define VAL_GPIOH_MODER             0x00000000
#define VAL_GPIOH_OTYPER            0x00000000
#define VAL_GPIOH_OSPEEDR           0x00000000
#define VAL_GPIOH_PUPDR             0x00000000
#define VAL_GPIOH_ODR               0x00000000
#define VAL_GPIOH_AFRL              0x00000000
#define VAL_GPIOH_AFRH              0x00000000

/*
 * Port I setup.
 */
#define VAL_GPIOI_MODER             0x00000000
#define VAL_GPIOI_OTYPER            0x00000000
#define VAL_GPIOI_OSPEEDR           0x00000000
#define VAL_GPIOI_PUPDR             0x00000000
#define VAL_GPIOI_ODR               0x00000000
#define VAL_GPIOI_AFRL              0x00000000
#define VAL_GPIOI_AFRH              0x00000000



#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
