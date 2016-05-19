#ifndef CONFIG_KROOZ_SD_H
#define CONFIG_KROOZ_SD_H

#define BOARD_KROOZ

/* KroozSD has a 25MHz external clock and 168MHz internal. */
#define EXT_CLK 24000000
#define AHB_CLK 168000000

/*
 * Onboard LEDs
 */

/* red, on PD11 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOD
#define LED_1_GPIO_PIN GPIO11
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* green, shared with JTAG_TRST */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOD
#define LED_2_GPIO_PIN GPIO12
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)

/*
 * not actual LEDS, used as GPIOs
 */
#if 0
/* PB4, Camera power On/Off */
#define CAM_SW_GPIO GPIOB
#define CAM_SW_GPIO_CLK RCC_GPIOB
#define CAM_SW_GPIO_PIN GPIO4
#define CAM_SW_AFIO_REMAP ((void)0)

/* PC2, Camera shot */
#define CAM_SH_GPIO GPIOC
#define CAM_SH_GPIO_CLK RCC_GPIOC
#define CAM_SH_GPIO_PIN GPIO2
#define CAM_SH_AFIO_REMAP ((void)0)

/* PC15, Camera video */
#define CAM_V_GPIO GPIOC
#define CAM_V_GPIO_CLK RCC_GPIOC
#define CAM_V_GPIO_PIN GPIO15
#define CAM_V_AFIO_REMAP ((void)0)

#define BEEPER_GPIO GPIOC
#define BEEPER_GPIO_CLK RCC_GPIOC
#define BEEPER_GPIO_PIN GPIO14
#define BEEPER_AFIO_REMAP ((void)0)
#endif

/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

//#define DefaultVoltageOfAdc(adc) (0.0088623*adc)
#define DefaultVoltageOfAdc(adc) (0.008862*2600)


/* UART */
//sonar
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_RX GPIOB
#define UART1_GPIO_RX GPIO7
//#define UART1_GPIO_PORT_TX GPIOB
//#define UART1_GPIO_TX GPIO6

#define UART2_GPIO_AF GPIO_AF7
#define UART2_GPIO_PORT_RX GPIOD
#define UART2_GPIO_RX GPIO6
#define UART2_GPIO_PORT_TX GPIOD
#define UART2_GPIO_TX GPIO5

#define UART3_GPIO_AF GPIO_AF7
#define UART3_GPIO_PORT_RX GPIOD
#define UART3_GPIO_RX GPIO9
#define UART3_GPIO_PORT_TX GPIOD
#define UART3_GPIO_TX GPIO8

#define UART4_GPIO_AF GPIO_AF8
#define UART4_GPIO_PORT_RX GPIOA
#define UART4_GPIO_RX GPIO1
#define UART4_GPIO_PORT_TX GPIOA
#define UART4_GPIO_TX GPIO0

#define UART6_GPIO_AF GPIO_AF8
#define UART6_GPIO_PORT_RX GPIOC
#define UART6_GPIO_RX GPIO7
#define UART6_GPIO_PORT_TX GPIOC
#define UART6_GPIO_TX GPIO6

/* SPI */
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOB
#define SPI1_GPIO_MOSI GPIO5
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5

#define SPI2_GPIO_AF GPIO_AF5
#define SPI2_GPIO_PORT_MISO GPIOB
#define SPI2_GPIO_MISO GPIO14
#define SPI2_GPIO_PORT_MOSI GPIOB
#define SPI2_GPIO_MOSI GPIO15
#define SPI2_GPIO_PORT_SCK GPIOB
#define SPI2_GPIO_SCK GPIO10

#define SPI_SELECT_SLAVE0_PORT GPIOC	//adiv1: gyro_xy_cs  SPI2 set toPC13; SPI1 set to PB7
#define SPI_SELECT_SLAVE0_PIN GPIO13
#define SPI_SELECT_SLAVE1_PORT GPIOD	//ms5611_cs //V1-PE1;V2-PD7
#define SPI_SELECT_SLAVE1_PIN GPIO7
#define SPI_SELECT_SLAVE2_PORT GPIOE	//gyro_z_cs
#define SPI_SELECT_SLAVE2_PIN GPIO4
#define SPI_SELECT_SLAVE3_PORT GPIOE	//adxl345_cs
#define SPI_SELECT_SLAVE3_PIN GPIO5

/* I2C mapping */
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO8
#define I2C1_GPIO_SDA GPIO9

//#define I2C2_GPIO_PORT GPIOB
//#define I2C2_GPIO_SCL GPIO10
//#define I2C2_GPIO_SDA GPIO11

#define I2C3_GPIO_PORT_SCL GPIOA
#define I2C3_GPIO_PORT_SDA GPIOC
#define I2C3_GPIO_SCL GPIO8
#define I2C3_GPIO_SDA GPIO9

/*mb1242 i2c */
#define I2C_SONAR_GPIO_SDA_PORT GPIOD
#define I2C_SONAR_GPIO_SCL_PORT GPIOD
#define I2C_SONAR_GPIO_SDA GPIO0
#define I2C_SONAR_GPIO_SCL GPIO1


/* Onboard ADCs */
#define USE_AD_TIM5 1

/* provide defines that can be used to access the ADC_x in the code or airframe file
 * these directly map to the index number of the 4 adc channels defined above
 * 4th (index 3) is used for bat monitoring by default
 */
/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_1
#endif

#define ADC_CHANNEL_CAM1    ADC_2

/* provide defines that can be used to access the ADC_x in the code or airframe file
 * these directly map to the index number of the 4 adc channels defined above
 * 4th (index 3) is used for bat monitoring by default
 */
 // Internal ADC for battery enabled by default
#ifndef USE_ADC_1
#define USE_ADC_1 1
#endif
#if USE_ADC_1
#define AD1_1_CHANNEL 3
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOA
#define ADC_1_GPIO_PIN GPIO3
#endif

#if USE_ADC_2
#define AD1_2_CHANNEL 4
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOA
#define ADC_2_GPIO_PIN GPIO4
#endif

#if USE_ADC_3
#define AD1_3_CHANNEL 8
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOB
#define ADC_3_GPIO_PIN GPIO0
#endif

#if USE_ADC_4
#define AD1_4_CHANNEL 9
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOB
#define ADC_4_GPIO_PIN GPIO1
#endif

#if USE_ADC_5
#define AD1_4_CHANNEL 10
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOC
#define ADC_4_GPIO_PIN GPIO0
#endif

#if USE_ADC_6
#define AD1_4_CHANNEL 12
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOC
#define ADC_4_GPIO_PIN GPIO2
#endif

#if USE_ADC_7
#define AD1_4_CHANNEL 13
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOC
#define ADC_4_GPIO_PIN GPIO3
#endif


/* by default activate onboard baro */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif


/* PWM */
#define PWM_USE_TIM1 1
#define PWM_USE_TIM4 1
//#define PWM_USE_TIM5 1

#define USE_PWM0 1
#define USE_PWM1 1
#define USE_PWM2 1
#define USE_PWM3 1
#define USE_PWM4 1
#define USE_PWM5 1
#define USE_PWM6 0
#define USE_PWM7 0

#define ACTUATORS_PWM_NB 8

// PWM_SERVO_x is the index of the servo in the actuators_pwm_values array
#if USE_PWM0
#define PWM_SERVO_0 0
#define PWM_SERVO_0_TIMER TIM1
#define PWM_SERVO_0_GPIO GPIOE
#define PWM_SERVO_0_PIN GPIO9
#define PWM_SERVO_0_AF GPIO_AF1
#define PWM_SERVO_0_OC TIM_OC1
#define PWM_SERVO_0_OC_BIT (1<<0)
#else
#define PWM_SERVO_0_OC_BIT 0
#endif

#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_TIMER TIM1
#define PWM_SERVO_1_GPIO GPIOE
#define PWM_SERVO_1_PIN GPIO11
#define PWM_SERVO_1_AF GPIO_AF1
#define PWM_SERVO_1_OC TIM_OC2
#define PWM_SERVO_1_OC_BIT (1<<1)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_TIMER TIM1
#define PWM_SERVO_2_GPIO GPIOE
#define PWM_SERVO_2_PIN GPIO13
#define PWM_SERVO_2_AF GPIO_AF1
#define PWM_SERVO_2_OC TIM_OC3
#define PWM_SERVO_2_OC_BIT (1<<2)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if 0
#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_TIMER TIM4
#define PWM_SERVO_2_GPIO GPIOD
#define PWM_SERVO_2_PIN GPIO14
#define PWM_SERVO_2_AF GPIO_AF2
#define PWM_SERVO_2_OC TIM_OC3
#define PWM_SERVO_2_OC_BIT (1<<2)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif
#endif

#if USE_PWM3
#define PWM_SERVO_3 3
#define PWM_SERVO_3_TIMER TIM1
#define PWM_SERVO_3_GPIO GPIOE
#define PWM_SERVO_3_PIN GPIO14
#define PWM_SERVO_3_AF GPIO_AF1
#define PWM_SERVO_3_OC TIM_OC4
#define PWM_SERVO_3_OC_BIT (1<<3)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#if USE_PWM4
#define PWM_SERVO_4 4
#define PWM_SERVO_4_TIMER TIM4
#define PWM_SERVO_4_GPIO GPIOB
#define PWM_SERVO_4_PIN GPIO6
#define PWM_SERVO_4_AF GPIO_AF2
#define PWM_SERVO_4_OC TIM_OC1
#define PWM_SERVO_4_OC_BIT (1<<0)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#if USE_PWM5
#define PWM_SERVO_5 5
#define PWM_SERVO_5_TIMER TIM4
#define PWM_SERVO_5_GPIO GPIOD
#define PWM_SERVO_5_PIN GPIO13
#define PWM_SERVO_5_AF GPIO_AF2
#define PWM_SERVO_5_OC TIM_OC2
#define PWM_SERVO_5_OC_BIT (1<<1)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif

#if USE_PWM6
#define PWM_SERVO_6 6
#define PWM_SERVO_6_TIMER TIM4
#define PWM_SERVO_6_GPIO GPIOD
#define PWM_SERVO_6_PIN GPIO14
#define PWM_SERVO_6_AF GPIO_AF2
#define PWM_SERVO_6_OC TIM_OC3
#define PWM_SERVO_6_OC_BIT (1<<2)
#else
#define PWM_SERVO_6_OC_BIT 0
#endif

#if USE_PWM7
#define PWM_SERVO_7 7
#define PWM_SERVO_7_TIMER TIM4
#define PWM_SERVO_7_GPIO GPIOD
#define PWM_SERVO_7_PIN GPIO15
#define PWM_SERVO_7_AF GPIO_AF2
#define PWM_SERVO_7_OC TIM_OC4
#define PWM_SERVO_7_OC_BIT (1<<3)
#else
#define PWM_SERVO_7_OC_BIT 0
#endif

#define PWM_TIM1_CHAN_MASK (PWM_SERVO_0_OC_BIT|PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT|PWM_SERVO_3_OC_BIT)
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_4_OC_BIT|PWM_SERVO_5_OC_BIT|PWM_SERVO_6_OC_BIT|PWM_SERVO_7_OC_BIT)

/* PPM */
#define USE_PPM_TIM2 1

#define PPM_CHANNEL         TIM_IC1
#define PPM_TIMER_INPUT     TIM_IC_IN_TI1
#define PPM_IRQ             NVIC_TIM2_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC1IE
#define PPM_CC_IF           TIM_SR_CC1IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO15
#define PPM_GPIO_AF         GPIO_AF1

/*
 * Spektrum
 */
/* The line that is pulled low at power up to initiate the bind process */
#define SPEKTRUM_BIND_PIN GPIO8
#define SPEKTRUM_BIND_PIN_PORT GPIOE

#define ADXRS290_SENS
#ifdef ADXRS290_SENS
//adxrs290 xy int pin
#define ADXRS_XY_EOC_PIN GPIO0
#define ADXRS_XY_EOC_PIN_PORT GPIOE
#define ADXRS_Z_EOC_PIN GPIO7
#define ADXRS_Z_EOC_PIN_PORT GPIOE
#endif //ADXRS290_SENS


/* bat manager option */
#define BAT_MANAGER_OPTION

/* ops system option*/
#define OPS_OPTION
#define OPS_PERIODIC_FREQUENCY 100

/*monitoring system option*/
#define MONITORING_OPTION
#define MONITORING_FREQUENCY 2

/* bat manager option */
#define BAT_MANAGER_OPTION
#define I2C_BM_GPIO_SCL_PORT GPIOA
#define I2C_BM_GPIO_SDA_PORT GPIOC
#define I2C_BM_GPIO_SCL GPIO2
#define I2C_BM_GPIO_SDA GPIO1

#define HMC5983_OPTION

/* actuator power control */
#define DEBUG_GPIO 	GPIOE,GPIO2 //0-enable,1-disable.

#endif /* CONFIG_KROOZ_SD_H */
