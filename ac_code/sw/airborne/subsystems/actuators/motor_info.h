#ifndef MOTOR_INFO_H
#define MOTOR_INFO_H

#include "mcu_periph/uart.h"
#define START_BYTE1		0xAA
#define START_BYTE2 	0XAA
#define MAX_DATA_NUM 	11;	//2*6-1
enum RX_STATUS
{
	RX_IDLE = 0,
	RX_START,
	RX_DATA,
	RX_CS
};

struct UART_MOTOR_TYPE
{
	uint8_t cs;
	uint8_t data[12];
	uint8_t rx_index;
	bool_t	rx_available;
	enum RX_STATUS rx_status;
	uint16_t speed[6];
};

extern struct UART_MOTOR_TYPE motor_info;
extern void read_motor_info(void);
extern void motor_info_init(void);
#endif
