
#include "motor_info.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/link_device.h"


struct UART_MOTOR_TYPE motor_info;
void motor_parse_char(uint8_t c)
{
	switch(motor_info.rx_status)
	{
	case RX_IDLE:
	{
		if(c == START_BYTE1)
		{
			motor_info.rx_status = RX_START;
			motor_info.cs = START_BYTE1;
		}
		break;
	}
	case RX_START:
	{
		if(c == START_BYTE2)
		{
			motor_info.rx_status = RX_DATA;
			motor_info.cs += START_BYTE2;
		}
		break;
	}

	case RX_DATA:
	{
		motor_info.data[motor_info.rx_index ++] = c;
		motor_info.cs += c;
		if(motor_info.rx_index == 12)
		{
			motor_info.rx_status = RX_CS;
		}

		break;
	}
	case RX_CS:
	{
		if(motor_info.cs == c)
		{
			motor_info.rx_available = TRUE;
		}
		else
		{
			motor_info.rx_available = FALSE;
			motor_info.rx_index = 0;
		}
		motor_info.rx_status = RX_IDLE;
		break;
	}
	}
}
void read_motor_info(void)
{
	struct link_device *dev = &((RADAR_DEVICE).device);
	while (dev->char_available(dev->periph))
	{
		motor_parse_char(dev->get_byte(dev->periph));
		if( motor_info.rx_available )
		{
			for(uint8_t i=0; i<6; i++)
			{
				motor_info.speed[i] = motor_info.data[i*2] + motor_info.data[i*2+1] * 256;
			}
			motor_info.rx_available = FALSE;
		}
	}
}
void motor_info_init(void)
{
	motor_info.rx_available = FALSE;
}
