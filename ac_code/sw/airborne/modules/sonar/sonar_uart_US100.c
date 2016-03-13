/*
 * Copyright (C) 2010  Gautier Hattenberger, 2013 Tobias Münch
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
 *
 */

#include "modules/sonar/sonar_uart_US100.h"
#include "generated/airframe.h"
//#include "mcu_periph/adc.h"
#include "subsystems/abi.h"
//#include "subsystems/abi_sender_ids.h"
#ifdef SITL
#include "state.h"
#endif
#include"modules/sonar/agl_dist.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "std.h"

#define SonarLinkDev  (uart2.device)
#define SonarLinkTransmit(c) SonarLinkDev.put_byte(SonarLinkDev.periph, c)
#define SonarLinkChAvailable() SonarLinkDev.char_available(SonarLinkDev.periph)
#define SonarLinkGetch() SonarLinkDev.get_byte(SonarLinkDev.periph)

struct sonar_data Sonar_US100;

void sonar_get_distance(void)
{ static bool exchange=true;
  if(exchange)
  { sonar_uart_send_order(); }
  else
  { sonar_uart_read(); }
  exchange=!exchange;  
}

void sonar_uart_read(void)
{  Sonar_US100.distance_mm=0;
   uint8_t n_bytes=0;
   Sonar_US100.distance_m=0.0;

 while(SonarLinkChAvailable())
 { if(n_bytes<2)
 	{  Sonar_US100.distance_mm|=( SonarLinkGetch()<<(8*(1-n_bytes)) );
       n_bytes++;
    }
   else 
   	{  SonarLinkGetch();
	   n_bytes++;
    }
 }
 
 if(n_bytes!=2) return;
   Sonar_US100.distance_m=(float)Sonar_US100.distance_mm/1000.0;
	 
	  
 if(Sonar_US100.distance_m<0.05||Sonar_US100.distance_m>7)
   {   //request above 0.1m
     return;
   }
 
  // Send ABI message
  AbiSendMsgAGL(AGL_SONAR_ADC_ID, Sonar_US100.distance_m);
  //AbiSendMsgBARO_ABS(BARO_BOARD_SENDER_ID, Sonar_US100.distance_m);

#ifdef SENSOR_SYNC_SEND_SONAR   //Send Telemetry report  
     xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
     DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &Sonar_US100.distance_mm, &agl_dist_value_filtered);//&Sonar_US100.distance_m);
#endif 
}


void sonar_uart_send_order(void)
{
  SonarLinkTransmit(0x55); //send getdata oder(0x55) to sonar
    
}


