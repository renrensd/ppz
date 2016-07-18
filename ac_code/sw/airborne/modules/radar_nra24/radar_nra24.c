/* it under the terms of the GNU General Public License as published by
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

#include "modules/radar_nra24/radar_nra24.h"
#include "generated/airframe.h"
//#include "mcu_periph/adc.h"
#include "subsystems/abi.h"
#include <math.h>
//#include "subsystems/abi_sender_ids.h"
#ifdef SITL
#include "state.h"
#endif

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif
#include"modules/sonar/agl_dist.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "std.h"
#include "modules/laser/laser_r2100.h"
#define radar_nra24_Dev  (uart3.device)
#define radar_nra24_Transmit(c) radar_nra24_Dev.put_byte(radar_nra24_Dev.periph, c)
#define radar_nra24_ChAvailable() radar_nra24_Dev.char_available(radar_nra24_Dev.periph)
#define radar_nra24_Getch() radar_nra24_Dev.get_byte(radar_nra24_Dev.periph)


#define RADAR_NRA24_STX 	 0xAA
#define RADAR_NRA24_SAT1  0x0C
#define RADAR_NRA24_SAT2  0x07
#define RADAR_NRA24_END     0x55

#define RADAR_NRA24_STAE0   0
#define RADAR_NRA24_STAE1    1
#define RADAR_NRA24_STAE2    2
#define RADAR_NRA24_STAE3    3
#define RADAR_NRA24_STAE4    4
#define RADAR_NRA24_STAE5    5
#define RADAR_NRA24_STAE6    6

struct RADAR_NRA24  radar_nra24;
struct RADAR_NRA24_DATA radar_nra24_data;

static inline void parse_rada_nra24r(struct RADAR_NRA24 *t, uint8_t c)
{
	  switch (t->status) 
	  {
	    	case RADAR_NRA24_STAE0:
	     	 	if (c == RADAR_NRA24_STX) 
			 {
				t->status++;
	     		 }
	     	 break;
	    	case RADAR_NRA24_STAE1:
			 if (c == RADAR_NRA24_STX) 
		 	{
	        			t->status++;
	      		 }
		 	 else
		  	{
				t->status = RADAR_NRA24_STAE0;
		  	}
	      	break;
		case RADAR_NRA24_STAE2:
		 	 if (c == RADAR_NRA24_SAT1) 
		  	{
	       			 t->status++;
	     		 }
		  	else
		  	{
				t->status = RADAR_NRA24_STAE0;
		 	 }
	     	 break;
	    	case RADAR_NRA24_STAE3:
		  	if (c == RADAR_NRA24_SAT2) 
		  	{
	        		t->status++;
				t->payload_idx = 0;
	     	 	}
		 	 else
		  	{
				t->status = RADAR_NRA24_STAE0;
		  	}
	      	break;
		case RADAR_NRA24_STAE4:	
	      		t->payload[t->payload_idx] = c;
	      		t->payload_idx++;
	     		 if (t->payload_idx == RADAR_NRA24_PAYLOAD_LEN) 
		  	{
	       			 t->status++;
	      		}
	      	break;
	   	case RADAR_NRA24_STAE5:
			if (c == RADAR_NRA24_END) 
		  	{
	        			t->status++;
	     	 	}
		 	 else
		  	{
				t->status = RADAR_NRA24_STAE0;
		  	}
			break;
	    case RADAR_NRA24_STAE6:
			if (c == RADAR_NRA24_END) 
		  	{
	        			 t->msg_received = TRUE; 
	     	 	}
		 	t->status = RADAR_NRA24_STAE0;
	    	default:
	      		t->status = RADAR_NRA24_STAE0;
	  }
}
//extern struct LASER_R2100_DATA laser_data;
void radar_nra24_get_data(void)
{  
	//uint8_t i;
	static float radar_data_buff=0.3;
	float data_rate;
	if ( radar_nra24_ChAvailable() )
	{
		while ( !radar_nra24.msg_received && radar_nra24_ChAvailable()  ) 
		{
	      		parse_rada_nra24r( &radar_nra24, radar_nra24_Getch() );
	    	}
	    	if (radar_nra24.msg_received)
		{
			radar_nra24.msg_received = FALSE;
			radar_nra24_data.dis=radar_nra24.payload[2]*256+radar_nra24.payload[3];
			radar_nra24_data.dis_m=((float)radar_nra24_data.dis)*0.01+0.15;
		       #if 0
			data_rate=radar_nra24_data.dis_m-radar_data_buff;
			if(fabs(data_rate)<0.5)
				radar_data_buff=radar_nra24_data.dis_m;
			else
				radar_nra24_data.dis_m=radar_data_buff;
		       #endif
			AbiSendMsgRADAR_24(AGL_NRA_24_ID,radar_nra24_data.dis_m);
			#if PERIODIC_TELEMETRY
			    RunOnceEvery(10,   {
			    xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
			    DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &radar_nra24_data.dis,&radar_nra24_data.dis_m);}    );
			#endif
	   	 }
	}
}

