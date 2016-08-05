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


#define RADAR_NRA24_STX   0xAA
#define RADAR_NRA24_SAT1  0x0C
#define RADAR_NRA24_SAT2  0x07
#define RADAR_NRA24_END   0x55

#define RADAR_NRA24_STAE0    0
#define RADAR_NRA24_STAE1    1
#define RADAR_NRA24_STAE2    2
#define RADAR_NRA24_STAE3    3
#define RADAR_NRA24_STAE4    4
#define RADAR_NRA24_STAE5    5
#define RADAR_NRA24_STAE6    6
#define RADINTDIS 20
#define RADDISSPEED 10

uint16_t radar_intdistance = RADINTDIS; 
uint16_t radar_distance_speed = RADDISSPEED;
static uint16_t radar_disbuf = RADINTDIS;

struct RADAR_NRA24  radar_nra24;
struct RADAR_NRA24_DATA radar_nra24_data;

float  RADAR24_DILUT_SPEED_R;     //dilution of speed
float  RADAR24_DILUT_fILTER_H;    //dilution of filter
float  RADAR24_STER_LENGTH;     //tracing step length
float Radar24_G;
int8_t model_change;

#define Sign(_x) ((_x) > 0 ? 1 : (-1))

float RADAR24_x1 = 0.3;
float RADAR24_x2 = 0.3;
float CUT_OFF;


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


static void Differential_equa(float signal,float *x1,float *x2)
{
	float Detar0,A0,Y,A1,A2,A,FH,f;
	Detar0=RADAR24_DILUT_SPEED_R*RADAR24_DILUT_fILTER_H*RADAR24_DILUT_fILTER_H;
	A0=RADAR24_DILUT_fILTER_H*(*x2);
	Y=(*x1-signal)+A0;
	A1=sqrt(Detar0*(Detar0+8*fabs(Y)));
	A2=A0+Sign(Y)*(A1-Detar0)/2;
	f=(Sign(Y+Detar0)-Sign(Y-Detar0))/2;
	A=(A0+Y)*f+A2*(1-f);
	f=(Sign(A+Detar0)-Sign(A-Detar0))/2;
	FH=-RADAR24_DILUT_SPEED_R*(A/Detar0)*f-RADAR24_DILUT_SPEED_R*Sign(A)*(1-f);
	*x2=*x2+RADAR24_STER_LENGTH*FH;
	*x1=*x1+RADAR24_STER_LENGTH* (*x2);
}


unsigned char FILTER_NUM;  //
uint16_t Radar_Smooth_MA(uint16_t radar_in)  //
{
	static uint8_t 	filter_cnt=0;
	static uint16_t	radar_BUF[50];
	uint32_t temp1=0;
	uint8_t i;
	radar_BUF[filter_cnt] = radar_in;
	uint16_t radar_out;
	for(i=0;i<FILTER_NUM;i++)
	{
		temp1 += radar_BUF[i];
	}
	radar_out= temp1 / FILTER_NUM;
	filter_cnt++;
	if(filter_cnt==FILTER_NUM)	filter_cnt=0;
	return radar_out;
}




//-----Butterworth-----//

static float b_alt[3];
static float a_alt[3];

#define M_PI_F 3.1415926


void LPF2pSetCutoffFreq(float sample_freq, float cutoff_freq) //
{
    float fr =0;  
    float ohm =0;
    float c =0;
	
    fr= sample_freq/cutoff_freq;
    ohm=tanf(M_PI_F/fr);
    c=1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;

    b_alt[0] = ohm*ohm/c;
    b_alt[1] = 2.0f*b_alt[0];
    b_alt[2] = b_alt[0];
	a_alt[0] =0.0;
    a_alt[1] = 2.0f*(ohm*ohm-1.0f)/c;
    a_alt[2] = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;

}

static float     _delay_element_11;        // buffered sample -1
static float     _delay_element_21;        // buffered sample -2

float LPF2pApply(float sample)  //
{
     float delay_element_0 = 0, output=0;

     delay_element_0 = sample - _delay_element_11 * a_alt[1] - _delay_element_21 * a_alt[2];
				
     //if (isnan(delay_element_0) || isinf(delay_element_0)) {
						
	//delay_element_0 = sample;
	//}
     output = delay_element_0 * b_alt[0] + _delay_element_11 * b_alt[1] + _delay_element_21 * b_alt[2];
				
     _delay_element_21 = _delay_element_11;
     _delay_element_11 = delay_element_0;
     return output;
}




void radar_nra24_get_data(void)
{  
	static uint16_t i=1;
	float data_rate,data_meas;
	if ( radar_nra24_ChAvailable() )
	{
		while ( !radar_nra24.msg_received && radar_nra24_ChAvailable()  ) 
		{
	      	parse_rada_nra24r( &radar_nra24, radar_nra24_Getch() );
	    }
		
	    if (radar_nra24.msg_received)
		{
			radar_nra24.msg_received = FALSE;
			radar_nra24_data.dis = (radar_nra24.payload[2]<<8) | radar_nra24.payload[3]; //unit:cm
            data_meas = radar_nra24_data.dis*0.01 + Radar24_G; //unit:m
			
			if (radar_nra24_data.dis<=10||radar_nra24_data.dis>=2000)
			{
				radar_nra24_data.dis = radar_disbuf;
				i++;
			}
			else if (abs(radar_nra24_data.dis-radar_disbuf)>=i*radar_distance_speed)
			{
				radar_nra24_data.dis=radar_disbuf;
				i++;
			}
			else 
			{
				radar_disbuf=radar_nra24_data.dis;
				i=1;
			}
			
			if(model_change==RADAR_DIFF)
			{
				Differential_equa((float)radar_nra24_data.dis*0.01,&RADAR24_x1,&RADAR24_x2);//m
				radar_nra24_data.dis_m=RADAR24_x1+Radar24_G;//m
			}
            else if(model_change==RADAR_SMOOTH_MA)
			{
				radar_nra24_data.dis_m=(float)Radar_Smooth_MA(radar_nra24_data.dis)*0.01+Radar24_G;
            }
			
			else if(model_change==RADAR_BUTTERWORTH)
			{
				//LPF2pSetCutoffFreq(50.0f,CUT_OFF);   //move to initial for CPU efficience
				radar_nra24_data.dis_m = LPF2pApply((float)radar_nra24_data.dis)*0.01 + Radar24_G;
	    	}
			else
			{
				 radar_nra24_data.dis_m=((float)radar_nra24_data.dis)*0.01+Radar24_G;
			}
			//radar_nra24_data.dis_m=((float)radar_nra24_data.dis)*0.01+Radar24_G;
			AbiSendMsgRADAR_24(AGL_NRA_24_ID,radar_nra24_data.dis_m);
			//data_meas=radar_nra24_data.dis*0.01+Radar24_G;//m
			#if PERIODIC_TELEMETRY
		    RunOnceEvery(10,   {
		    xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
		    DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &radar_nra24_data.dis, &radar_nra24_data.dis_m);}    );
			#endif
	   	 }
	}
}
void Init_radar24(void)
{	
	RADAR24_DILUT_SPEED_R=RADAR24_SPEED_R;	 //dilution of speed
	RADAR24_DILUT_fILTER_H=RADAR24_fILTER_H;	  //dilution of filter
	RADAR24_STER_LENGTH=RADAR24_LENGTH; 	//tracing step length
	Radar24_G=RADAR24_G;
	CUT_OFF=CUT_OFF_PV;
	model_change = RADAR_BUTTERWORTH;
	if(model_change == RADAR_BUTTERWORTH)
	{
		LPF2pSetCutoffFreq(50.0f,CUT_OFF);
	}
	FILTER_NUM=NUM_FIL;
}

