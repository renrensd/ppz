/** \file baro_scp_i2c.c
 *  \brief VTI SCP1000 I2C sensor interface
 *
 *   This reads the values for pressure and temperature from the VTI SCP1000 sensor through I2C.
 */


#include "modules/optical_flow/px4_flow_i2c.h"
#include "mcu_periph/sys_time.h"

#include "subsystems/abi.h"
#include "messages.h"

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_px4flow(struct transport_tx *trans, struct link_device *dev)
{
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	pprz_msg_send_PX4FLOW(trans, dev, AC_ID,
												&px4_flow.data.frame_count,
												&px4_flow.data.sum_x,
												&px4_flow.data.sum_y,
												&px4_flow.data.flow_comp_m_x,
												&px4_flow.data.flow_comp_m_y,
												&px4_flow.data.qual,
												&px4_flow.data.sonar_timestamp,
												&px4_flow.data.ground_distance);
}
#endif

#ifndef USE_FLOW
#define USE_FLOW 1
#endif

#ifndef PX4_FLOW_I2C_DEV
#define PX4_FLOW_I2C_DEV i2c1
#endif

#define PX4_FLOW_SLAVE_ADDR 0x84

struct Px4_flow_I2c px4_flow;



void px4_flow_i2c_init(struct Px4_flow_I2c *ms, struct i2c_periph *i2c_p, uint8_t addr)
{
	/* set i2c_peripheral */
	ms->i2c_p = i2c_p;

	/* slave address */
	ms->i2c_trans.slave_addr = addr;
	/* set initial status: Success or Done */
	ms->i2c_trans.status = I2CTransDone;

	//ms->data_available = FALSE;
	//ms->initialized = FALSE;
	//ms->status = FLOW_STATUS_UNINIT;
	//ms->prom_cnt = 0;
}

void px4_flow_init(void)
{
	px4_flow_i2c_init(&px4_flow, &PX4_FLOW_I2C_DEV, PX4_FLOW_SLAVE_ADDR);
	px4_flow.data.sum_x=0.0;
	px4_flow.data.sum_y=0.0;
#if PERIODIC_TELEMETRY
	register_periodic_telemetry(DefaultPeriodic, "PX4FLOW", send_px4flow);
#endif // DOWNLINK
}

void px4_flow_periodic(void)
{
	static bool state_flow=0;
	/*take turns doing(transceive and get data,fre/2 */
	switch(state_flow)
	{
	case 0://request 22bytes data
	{
		if(px4_flow.i2c_trans.status == I2CTransDone)
		{
			px4_flow.i2c_trans.buf[0]=0x00;  //register addr
			i2c_transceive(&PX4_FLOW_I2C_DEV,&px4_flow.i2c_trans,PX4_FLOW_SLAVE_ADDR,1,22);
		}
		state_flow=1;
		break;
	}
	case 1:
	{
		if(px4_flow.i2c_trans.status == I2CTransSuccess)
		{
			//read data
			px4_flow.data.frame_count=(px4_flow.i2c_trans.buf[1]<<8 | px4_flow.i2c_trans.buf[0]);
			px4_flow.data.pixel_flow_x_sum=(px4_flow.i2c_trans.buf[3]<<8 | px4_flow.i2c_trans.buf[2]);
			px4_flow.data.pixel_flow_y_sum=(px4_flow.i2c_trans.buf[5]<<8 | px4_flow.i2c_trans.buf[4]);
			px4_flow.data.flow_comp_m_x=-(px4_flow.i2c_trans.buf[7]<<8 | px4_flow.i2c_trans.buf[6]);  //unit=mm/s(last two frame)
			px4_flow.data.flow_comp_m_y=-(px4_flow.i2c_trans.buf[9]<<8 | px4_flow.i2c_trans.buf[8]);  //orientation
			px4_flow.data.qual=(px4_flow.i2c_trans.buf[11]<<8 | px4_flow.i2c_trans.buf[10]);
			//px4_flow.data.gyro_x_rate=(px4_flow.i2c_trans.buf[13]<<8 | px4_flow.i2c_trans.buf[12]);
			//px4_flow.data.gyro_y_rate=(px4_flow.i2c_trans.buf[15]<<8 | px4_flow.i2c_trans.buf[14]);
			//px4_flow.data.gyro_z_rate=(px4_flow.i2c_trans.buf[17]<<8 | px4_flow.i2c_trans.buf[16]);
			//px4_flow.data.gyro_range=px4_flow.i2c_trans.buf[18];
			px4_flow.data.sonar_timestamp=px4_flow.i2c_trans.buf[19];
			px4_flow.data.ground_distance=(px4_flow.i2c_trans.buf[21]<<8 | px4_flow.i2c_trans.buf[20])/1000.0;	 //unit=m

			px4_flow.data.sum_x=
				px4_flow.data.ground_distance*px4_flow.data.pixel_flow_x_sum*0.51/1000.0;   //unit=m
			px4_flow.data.sum_y=
				px4_flow.data.ground_distance*px4_flow.data.pixel_flow_y_sum*0.51/1000.0;

			if(px4_flow.data.ground_distance>0.022)  //distance lower 0.022m,give up
			{
				AbiSendMsgAGL(AGL_SONAR_ADC_ID, px4_flow.data.ground_distance);  //abi send sonar data
				AbiSendMsgFLOW(FLOW_PX4_ID, &px4_flow.data);  //abi send flow data
			}
			px4_flow.i2c_trans.status = I2CTransDone;
		}
		else if(px4_flow.i2c_trans.status == I2CTransFailed)
			px4_flow.i2c_trans.status = I2CTransDone;
		state_flow=0;
		break;
	}
	default:
		state_flow=0;
		break;

	}

}


