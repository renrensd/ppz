#ifndef PX4_FLOW_I2C_H
#define PX4_FLOW_I2C_H

#include "std.h"
#include "mcu_periph/i2c.h"

/*
enum Px4flow_Status {
  FLOW_STATUS_UNINIT,
  FLOW_STATUS_SENDED_ADD,
  FLOW_STATUS_RECEIVED
};*/
struct Px4_flow_Data
{
	//uint8_t flow_buf[22];
	uint16_t frame_count;// counts created I2C frames [#frames]
	int16_t pixel_flow_x_sum;// latest x flow measurement in pixels*10 [pixels]
	int16_t pixel_flow_y_sum;// latest y flow measurement in pixels*10 [pixels]
	int16_t flow_comp_m_x;// x velocity*1000 [meters/sec]
	int16_t flow_comp_m_y;// y velocity*1000 [meters/sec]
	uint8_t qual;// Optical flow quality / confidence [0: bad, 255: maximum quality]
	//int16_t gyro_x_rate; // latest gyro x rate [rad/sec]
	//int16_t gyro_y_rate; // latest gyro y rate [rad/sec]
	//int16_t gyro_z_rate; // latest gyro z rate [rad/sec]
	//uint8_t gyro_range; // gyro range [0 .. 7] equals [50 deg/sec .. 2000 deg/sec]
	uint8_t sonar_timestamp;// time since last sonar update [milliseconds]
	float   ground_distance;// Ground distance in meters*1000 [meters].
	//int16_t pixel_flow_x_integral;
	//int16_t pixel_flow_y_integral;
	//int16_t gyro_x_rate_integral;
	//int16_t gyro_y_rate_integral;
	float sum_x;
	float sum_y;
	//float sum_gyro_x;
	//float sum_gyro_y;
};

struct Px4_flow_I2c
{
	struct i2c_periph *i2c_p;
	struct i2c_transaction i2c_trans;
	//enum Px4flow_Status status;
	//bool_t initialized;                 ///< config done flag
	//volatile bool_t data_available;     ///< data ready flag
	struct Px4_flow_Data data;
	//int32_t prom_cnt;                   ///< number of bytes read from PROM using?
};

extern struct Px4_flow_I2c px4_flow;

//void px4flow_i2c_init(struct Px4flow_I2c *ms, struct i2c_periph *i2c_p, uint8_t addr);
void px4_flow_init(void);
void px4_flow_periodic(void);
void px4_flow_i2c_init(struct Px4_flow_I2c *ms, struct i2c_periph *i2c_p, uint8_t addr);
#endif
