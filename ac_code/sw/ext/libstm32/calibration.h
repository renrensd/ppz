#ifndef __CALIBRATION_H
#define __CALIBRATION_H

struct CALI_INFO
{
	uint8_t mag_entry_flag; /* TRUE:entry calibration state, FALSE:exit calibration state*/
	uint8_t mag_state;
	uint16_t mag_txt_len;
	uint8_t mag_cnt;
};

enum CALI_MAG_STATE_PARAM
{
	CALI_MAG_STATE_INIT = 0x00,
	CALI_MAG_STATE_WAIT_FINISHED = 0x01,
	CALI_MAG_STATE_FINISHED_SUCCESS = 0x02,
	CALI_MAG_STATE_FINISHED_FAIL = 0x03,
};

#define CALI_MAG_TIMEROUT 15000 /* uint:ms */
#define CALI_MAG_RAW_CNT 4


extern void cali_fatfs_init(void);
extern void cali_magraw_to_txt(int32_t x1, int32_t y1, int32_t z1);
extern void cali_task(void);
extern void cali_mag_begin(void);
extern void cali_mag_end(void);
extern void cali_mag_state_init(void);

#endif


