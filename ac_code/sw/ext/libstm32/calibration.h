#ifndef __CALIBRATION_H
#define __CALIBRATION_H

extern void cali_fatfs_init(void);
extern void cali_magraw_to_txt(int32_t x1, int32_t y1, int32_t z1);
extern void cali_task(void);

#endif


