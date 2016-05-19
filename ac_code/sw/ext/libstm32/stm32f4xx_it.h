//V1.0.0
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include "stm32f4xx.h"

extern void sdio_isr(void);
extern void dma2_stream3_isr(void);

#ifdef __cplusplus
}
#endif

#endif 

