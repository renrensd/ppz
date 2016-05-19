#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "sdio_sd.h"

void sdio_isr(void)
{
    SD_ProcessIRQSrc();
}

void dma2_stream3_isr(void)
{
    SD_ProcessDMAIRQ();
}

