#ifndef __BSP_AD9833_H
#define __BSP_AD9833_H

#include "stm32f4xx_hal.h"



void AD9833_Init(SPI_HandleTypeDef *hspi);
void AD9833_SetFrequency(uint32_t frequency);
void AD9833_SetWaveform(uint8_t waveform);

#endif /* __BSP_AD9833_H */