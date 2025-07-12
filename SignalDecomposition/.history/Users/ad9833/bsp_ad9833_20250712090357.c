#ifndef __BSP_AD9833_H
#define __BSP_AD9833_H

#include "stm32f4xx_hal.h"

#define AD9833_FSYNC_PORT    GPIOA
#define AD9833_FSYNC_PIN     GPIO_PIN_4
#define AD9833_SCLK_PORT     GPIOA
#define AD9833_SCLK_PIN      GPIO_PIN_5
#define AD9833_SDATA_PORT    GPIOA
#define AD9833_SDATA_PIN     GPIO_PIN_7

/******************************************************************************/
/* AD9833                                                                    */
/******************************************************************************/
/* ¼Ä´æÆ÷ */

#define AD9833_REG_CMD		(0 << 14)
#define AD9833_REG_FREQ0	(1 << 14)
#define AD9833_REG_FREQ1	(2 << 14)
#define AD9833_REG_PHASE0	(6 << 13)
#define AD9833_REG_PHASE1	(7 << 13)




#endif /* __BSP_AD9833_H */