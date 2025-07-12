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
/* 寄存器 */

#define AD9833_REG_CMD		(0 << 14)
#define AD9833_REG_FREQ0	(1 << 14)
#define AD9833_REG_FREQ1	(2 << 14)
#define AD9833_REG_PHASE0	(6 << 13)
#define AD9833_REG_PHASE1	(7 << 13)

/* 命令控制位 */

#define AD9833_B28				(1 << 13)       // 连续读写控制位
#define AD9833_HLB				(1 << 12)       // 高低字节控制位
#define AD9833_FSEL0			(0 << 11)       // 频率选择位0
#define AD9833_FSEL1			(1 << 11)       // 频率选择位1
#define AD9833_PSEL0			(0 << 10)       // 相位选择位0
#define AD9833_PSEL1			(1 << 10)       // 相位选择位1
#define AD9833_PIN_SW			(1 << 9)       // 引脚开关
#define AD9833_RESET			(1 << 8)        
#define AD9833_SLEEP1			(1 << 7)
#define AD9833_SLEEP12		(1 << 6)
#define AD9833_OPBITEN		(1 << 5)
#define AD9833_SIGN_PIB		(1 << 4)
#define AD9833_DIV2				(1 << 3)
#define AD9833_MODE				(1 << 1)

#define AD9833_OUT_SINUS		((0 << 5) | (0 << 1) | (0 << 3))//正弦波 
#define AD9833_OUT_TRIANGLE	((0 << 5) | (1 << 1) | (0 << 3))//三角波
#define AD9833_OUT_MSB			((1 << 5) | (0 << 1) | (1 << 3)) //方波
#define AD9833_OUT_MSB2			((1 << 5) | (0 << 1) | (0 << 3))



#endif /* __BSP_AD9833_H */