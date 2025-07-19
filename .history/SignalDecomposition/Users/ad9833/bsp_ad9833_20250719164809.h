#ifndef __BSP_AD9833_H
#define __BSP_AD9833_H

#include "stm32f4xx_hal.h"

#define AD9833_1_FSYNC_PORT    GPIOC
#define AD9833_1_FSYNC_PIN     GPIO_PIN_0
#define AD9833_1_SCLK_PORT     GPIOC
#define AD9833_1_SCLK_PIN      GPIO_PIN_1
#define AD9833_1_SDATA_PORT    GPIOC
#define AD9833_1_SDATA_PIN     GPIO_PIN_2

#define AD9833_2_FSYNC_PORT    GPIOB
#define AD9833_2_FSYNC_PIN     GPIO_PIN_0
#define AD9833_2_SCLK_PORT     GPIOB
#define AD9833_2_SCLK_PIN      GPIO_PIN_1
#define AD9833_2_SDATA_PORT    GPIOB
#define AD9833_2_SDATA_PIN     GPIO_PIN_2

/******************************************************************************/
/* AD9833                                                                    */
/******************************************************************************/
/* 寄存器 */

#define AD9833_REG_CMD		(0 << 14)
#define AD9833_REG_FREQ0	(1 << 14)
#define AD9833_REG_FREQ1	(2 << 14)
#define AD9833_REG_PHASE0	(6 << 13)
#define AD9833_REG_PHASE1	(7 << 13)

// 模拟SPI的GPIO操作宏
#define AD9833_1_FSYNC_HIGH()   HAL_GPIO_WritePin(AD9833_1_FSYNC_PORT, AD9833_1_FSYNC_PIN, GPIO_PIN_SET)
#define AD9833_1_FSYNC_LOW()    HAL_GPIO_WritePin(AD9833_1_FSYNC_PORT, AD9833_1_FSYNC_PIN, GPIO_PIN_RESET)

#define AD9833_1_SCLK_HIGH()    HAL_GPIO_WritePin(AD9833_1_SCLK_PORT, AD9833_1_SCLK_PIN, GPIO_PIN_SET)
#define AD9833_1_SCLK_LOW()     HAL_GPIO_WritePin(AD9833_1_SCLK_PORT, AD9833_1_SCLK_PIN, GPIO_PIN_RESET)

#define AD9833_1_SDATA_HIGH()   HAL_GPIO_WritePin(AD9833_1_SDATA_PORT, AD9833_1_SDATA_PIN, GPIO_PIN_SET)
#define AD9833_1_SDATA_LOW()    HAL_GPIO_WritePin(AD9833_1_SDATA_PORT, AD9833_1_SDATA_PIN, GPIO_PIN_RESET)

#define AD9833_2_FSYNC_HIGH()   HAL_GPIO_WritePin(AD9833_2_FSYNC_PORT, AD9833_2_FSYNC_PIN, GPIO_PIN_SET)
#define AD9833_2_FSYNC_LOW()    HAL_GPIO_WritePin(AD9833_2_FSYNC_PORT, AD9833_2_FSYNC_PIN, GPIO_PIN_RESET)

#define AD9833_2_SCLK_HIGH()    HAL_GPIO_WritePin(AD9833_2_SCLK_PORT, AD9833_2_SCLK_PIN, GPIO_PIN_SET)
#define AD9833_2_SCLK_LOW()     HAL_GPIO_WritePin(AD9833_2_SCLK_PORT, AD9833_2_SCLK_PIN, GPIO_PIN_RESET)

#define AD9833_2_SDATA_HIGH()   HAL_GPIO_WritePin(AD9833_2_SDATA_PORT, AD9833_2_SDATA_PIN, GPIO_PIN_SET)
#define AD9833_2_SDATA_LOW()    HAL_GPIO_WritePin(AD9833_2_SDATA_PORT, AD9833_2_SDATA_PIN, GPIO_PIN_RESET)

/* 命令控制位 */

#define AD9833_B28				(1 << 13)       // 连续读写控制位
#define AD9833_HLB				(1 << 12)       // 高低字节控制位
#define AD9833_FSEL0			(0 << 11)       // 频率选择位0
#define AD9833_FSEL1			(1 << 11)       // 频率选择位1
#define AD9833_PSEL0			(0 << 10)       // 相位选择位0
#define AD9833_PSEL1			(1 << 10)       // 相位选择位1
#define AD9833_PIN_SW			(1 << 9)       // 引脚开关
#define AD9833_RESET			(1 << 8)        // 复位控制位
#define AD9833_SLEEP1			(1 << 7)        // 睡眠控制位1
#define AD9833_SLEEP12		    (1 << 6)            // 睡眠控制位2
#define AD9833_OPBITEN		    (1 << 5)            // 输出位使能
#define AD9833_SIGN_PIB		    (1 << 4)            // 正弦波输出选择
#define AD9833_DIV2				(1 << 3)            // 除以2
#define AD9833_MODE				(1 << 1)            // 模式选择

#define AD9833_OUT_SINUS		((0 << 5) | (0 << 1) | (0 << 3))//正弦波 
#define AD9833_OUT_TRIANGLE	((0 << 5) | (1 << 1) | (0 << 3))//三角波
#define AD9833_OUT_MSB			((1 << 5) | (0 << 1) | (1 << 3)) //方波
#define AD9833_OUT_MSB2			((1 << 5) | (0 << 1) | (0 << 3))

void AD9833_1_GPIO_Init(void);//初始化IO口
void AD9833_1_Init(void);//初始化IO口及寄存器，在main函数中调用，可以每次开始前都初始化

void AD9833_1_Reset(void);			//置位AD9833的复位位
void AD9833_1_ClearReset(void);	//清除AD9833的 复位位

void AD9833_1_SetRegisterValue(unsigned short regValue);												//将值写入寄存器
void AD9833_1_SetFrequency(unsigned short reg, float fout,unsigned short type);	                        //写入频率寄存器
void AD9833_1_SetPhase(unsigned short reg, unsigned short val);									        //写入相位寄存器

void AD9833_1_Setup(unsigned short freq,unsigned short phase,unsigned short type);	                    //选择频率、相位和波形类型
void AD9833_1_SetFrequencyQuick(float fout,unsigned short type);	                                    //设置频率及波形类型

void AD9833_2_GPIO_Init(void);//初始化IO口
void AD9833_2_Init(void);//初始化IO口及寄存器，在main函数中调用，可以每次开始前都初始化

void AD9833_2_Reset(void);			//置位AD9833的复位位
void AD9833_2_ClearReset(void);	//清除AD9833的 复位位

void AD9833_2_SetRegisterValue(unsigned short regValue);												//将值写入寄存器
void AD9833_2_SetFrequency(unsigned short reg, float fout,unsigned short type);	                        //写入频率寄存器
void AD9833_2_SetPhase(unsigned short reg, unsigned short val);									        //写入相位寄存器

void AD9833_2_Setup(unsigned short freq,unsigned short phase,unsigned short type);	                    //选择频率、相位和波形类型
void AD9833_2_SetFrequencyQuick(float fout,unsigned short type);	                                    //设置频率及波形类型

void AD9833_1_Config(float fout, uint16_t waveform, uint16_t phase);
void AD9833_2_Config(float fout, uint16_t waveform, uint16_t phase);

#endif /* __BSP_AD9833_H */
