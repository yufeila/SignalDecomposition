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
/* �Ĵ��� */

#define AD9833_REG_CMD		(0 << 14)
#define AD9833_REG_FREQ0	(1 << 14)
#define AD9833_REG_FREQ1	(2 << 14)
#define AD9833_REG_PHASE0	(6 << 13)
#define AD9833_REG_PHASE1	(7 << 13)

// ģ��SPI��GPIO������
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

/* �������λ */

#define AD9833_B28				(1 << 13)       // ������д����λ
#define AD9833_HLB				(1 << 12)       // �ߵ��ֽڿ���λ
#define AD9833_FSEL0			(0 << 11)       // Ƶ��ѡ��λ0
#define AD9833_FSEL1			(1 << 11)       // Ƶ��ѡ��λ1
#define AD9833_PSEL0			(0 << 10)       // ��λѡ��λ0
#define AD9833_PSEL1			(1 << 10)       // ��λѡ��λ1
#define AD9833_PIN_SW			(1 << 9)       // ���ſ���
#define AD9833_RESET			(1 << 8)        // ��λ����λ
#define AD9833_SLEEP1			(1 << 7)        // ˯�߿���λ1
#define AD9833_SLEEP12		    (1 << 6)            // ˯�߿���λ2
#define AD9833_OPBITEN		    (1 << 5)            // ���λʹ��
#define AD9833_SIGN_PIB		    (1 << 4)            // ���Ҳ����ѡ��
#define AD9833_DIV2				(1 << 3)            // ����2
#define AD9833_MODE				(1 << 1)            // ģʽѡ��

#define AD9833_OUT_SINUS		((0 << 5) | (0 << 1) | (0 << 3))//���Ҳ� 
#define AD9833_OUT_TRIANGLE	((0 << 5) | (1 << 1) | (0 << 3))//���ǲ�
#define AD9833_OUT_MSB			((1 << 5) | (0 << 1) | (1 << 3)) //����
#define AD9833_OUT_MSB2			((1 << 5) | (0 << 1) | (0 << 3))

void AD9833_1_GPIO_Init(void);//��ʼ��IO��
void AD9833_1_Init(void);//��ʼ��IO�ڼ��Ĵ�������main�����е��ã�����ÿ�ο�ʼǰ����ʼ��

void AD9833_1_Reset(void);			//��λAD9833�ĸ�λλ
void AD9833_1_ClearReset(void);	//���AD9833�� ��λλ

void AD9833_1_SetRegisterValue(unsigned short regValue);												//��ֵд��Ĵ���
void AD9833_1_SetFrequency(unsigned short reg, float fout,unsigned short type);	                        //д��Ƶ�ʼĴ���
void AD9833_1_SetPhase(unsigned short reg, unsigned short val);									        //д����λ�Ĵ���

void AD9833_1_Setup(unsigned short freq,unsigned short phase,unsigned short type);	                    //ѡ��Ƶ�ʡ���λ�Ͳ�������
void AD9833_1_SetFrequencyQuick(float fout,unsigned short type);	                                    //����Ƶ�ʼ���������

void AD9833_2_GPIO_Init(void);//��ʼ��IO��
void AD9833_2_Init(void);//��ʼ��IO�ڼ��Ĵ�������main�����е��ã�����ÿ�ο�ʼǰ����ʼ��

void AD9833_2_Reset(void);			//��λAD9833�ĸ�λλ
void AD9833_2_ClearReset(void);	//���AD9833�� ��λλ

void AD9833_2_SetRegisterValue(unsigned short regValue);												//��ֵд��Ĵ���
void AD9833_2_SetFrequency(unsigned short reg, float fout,unsigned short type);	                        //д��Ƶ�ʼĴ���
void AD9833_2_SetPhase(unsigned short reg, unsigned short val);									        //д����λ�Ĵ���

void AD9833_2_Setup(unsigned short freq,unsigned short phase,unsigned short type);	                    //ѡ��Ƶ�ʡ���λ�Ͳ�������
void AD9833_2_SetFrequencyQuick(float fout,unsigned short type);	                                    //����Ƶ�ʼ���������

void AD9833_1_Config(float fout, uint16_t waveform, uint16_t phase);
void AD9833_2_Config(float fout, uint16_t waveform, uint16_t phase);

#endif /* __BSP_AD9833_H */
