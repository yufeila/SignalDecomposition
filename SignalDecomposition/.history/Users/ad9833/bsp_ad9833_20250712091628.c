#include "bsp_ad9833.h"

//ʱ������Ϊ25 MHzʱ�� ����ʵ��0.1 Hz�ķֱ��ʣ���ʱ������Ϊ1 MHzʱ�������ʵ��0.004 Hz�ķֱ��ʡ�
//�����ο�ʱ���޸Ĵ˴����ɡ�
#define FCLK 25000000	//���òο�ʱ��25MHz����Ĭ�ϰ��ؾ���Ƶ��25Mhz��

#define RealFreDat    268435456.0/FCLK//�ܵĹ�ʽΪ Fout=��Fclk/2��28�η���*28λ�Ĵ�����ֵ

/************************************************************
** �������� ��void AD983_GPIO_Init(void)  
** �������� ����ʼ������AD9833��Ҫ�õ���IO�ڣ�HAL��汾��
** ��ڲ��� ����
** ���ڲ��� ����
** ����˵�� ��ʹ�ú궨��Ķ˿ں�����
**************************************************************/

#define AD9833_FSYNC_PORT    GPIOA
#define AD9833_FSYNC_PIN     GPIO_PIN_4
#define AD9833_SCLK_PORT     GPIOA
#define AD9833_SCLK_PIN      GPIO_PIN_5
#define AD9833_SDATA_PORT    GPIOA
#define AD9833_SDATA_PIN     GPIO_PIN_7

void AD983_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // ʹ��GPIOAʱ��
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // ����FSYNC��SCLK��SDATAΪ�������
    GPIO_InitStruct.Pin = AD9833_FSYNC_PIN | AD9833_SCLK_PIN | AD9833_SDATA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(AD9833_FSYNC_PORT, &GPIO_InitStruct);
}
