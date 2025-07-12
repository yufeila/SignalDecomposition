#include "bsp_ad9833.h"

//ʱ������Ϊ25 MHzʱ�� ����ʵ��0.1 Hz�ķֱ��ʣ���ʱ������Ϊ1 MHzʱ�������ʵ��0.004 Hz�ķֱ��ʡ�
//�����ο�ʱ���޸Ĵ˴����ɡ�
#define FCLK 25000000	//���òο�ʱ��25MHz����Ĭ�ϰ��ؾ���Ƶ��25Mhz��

#define RealFreDat    268435456.0/FCLK//�ܵĹ�ʽΪ Fout=��Fclk/2��28�η���*28λ�Ĵ�����ֵ

/************************************************************
** �������� ��void AD983_GPIO_Init(void)  
** �������� ����ʼ������AD9833��Ҫ�õ���IO��
** ��ڲ��� ����
** ���ڲ��� ����
** ����˵�� ����
**************************************************************/

void AD983_GPIO_Init(void) 
{

    GPIO_InitTypeDef GPIO_InitStructure ; 
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //ʹ��PA�˿�ʱ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5; 

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ; 

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ; 

    GPIO_Init(GPIOA ,&GPIO_InitStructure) ; 
} 
