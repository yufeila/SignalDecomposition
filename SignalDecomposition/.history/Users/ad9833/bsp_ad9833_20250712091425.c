#include "bsp_ad9833.h"

//时钟速率为25 MHz时， 可以实现0.1 Hz的分辨率；而时钟速率为1 MHz时，则可以实现0.004 Hz的分辨率。
//调整参考时钟修改此处即可。
#define FCLK 25000000	//设置参考时钟25MHz，板默认板载晶振频率25Mhz。

#define RealFreDat    268435456.0/FCLK//总的公式为 Fout=（Fclk/2的28次方）*28位寄存器的值

/************************************************************
** 函数名称 ：void AD983_GPIO_Init(void)  
** 函数功能 ：初始化控制AD9833需要用到的IO口
** 入口参数 ：无
** 出口参数 ：无
** 函数说明 ：无
**************************************************************/

void AD983_GPIO_Init(void) 
{

    GPIO_InitTypeDef GPIO_InitStructure ; 
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //使能PA端口时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5; 

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ; 

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ; 

    GPIO_Init(GPIOA ,&GPIO_InitStructure) ; 
} 
