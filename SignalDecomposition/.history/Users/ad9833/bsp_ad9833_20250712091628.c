#include "bsp_ad9833.h"

//时钟速率为25 MHz时， 可以实现0.1 Hz的分辨率；而时钟速率为1 MHz时，则可以实现0.004 Hz的分辨率。
//调整参考时钟修改此处即可。
#define FCLK 25000000	//设置参考时钟25MHz，板默认板载晶振频率25Mhz。

#define RealFreDat    268435456.0/FCLK//总的公式为 Fout=（Fclk/2的28次方）*28位寄存器的值

/************************************************************
** 函数名称 ：void AD983_GPIO_Init(void)  
** 函数功能 ：初始化控制AD9833需要用到的IO口（HAL库版本）
** 入口参数 ：无
** 出口参数 ：无
** 函数说明 ：使用宏定义的端口和引脚
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

    // 使能GPIOA时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // 配置FSYNC、SCLK、SDATA为推挽输出
    GPIO_InitStruct.Pin = AD9833_FSYNC_PIN | AD9833_SCLK_PIN | AD9833_SDATA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(AD9833_FSYNC_PORT, &GPIO_InitStruct);
}
