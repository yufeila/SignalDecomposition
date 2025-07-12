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
void AD9833_1_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 使能GPIOA时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // 配置FSYNC、SCLK、SDATA为推挽输出
    GPIO_InitStruct.Pin = AD9833_1_FSYNC_PIN | AD9833_1_SCLK_PIN | AD9833_1_SDATA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(AD9833_1_FSYNC_PORT, &GPIO_InitStruct);
}

/**********************************************************************************************
** 函数名称 ：unsigned char AD9833_SPI_Write(unsigned char* data,unsigned char bytesNumber)
** 函数功能 ：使用模拟SPI向AD9833写数据
** 入口参数 ：* data:写入数据缓冲区,第一个字节是寄存器地址；第二个字节开始要写入的数据。
                        bytesNumber: 要写入的字节数
** 出口参数 ：无
** 函数说明 ：无
************************************************************************************************/
unsigned char AD9833_1_SPI_Write(unsigned char* data,unsigned char bytesNumber)
{
    unsigned char i, j; 
    unsigned char writeData[5] = {0, 0, 0, 0, 0};

    AD9833_1_SCLK_HIGH(); 
    AD9833_1_FSYNC_LOW();

    for(i = 0; i < bytesNumber; i++)
    {
        writeData[i] = data[i + 1];
    }

    for(i = 0; i < bytesNumber; i++) 
    {
        for(j = 0; j < 8; j++)      
        { 
            if(writeData[i] & 0x80) 
                AD9833_1_SDATA_HIGH(); 
            else 
                AD9833_1_SDATA_LOW(); 

            AD9833_1_SCLK_LOW(); 
            writeData[i] <<= 1; 
            AD9833_1_SCLK_HIGH(); 
        } 
    }
    AD9833_1_SDATA_HIGH(); 
    AD9833_1_FSYNC_HIGH(); 

    return i;
}

/*****************************************************************************************
** 函数名称 ：void AD9833_1_SetRegisterValue(unsigned short regValue)
** 函数功能 ：将值写入寄存器
** 入口参数 ：regValue：要写入寄存器的值。(16位寄存器)
** 出口参数 ：无
** 函数说明 ：无
*******************************************************************************************/
void AD9833_1_SetRegisterValue(unsigned short regValue)
{
	unsigned char data[5] = {0x03, 0x00, 0x00};	
	
	data[1] = (unsigned char)((regValue & 0xFF00) >> 8);	//MSB
	data[2] = (unsigned char)((regValue & 0x00FF) >> 0);	//LSB
	AD9833_1_SPI_Write(data,2);
}

/************************************************************
** 函数名称 ：void AD9833_1_Init(void)  
** 函数功能 ：初始化控制AD9833需要用到的IO口及寄存器
** 入口参数 ：无
** 出口参数 ：无
** 函数说明 ：无
**************************************************************/
void AD9833_1_Init(void)
{
    AD9833_1_GPIO_Init();
    AD9833_1_SetRegisterValue(AD9833_REG_CMD | AD9833_RESET);
}

/*****************************************************************************************
** 函数名称 ：void AD9833_Reset(void)  
** 函数功能 ：设置AD9833的复位位
** 入口参数 ：无
** 出口参数 ：无
** 函数说明 ：无
*******************************************************************************************/
void AD9833_1_Reset(void)
{
	AD9833_1_SetRegisterValue(AD9833_REG_CMD | AD9833_RESET);
	delay_ms(10);
}

/*****************************************************************************************
** 函数名称 ：void AD9833_1_ClearReset(void)  
** 函数功能 ：清除AD9833的复位位。
** 入口参数 ：无
** 出口参数 ：无
** 函数说明 ：无
*******************************************************************************************/
void AD9833_1_ClearReset(void)
{
	AD9833_1_SetRegisterValue(AD9833_REG_CMD);
}

/*****************************************************************************************
** 函数名称 ：void AD9833_1_SetFrequencyQuick(float fout,unsigned short type)
** 函数功能 ：写入频率寄存器
** 入口参数 ：val：要写入的频率值。
**						type：波形类型；AD9833_OUT_SINUS正弦波、AD9833_OUT_TRIANGLE三角波、AD9833_OUT_MSB方波
** 出口参数 ：无
** 函数说明 ：时钟速率为25 MHz时， 可以实现0.1 Hz的分辨率；而时钟速率为1 MHz时，则可以实现0.004 Hz的分辨率。
*******************************************************************************************/
void AD9833_1_SetFrequencyQuick(float fout,unsigned short type)
{
	AD9833_1_SetFrequency(AD9833_REG_FREQ0, fout,type);
}

/*****************************************************************************************
** 函数名称 ：void AD9833_1_SetFrequency(unsigned short reg, float fout,unsigned short type)
** 函数功能 ：写入频率寄存器
** 入口参数 ：reg：要写入的频率寄存器。（16位）
**						val：要写入的值。
**						type：波形类型；AD9833_OUT_SINUS正弦波、AD9833_OUT_TRIANGLE三角波、AD9833_OUT_MSB方波
** 出口参数 ：无
** 函数说明 ：无
*******************************************************************************************/
void AD9833_1_SetFrequency(unsigned short reg, float fout,unsigned short type)
{
	unsigned short freqHi = reg;
	unsigned short freqLo = reg;
	unsigned long val=RealFreDat*fout;
	freqHi |= (val & 0xFFFC000) >> 14 ; //高14位
	freqLo |= (val & 0x3FFF);           //低14位
	AD9833_1_SetRegisterValue(AD9833_B28|type);
	AD9833_1_SetRegisterValue(freqLo);
	AD9833_1_SetRegisterValue(freqHi);
}

/*****************************************************************************************
** 函数名称 ：void AD9833_SetPhase(unsigned short reg, unsigned short val)
** 函数功能 ：写入相位寄存器。
** 入口参数 ：reg：要写入的相位寄存器。
**						val：要写入的值。
** 出口参数 ：无
** 函数说明 ：无
*******************************************************************************************/
void AD9833_SetPhase(unsigned short reg, unsigned short val)
{
	unsigned short phase = reg;
	phase |= val;
	AD9833_1_SetRegisterValue(phase);
}

/*****************************************************************************************
** 函数名称 ：void AD9833_1_Setup(unsigned short freq, unsigned short phase,unsigned short type)
** 函数功能 ：写入相位寄存器。
** 入口参数 ：freq：使用的频率寄存器。
							phase：使用的相位寄存器。
							type：要输出的波形类型。
** 出口参数 ：无
** 函数说明 ：无
*******************************************************************************************/
void AD9833_1_Setup(unsigned short freq, unsigned short phase,unsigned short type)
{
	unsigned short val = 0;
	
	val = freq | phase | type;
	AD9833_1_SetRegisterValue(val);
}

/*****************************************************************************************
** 函数名称 ：void AD9833_SetWave(unsigned short type)
** 函数功能 ：设置要输出的波形类型。
** 入口参数 ：type：要输出的波形类型。
** 出口参数 ：无
** 函数说明 ：无
*******************************************************************************************/
void AD9833_SetWave(unsigned short type)
{
	AD9833_1_SetRegisterValue(type);
}

// 主函数调用方式
//	AD9833_1_Init();//IO口及AD9833寄存器初始化
	
//  频率入口参数为float，可使信号的频率更精确
//	AD9833_1_SetFrequencyQuick(1000.0,AD9833_OUT_SINUS);//写输出频率1000.0Hz,输出正弦波
//	AD9833_1_SetFrequencyQuick(1000.0,AD9833_OUT_TRIANGLE);//写输出频率1000.0Hz,输出三角波
//	AD9833_1_SetFrequencyQuick(1000.0,AD9833_OUT_MSB);//写输出频率1000.0Hz,输出方波
