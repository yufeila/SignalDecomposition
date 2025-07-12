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
void AD9833_1_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // ʹ��GPIOAʱ��
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // ����FSYNC��SCLK��SDATAΪ�������
    GPIO_InitStruct.Pin = AD9833_1_FSYNC_PIN | AD9833_1_SCLK_PIN | AD9833_1_SDATA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(AD9833_1_FSYNC_PORT, &GPIO_InitStruct);
}

/**********************************************************************************************
** �������� ��unsigned char AD9833_SPI_Write(unsigned char* data,unsigned char bytesNumber)
** �������� ��ʹ��ģ��SPI��AD9833д����
** ��ڲ��� ��* data:д�����ݻ�����,��һ���ֽ��ǼĴ�����ַ���ڶ����ֽڿ�ʼҪд������ݡ�
                        bytesNumber: Ҫд����ֽ���
** ���ڲ��� ����
** ����˵�� ����
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
** �������� ��void AD9833_1_SetRegisterValue(unsigned short regValue)
** �������� ����ֵд��Ĵ���
** ��ڲ��� ��regValue��Ҫд��Ĵ�����ֵ��(16λ�Ĵ���)
** ���ڲ��� ����
** ����˵�� ����
*******************************************************************************************/
void AD9833_1_SetRegisterValue(unsigned short regValue)
{
	unsigned char data[5] = {0x03, 0x00, 0x00};	
	
	data[1] = (unsigned char)((regValue & 0xFF00) >> 8);	//MSB
	data[2] = (unsigned char)((regValue & 0x00FF) >> 0);	//LSB
	AD9833_1_SPI_Write(data,2);
}

/************************************************************
** �������� ��void AD9833_1_Init(void)  
** �������� ����ʼ������AD9833��Ҫ�õ���IO�ڼ��Ĵ���
** ��ڲ��� ����
** ���ڲ��� ����
** ����˵�� ����
**************************************************************/
void AD9833_1_Init(void)
{
    AD9833_1_GPIO_Init();
    AD9833_1_SetRegisterValue(AD9833_REG_CMD | AD9833_RESET);
}

/*****************************************************************************************
** �������� ��void AD9833_Reset(void)  
** �������� ������AD9833�ĸ�λλ
** ��ڲ��� ����
** ���ڲ��� ����
** ����˵�� ����
*******************************************************************************************/
void AD9833_1_Reset(void)
{
	AD9833_1_SetRegisterValue(AD9833_REG_CMD | AD9833_RESET);
	delay_ms(10);
}

/*****************************************************************************************
** �������� ��void AD9833_1_ClearReset(void)  
** �������� �����AD9833�ĸ�λλ��
** ��ڲ��� ����
** ���ڲ��� ����
** ����˵�� ����
*******************************************************************************************/
void AD9833_1_ClearReset(void)
{
	AD9833_1_SetRegisterValue(AD9833_REG_CMD);
}

/*****************************************************************************************
** �������� ��void AD9833_1_SetFrequencyQuick(float fout,unsigned short type)
** �������� ��д��Ƶ�ʼĴ���
** ��ڲ��� ��val��Ҫд���Ƶ��ֵ��
**						type���������ͣ�AD9833_OUT_SINUS���Ҳ���AD9833_OUT_TRIANGLE���ǲ���AD9833_OUT_MSB����
** ���ڲ��� ����
** ����˵�� ��ʱ������Ϊ25 MHzʱ�� ����ʵ��0.1 Hz�ķֱ��ʣ���ʱ������Ϊ1 MHzʱ�������ʵ��0.004 Hz�ķֱ��ʡ�
*******************************************************************************************/
void AD9833_1_SetFrequencyQuick(float fout,unsigned short type)
{
	AD9833_1_SetFrequency(AD9833_REG_FREQ0, fout,type);
}

/*****************************************************************************************
** �������� ��void AD9833_1_SetFrequency(unsigned short reg, float fout,unsigned short type)
** �������� ��д��Ƶ�ʼĴ���
** ��ڲ��� ��reg��Ҫд���Ƶ�ʼĴ�������16λ��
**						val��Ҫд���ֵ��
**						type���������ͣ�AD9833_OUT_SINUS���Ҳ���AD9833_OUT_TRIANGLE���ǲ���AD9833_OUT_MSB����
** ���ڲ��� ����
** ����˵�� ����
*******************************************************************************************/
void AD9833_1_SetFrequency(unsigned short reg, float fout,unsigned short type)
{
	unsigned short freqHi = reg;
	unsigned short freqLo = reg;
	unsigned long val=RealFreDat*fout;
	freqHi |= (val & 0xFFFC000) >> 14 ; //��14λ
	freqLo |= (val & 0x3FFF);           //��14λ
	AD9833_1_SetRegisterValue(AD9833_B28|type);
	AD9833_1_SetRegisterValue(freqLo);
	AD9833_1_SetRegisterValue(freqHi);
}

/*****************************************************************************************
** �������� ��void AD9833_SetPhase(unsigned short reg, unsigned short val)
** �������� ��д����λ�Ĵ�����
** ��ڲ��� ��reg��Ҫд�����λ�Ĵ�����
**						val��Ҫд���ֵ��
** ���ڲ��� ����
** ����˵�� ����
*******************************************************************************************/
void AD9833_SetPhase(unsigned short reg, unsigned short val)
{
	unsigned short phase = reg;
	phase |= val;
	AD9833_1_SetRegisterValue(phase);
}

/*****************************************************************************************
** �������� ��void AD9833_1_Setup(unsigned short freq, unsigned short phase,unsigned short type)
** �������� ��д����λ�Ĵ�����
** ��ڲ��� ��freq��ʹ�õ�Ƶ�ʼĴ�����
							phase��ʹ�õ���λ�Ĵ�����
							type��Ҫ����Ĳ������͡�
** ���ڲ��� ����
** ����˵�� ����
*******************************************************************************************/
void AD9833_1_Setup(unsigned short freq, unsigned short phase,unsigned short type)
{
	unsigned short val = 0;
	
	val = freq | phase | type;
	AD9833_1_SetRegisterValue(val);
}

/*****************************************************************************************
** �������� ��void AD9833_SetWave(unsigned short type)
** �������� ������Ҫ����Ĳ������͡�
** ��ڲ��� ��type��Ҫ����Ĳ������͡�
** ���ڲ��� ����
** ����˵�� ����
*******************************************************************************************/
void AD9833_SetWave(unsigned short type)
{
	AD9833_1_SetRegisterValue(type);
}

// ���������÷�ʽ
//	AD9833_1_Init();//IO�ڼ�AD9833�Ĵ�����ʼ��
	
//  Ƶ����ڲ���Ϊfloat����ʹ�źŵ�Ƶ�ʸ���ȷ
//	AD9833_1_SetFrequencyQuick(1000.0,AD9833_OUT_SINUS);//д���Ƶ��1000.0Hz,������Ҳ�
//	AD9833_1_SetFrequencyQuick(1000.0,AD9833_OUT_TRIANGLE);//д���Ƶ��1000.0Hz,������ǲ�
//	AD9833_1_SetFrequencyQuick(1000.0,AD9833_OUT_MSB);//д���Ƶ��1000.0Hz,�������
