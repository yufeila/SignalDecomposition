#include "data_process.h"

void SignalOutputAndDisplay(void)
{
    // ��ⰴ��״̬
    Detect_KeyPress();

    // ����źŷֽ��־�����ã�ִ���źŷֽ⴦��
    if (signal_deconposition_flag)
    {
        signal_deconposition_flag = 0; // ���ñ�־

        Data_Process();

        Update_LCD_Display();

        DDS_Output();
    }
}

void Data_Process(void)
{
    // 1. ��������ADC+DMA+TIM�ɼ��������޸�����ʱ��
    ADC_BufferReadyFlag = BUFFER_READY_FLAG_NONE;



    // ������ADC+DMA��ȷ��׼������
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, BUF_SIZE) != HAL_OK)
    {
        Error_Handler();
    }       
}