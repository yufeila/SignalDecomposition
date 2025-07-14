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

    // ������ܵ�DMA��־λ
    __HAL_DMA_CLEAR_FLAG(&hdma_adc1, DMA_FLAG_TCIF0_4);

    // ������ADC+DMA��ȷ��׼������
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, BUF_SIZE) != HAL_OK)
    {
        Error_Handler();
    }       

    // ������ʱ������һ��TRGO�����������ں���
    HAL_TIM_Base_Start(&htim2);

    // 2. �ȴ��������ݲɼ����
    uint32_t timeout = 0;
    while (ADC_BufferReadyFlag != BUFFER_READY_FLAG_FULL && timeout < 1000)
    {
        HAL_Delay(1);  // ������ʱ������CPUռ�ù���
        timeout++;
    }

    // 3. ����Ƿ�ʱ
    if (timeout >= 1000)
    {
        // ��ʱ����ֹͣADC+TIM��������������
        HAL_ADC_Stop_DMA(&hadc1);
        HAL_TIM_Base_Stop(&htim2);
        return 0;
    }

    // 4. ֹͣ��ǰ�ִε�ADC+TIM��Ϊ�´�������׼����
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_TIM_Base_Stop(&htim2);

    // ���URSλ���ָ�Ĭ����Ϊ
    htim2.Instance->CR1 &= ~TIM_CR1_URS;

    // 5. �����ֲɼ�������
    const uint16_t *src = (const uint16_t *)&adc_buffer[0];
    float adc_zeroed[BUF_SIZE];
    adc_zero_bias(src, adc_zeroed, BUF_SIZE);

    // 6. ʹ��go
    float fund_freq,fund_amp;
    fft_hann_zero_interp(adc_zeroed, &fund_freq, &fund_amp);    
    
    // 6. ���һ�Ƶ
    goertzel_cfg_t target_freq_cfg;

    float harm3_mag, harm3_phase;
    goertzel_init(&target_freq_cfg, FFT_SIZE, fund_freq, FS_HZ);

    
    goertzel_process_f32(&target_freq_cfg, adc_zeroed, &harm3_mag, &harm3_phase);

    float harm5_mag, harm5_phase;
    goertzel_init(&target_freq_cfg, FFT_SIZE, fund_freq * 3.0f, FS_HZ);
    goertzel_process_f32(&target_freq_cfg, adc_zeroed, &harm5_mag, &harm5_phase);


}