#include "data_process.h"

/* �ڲ���̬���������� FFT/Goertzel ����һ�� ADC ���� */
static float adc_zeroed[BUF_SIZE];     /* ȥֱ�� & �����ѹ��ĸ������� */

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

    // 6. ʹ��goertzel�㷨���û�Ƶ���Ƹߴ�г��
    // 6.1 ��ȡ��Ƶ
    float fund_freq,fund_amp;
    fft_hann_zero_interp(adc_zeroed, &fund_freq, &fund_amp);    
    goertzel_cfg_t target_freq_cfg;

    float harm1_mag, harm1_phase;
    goertzel_init(&target_freq_cfg, FFT_SIZE, fund_freq*1.0f, FS_HZ);
    goertzel_process_f32(&target_freq_cfg, adc_zeroed, &harm1_mag, &harm1_phase);


    // 6.2 ��ȡ3��г������
    float harm3_mag, harm3_phase;
    goertzel_init(&target_freq_cfg, FFT_SIZE, fund_freq*3.0f, FS_HZ);
    goertzel_process_f32(&target_freq_cfg, adc_zeroed, &harm3_mag, &harm3_phase);

    // 6.3 ��ȡ5��г������
    float harm5_mag, harm5_phase;
    goertzel_init(&target_freq_cfg, FFT_SIZE, fund_freq * 5.0f, FS_HZ);
    goertzel_process_f32(&target_freq_cfg, adc_zeroed, &harm5_mag, &harm5_phase);


}

/* _buf: ADC ԭʼ uint16[]���������� FFT_SIZE */
void analyse_two_signals(const uint16_t *_buf, Signal_t *sig1, Signal_t *sig2)
{
    /* 1. ȥ��ƫ & ����Ϊ��ѹ */
    adc_zero_bias(_buf, adc_zeroed, FFT_SIZE);

    /* 2. FFT �ҵ��������� */
    float f1, A1, f2, A2;
    fft_top2_hann_zero_interp(adc_zeroed, &f1, &A1, &f2, &A2, FFT_SIZE);

    /* 3. ��ÿ��������Goertzel ���� 1��/3��/5�� ��ֵ */
    goertzel_cfg_t g1, g3, g5;
    float mag1, ph1, mag3, ph3, mag5, ph5;

    /* --- helper lambda��C99 ��ģ�⣩--- */
    #define ANALYSE_ONE(_f_base, _sig_out)                                     \
        do {                                                                   \
            /* fundamental */                                                  \
            goertzel_init(&g1, N_RAW, (_f_base), FS_HZ);                       \
            goertzel_process_f32(&g1, adc_zeroed, &mag1, &ph1);                \
            /* 3rd harmonic */                                                 \
            goertzel_init(&g3, N_RAW, 3.0f*(_f_base), FS_HZ);                  \
            goertzel_process_f32(&g3, adc_zeroed, &mag3, &ph3);                \
            /* 5th harmonic */                                                 \
            goertzel_init(&g5, N_RAW, 5.0f*(_f_base), FS_HZ);                  \
            goertzel_process_f32(&g5, adc_zeroed, &mag5, &ph5);                \
                                                                               \
            float r3 = mag3 / mag1;                                            \
            float r5 = mag5 / mag1;                                            \
            (_sig_out)->freq = (_f_base);                                      \
            if (r3 < 0.03f) {                                                  \
                (_sig_out)->wave_form = SINC_WAVE;                             \
            } else if (r3 > 0.07f && r3 < 0.15f && r5 > 0.02f && r5 < 0.06f) { \
                (_sig_out)->wave_form = TRIANGLE_WAVE;                         \
            } else {                                                           \
                /* �������ߣ��ɼ� LOG ����δ֪ */                             \
                (_sig_out)->wave_form = 0;                                     \
            }                                                                  \
        } while(0)

    ANALYSE_ONE(f1, sig1);
    ANALYSE_ONE(f2, sig2);
}