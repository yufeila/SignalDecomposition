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

    // 6. ���������źŲ���
    Signal_t sig1, sig2;
    analyse_two_signals(src, &sig1, &sig2);
    
    // 7. ��ʾ���ݷ������
    Signal_Info_Display(&sig1, &sig2);

    // 8. DDSģ�������ź�
    DDS_Output(&sig1, &sig2);

}

/* _buf: ADC ԭʼ uint16[]���������� FFT_SIZE */
void analyse_two_signals(const uint16_t *_buf, Signal_t *sig1, Signal_t *sig2)
{
    /* 1. ȥ��ƫ & ����Ϊ��ѹ */
    adc_zero_bias(_buf, adc_zeroed, FFT_SIZE);

    /* 2. FFT �ҵ��������� */
    /* �ȵõ�����������Ƶ�ʺͷ��� */
    float f1, A1, f2, A2;
    fft_top2_hann_zero_interp(adc_zeroed, &f1, &A1, &f2, &A2);

    /* --- helper lambda��C99 ��ģ�⣩--- */
    #define ANALYSE_ONE(_f_base, _A_base, _f_other, _A_other, _sig_out)                
    do {                                                                               \
        goertzel_cfg_fomega_t g1, g3, g5;                                                  \
        float mag1, ph1, mag3, ph3, mag5, ph5;                                         \
                                                                                    \
        /* -------- һ���԰� 1��/3��/5�� ��ֵ����� -------- */                          \
        goertzel_init_fomega(&g1, FFT_SIZE, 1.0f * (_f_base),  FS_HZ);                        \
        goertzel_init_fomega(&g3, FFT_SIZE, 3.0f * (_f_base),  FS_HZ);                        \
        goertzel_init_fomega(&g5, FFT_SIZE, 5.0f * (_f_base),  FS_HZ);                        \
        goertzel_process_f32omega(&g1, adc_zeroed, &mag1, &ph1);                            \
        goertzel_process_f32omega(&g3, adc_zeroed, &mag3, &ph3);                            \
        goertzel_process_f32omega(&g5, adc_zeroed, &mag5, &ph5);                            \
                                                                                    \
        /* -------- �� 3�� �� 5�� ����һ�����ص��������ȵ��� -------- */                 \
        const float EPS = 0.4f;   /* half-bin ���� */                                  \
        if (fabsf(3.0f*(_f_base) - (_f_other)) < EPS)  mag3 = fabsf(mag3 - _A_other);  \
        if (fabsf(5.0f*(_f_base) - (_f_other)) < EPS)  mag5 = fabsf(mag5 - _A_other);  \
                                                                                    \
        float r3 = mag3 / mag1;                                                        \
        float r5 = mag5 / mag1;                                                        \
                                                                                    \
        (_sig_out)->freq = (_f_base);                                                  \
        if (r3 < 0.04f && r5 < 0.02f) {                                                \
            (_sig_out)->wave_form = SINC_WAVE;                                         \
        } else if (fabsf(r3 - 0.1111f) < 0.03f && fabsf(r5 - 0.0400f) < 0.02f) {       \
            (_sig_out)->wave_form = TRIANGLE_WAVE;                                     \
        } else {                                                                       \
            (_sig_out)->wave_form = 0; /* δ֪/��ϣ��� log */                         \
        }                                                                              \
    } while (0)


    /* �����Ϊ sig1���ڶ����Ϊ sig2 */
    ANALYSE_ONE(f1, A1, f2, A2, sig1);
    ANALYSE_ONE(f2, A2, f1, A1, sig2);
}

/* ===== ��ʾ���� ===== */
void Signal_Info_Display(Signal_t *sig1, Signal_t *sig2)
{
    LCD_Display_Title_Center("Signal Info", 10);
    
    // ��ʾ�ź�1��Ƶ�ʺ�����
    LCD_ShowString(10, 30, 200, 16, 16, (uint8_t*)"Signal 1:");
    LCD_ShowString(10, 50, 200, 16, 16, (uint8_t*)"Frequency: %.2f Hz", sig1->freq);
    LCD_ShowString(10, 70, 200, 16, 16, (uint8_t*)"Type: %d", sig1->wave_form);

    // ��ʾ�ź�2��Ƶ�ʺ�����
    LCD_ShowString(10, 90, 200, 16, 16, (uint8_t*)"Signal 2:");
    LCD_ShowString(10, 110, 200, 16, 16, (uint8_t*)"Frequency: %.2f Hz", sig2->freq);
    LCD_ShowString(10, 130, 200, 16, 16, (uint8_t*)"Type: %d", sig2->wave_form);
}


/**
 * @brief ��LCD��Ļ����������ʾ����
 * @param title Ҫ��ʾ�ı����ַ���
 * @param y_pos ������Y���ϵ�λ�ã���ѡ��Ĭ�Ͻ���10-20��
 * @retval None
 */
void LCD_Display_Title_Center(const char* title, uint16_t y_pos)
{
    // ��Ļ�ߴ綨��
    #define SCREEN_WIDTH  240
    #define SCREEN_CENTER_X (SCREEN_WIDTH / 2)  // 120
    
    // �������
    #define FONT_WIDTH  8   // 16x8������ַ����
    #define FONT_HEIGHT 16  // 16x8������ַ��߶�
    
    // �����ַ�������
    uint16_t str_len = strlen(title);
    
    // �����ַ��������ؿ��
    uint16_t total_width = str_len * FONT_WIDTH;
    
    // ������ʼX����(ȷ���ַ������Ķ��뵽��Ļ����)
    uint16_t start_x = SCREEN_CENTER_X - (total_width / 2);
    
    // �߽��飬ȷ���ַ������ᳬ����Ļ�߽�
    if(start_x > SCREEN_WIDTH) start_x = 0;  // ��ֹ����
    if(start_x + total_width > SCREEN_WIDTH) start_x = SCREEN_WIDTH - total_width;
    
    // ������ʾ��ɫ
    POINT_COLOR = BLACK;    // ��ɫ����
    BACK_COLOR = WHITE;     // ��ɫ����

    // ��ʾ�����ַ���
    LCD_ShowString(start_x, y_pos, total_width, FONT_HEIGHT, 16, (uint8_t*)title);
}

void DDS_Output(Signal_t *sig1, Signal_t *sig2)
{
    AD9833_1_Init();
    AD9833_2_Init();

    AD9833_1_Set_Frequency(sig1->freq);
    AD9833_2_Set_Frequency(sig2->freq);

    AD9833_1_Set_Amplitude(sig1->amp);
    AD9833_2_Set_Amplitude(sig2->amp);
}