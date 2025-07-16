#include "data_process.h"
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>     // ����sprintf
#include "adc.h"

/* �ڲ���̬���������� FFT/Goertzel ����һ�� ADC ���� */
static float adc_zeroed[BUF_SIZE];     /* ȥֱ�� & �����ѹ��ĸ������� */
uint16_t calibration_flag = 0;      /* Ƶ��У׼��־λ */

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;

extern volatile uint16_t calibration_buffer_A[CALIBRATION_BUF_SIZE];
extern volatile uint8_t calibration_buffer_A_ready;
extern volatile uint16_t calibration_buffer_B[CALIBRATION_BUF_SIZE];
extern volatile uint8_t calibration_buffer_B_ready;

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
    }

    // 4. ֹͣ��ǰ�ִε�ADC+TIM��Ϊ�´�������׼����
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_TIM_Base_Stop(&htim2);

    // ���URSλ���ָ�Ĭ����Ϊ
    htim2.Instance->CR1 &= ~TIM_CR1_URS;

    // 5. �����ֲɼ�������
    const uint16_t *src = (const uint16_t *)&adc_buffer[0];

    // 6. ���������źŲ���
    
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
	fft_top5_hann_zero_nointp(adc_zeroed);

    /* 2. FFT �ҵ��������� */
    /* �ȵõ�����������Ƶ�ʺͷ��� */
    float f1, A1, f2, A2;
    fft_top2_hann_zero_interp(adc_zeroed, &f1, &A1, &f2, &A2);
	
	/* ���ȸ����� 2 λС�����ɰ���Ҫ�ĳ� %.1f��%.3f �� */
	printf("F1 = %.2f Hz,  A1 = %.2f,  F2 = %.2f Hz,  A2 = %.2f\r\n",
    f1, A1, f2, A2);

	#define SINC_BASE_MAG 0.45

    /* --- helper lambda��C99 ��ģ�⣩--- */
    #define ANALYSE_ONE(_f_base, _A_base, _f_other, _A_other, _sig_out)               \
    do {                                                                              \
        goertzel_cfg_fomega_t g1, g3, g5;                                             \
        float mag1, ph1, mag3, ph3, mag5, ph5;                                        \
		/* -------- һ���԰� 1��/3��/5�� ��ֵ����� -------- */                        \
        goertzel_init_fomega(&g1, FFT_SIZE, 1.0f * (_f_base),  FS_HZ);                \
        goertzel_init_fomega(&g3, FFT_SIZE, 3.0f * (_f_base),  FS_HZ);                \
        goertzel_init_fomega(&g5, FFT_SIZE, 5.0f * (_f_base),  FS_HZ);                \
        goertzel_process_f32omega(&g1, adc_zeroed, &mag1, &ph1);                      \
        goertzel_process_f32omega(&g3, adc_zeroed, &mag3, &ph3);                      \
        goertzel_process_f32omega(&g5, adc_zeroed, &mag5, &ph5);                      \
		printf("f_base = %.4f,f_other = %.4f, delta_f = %.4f\r\n",_f_base, _f_other, 3*_f_base - _f_other);\
        printf("mag3:%.3f(before half bin cutoff)\r\n", mag3);                            \
        /* -------- �� 3�� �� 5�� ����һ�����ص��������ȵ��� -------- */                \
        const float EPS = 2.0f;   /* half-bin ���� */                                  \
        if (fabsf(3.0f*(_f_base) - (_f_other)) < EPS)  mag3 = fabsf(mag3 - _A_other);  \
		printf("f = %.2f ,mag1 = %.3f ,mag3 = %.3f, mag5 = %.3f\r\n", _f_base,mag1,mag3, mag5);\
        float r3 = mag3 / mag1;                                                        \
        float r5 = mag5 / mag1;                                                        \
        printf("f = %.2f ,r3 = %.3f, r5 = %.3f\r\n", _f_base,r3, r5);				\
        (_sig_out)->freq = (_f_base);                                                  \
        if (r3 < 0.04f && r5 < 0.02f) {                                                \
            (_sig_out)->wave_form = SINC_WAVE;                                         \
        } else if ((fabsf(r3 - 0.1111f) < 0.03f && fabsf(r5 - 0.0400f) < 0.02f)||_A_base <= SINC_BASE_MAG) {       \
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
	LCD_Clear(WHITE);
    LCD_Display_Title_Center("Signal Info", 10);
    
	char buf[32];
	
    // ��ʾ�ź�1��Ƶ�ʺ�����
    LCD_ShowString(10, 30, 200, 16, 16, (uint8_t*)"Signal 1:");
	sprintf(buf, "Frequency: %.2f Hz", sig1->freq);
    LCD_ShowString(10, 50, 200, 16, 16, (uint8_t*)buf);
	sprintf(buf, "Type: %d", sig1->wave_form);
    LCD_ShowString(10, 70, 200, 16, 16, (uint8_t*)buf);

    // ��ʾ�ź�2��Ƶ�ʺ�����
    LCD_ShowString(10, 90, 200, 16, 16, (uint8_t*)"Signal 2:");
	sprintf(buf, "Frequency: %.2f Hz", sig2->freq);
    LCD_ShowString(10, 110, 200, 16, 16, (uint8_t*)buf);
	sprintf(buf, "Type: %d", sig2->wave_form);
    LCD_ShowString(10, 130, 200, 16, 16, (uint8_t*)buf);
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
    AD9833_1_GPIO_Init();
    AD9833_2_GPIO_Init();

    if (sig1->wave_form == SINC_WAVE)
    {
        AD9833_1_Config(sig1->freq + FREQ_TUNNING, AD9833_OUT_SINUS);
    }
    else if (sig1->wave_form == TRIANGLE_WAVE)
    {
        AD9833_1_Config(sig1->freq + FREQ_TUNNING, AD9833_OUT_TRIANGLE);
    }
	else
	{
		AD9833_1_Reset();
	}

    if (sig2->wave_form == SINC_WAVE)
    {
        AD9833_2_Config(sig2->freq + FREQ_TUNNING, AD9833_OUT_SINUS);
    }
    else if (sig2->wave_form == TRIANGLE_WAVE)
    {
        AD9833_2_Config(sig2->freq + FREQ_TUNNING, AD9833_OUT_TRIANGLE);
    }
	else
	{
		AD9833_2_Reset();
	}

}

/* ���ڽ�����Ϣģ�� */
uint8_t get_message(uint8_t *buf, uint16_t len, uint16_t *p_deg)
{
    /* 1. ���Ȳ��ܳ��� 3��"180"�� */
    if (len == 0 || len > 3) return 0;

    /* 2. �����ַ����������� */
    for (uint16_t i = 0; i < len; i++) {
        if (!isdigit(buf[i])) return 0;
    }

    /* 3. ���� + ת���� */
    char tmp[4] = {0};        // len �� [1,3], ��һ�� '\0'
    memcpy(tmp, buf, len);
    uint32_t val = strtoul(tmp, NULL, 10);
    if (val > 180) return 0;

    *p_deg = (uint16_t)val;
    return 1;
}

// ��غ�������
//void X9C_Init(void);
//void X9C103_SetResistance(float resistance);
//void X9C503_SetResistance(float resistance);

void config_digital_potentiometer(uint16_t deg)
{
    float r1_103 = 0;
    float r2_503 = 0;

    /* ������λ�Ǽ������ֵ */

    X9C_Init();
    X9C103_SetResistance(r1_103);
    X9C503_SetResistance(r2_503);
}

void Calibration_Frequency(void)
{
	// 1. ��������ADC+DMA+TIM�ɼ��������޸�����ʱ��
    calibration_buffer_A_ready = 0;
    calibration_buffer_B_ready = 0;

    // ������ܵ�DMA��־λ
    __HAL_DMA_CLEAR_FLAG(&hdma_adc2, DMA_FLAG_TCIF0_4);
    __HAL_DMA_CLEAR_FLAG(&hdma_adc3, DMA_FLAG_TCIF0_4);

    // ������ADC+DMA��ȷ��׼������
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*)calibration_buffer_A, CALIBRATION_BUF_SIZE) != HAL_OK)
    {
        Error_Handler();
    }       

    if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*)calibration_buffer_B, CALIBRATION_BUF_SIZE) != HAL_OK)
    {
        Error_Handler();
    }       

    // ������ʱ������һ��TRGO�����������ں���
    HAL_TIM_Base_Start(&htim8);

    // 2. �ȴ��������ݲɼ����
    uint32_t timeout = 0;
    while (calibration_buffer_A_ready != 1 && calibration_buffer_B_ready != 1 && timeout < 1000)
    {
        HAL_Delay(1);  // ������ʱ������CPUռ�ù���
        timeout++;
    }

    // 3. ����Ƿ�ʱ
    if (timeout >= 1000)
    {
        // ��ʱ����ֹͣADC+TIM��������������
        HAL_ADC_Stop_DMA(&hadc2);
        HAL_ADC_Stop_DMA(&hadc3);
        HAL_TIM_Base_Stop(&htim8);
    }

    // 4. ֹͣ��ǰ�ִε�ADC+TIM��Ϊ�´�������׼����
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADC_Stop_DMA(&hadc3);
    HAL_TIM_Base_Stop(&htim8);

    // ���URSλ���ָ�Ĭ����Ϊ
    htim8.Instance->CR1 &= ~TIM_CR1_URS;

    // 5. �����ֲɼ�������
    static float buf1_A[CALIBRATION_BUF_SIZE];
    static float buf2_A[CALIBRATION_BUF_SIZE];
    static float buf1_B[CALIBRATION_BUF_SIZE];
    static float buf2_B[CALIBRATION_BUF_SIZE];
    DemuxADCData(calibration_buffer_A, buf1_A, buf2_A, CALIBRATION_BUF_SIZE);
    DemuxADCData(calibration_buffer_B, buf1_B, buf2_B, CALIBRATION_BUF_SIZE);

    // 6. FFT����У׼
}

/**
 * @brief ����ADC���ݵ�2���������У�������ѹת����
 * @param src ָ��Դ���������ָ�룬��������ͨ����ADC���ݡ�
 * @param buf1 ָ��ͨ��1���ݻ�������ָ�롣
 * @param buf2 ָ��ͨ��2���ݻ�������ָ�롣
 * @param len ÿ���������ĳ��ȣ���ÿ��ͨ�������ݵ���������
 */
static void DemuxADCData(const uint16_t *src,
                  float *buf1,
                  float *buf2,
                  uint16_t len)
{
    // ��һ����ת��Ϊ��ѹ������ƽ��ֵ��DC������
    float sum1 = 0.0f, sum2 = 0.0f;
    
    for(uint16_t i = 0; i < len; i++)
    {
        float volt1 = (float)src[2*i] * ADC_LSB_VOLT;
        float volt2 = (float)src[2*i+1] * ADC_LSB_VOLT;
        
        buf1[i] = volt1;
        buf2[i] = volt2;
        
        sum1 += volt1;
        sum2 += volt2;
    }
    
    // �ڶ�����ΪACͨ��ȥ��DC������FFT��Ҫ��0Ϊ���ĵ��źţ�
    float dc1 = sum1 / (float)len;
    float dc2 = sum2 / (float)len;
    
    for(uint16_t i = 0; i < len; i++)
    {
        buf1[i] -= dc1;  // ȥ��DC�������õ���AC�ź�
        buf2[i] -= dc2;  // ȥ��DC�������õ���AC�ź�
        // buf3���ֲ��䣬��Ϊ������DC�ź�
    }
}


