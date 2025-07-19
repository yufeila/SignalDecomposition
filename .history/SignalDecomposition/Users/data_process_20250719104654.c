#include "data_process.h"
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>     // ����sprintf
#include "adc.h"


/* ------------ ȫ��/��̬ ------------------- */
/* �ڲ���̬���������� FFT/Goertzel ����һ�� ADC ���� */
static float adc_zeroed[BUF_SIZE];     /* ȥֱ�� & �����ѹ��ĸ������� */
Signal_t sig1, sig2;
uint32_t FTW1_cur = 0, FTW2_cur = 0;
static float    pllA_int = 0, pllB_int = 0;
float freq_tunning_A_Sinus[TUNNING_SIZE_A]={8.4,12.1,11.255855,10.3,9.5,8.7,12.5,11.6};
float freq_tunning_B_Sinus[TUNNING_SIZE_B]={13.05,12.55,11.9,11.3,10.8,4.9,4.3,3.7};
float freq_tunning_A_Triangle[TUNNING_SIZE_A]={8.4,12.1,11.255855,10.3,9.5,8.7,12.5,11.6};
float freq_tunning_B_Triangle[TUNNING_SIZE_B]={13.05,12.55,11.9,11.3,10.8,4.9,4.3,3.7};
PhaseConfig_t phase_config;


/* Ƶ�ʲ����˲���״̬ */
static float fA_filtered = 40000.0f;
static float fApr_filtered = 40000.0f;
static float fB_filtered = 70000.0f;
static float fBpr_filtered = 70000.0f;
static uint8_t filter_init = 0;

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;

extern volatile uint16_t calibration_buffer_A[CALIBRATION_BUF_SIZE];
extern volatile uint8_t calibration_buffer_A_ready;
extern volatile uint16_t calibration_buffer_B[CALIBRATION_BUF_SIZE];
extern volatile uint8_t calibration_buffer_B_ready;

static void DemuxADCData(const uint16_t *src,
                  float *buf1,
                  float *buf2,
                  uint16_t len);
int find_zero_crossings(const float *x, int N, float *zc_idx, int max_zc);
static inline void AD9833_WriteFTW1(uint32_t ftw);
static inline void AD9833_WriteFTW2(uint32_t ftw);

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

	printf("\r\n");

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

	/* ------- ͨ�� 1 ------- */
    if (sig1->wave_form == SINC_WAVE)
    {
        if(fabs(sig1->freq - 20000) <= 400)
        {
            AD9833_1_Config(sig1->freq + freq_tunning_A_Sinus[0], AD9833_OUT_SINUS);
        }
        else if(fabs(sig1->freq - 30000) <= 400)
        {
            AD9833_1_Config(sig1->freq + freq_tunning_A_Sinus[1], AD9833_OUT_SINUS);
        }
        else if(fabs(sig1->freq - 40000) <= 400)
        {
            AD9833_1_Config(sig1->freq + freq_tunning_A_Sinus[2], AD9833_OUT_SINUS);
        }
        else if(fabs(sig1->freq - 50000) <= 400)
        {
            AD9833_1_Config(sig1->freq + freq_tunning_A_Sinus[3], AD9833_OUT_SINUS);
        }
        else if(fabs(sig1->freq - 60000) <= 400)
        {
            AD9833_1_Config(sig1->freq + freq_tunning_A_Sinus[4], AD9833_OUT_SINUS);
        }
        else if(fabs(sig1->freq - 70000) <= 400)
        {
            AD9833_1_Config(sig1->freq + freq_tunning_A_Sinus[5], AD9833_OUT_SINUS);
        }
        else if(fabs(sig1->freq - 80000) <= 400)
    }
    else if (sig1->wave_form == TRIANGLE_WAVE)
    {
		if(fabs(sig1->freq - 20000) <= 400)
        {
			AD9833_1_Config(sig1->freq + freq_tunning_A_Triangle[0], AD9833_OUT_TRIANGLE);
        }
		else if(fabs(sig1->freq - 30000) <= 400)
			AD9833_1_Config(sig1->freq + freq_tunning_A_Triangle[1], AD9833_OUT_TRIANGLE);
		else if(fabs(sig1->freq - 40000) <= 400)
			AD9833_1_Config(sig1->freq + freq_tunning_A_Triangle[2], AD9833_OUT_TRIANGLE);
		else if(fabs(sig1->freq - 50000) <= 400)
			AD9833_1_Config(sig1->freq + freq_tunning_A_Triangle[3], AD9833_OUT_TRIANGLE);
		else if(fabs(sig1->freq - 60000) <= 400)
			AD9833_1_Config(sig1->freq + freq_tunning_A_Triangle[4], AD9833_OUT_TRIANGLE);
		else if(fabs(sig1->freq - 70000) <= 400)
			AD9833_1_Config(sig1->freq + freq_tunning_A_Triangle[5], AD9833_OUT_TRIANGLE);
		else if(fabs(sig1->freq - 80000) <= 400)
			AD9833_1_Config(sig1->freq + freq_tunning_A_Triangle[6], AD9833_OUT_TRIANGLE);
		else if(fabs(sig1->freq - 90000) <= 400)
			AD9833_1_Config(sig1->freq + freq_tunning_A_Triangle[7], AD9833_OUT_TRIANGLE);		
		else
			AD9833_1_Config(sig1->freq + FREQ_TUNNING_A, AD9833_OUT_TRIANGLE);
		
    }
	else
	{
		AD9833_1_Reset();
	}
	/* �����ʼ FTW1 */
	FTW1_cur = (uint32_t)((sig1->freq + FREQ_TUNNING_A) * 268435456.0f / ADCLK);

	/* ------- ͨ�� 2 ------- */
    if (sig2->wave_form == SINC_WAVE)
    {
        if(fabs(sig2->freq - 30000) <= 400)
        {
            AD9833_2_Config(sig2->freq + freq_tunning_B_Sinus[0], AD9833_OUT_SINUS);
        }
        else if(fabs(sig2->freq - 40000) <= 400)
        {
            AD9833_2_Config(sig2->freq + freq_tunning_B_Sinus[1], AD9833_OUT_SINUS);
        }
        else if(fabs(sig2->freq - 50000) <= 400)
        {
            AD9833_2_Config(sig2->freq + freq_tunning_B_Sinus[2], AD9833_OUT_SINUS);
        }
        else if(fabs(sig2->freq - 60000) <= 400)
        {
            AD9833_2_Config(sig2->freq + freq_tunning_B_Sinus[3], AD9833_OUT_SINUS);
        }
        else if(fabs(sig2->freq - 70000) <= 400)
        {
            AD9833_2_Config(sig2->freq + freq_tunning_B_Sinus[4], AD9833_OUT_SINUS);
        }
        else if(fabs(sig2->freq - 80000) <= 400)
        {
            AD9833_2_Config(sig2->freq + freq_tunning_B_Sinus[5], AD9833_OUT_SINUS);
        }
        else if(fabs(sig2->freq - 90000) <= 400)
        {
            AD9833_2_Config(sig2->freq + freq_tunning_B_Sinus[6], AD9833_OUT_SINUS);
        }
        else if(fabs(sig2->freq - 100000) <= 400)
        {
            AD9833_2_Config(sig2->freq + freq_tunning_B_Sinus[7], AD9833_OUT_SINUS);
        }
        else
        {
            AD9833_2_Config(sig2->freq + FREQ_TUNNING_B, AD9833_OUT_SINUS);
        }
    }
    else if (sig2->wave_form == TRIANGLE_WAVE)
    {
		if(fabs(sig2->freq - 30000) <= 400)
			AD9833_2_Config(sig2->freq + freq_tunning_B_Triangle[0], AD9833_OUT_TRIANGLE);
		else if(fabs(sig2->freq - 40000) <= 400)
			AD9833_2_Config(sig2->freq + freq_tunning_B_Triangle[1], AD9833_OUT_TRIANGLE);
		else if(fabs(sig2->freq - 50000) <= 400)
			AD9833_2_Config(sig2->freq + freq_tunning_B_Triangle[2], AD9833_OUT_TRIANGLE);
		else if(fabs(sig2->freq - 60000) <= 400)
			AD9833_2_Config(sig2->freq + freq_tunning_B_Triangle[3], AD9833_OUT_TRIANGLE);
		else if(fabs(sig2->freq - 70000) <= 400)
			AD9833_2_Config(sig2->freq + freq_tunning_B_Triangle[4], AD9833_OUT_TRIANGLE);
		else if(fabs(sig2->freq - 80000) <= 400)
			AD9833_2_Config(sig2->freq + freq_tunning_B_Triangle[5], AD9833_OUT_TRIANGLE);
		else if(fabs(sig2->freq - 90000) <= 400)
			AD9833_2_Config(sig2->freq + freq_tunning_B_Triangle[6], AD9833_OUT_TRIANGLE);
		else if(fabs(sig2->freq - 100000) <= 400)
			AD9833_2_Config(sig2->freq + freq_tunning_B_Triangle[7], AD9833_OUT_TRIANGLE);
		else
			AD9833_2_Config(sig2->freq + FREQ_TUNNING_B, AD9833_OUT_TRIANGLE);
		
    }
	else
	{
		AD9833_2_Reset();
	}
	FTW2_cur = (uint32_t)((sig2->freq + FREQ_TUNNING_B) * 268435456.0f / ADCLK);

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

void StartSampling(void)
{
    /* 1. �� DMA ��־ */
    __HAL_DMA_CLEAR_FLAG(&hdma_adc2, DMA_FLAG_TCIF0_4);
    __HAL_DMA_CLEAR_FLAG(&hdma_adc3, DMA_FLAG_TCIF0_4);

    /* 2. ���� ADC+DMA (NORMAL) */
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)calibration_buffer_A, CALIBRATION_BUF_SIZE);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)calibration_buffer_B, CALIBRATION_BUF_SIZE);

    /* 3. �� TIM8 ����һ�� TRGO ���� */
    HAL_TIM_Base_Start(&htim8);                // TIM8 �����ô���һ�� N ��
}

// �����µĿ�����������״̬����
static void FLL_Controller_Update(float error, uint32_t *ftw, FLL_PI_Controller_t *controller);

// Ϊͨ��A��B�ֱ��������״̬
static FLL_PI_Controller_t fll_controller_A;
static FLL_PI_Controller_t fll_controller_B;

/* ������Ϊ��ȷƵ�ʲ���������λ������ */
static Phase_Tracker_t phase_tracker_A;
static Phase_Tracker_t phase_tracker_Apr;
static Phase_Tracker_t phase_tracker_B;
static Phase_Tracker_t phase_tracker_Bpr;

/* ��������֡Ƶ��ƽ���� */
static Freq_Averager_t freq_averager_A;
static Freq_Averager_t freq_averager_Apr;
static Freq_Averager_t freq_averager_B;
static Freq_Averager_t freq_averager_Bpr;
static uint32_t frame_counter = 0;  /* ȫ��֡������ */

/**
 * @brief ��Ƶ��ƽ��������µ�Ƶ�ʲ���ֵ
 * @param averager Ƶ��ƽ����ָ��
 * @param new_freq �µ�Ƶ�ʲ���ֵ
 * @return ��������������򷵻�1�����򷵻�0
 */
uint8_t add_frequency_measurement(Freq_Averager_t *averager, float new_freq)
{
    /* ����Ƶ����ӵ������� */
    averager->freq_buffer[averager->frame_count] = new_freq;
    averager->frame_count++;
    
    /* ��黺�����Ƿ����� */
    if (averager->frame_count >= FREQ_AVERAGE_FRAMES) {
        averager->buffer_full = 1;
        averager->frame_count = 0;  /* ���ü�������ѭ��ʹ�û����� */
        
        /* ����ƽ��Ƶ�� */
        float sum = 0.0f;
        for (uint8_t i = 0; i < FREQ_AVERAGE_FRAMES; i++) {
            sum += averager->freq_buffer[i];
        }
        averager->averaged_freq = sum / FREQ_AVERAGE_FRAMES;
        
        return 1;  /* ����������������ʹ��ƽ��ֵ */
    }
    
    return 0;  /* ������δ���������ռ����� */
}

/**
 * @brief ʹ��Goertzel + ��λ΢�ַ����о�ȷƵ�ʲ���
 * @param signal_data �����ź����� (��ȥֱ��)
 * @param N ���ݵ���
 * @param target_freq Ŀ��Ƶ�� (Hz) - ���Դֲ���
 * @param tracker ��λ������״̬
 * @return ��ȷ��Ƶ�ʹ���ֵ (Hz)
 */
float precise_frequency_measurement(const float *signal_data, int N, float target_freq, Phase_Tracker_t *tracker)
{
    /* 1. ʹ��Goertzel�㷨����Ŀ��Ƶ�ʵĸ������ */
    goertzel_cfg_fomega_t goertzel_cfg;
    float magnitude, phase;
    
    // ��ʼ��Goertzel����
    goertzel_init_fomega(&goertzel_cfg, N, target_freq, CALIBRATION_SAMPLE_FREQ);
    
    // ִ��Goertzel�任���õ����Ⱥ���λ
    goertzel_process_f32omega(&goertzel_cfg, signal_data, &magnitude, &phase);
    
    /* 2. ��λ������Ƶ�ʹ��� */
    float freq_estimate = target_freq; // Ĭ�Ϸ���Ŀ��Ƶ��
    
    if (tracker->phase_valid) {
        // ������λ��
        float delta_phase = phase - tracker->last_phase;
        
        // ������λ��Ծ (phase wrapping)
        while (delta_phase > 3.14159265f) {
            delta_phase -= 2.0f * 3.14159265f;
        }
        while (delta_phase < -3.14159265f) {
            delta_phase += 2.0f * 3.14159265f;
        }
        
        // ������λ����㾫ȷƵ��
        // f_est = f0 + ���ա�Fs/(2�С�N)
        float freq_correction = delta_phase * CALIBRATION_SAMPLE_FREQ / (2.0f * 3.14159265f * N);
        freq_estimate = target_freq + freq_correction;
        
        // �򵥵�IIR�˲�ƽ��Ƶ�ʹ��� - ����alphaʹ��ʷ�����и���Ȩ�أ�ϵͳ��ƽ��
        const float alpha = 0.4f; 
        tracker->freq_estimate = tracker->freq_estimate * (1.0f - alpha) + freq_estimate * alpha;
        freq_estimate = tracker->freq_estimate;
        
    } else {
        // ��һ�����У���ʼ��
        tracker->freq_estimate = target_freq;
        tracker->phase_valid = 1;
    }
    
    // ������λ��ʷ
    tracker->last_phase = phase;
    
    return freq_estimate;
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
    /* ͣ TIM, �� CNT, ���� */
	HAL_TIM_Base_Stop(&htim8);
	__HAL_TIM_SET_COUNTER(&htim8, 0);   // �ؼ����� CNT ����
	HAL_TIM_Base_Start(&htim8);         // ��һ֡�� 0 ��ʼ


    // 2. �ȴ��������ݲɼ����
    uint32_t timeout = 0;
    while (!(calibration_buffer_A_ready && calibration_buffer_B_ready) && timeout < 1000)
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
        return;
    }

    // 4. ֹͣ��ǰ�ִε�ADC+TIM��Ϊ�´�������׼����
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADC_Stop_DMA(&hadc3);
    HAL_TIM_Base_Stop(&htim8);

    // ���URSλ���ָ�Ĭ����Ϊ
    htim8.Instance->CR1 &= ~TIM_CR1_URS;

    // 5. �����ֲɼ�������
    static float buf_A[CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE];
    static float buf_Apr[CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE];
    static float buf_B[CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE];
    static float buf_Bpr[CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE];
    DemuxADCData((const uint16_t *)calibration_buffer_A, buf_A, buf_Apr, CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE);
    DemuxADCData((const uint16_t *)calibration_buffer_B, buf_B, buf_Bpr, CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE);


    // 6. �㽻���
    static float zc_idx_A[MAX_ZC];
    static float zc_idx_Apr[MAX_ZC];
    static float zc_idx_B[MAX_ZC];
    static float zc_idx_Bpr[MAX_ZC];

    // ������һ�ε���ʱ���㣬��ֹʹ���ϴβ�������
    static uint8_t is_first_run = 1;
    if (is_first_run) {
        memset(zc_idx_A, 0, sizeof(zc_idx_A));
        memset(zc_idx_Apr, 0, sizeof(zc_idx_Apr));
        memset(zc_idx_B, 0, sizeof(zc_idx_B));
        memset(zc_idx_Bpr, 0, sizeof(zc_idx_Bpr));
        is_first_run = 0;
    }

    uint16_t na = find_zero_crossings(buf_A, CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE, zc_idx_A, MAX_ZC);
    uint16_t nap = find_zero_crossings(buf_Apr, CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE, zc_idx_Apr, MAX_ZC);
    uint16_t nb = find_zero_crossings(buf_B, CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE, zc_idx_B, MAX_ZC);
    uint16_t nbp = find_zero_crossings(buf_Bpr, CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE, zc_idx_Bpr, MAX_ZC);

	if(na <= MAVG || nap <= MAVG || nb <= MAVG || nbp <= MAVG) return; /* ���ݲ��� */

    /* 2. ��Ƶ�ʼ��㣺ƽ�� MAVG ���� (����ȷ������Ƶ��) */
	float Ta   = (zc_idx_A [na-1]  - zc_idx_A [na-1-MAVG ]) / MAVG / CALIBRATION_SAMPLE_FREQ;
    float Tap  = (zc_idx_Apr[nap-1]- zc_idx_Apr[nap-1-MAVG]) / MAVG / CALIBRATION_SAMPLE_FREQ;
	float Tb  = (zc_idx_B [nb-1]  - zc_idx_B [nb-1-MAVG ]) / MAVG / CALIBRATION_SAMPLE_FREQ;
	float Tbp = (zc_idx_Bpr[nbp-1]- zc_idx_Bpr[nbp-1-MAVG]) / MAVG / CALIBRATION_SAMPLE_FREQ;

    float fA_coarse   = 1.0f/Ta;
    float fApr_coarse = 1.0f/Tap;
    float fB_coarse   = 1.0f/Tb;
    float fBpr_coarse = 1.0f/Tbp;
	
	printf("Coarse freq: fA=%.2f, fApr=%.2f, fB=%.2f, fBpr=%.2f Hz\r\n", 
           fA_coarse, fApr_coarse, fB_coarse, fBpr_coarse);

    /* 3. ��ȷƵ�ʲ�����ʹ��Goertzel��λ΢�ַ� */
    float fA_precise   = precise_frequency_measurement(buf_A,   CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE, fA_coarse,   &phase_tracker_A);
    float fApr_precise = precise_frequency_measurement(buf_Apr, CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE, fApr_coarse, &phase_tracker_Apr);
    float fB_precise   = precise_frequency_measurement(buf_B,   CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE, fB_coarse,   &phase_tracker_B);
    float fBpr_precise = precise_frequency_measurement(buf_Bpr, CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE, fBpr_coarse, &phase_tracker_Bpr);

    printf("Precise freq: fA=%.3f, fApr=%.3f, fB=%.3f, fBpr=%.3f Hz\r\n", 
           fA_precise, fApr_precise, fB_precise, fBpr_precise);
	printf("Zero crossings: na=%d, nap=%d, nb=%d, nbp=%d\r\n", na, nap, nb, nbp);
    
    /* 4. ��Ƶ�ʲ���ֵ��ӵ���֡ƽ�������� */
    uint8_t avgA_ready   = add_frequency_measurement(&freq_averager_A,   fA_precise);
    uint8_t avgApr_ready = add_frequency_measurement(&freq_averager_Apr, fApr_precise);
    uint8_t avgB_ready   = add_frequency_measurement(&freq_averager_B,   fB_precise);
    uint8_t avgBpr_ready = add_frequency_measurement(&freq_averager_Bpr, fBpr_precise);
    
    frame_counter++;
    printf("Frame %lu: Collected frequency data\r\n", frame_counter);

    /* 5. ֻ�е�����ͨ����ƽ��������������ʱ���Ž���FLL���������� */
    if (avgA_ready && avgApr_ready && avgB_ready && avgBpr_ready) {
        
        /* ʹ��ƽ�����Ƶ��ֵ */
        float fA_avg   = freq_averager_A.averaged_freq;
        float fApr_avg = freq_averager_Apr.averaged_freq;
        float fB_avg   = freq_averager_B.averaged_freq;
        float fBpr_avg = freq_averager_Bpr.averaged_freq;
        
        printf("=== AVERAGED FREQUENCIES (Frame %lu) ===\r\n", frame_counter);
        printf("Averaged freq: fA=%.4f, fApr=%.4f, fB=%.4f, fBpr=%.4f Hz\r\n", 
               fA_avg, fApr_avg, fB_avg, fBpr_avg);

        /* 6. Ƶ����㣺ʹ��ƽ�����Ƶ�� */
        float dfA = fA_avg - fApr_avg;
        float dfB = fB_avg - fBpr_avg;
        
        printf("Averaged frequency errors: dfA = %.5f Hz, dfB = %.5f Hz\r\n", dfA, dfB);
        
//        /* 7. ����FLL������ */
//        FLL_Controller_Update(dfA, &FTW1_cur, &fll_controller_A);
//        FLL_Controller_Update(dfB, &FTW2_cur, &fll_controller_B);
        
        printf("=== FLL CONTROLLER UPDATED ===\r\n");
    } else {
        printf("Collecting data... A:%d Apr:%d B:%d Bpr:%d\r\n", 
               avgA_ready, avgApr_ready, avgB_ready, avgBpr_ready);
    }
	
	printf("\r\n");
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


// �㽻��⣨Ƕ��ʽ�棬��malloc/free��
// x: ����float���飬N:���鳤��
// zc_idx: �û��ṩ���㽻���buffer
// max_zc: buffer��������
// ���أ�ʵ�ʼ�⵽���㽻����
int find_zero_crossings(const float *x, int N, float *zc_idx, int max_zc)
{
    int count = 0;
    for (int i = 0; i < N - 1 && count < max_zc; ++i) {
        // ֻ��������㽻�㣨�Ӹ�����������MATLAB����һ��
        if (x[i] < 0 && x[i+1] >= 0) { // �Ӹ�����
            float frac = x[i] / (x[i] - x[i+1]);
            zc_idx[count++] = i + frac; // ����������
        }
    }
    return count;
}

//������� ftw��Frequency Tuning Word���� AD9833 �� 28 λƵ�ʿ�����
static inline void AD9833_WriteFTW1(uint32_t ftw)
{
    // FREQ0 ��14λ
    uint16_t freq0_lsb = AD9833_REG_FREQ0 | (ftw & 0x3FFF);
    // FREQ0 ��14λ
    uint16_t freq0_msb = AD9833_REG_FREQ0 | ((ftw >> 14) & 0x3FFF);

    AD9833_1_SetRegisterValue(freq0_lsb);
    AD9833_1_SetRegisterValue(freq0_msb);
}
//������� ftw��Frequency Tuning Word���� AD9833 �� 28 λƵ�ʿ�����
static inline void AD9833_WriteFTW2(uint32_t ftw)
{
    uint16_t freq0_lsb = AD9833_REG_FREQ0 | (ftw & 0x3FFF);
    uint16_t freq0_msb = AD9833_REG_FREQ0 | ((ftw >> 14) & 0x3FFF);

    AD9833_2_SetRegisterValue(freq0_lsb);
    AD9833_2_SetRegisterValue(freq0_msb);
}

/**
 * @brief FLL PI���������ĸ��º��� (��Anti-Windup) - �ع���
 * @param error ��ǰƵ����� (df)
 * @param ftw   ָ��ǰDDSƵ���ֵ�ָ��
 * @param controller ָ�������״̬��ָ��
 */
static void FLL_Controller_Update(float error, uint32_t *ftw, FLL_PI_Controller_t *controller)
{
    /* ���У׼��ֹͣ����ֱ�ӷ��� */
    if (controller->calibration_stopped) {
        return;
    }

    /* ���Ƶ���Ƿ��ȶ� */
    if (fabsf(error) < FLL_STABLE_THRESHOLD_HZ) {
        controller->stable_counter++;
    } else {
        controller->stable_counter = 0; // ������ȶ������ü�����
    }

    /* ����ﵽ�ȶ���������ֹͣУ׼ */
    if (controller->stable_counter >= STABLE_FRAME_COUNT) {
        controller->calibration_stopped = 1;
        printf(">>> FLL for controller %s has stopped due to stability. <<<\n",
               (controller == &fll_controller_A) ? "A" : "B");
        return; // ֹͣ���θ���
    }

    float p_term = FLL_KP * error; // ���������

    /* 1. �����жϣ��������ڣ����ñ���(P)�����������(I)�����������̬��� */
    if (fabsf(error) < FLL_DEAD_ZONE_HZ) {
        p_term = 0.0f;
    }
    
    /* 1.1. ���������ã�����Сʱ�����û�������ֹ����Ư�� */
    if (fabsf(error) < FLL_RESET_THRESHOLD_HZ) {
        controller->integrator = 0.0f;
    }
    
    /* 2. ���� PI ��� */
    float output = p_term + controller->integrator;
    
    /* 3. ����޷� (����) */
    float output_saturated = output;
    if (output_saturated > FLL_MAX_STEP_HZ) {
        output_saturated = FLL_MAX_STEP_HZ;
    } else if (output_saturated < -FLL_MAX_STEP_HZ) {
        output_saturated = -FLL_MAX_STEP_HZ;
    }
    
    /* 4. ���������� (�� Anti-Windup) */
    // �����������ۻ�ԭʼ���(FLL_KI * error)��
    // ���������޷�������������ȥ�����Ĳ���(output - output_saturated)��
    // �Ӷ���ֹ�������ڱ���״̬�¼����ۻ���
    controller->integrator += FLL_KI * error + FLL_ANTI_WINDUP_GAIN * (output_saturated - output);
    
    /* 5. ������Ӳ�޷� - ��ֹ�����������ۻ�������� */
    if (controller->integrator > FLL_INTEGRATOR_MAX) {
        controller->integrator = FLL_INTEGRATOR_MAX;
    } else if (controller->integrator < FLL_INTEGRATOR_MIN) {
        controller->integrator = FLL_INTEGRATOR_MIN;
    }

    /* 6. ���� DDS Ƶ���� */
    int32_t dFTW = (int32_t)(output_saturated * (268435456.0f / ADCLK));
    *ftw += dFTW;
    
    // �������ĸ�ͨ���Ŀ����������ö�Ӧ��д����
    if (controller == &fll_controller_A) {
        AD9833_WriteFTW1(*ftw);
    } else {
        AD9833_WriteFTW2(*ftw);
    }

    printf("FLL_Update: err=%.2f, Kp=%.2f, Ki=%.2f, out=%.2f, sat_out=%.2f, int=%.2f,p_term = %.2f\n",
           error, FLL_KP, FLL_KI, output, output_saturated, controller->integrator,p_term);
}
