#include "data_process.h"
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>     // 用于sprintf
#include "adc.h"


/* ------------ 全局/静态 ------------------- */
/* 内部静态缓冲区，与 FFT/Goertzel 共用一份 ADC 数据 */
static float adc_zeroed[BUF_SIZE];     /* 去直流 & 换算电压后的浮点序列 */
Signal_t sig1, sig2;
uint32_t FTW1_cur = 0, FTW2_cur = 0;
static float    pllA_int = 0, pllB_int = 0;
float freq_tunning_A_Sinus[TUNNING_SIZE_A]={8.4,12.1,11.255855,10.3,9.5,8.7,12.5,11.6};
float freq_tunning_B_Sinus[TUNNING_SIZE_B]={13.05,12.55,11.9,11.3,10.8,4.9,4.3,3.7};
float freq_tunning_A_Triangle[TUNNING_SIZE_A]={8.4,12.1,11.255855,10.3,9.5,8.7,12.5,11.6};
float freq_tunning_B_Triangle[TUNNING_SIZE_B]={13.05,12.55,11.9,11.3,10.8,4.9,4.3,3.7};
PhaseConfig_t phase_config;


/* 频率测量滤波器状态 */
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
    // 1. 启动本轮ADC+DMA+TIM采集（彻底修复启动时序）
    ADC_BufferReadyFlag = BUFFER_READY_FLAG_NONE;

    // 清除可能的DMA标志位
    __HAL_DMA_CLEAR_FLAG(&hdma_adc1, DMA_FLAG_TCIF0_4);

    // 先启动ADC+DMA，确保准备就绪
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, BUF_SIZE) != HAL_OK)
    {
        Error_Handler();
    }       

    // 启动定时器，第一次TRGO将在完整周期后发生
    HAL_TIM_Base_Start(&htim2);

    // 2. 等待本轮数据采集完成
    uint32_t timeout = 0;
    while (ADC_BufferReadyFlag != BUFFER_READY_FLAG_FULL && timeout < 1000)
    {
        HAL_Delay(1);  // 短暂延时，避免CPU占用过高
        timeout++;
    }

    // 3. 检查是否超时
    if (timeout >= 1000)
    {
        // 超时处理：停止ADC+TIM，返回无新数据
        HAL_ADC_Stop_DMA(&hadc1);
        HAL_TIM_Base_Stop(&htim2);
    }

    // 4. 停止当前轮次的ADC+TIM（为下次启动做准备）
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_TIM_Base_Stop(&htim2);

    // 清除URS位，恢复默认行为
    htim2.Instance->CR1 &= ~TIM_CR1_URS;

    // 5. 处理本轮采集的数据
    const uint16_t *src = (const uint16_t *)&adc_buffer[0];

    // 6. 生成两个信号参数
    
    analyse_two_signals(src, &sig1, &sig2);
    
    // 7. 显示数据分析结果
    Signal_Info_Display(&sig1, &sig2);

    // 8. DDS模块生成信号
    DDS_Output(&sig1, &sig2);

	printf("\r\n");

}

/* _buf: ADC 原始 uint16[]，长度至少 FFT_SIZE */
void analyse_two_signals(const uint16_t *_buf, Signal_t *sig1, Signal_t *sig2)
{
    /* 1. 去零偏 & 换算为电压 */
    adc_zero_bias(_buf, adc_zeroed, FFT_SIZE);
	fft_top5_hann_zero_nointp(adc_zeroed);

    /* 2. FFT 找到两个基波 */
    /* 先得到两个基波的频率和幅度 */
    float f1, A1, f2, A2;
    fft_top2_hann_zero_interp(adc_zeroed, &f1, &A1, &f2, &A2);
	
	/* 精度各保留 2 位小数，可按需要改成 %.1f、%.3f 等 */
	printf("F1 = %.2f Hz,  A1 = %.2f,  F2 = %.2f Hz,  A2 = %.2f\r\n",
    f1, A1, f2, A2);

	#define SINC_BASE_MAG 0.45

    /* --- helper lambda（C99 宏模拟）--- */
    #define ANALYSE_ONE(_f_base, _A_base, _f_other, _A_other, _sig_out)               \
    do {                                                                              \
        goertzel_cfg_fomega_t g1, g3, g5;                                             \
        float mag1, ph1, mag3, ph3, mag5, ph5;                                        \
		/* -------- 一次性把 1×/3×/5× 幅值算出来 -------- */                        \
        goertzel_init_fomega(&g1, FFT_SIZE, 1.0f * (_f_base),  FS_HZ);                \
        goertzel_init_fomega(&g3, FFT_SIZE, 3.0f * (_f_base),  FS_HZ);                \
        goertzel_init_fomega(&g5, FFT_SIZE, 5.0f * (_f_base),  FS_HZ);                \
        goertzel_process_f32omega(&g1, adc_zeroed, &mag1, &ph1);                      \
        goertzel_process_f32omega(&g3, adc_zeroed, &mag3, &ph3);                      \
        goertzel_process_f32omega(&g5, adc_zeroed, &mag5, &ph5);                      \
		printf("f_base = %.4f,f_other = %.4f, delta_f = %.4f\r\n",_f_base, _f_other, 3*_f_base - _f_other);\
        printf("mag3:%.3f(before half bin cutoff)\r\n", mag3);                            \
        /* -------- 若 3× 或 5× 与另一基波重叠，做幅度抵消 -------- */                \
        const float EPS = 2.0f;   /* half-bin 宽容 */                                  \
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
            (_sig_out)->wave_form = 0; /* 未知/混合，可 log */                         \
        }                                                                              \
    } while (0)


    /* 最大峰记为 sig1，第二峰记为 sig2 */
    ANALYSE_ONE(f1, A1, f2, A2, sig1);
    ANALYSE_ONE(f2, A2, f1, A1, sig2);
	
	
}

/* ===== 显示函数 ===== */
void Signal_Info_Display(Signal_t *sig1, Signal_t *sig2)
{
	LCD_Clear(WHITE);
    LCD_Display_Title_Center("Signal Info", 10);
    
	char buf[32];
	
    // 显示信号1的频率和类型
    LCD_ShowString(10, 30, 200, 16, 16, (uint8_t*)"Signal 1:");
	sprintf(buf, "Frequency: %.2f Hz", sig1->freq);
    LCD_ShowString(10, 50, 200, 16, 16, (uint8_t*)buf);
	sprintf(buf, "Type: %d", sig1->wave_form);
    LCD_ShowString(10, 70, 200, 16, 16, (uint8_t*)buf);

    // 显示信号2的频率和类型
    LCD_ShowString(10, 90, 200, 16, 16, (uint8_t*)"Signal 2:");
	sprintf(buf, "Frequency: %.2f Hz", sig2->freq);
    LCD_ShowString(10, 110, 200, 16, 16, (uint8_t*)buf);
	sprintf(buf, "Type: %d", sig2->wave_form);
    LCD_ShowString(10, 130, 200, 16, 16, (uint8_t*)buf);
}


/**
 * @brief 在LCD屏幕顶部中心显示标题
 * @param title 要显示的标题字符串
 * @param y_pos 标题在Y轴上的位置（可选，默认建议10-20）
 * @retval None
 */
void LCD_Display_Title_Center(const char* title, uint16_t y_pos)
{
    // 屏幕尺寸定义
    #define SCREEN_WIDTH  240
    #define SCREEN_CENTER_X (SCREEN_WIDTH / 2)  // 120
    
    // 字体参数
    #define FONT_WIDTH  8   // 16x8字体的字符宽度
    #define FONT_HEIGHT 16  // 16x8字体的字符高度
    
    // 计算字符串长度
    uint16_t str_len = strlen(title);
    
    // 计算字符串总像素宽度
    uint16_t total_width = str_len * FONT_WIDTH;
    
    // 计算起始X坐标(确保字符串中心对齐到屏幕中心)
    uint16_t start_x = SCREEN_CENTER_X - (total_width / 2);
    
    // 边界检查，确保字符串不会超出屏幕边界
    if(start_x > SCREEN_WIDTH) start_x = 0;  // 防止下溢
    if(start_x + total_width > SCREEN_WIDTH) start_x = SCREEN_WIDTH - total_width;
    
    // 设置显示颜色
    POINT_COLOR = BLACK;    // 黑色字体
    BACK_COLOR = WHITE;     // 白色背景

    // 显示标题字符串
    LCD_ShowString(start_x, y_pos, total_width, FONT_HEIGHT, 16, (uint8_t*)title);
}

void DDS_Output(Signal_t *sig1, Signal_t *sig2)
{
    AD9833_1_GPIO_Init();
    AD9833_2_GPIO_Init();

	/* ------- 通道 1 ------- */
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
	/* 计算初始 FTW1 */
	FTW1_cur = (uint32_t)((sig1->freq + FREQ_TUNNING_A) * 268435456.0f / ADCLK);

	/* ------- 通道 2 ------- */
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

/* 串口接收消息模块 */
uint8_t get_message(uint8_t *buf, uint16_t len, uint16_t *p_deg)
{
    /* 1. 长度不能超过 3（"180"） */
    if (len == 0 || len > 3) return 0;

    /* 2. 所有字符必须是数字 */
    for (uint16_t i = 0; i < len; i++) {
        if (!isdigit(buf[i])) return 0;
    }

    /* 3. 拷贝 + 转整数 */
    char tmp[4] = {0};        // len ∈ [1,3], 多一个 '\0'
    memcpy(tmp, buf, len);
    uint32_t val = strtoul(tmp, NULL, 10);
    if (val > 180) return 0;

    *p_deg = (uint16_t)val;
    return 1;
}

// 相关函数声明
//void X9C_Init(void);
//void X9C103_SetResistance(float resistance);
//void X9C503_SetResistance(float resistance);

void config_digital_potentiometer(uint16_t deg)
{
    float r1_103 = 0;
    float r2_503 = 0;

    /* 根据相位角计算电阻值 */

    X9C_Init();
    X9C103_SetResistance(r1_103);
    X9C503_SetResistance(r2_503);
}

void StartSampling(void)
{
    /* 1. 清 DMA 标志 */
    __HAL_DMA_CLEAR_FLAG(&hdma_adc2, DMA_FLAG_TCIF0_4);
    __HAL_DMA_CLEAR_FLAG(&hdma_adc3, DMA_FLAG_TCIF0_4);

    /* 2. 启动 ADC+DMA (NORMAL) */
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)calibration_buffer_A, CALIBRATION_BUF_SIZE);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)calibration_buffer_B, CALIBRATION_BUF_SIZE);

    /* 3. 让 TIM8 产生一次 TRGO 序列 */
    HAL_TIM_Base_Start(&htim8);                // TIM8 已配置触发一次 N 点
}

// 声明新的控制器函数和状态变量
static void FLL_Controller_Update(float error, uint32_t *ftw, FLL_PI_Controller_t *controller);

// 为通道A和B分别定义控制器状态
static FLL_PI_Controller_t fll_controller_A;
static FLL_PI_Controller_t fll_controller_B;

/* 新增：为精确频率测量定义相位跟踪器 */
static Phase_Tracker_t phase_tracker_A;
static Phase_Tracker_t phase_tracker_Apr;
static Phase_Tracker_t phase_tracker_B;
static Phase_Tracker_t phase_tracker_Bpr;

/* 新增：多帧频率平均器 */
static Freq_Averager_t freq_averager_A;
static Freq_Averager_t freq_averager_Apr;
static Freq_Averager_t freq_averager_B;
static Freq_Averager_t freq_averager_Bpr;
static uint32_t frame_counter = 0;  /* 全局帧计数器 */

/**
 * @brief 向频率平均器添加新的频率测量值
 * @param averager 频率平均器指针
 * @param new_freq 新的频率测量值
 * @return 如果缓冲区已满则返回1，否则返回0
 */
uint8_t add_frequency_measurement(Freq_Averager_t *averager, float new_freq)
{
    /* 将新频率添加到缓冲区 */
    averager->freq_buffer[averager->frame_count] = new_freq;
    averager->frame_count++;
    
    /* 检查缓冲区是否已满 */
    if (averager->frame_count >= FREQ_AVERAGE_FRAMES) {
        averager->buffer_full = 1;
        averager->frame_count = 0;  /* 重置计数器，循环使用缓冲区 */
        
        /* 计算平均频率 */
        float sum = 0.0f;
        for (uint8_t i = 0; i < FREQ_AVERAGE_FRAMES; i++) {
            sum += averager->freq_buffer[i];
        }
        averager->averaged_freq = sum / FREQ_AVERAGE_FRAMES;
        
        return 1;  /* 缓冲区已满，可以使用平均值 */
    }
    
    return 0;  /* 缓冲区未满，继续收集数据 */
}

/**
 * @brief 使用Goertzel + 相位微分法进行精确频率测量
 * @param signal_data 输入信号数据 (已去直流)
 * @param N 数据点数
 * @param target_freq 目标频率 (Hz) - 来自粗测结果
 * @param tracker 相位跟踪器状态
 * @return 精确的频率估计值 (Hz)
 */
float precise_frequency_measurement(const float *signal_data, int N, float target_freq, Phase_Tracker_t *tracker)
{
    /* 1. 使用Goertzel算法计算目标频率的复数输出 */
    goertzel_cfg_fomega_t goertzel_cfg;
    float magnitude, phase;
    
    // 初始化Goertzel配置
    goertzel_init_fomega(&goertzel_cfg, N, target_freq, CALIBRATION_SAMPLE_FREQ);
    
    // 执行Goertzel变换，得到幅度和相位
    goertzel_process_f32omega(&goertzel_cfg, signal_data, &magnitude, &phase);
    
    /* 2. 相位差计算和频率估计 */
    float freq_estimate = target_freq; // 默认返回目标频率
    
    if (tracker->phase_valid) {
        // 计算相位差
        float delta_phase = phase - tracker->last_phase;
        
        // 处理相位跳跃 (phase wrapping)
        while (delta_phase > 3.14159265f) {
            delta_phase -= 2.0f * 3.14159265f;
        }
        while (delta_phase < -3.14159265f) {
            delta_phase += 2.0f * 3.14159265f;
        }
        
        // 根据相位差计算精确频率
        // f_est = f0 + Δφ·Fs/(2π·N)
        float freq_correction = delta_phase * CALIBRATION_SAMPLE_FREQ / (2.0f * 3.14159265f * N);
        freq_estimate = target_freq + freq_correction;
        
        // 简单的IIR滤波平滑频率估计 - 降低alpha使历史数据有更大权重，系统更平滑
        const float alpha = 0.4f; 
        tracker->freq_estimate = tracker->freq_estimate * (1.0f - alpha) + freq_estimate * alpha;
        freq_estimate = tracker->freq_estimate;
        
    } else {
        // 第一次运行，初始化
        tracker->freq_estimate = target_freq;
        tracker->phase_valid = 1;
    }
    
    // 更新相位历史
    tracker->last_phase = phase;
    
    return freq_estimate;
}


void Calibration_Frequency(void)
{
	// 1. 启动本轮ADC+DMA+TIM采集（彻底修复启动时序）
    calibration_buffer_A_ready = 0;
    calibration_buffer_B_ready = 0;

    // 清除可能的DMA标志位
    __HAL_DMA_CLEAR_FLAG(&hdma_adc2, DMA_FLAG_TCIF0_4);
    __HAL_DMA_CLEAR_FLAG(&hdma_adc3, DMA_FLAG_TCIF0_4);

    // 先启动ADC+DMA，确保准备就绪
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*)calibration_buffer_A, CALIBRATION_BUF_SIZE) != HAL_OK)
    {
        Error_Handler();
    }       

    if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*)calibration_buffer_B, CALIBRATION_BUF_SIZE) != HAL_OK)
    {
        Error_Handler();
    }       

    // 启动定时器，第一次TRGO将在完整周期后发生
    /* 停 TIM, 清 CNT, 再启 */
	HAL_TIM_Base_Stop(&htim8);
	__HAL_TIM_SET_COUNTER(&htim8, 0);   // 关键：把 CNT 清零
	HAL_TIM_Base_Start(&htim8);         // 下一帧从 0 开始


    // 2. 等待本轮数据采集完成
    uint32_t timeout = 0;
    while (!(calibration_buffer_A_ready && calibration_buffer_B_ready) && timeout < 1000)
    {
        HAL_Delay(1);  // 短暂延时，避免CPU占用过高
        timeout++;
    }

    // 3. 检查是否超时
    if (timeout >= 1000)
    {
        // 超时处理：停止ADC+TIM，返回无新数据
        HAL_ADC_Stop_DMA(&hadc2);
        HAL_ADC_Stop_DMA(&hadc3);
        HAL_TIM_Base_Stop(&htim8);
        return;
    }

    // 4. 停止当前轮次的ADC+TIM（为下次启动做准备）
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADC_Stop_DMA(&hadc3);
    HAL_TIM_Base_Stop(&htim8);

    // 清除URS位，恢复默认行为
    htim8.Instance->CR1 &= ~TIM_CR1_URS;

    // 5. 处理本轮采集的数据
    static float buf_A[CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE];
    static float buf_Apr[CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE];
    static float buf_B[CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE];
    static float buf_Bpr[CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE];
    DemuxADCData((const uint16_t *)calibration_buffer_A, buf_A, buf_Apr, CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE);
    DemuxADCData((const uint16_t *)calibration_buffer_B, buf_B, buf_Bpr, CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE);


    // 6. 零交检测
    static float zc_idx_A[MAX_ZC];
    static float zc_idx_Apr[MAX_ZC];
    static float zc_idx_B[MAX_ZC];
    static float zc_idx_Bpr[MAX_ZC];

    // 函数第一次调用时清零，防止使用上次残留数据
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

	if(na <= MAVG || nap <= MAVG || nb <= MAVG || nbp <= MAVG) return; /* 数据不足 */

    /* 2. 粗频率计算：平均 MAVG 周期 (用于确定大致频率) */
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

    /* 3. 精确频率测量：使用Goertzel相位微分法 */
    float fA_precise   = precise_frequency_measurement(buf_A,   CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE, fA_coarse,   &phase_tracker_A);
    float fApr_precise = precise_frequency_measurement(buf_Apr, CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE, fApr_coarse, &phase_tracker_Apr);
    float fB_precise   = precise_frequency_measurement(buf_B,   CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE, fB_coarse,   &phase_tracker_B);
    float fBpr_precise = precise_frequency_measurement(buf_Bpr, CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE, fBpr_coarse, &phase_tracker_Bpr);

    printf("Precise freq: fA=%.3f, fApr=%.3f, fB=%.3f, fBpr=%.3f Hz\r\n", 
           fA_precise, fApr_precise, fB_precise, fBpr_precise);
	printf("Zero crossings: na=%d, nap=%d, nb=%d, nbp=%d\r\n", na, nap, nb, nbp);
    
    /* 4. 将频率测量值添加到多帧平均缓冲区 */
    uint8_t avgA_ready   = add_frequency_measurement(&freq_averager_A,   fA_precise);
    uint8_t avgApr_ready = add_frequency_measurement(&freq_averager_Apr, fApr_precise);
    uint8_t avgB_ready   = add_frequency_measurement(&freq_averager_B,   fB_precise);
    uint8_t avgBpr_ready = add_frequency_measurement(&freq_averager_Bpr, fBpr_precise);
    
    frame_counter++;
    printf("Frame %lu: Collected frequency data\r\n", frame_counter);

    /* 5. 只有当所有通道的平均缓冲区都已满时，才进行FLL控制器更新 */
    if (avgA_ready && avgApr_ready && avgB_ready && avgBpr_ready) {
        
        /* 使用平均后的频率值 */
        float fA_avg   = freq_averager_A.averaged_freq;
        float fApr_avg = freq_averager_Apr.averaged_freq;
        float fB_avg   = freq_averager_B.averaged_freq;
        float fBpr_avg = freq_averager_Bpr.averaged_freq;
        
        printf("=== AVERAGED FREQUENCIES (Frame %lu) ===\r\n", frame_counter);
        printf("Averaged freq: fA=%.4f, fApr=%.4f, fB=%.4f, fBpr=%.4f Hz\r\n", 
               fA_avg, fApr_avg, fB_avg, fBpr_avg);

        /* 6. 频差计算：使用平均后的频率 */
        float dfA = fA_avg - fApr_avg;
        float dfB = fB_avg - fBpr_avg;
        
        printf("Averaged frequency errors: dfA = %.5f Hz, dfB = %.5f Hz\r\n", dfA, dfB);
        
//        /* 7. 调用FLL控制器 */
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
 * @brief 分离ADC数据到2个缓冲区中，并做电压转换。
 * @param src 指向源数据数组的指针，包含所有通道的ADC数据。
 * @param buf1 指向通道1数据缓冲区的指针。
 * @param buf2 指向通道2数据缓冲区的指针。
 * @param len 每个缓冲区的长度（即每个通道的数据点数量）。
 */
static void DemuxADCData(const uint16_t *src,
                  float *buf1,
                  float *buf2,
                  uint16_t len)
{
    // 第一步：转换为电压并计算平均值（DC分量）
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
    
    // 第二步：为AC通道去除DC分量（FFT需要以0为中心的信号）
    float dc1 = sum1 / (float)len;
    float dc2 = sum2 / (float)len;
    
    for(uint16_t i = 0; i < len; i++)
    {
        buf1[i] -= dc1;  // 去除DC分量，得到纯AC信号
        buf2[i] -= dc2;  // 去除DC分量，得到纯AC信号
        // buf3保持不变，因为它就是DC信号
    }

}


// 零交检测（嵌入式版，无malloc/free）
// x: 输入float数组，N:数组长度
// zc_idx: 用户提供的零交输出buffer
// max_zc: buffer长度上限
// 返回：实际检测到的零交数量
int find_zero_crossings(const float *x, int N, float *zc_idx, int max_zc)
{
    int count = 0;
    for (int i = 0; i < N - 1 && count < max_zc; ++i) {
        // 只检测上升零交点（从负到正），与MATLAB保持一致
        if (x[i] < 0 && x[i+1] >= 0) { // 从负到正
            float frac = x[i] / (x[i] - x[i+1]);
            zc_idx[count++] = i + frac; // 分数采样点
        }
    }
    return count;
}

//输入参数 ftw（Frequency Tuning Word）是 AD9833 的 28 位频率控制字
static inline void AD9833_WriteFTW1(uint32_t ftw)
{
    // FREQ0 低14位
    uint16_t freq0_lsb = AD9833_REG_FREQ0 | (ftw & 0x3FFF);
    // FREQ0 高14位
    uint16_t freq0_msb = AD9833_REG_FREQ0 | ((ftw >> 14) & 0x3FFF);

    AD9833_1_SetRegisterValue(freq0_lsb);
    AD9833_1_SetRegisterValue(freq0_msb);
}
//输入参数 ftw（Frequency Tuning Word）是 AD9833 的 28 位频率控制字
static inline void AD9833_WriteFTW2(uint32_t ftw)
{
    uint16_t freq0_lsb = AD9833_REG_FREQ0 | (ftw & 0x3FFF);
    uint16_t freq0_msb = AD9833_REG_FREQ0 | ((ftw >> 14) & 0x3FFF);

    AD9833_2_SetRegisterValue(freq0_lsb);
    AD9833_2_SetRegisterValue(freq0_msb);
}

/**
 * @brief FLL PI控制器核心更新函数 (带Anti-Windup) - 重构版
 * @param error 当前频率误差 (df)
 * @param ftw   指向当前DDS频率字的指针
 * @param controller 指向控制器状态的指针
 */
static void FLL_Controller_Update(float error, uint32_t *ftw, FLL_PI_Controller_t *controller)
{
    /* 如果校准已停止，则直接返回 */
    if (controller->calibration_stopped) {
        return;
    }

    /* 检查频率是否稳定 */
    if (fabsf(error) < FLL_STABLE_THRESHOLD_HZ) {
        controller->stable_counter++;
    } else {
        controller->stable_counter = 0; // 如果不稳定，重置计数器
    }

    /* 如果达到稳定条件，则停止校准 */
    if (controller->stable_counter >= STABLE_FRAME_COUNT) {
        controller->calibration_stopped = 1;
        printf(">>> FLL for controller %s has stopped due to stability. <<<\n",
               (controller == &fll_controller_A) ? "A" : "B");
        return; // 停止本次更新
    }

    float p_term = FLL_KP * error; // 计算比例项

    /* 1. 死区判断：在死区内，禁用比例(P)项，但保留积分(I)项继续消除稳态误差 */
    if (fabsf(error) < FLL_DEAD_ZONE_HZ) {
        p_term = 0.0f;
    }
    
    /* 1.1. 积分器重置：当误差极小时，重置积分器防止长期漂移 */
    if (fabsf(error) < FLL_RESET_THRESHOLD_HZ) {
        controller->integrator = 0.0f;
    }
    
    /* 2. 计算 PI 输出 */
    float output = p_term + controller->integrator;
    
    /* 3. 输出限幅 (饱和) */
    float output_saturated = output;
    if (output_saturated > FLL_MAX_STEP_HZ) {
        output_saturated = FLL_MAX_STEP_HZ;
    } else if (output_saturated < -FLL_MAX_STEP_HZ) {
        output_saturated = -FLL_MAX_STEP_HZ;
    }
    
    /* 4. 积分器更新 (带 Anti-Windup) */
    // 积分器持续累积原始误差(FLL_KI * error)。
    // 如果输出被限幅，抗饱和项会减去超出的部分(output - output_saturated)，
    // 从而防止积分器在饱和状态下继续累积。
    controller->integrator += FLL_KI * error + FLL_ANTI_WINDUP_GAIN * (output_saturated - output);
    
    /* 5. 积分器硬限幅 - 防止积分器无限累积导致溢出 */
    if (controller->integrator > FLL_INTEGRATOR_MAX) {
        controller->integrator = FLL_INTEGRATOR_MAX;
    } else if (controller->integrator < FLL_INTEGRATOR_MIN) {
        controller->integrator = FLL_INTEGRATOR_MIN;
    }

    /* 6. 更新 DDS 频率字 */
    int32_t dFTW = (int32_t)(output_saturated * (268435456.0f / ADCLK));
    *ftw += dFTW;
    
    // 根据是哪个通道的控制器来调用对应的写函数
    if (controller == &fll_controller_A) {
        AD9833_WriteFTW1(*ftw);
    } else {
        AD9833_WriteFTW2(*ftw);
    }

    printf("FLL_Update: err=%.2f, Kp=%.2f, Ki=%.2f, out=%.2f, sat_out=%.2f, int=%.2f,p_term = %.2f\n",
           error, FLL_KP, FLL_KI, output, output_saturated, controller->integrator,p_term);
}
