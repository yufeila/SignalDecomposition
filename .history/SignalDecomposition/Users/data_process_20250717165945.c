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
	/* 计算初始 FTW1 */
	FTW1_cur = (uint32_t)((sig1->freq + FREQ_TUNNING) * 268435456.0f / ADCLK);

	/* ------- 通道 2 ------- */
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
	FTW2_cur = (uint32_t)((sig2->freq + FREQ_TUNNING) * 268435456.0f / ADCLK);

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
    float zc_idx_A[MAX_ZC];
    float zc_idx_Apr[MAX_ZC];
    float zc_idx_B[MAX_ZC];
    float zc_idx_Bpr[MAX_ZC];

    uint16_t na = find_zero_crossings(buf_A, CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE, zc_idx_A, MAX_ZC);
    uint16_t nap = find_zero_crossings(buf_Apr, CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE, zc_idx_Apr, MAX_ZC);
    uint16_t nb = find_zero_crossings(buf_B, CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE, zc_idx_B, MAX_ZC);
    uint16_t nbp = find_zero_crossings(buf_Bpr, CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE, zc_idx_Bpr, MAX_ZC);

	if(na <= MAVG || nap <= MAVG || nb <= MAVG || nbp <= MAVG) return; /* 数据不足 */

    /* 2. 频率计算：平均 MAVG 周期 */
	float Ta   = (zc_idx_A [na-1]  - zc_idx_A [na-1-MAVG ]) / MAVG / CALIBRATION_SAMPLE_FREQ;
    float Tap  = (zc_idx_Apr[nap-1]- zc_idx_Apr[nap-1-MAVG]) / MAVG / CALIBRATION_SAMPLE_FREQ;
	float Tb  = (zc_idx_B [nb-1]  - zc_idx_B [nb-1-MAVG ]) / MAVG / CALIBRATION_SAMPLE_FREQ;
	float Tbp = (zc_idx_Bpr[nbp-1]- zc_idx_Bpr[nbp-1-MAVG]) / MAVG / CALIBRATION_SAMPLE_FREQ;

    float fA   = 1.0f/Ta;
    float fApr = 1.0f/Tap;
    float fB   = 1.0f/Tb;
    float fBpr = 1.0f/Tbp;
	
	/* 频率测量低通滤波（减少噪声） */
	if (!filter_init) {
		fA_filtered = fA;
		fApr_filtered = fApr;
		fB_filtered = fB;
		fBpr_filtered = fBpr;
		filter_init = 1;
	} else {
		/* 外部信号源(A,B)使用更强滤波，DDS信号源(Apr,Bpr)使用较轻滤波 */
		fA_filtered = FREQ_FILTER_ALPHA_EXTERNAL * fA_filtered + (1.0f - FREQ_FILTER_ALPHA_EXTERNAL) * fA;
		fApr_filtered = FREQ_FILTER_ALPHA_DDS * fApr_filtered + (1.0f - FREQ_FILTER_ALPHA_DDS) * fApr;
		fB_filtered = FREQ_FILTER_ALPHA_EXTERNAL * fB_filtered + (1.0f - FREQ_FILTER_ALPHA_EXTERNAL) * fB;
		fBpr_filtered = FREQ_FILTER_ALPHA_DDS * fBpr_filtered + (1.0f - FREQ_FILTER_ALPHA_DDS) * fBpr;
	}
	
	printf("freq_A = %.2f Hz(raw:%.2f), freq_Apr = %.2f Hz(raw:%.2f), freq_B = %.2f Hz(raw:%.2f), freq_Bpr = %.2f Hz(raw:%.2f)\r\n", 
           fA_filtered, fA, fApr_filtered, fApr, fB_filtered, fB, fBpr_filtered, fBpr);
	printf("Zero crossings: na=%d, nap=%d, nb=%d, nbp=%d\r\n", na, nap, nb, nbp);



    /* 3. 频差 & PI 调节 */
    float dfA = fA_filtered - fApr_filtered;
    float dfB = fB_filtered - fBpr_filtered;
	
	printf("Frequency errors: dfA = %.3f Hz, dfB = %.3f Hz\r\n", dfA, dfB);
	
	/* 死区控制：频差太小时不调节（避免追踪噪声） */
	const float DEAD_ZONE = 2.0f; // 2Hz死区

    /* ---- 通道 A ---- */
	/* 死区控制 */
	if (fabsf(dfA) < DEAD_ZONE) dfA = 0;
	
	/* Anti-wind-up：当极性翻转时清积分 */
	if ((pllA_int > 0 && dfA < 0) || (pllA_int < 0 && dfA > 0))
		 pllA_int = 0;

	/* 积分并限幅 ±50 Hz（进一步减小积分限幅） */
	pllA_int += KI * dfA;
	if (pllA_int > 50) pllA_int = 50;
	if (pllA_int < -50) pllA_int = -50;

	float stepA = KP * dfA + pllA_int;

	/* 限幅：一次最大改 ±20 Hz（进一步减小步进限制） */
	if (stepA > 20)  stepA = 20;
	if (stepA < -20) stepA = -20;

    if(fabsf(dfA) > TOL_HZ){                     /* 还没锁定 */
        int32_t dFTW = (int32_t)(stepA * 268435456.0f / ADCLK);
        FTW1_cur += dFTW;
        AD9833_WriteFTW1(FTW1_cur);
    }

	/* ---- 通道 B ---- */
	/* 死区控制 */
	if (fabsf(dfB) < DEAD_ZONE) dfB = 0;
	
	/* Anti-wind-up：当极性翻转时清积分 */
	if ((pllB_int > 0 && dfB < 0) || (pllB_int < 0 && dfB > 0))
		pllB_int = 0;

	/* 积分并限幅 ±50 Hz（进一步减小积分限幅） */
	pllB_int += KI * dfB;
	if (pllB_int > 50)  pllB_int = 50;
	if (pllB_int < -50) pllB_int = -50;

	float stepB = KP * dfB + pllB_int;

	/* 限幅：一次最大改 ±20 Hz（进一步减小步进限制） */
	if (stepB > 20)  stepB = 20;
	if (stepB < -20) stepB = -20;

	if (fabsf(dfB) > TOL_HZ) {
		int32_t dFTW = (int32_t)(stepB * 268435456.0f / ADCLK);
		FTW2_cur += dFTW;
		AD9833_WriteFTW2(FTW2_cur);
	}

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
