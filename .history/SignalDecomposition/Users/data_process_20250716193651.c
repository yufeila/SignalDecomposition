#include "data_process.h"
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>     // 用于sprintf
#include "adc.h"

/* 内部静态缓冲区，与 FFT/Goertzel 共用一份 ADC 数据 */
static float adc_zeroed[BUF_SIZE];     /* 去直流 & 换算电压后的浮点序列 */
uint16_t calibration_flag = 0;      /* 频率校准标志位 */

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;

extern volatile uint16_t calibration_buffer_A[CALIBRATION_BUF_SIZE];
extern volatile uint8_t calibration_buffer_A_ready;
extern volatile uint16_t calibration_buffer_B[CALIBRATION_BUF_SIZE];
extern volatile uint8_t calibration_buffer_B_ready;

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
    HAL_TIM_Base_Start(&htim8);

    // 2. 等待本轮数据采集完成
    uint32_t timeout = 0;
    while (calibration_buffer_A_ready != 1 && calibration_buffer_B_ready != 1 && timeout < 1000)
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
    static float buf1_A[CALIBRATION_BUF_SIZE];
    static float buf2_A[CALIBRATION_BUF_SIZE];
    static float buf1_B[CALIBRATION_BUF_SIZE];
    static float buf2_B[CALIBRATION_BUF_SIZE];
    DemuxADCData(calibration_buffer_A, buf1_A, buf2_A, CALIBRATION_BUF_SIZE);
    DemuxADCData(calibration_buffer_B, buf1_B, buf2_B, CALIBRATION_BUF_SIZE);

    // 6. FFT进行校准
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


