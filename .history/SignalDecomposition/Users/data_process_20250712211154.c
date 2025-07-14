#include "data_process.h"

/* 内部静态缓冲区，与 FFT/Goertzel 共用一份 ADC 数据 */
static float adc_zeroed[BUF_SIZE];     /* 去直流 & 换算电压后的浮点序列 */

void SignalOutputAndDisplay(void)
{
    // 检测按键状态
    Detect_KeyPress();

    // 如果信号分解标志被设置，执行信号分解处理
    if (signal_deconposition_flag)
    {
        signal_deconposition_flag = 0; // 重置标志

        Data_Process();

        Update_LCD_Display();

        DDS_Output();
    }
}

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
        return 0;
    }

    // 4. 停止当前轮次的ADC+TIM（为下次启动做准备）
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_TIM_Base_Stop(&htim2);

    // 清除URS位，恢复默认行为
    htim2.Instance->CR1 &= ~TIM_CR1_URS;

    // 5. 处理本轮采集的数据
    const uint16_t *src = (const uint16_t *)&adc_buffer[0];

    // 6. 生成两个信号参数
    Signal_t sig1, sig2;
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

    /* 2. FFT 找到两个基波 */
    /* 先得到两个基波的频率和幅度 */
    float f1, A1, f2, A2;
    fft_top2_hann_zero_interp(adc_zeroed, &f1, &A1, &f2, &A2);

    /* --- helper lambda（C99 宏模拟）--- */
    #define ANALYSE_ONE(_f_base, _A_base, _f_other, _A_other, _sig_out)                
    do {                                                                               \
        goertzel_cfg_fomega_t g1, g3, g5;                                                  \
        float mag1, ph1, mag3, ph3, mag5, ph5;                                         \
                                                                                    \
        /* -------- 一次性把 1×/3×/5× 幅值算出来 -------- */                          \
        goertzel_init_fomega(&g1, FFT_SIZE, 1.0f * (_f_base),  FS_HZ);                        \
        goertzel_init_fomega(&g3, FFT_SIZE, 3.0f * (_f_base),  FS_HZ);                        \
        goertzel_init_fomega(&g5, FFT_SIZE, 5.0f * (_f_base),  FS_HZ);                        \
        goertzel_process_f32omega(&g1, adc_zeroed, &mag1, &ph1);                            \
        goertzel_process_f32omega(&g3, adc_zeroed, &mag3, &ph3);                            \
        goertzel_process_f32omega(&g5, adc_zeroed, &mag5, &ph5);                            \
                                                                                    \
        /* -------- 若 3× 或 5× 与另一基波重叠，做幅度抵消 -------- */                 \
        const float EPS = 0.4f;   /* half-bin 宽容 */                                  \
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
    LCD_Display_Title_Center("Signal Info", 10);
    
    // 显示信号1的频率和类型
    LCD_ShowString(10, 30, 200, 16, 16, (uint8_t*)"Signal 1:");
    LCD_ShowString(10, 50, 200, 16, 16, (uint8_t*)"Frequency: %.2f Hz", sig1->freq);
    LCD_ShowString(10, 70, 200, 16, 16, (uint8_t*)"Type: %d", sig1->wave_form);

    // 显示信号2的频率和类型
    LCD_ShowString(10, 90, 200, 16, 16, (uint8_t*)"Signal 2:");
    LCD_ShowString(10, 110, 200, 16, 16, (uint8_t*)"Frequency: %.2f Hz", sig2->freq);
    LCD_ShowString(10, 130, 200, 16, 16, (uint8_t*)"Type: %d", sig2->wave_form);
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
    AD9833_1_Init();
    AD9833_2_Init();

    AD9833_1_Set_Frequency(sig1->freq);
    AD9833_2_Set_Frequency(sig2->freq);

    AD9833_1_Set_Amplitude(sig1->amp);
    AD9833_2_Set_Amplitude(sig2->amp);
}