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
    float adc_zeroed[BUF_SIZE];
    adc_zero_bias(src, adc_zeroed, BUF_SIZE);

    // 6. 使用goertzel算法利用基频估计高次谐波
    // 6.1 获取基频
    float fund_freq,fund_amp;
    fft_hann_zero_interp(adc_zeroed, &fund_freq, &fund_amp);    
    goertzel_cfg_t target_freq_cfg;

    float harm1_mag, harm1_phase;
    goertzel_init(&target_freq_cfg, FFT_SIZE, fund_freq*1.0f, FS_HZ);
    goertzel_process_f32(&target_freq_cfg, adc_zeroed, &harm1_mag, &harm1_phase);


    // 6.2 获取3次谐波幅度
    float harm3_mag, harm3_phase;
    goertzel_init(&target_freq_cfg, FFT_SIZE, fund_freq*3.0f, FS_HZ);
    goertzel_process_f32(&target_freq_cfg, adc_zeroed, &harm3_mag, &harm3_phase);

    // 6.3 获取5次谐波幅度
    float harm5_mag, harm5_phase;
    goertzel_init(&target_freq_cfg, FFT_SIZE, fund_freq * 5.0f, FS_HZ);
    goertzel_process_f32(&target_freq_cfg, adc_zeroed, &harm5_mag, &harm5_phase);


}

/* _buf: ADC 原始 uint16[]，长度至少 FFT_SIZE */
void analyse_two_signals(const uint16_t *_buf, Signal_t *sig1, Signal_t *sig2)
{
    /* 1. 去零偏 & 换算为电压 */
    adc_zero_bias(_buf, adc_zeroed, FFT_SIZE);

    /* 2. FFT 找到两个基波 */
    float f1, A1, f2, A2;
    fft_top2_hann_zero_interp(adc_zeroed, &f1, &A1, &f2, &A2, FFT_SIZE);

    /* 3. 对每个基波，Goertzel 计算 1×/3×/5× 幅值 */
    goertzel_cfg_t g1, g3, g5;
    float mag1, ph1, mag3, ph3, mag5, ph5;

    /* --- helper lambda（C99 宏模拟）--- */
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
                /* 介于两者，可加 LOG 或设未知 */                             \
                (_sig_out)->wave_form = 0;                                     \
            }                                                                  \
        } while(0)

    ANALYSE_ONE(f1, sig1);
    ANALYSE_ONE(f2, sig2);
}