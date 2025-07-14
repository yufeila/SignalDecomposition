#include "data_process.h"

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

    // 6. 使用go
    float fund_freq,fund_amp;
    fft_hann_zero_interp(adc_zeroed, &fund_freq, &fund_amp);    
    
    // 6. 查找基频
    goertzel_cfg_t target_freq_cfg;

    float harm3_mag, harm3_phase;
    goertzel_init(&target_freq_cfg, FFT_SIZE, fund_freq, FS_HZ);

    
    goertzel_process_f32(&target_freq_cfg, adc_zeroed, &harm3_mag, &harm3_phase);

    float harm5_mag, harm5_phase;
    goertzel_init(&target_freq_cfg, FFT_SIZE, fund_freq * 3.0f, FS_HZ);
    goertzel_process_f32(&target_freq_cfg, adc_zeroed, &harm5_mag, &harm5_phase);


}