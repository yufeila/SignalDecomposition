/*
 * @Author: yyf 17786321727@163.com
 * @Date: 2025-07-12 15:11:29
 * @LastEditors: yyf 17786321727@163.com
 * @LastEditTime: 2025-07-17 21:52:16
 * @FilePath: /SignalDecomposition/Users/data_process.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __DATA_PROCESS_H
#define __DATA_PROCESS_H   

#include "main.h"
#include "tim.h"
#include "adc.h"
#include "usart.h"
#include "./key/key.h"
#include "./x9cxxx/bsp_x9cxxx.h"
#include "./ad9833/bsp_ad9833.h"
#include "./fft_hp_estimate/fft_hp_estimate.h"
#include "./goertzel/goertzel.h"
#include "./lcd/lcd.h"

#define SINC_WAVE 1
#define TRIANGLE_WAVE 2
#define FREQ_TUNNING 0
#define MAX_ZC 32


/* ------------ 用户可调参数 ---------------- */
#define CALIBRATION_SAMPLE_FREQ 600000.0f /* ADC采样率 (硬件固定) */
#define ADCLK                   25000000.0f /* AD9833 时钟 (Hz) */
#define MAVG                    50          /* 用 50 个周期做平均，增加频率测量稳定性 */
#define MAX_ZC                  64          /* 最大零交点检测数 (增加以容纳更长MAVG) */

/* FLL (锁频环) 控制器设计参数 */
#define FLL_BANDWIDTH_HZ        20.0f   /* 环路带宽(Hz): 决定响应速度，值越小越稳定，但响应越慢 (建议范围 5-50) */
#define FLL_DAMPING_RATIO       0.8f    /* 阻尼比: 决定超调量，0.707是临界阻尼，>0.7会更稳定 (建议范围 0.7-1.0) */
#define FLL_MAX_STEP_HZ         25.0f   /* 单次最大调节步进(Hz): 限制输出变化率 */
#define FLL_DEAD_ZONE_HZ        3.0f    /* 死区(Hz): 忽略此范围内的频率误差，防止追踪噪声 */


typedef struct{
    float freq;
    uint16_t wave_form;
}Signal_t;

/* PI控制器状态结构体 (用于FLL) */
typedef struct {
    float integrator;       /* 积分累加项 */
    float last_output;      /* 上一次的输出 (用于Anti-windup) */
} FLL_PI_Controller_t;

void Data_Process(void);
void DDS_Output(Signal_t *sig1, Signal_t *sig2);
void LCD_Display_Title_Center(const char* title, uint16_t y_pos);
void Signal_Info_Display(Signal_t *sig1, Signal_t *sig2);
void Update_LCD_Display(void);
void analyse_two_signals(const uint16_t *_buf, Signal_t *sig1, Signal_t *sig2);
void config_digital_potentiometer(uint16_t deg);
uint8_t get_message(uint8_t *buf, uint16_t len, uint16_t *p_deg);
void Calibration_Frequency(void);

#endif /* __DATA_PROCESS_H */
