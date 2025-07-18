/*
 * @Author: yyf 17786321727@163.com
 * @Date: 2025-07-12 15:11:29
 * @LastEditors: yyf 17786321727@163.com
 * @LastEditTime: 2025-07-18 09:50:34
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
#define MAVG                    15          /* 减少到15个周期，确保在新窗口下有足够数据 */
#define MAX_ZC                  200         /* 增加到200，适应更长的数据窗口 */

/* FLL (锁频环) 控制器设计参数 */
#define FLL_BANDWIDTH_HZ        12.0f   /* 环路带宽(Hz): 适当提高到12Hz，增强积分作用 */
#define FLL_DAMPING_RATIO       0.9f    /* 阻尼比: 保持0.9以抑制过冲 */
#define FLL_MAX_STEP_HZ         5.0f    /* 单次最大调节步进(Hz): 保持5Hz，避免剧烈跳变 */
#define FLL_DEAD_ZONE_HZ        1.0f    /* 死区(Hz): 减小到1Hz，让系统对小误差更敏感 */


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
