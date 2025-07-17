/*
 * @Author: yyf 17786321727@163.com
 * @Date: 2025-07-12 15:11:29
 * @LastEditors: yyf 17786321727@163.com
 * @LastEditTime: 2025-07-17 16:58:54
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
//#define FS            600000.0f          /* 采样率 */
#define CALIBRATION_SAMPLE_FREQ 600000.0f /* 采样率 */
#define MAVG          30                 /* 用 30 周期平均（进一步增加） */
#define FREQ_FILTER_ALPHA_EXTERNAL 0.85f /* 外部信号源滤波系数（更强滤波） */
#define FREQ_FILTER_ALPHA_DDS 0.0f       /* DDS信号滤波系数（较轻滤波） */
#define KP            0.005f             /* PI 比例系数（更保守） */
#define KI            0.0002f            /* PI 积分系数（更保守） */
#define TOL_HZ        1.0f               /* 判断锁定阈值 */
#define ADCLK         25000000.0f        /* AD9833 时钟 (Hz) */

typedef struct{
    float freq;
    uint16_t wave_form;
}Signal_t;

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
