/*
 * @Author: yyf 17786321727@163.com
 * @Date: 2025-07-12 15:11:29
 * @LastEditors: yyf 17786321727@163.com
 * @LastEditTime: 2025-07-17 21:25:15
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
#define MAVG          30                 /* 用 30 周期平均 */
#define FREQ_FILTER_ALPHA_EXTERNAL 0.85f /* 外部信号源滤波系数（更强滤波） */
#define FREQ_FILTER_ALPHA_DDS 0.3f       /* DDS信号滤波系数（轻度滤波） */

/* FLL参数 - 按离散控制理论设计 */
#define FLL_BANDWIDTH_HZ    50.0f        /* FLL带宽50Hz（适中的带宽） */
#define FLL_DAMPING         0.707f       /* 阻尼比（临界阻尼） */
#define FLL_UPDATE_PERIOD   (MAVG / 50000.0f)  /* 更新周期：假设50kHz信号 */
#define FTW_SCALE           (268435456.0f/25000000.0f)  /* 2^28/ADCLK */

/* 根据离散控制理论计算的参数 */
#define OMEGA_N             (2.0f * 3.14159f * FLL_BANDWIDTH_HZ)
#define KP_CONTINUOUS       (2.0f * FLL_DAMPING * OMEGA_N)
#define KI_CONTINUOUS       (OMEGA_N * OMEGA_N)

/* 离散化后的实际控制参数 */
#define KP                  (KP_CONTINUOUS / FTW_SCALE)     /* 比例增益 */
#define KI                  (KI_CONTINUOUS * FLL_UPDATE_PERIOD / FTW_SCALE)  /* 积分增益 */
#define K_AW                (1.0f / FLL_UPDATE_PERIOD)      /* Anti-wind-up增益 */

/* 统一的限幅参数 */
#define MAX_FREQ_STEP_HZ    30.0f        /* 最大频率步进30Hz */
#define MAX_FTW_STEP        (MAX_FREQ_STEP_HZ * FTW_SCALE)  /* 对应的FTW步进 */

#define TOL_HZ              2.0f         /* 判断锁定阈值 */
#define DEAD_ZONE_HZ        1.0f         /* 死区1Hz（针对FLL，不需要太大） */
#define ADCLK               25000000.0f  /* AD9833 时钟 (Hz) */

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
