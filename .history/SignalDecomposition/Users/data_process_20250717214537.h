/*
 * @Author: yyf 17786321727@163.com
 * @Date: 2025-07-12 15:11:29
 * @LastEditors: yyf 17786321727@163.com
 * @LastEditTime: 2025-07-17 21:45:26
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
#define MAVG          40                 /* 用 40 周期平均（进一步增加稳定性） */

/* 按照离散控制理论重新设计的FLL参数 */
#define TARGET_FREQ   50000.0f           /* 目标频率(Hz) */
#define FLL_BANDWIDTH 50.0f              /* FLL带宽(Hz) - 比O3建议的100Hz保守一点 */
#define DAMPING_RATIO 0.8f               /* 阻尼比 - 比0.707稍大，更稳定 */
#define UPDATE_PERIOD (MAVG / TARGET_FREQ) /* 环路更新周期 ≈ 0.8ms */
#define FTW_SCALE     (268435456.0f/25000000.0f) /* 2^28/ADCLK ≈ 10.737 */

/* 连续域设计后离散化 */
#define OMEGA_N       (2.0f * 3.14159f * FLL_BANDWIDTH)
#define KP_CONTINUOUS (2.0f * DAMPING_RATIO * OMEGA_N)
#define KI_CONTINUOUS (OMEGA_N * OMEGA_N)

/* 离散化后的实际控制器参数 */
#define KP            (KP_CONTINUOUS * 0.001f)  /* 归一化缩放 */
#define KI            (KI_CONTINUOUS * UPDATE_PERIOD * 0.001f)  /* 离散化+归一化 */
#define K_AW          (1.0f / UPDATE_PERIOD)    /* Anti-wind-up系数 */

/* 统一的限幅参数 */
#define MAX_FREQ_STEP 15.0f              /* 最大频率步进(Hz) */
#define FREQ_DEAD_ZONE 1.0f              /* 频率死区(Hz) */
#define TOL_HZ        2.0f               /* 判断锁定阈值 */

/* 滤波参数 */
#define FREQ_FILTER_ALPHA_EXTERNAL 0.92f /* 外部信号源滤波系数 */
#define FREQ_FILTER_ALPHA_DDS 0.5f       /* DDS信号滤波系数 */
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
