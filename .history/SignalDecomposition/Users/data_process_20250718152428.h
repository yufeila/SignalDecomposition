/*
 * @Author: yyf 17786321727@163.com
 * @Date: 2025-07-12 15:11:29
 * @LastEditors: yyf 17786321727@163.com
 * @LastEditTime: 2025-07-18 15:23:47
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


/* ------------ 用户可调参数 ---------------- */
#define CALIBRATION_SAMPLE_FREQ 600000.0f /* ADC采样率 (硬件固定) */
#define ADCLK                   25000000.0f /* AD9833 时钟 (Hz) */
#define MAVG                    20          /* 增加周期数以提高频率估算平滑度 */
#define MAX_ZC                  60         

/* FLL (锁频环) 控制器设计参数 */
#define FLL_LOOP_UPDATE_RATE_HZ 20.0f   /* FLL环路更新频率(Hz)，用于计算固定的Ts */
#define FLL_BANDWIDTH_HZ        0.01f    /* 环路带宽(Hz): 适当降低，使响应更平稳 */
#define FLL_DAMPING_RATIO       0.9f    /* 阻尼比: 保持0.9以抑制过冲 */
#define FLL_MAX_STEP_HZ         5.0f    /* 单次最大调节步进(Hz): 保持5Hz，避免剧烈跳变 */
#define FLL_DEAD_ZONE_HZ        0.1f    /* 死区(Hz): 大幅减小，让积分器可以消除小的稳态误差 */

/* 新增：多帧平均参数 */
#define FREQ_AVERAGE_FRAMES     3       /* 平均多少帧数据，可根据需要调整(4-16) */
#define FLL_UPDATE_INTERVAL     FREQ_AVERAGE_FRAMES  /* 每隔多少帧更新一次FLL */

/* FLL PI控制器离散化后的固定参数 */
#define FLL_TS                  (1.0f / FLL_LOOP_UPDATE_RATE_HZ)
#define FLL_OMEGA_N             (2.0f * 3.1415926f * FLL_BANDWIDTH_HZ)
#define FLL_KP                  (2.0f * FLL_DAMPING_RATIO * FLL_OMEGA_N)
#define FLL_KI                  (FLL_OMEGA_N * FLL_OMEGA_N * FLL_TS)
#define FLL_ANTI_WINDUP_GAIN    (1.0f / FLL_KP)

/* 新增：积分器保护参数 */
#define FLL_INTEGRATOR_MAX      100.0f   /* 积分器最大值 (Hz) */
#define FLL_INTEGRATOR_MIN      (-100.0f) /* 积分器最小值 (Hz) */
#define FLL_RESET_THRESHOLD_HZ  0.01f    /* 误差小于此值时重置积分器 (Hz) */


typedef struct{
    float freq;
    uint16_t wave_form;
}Signal_t;

/* PI控制器状态结构体 (用于FLL) */
typedef struct {
    float integrator;       /* 积分累加项 */
    float last_output;      /* 上一次的输出 (用于Anti-windup) */
} FLL_PI_Controller_t;

/* 新增：相位跟踪Goertzel状态结构体 */
typedef struct {
    float last_phase;       /* 上一次的相位值 (弧度) */
    uint8_t phase_valid;    /* 相位是否已初始化 */
    float freq_estimate;    /* 当前频率估计值 */
} Phase_Tracker_t;

/* 新增：多帧频率平均缓冲区结构体 */
typedef struct {
    float freq_buffer[FREQ_AVERAGE_FRAMES];  /* 频率测量缓冲区 */
    uint8_t frame_count;                      /* 当前帧计数 */
    uint8_t buffer_full;                      /* 缓冲区是否已满 */
    float averaged_freq;                      /* 平均后的频率 */
} Freq_Averager_t;

void Data_Process(void);
void DDS_Output(Signal_t *sig1, Signal_t *sig2);
void LCD_Display_Title_Center(const char* title, uint16_t y_pos);
void Signal_Info_Display(Signal_t *sig1, Signal_t *sig2);
void Update_LCD_Display(void);
void analyse_two_signals(const uint16_t *_buf, Signal_t *sig1, Signal_t *sig2);
void config_digital_potentiometer(uint16_t deg);
uint8_t get_message(uint8_t *buf, uint16_t len, uint16_t *p_deg);
void Calibration_Frequency(void);

/* 新增：精确频率测量函数 */
float precise_frequency_measurement(const float *signal_data, int N, float target_freq, Phase_Tracker_t *tracker);

/* 新增：多帧频率平均函数 */
uint8_t add_frequency_measurement(Freq_Averager_t *averager, float new_freq);

#endif /* __DATA_PROCESS_H */
