/*
 * @Author: yyf 17786321727@163.com
 * @Date: 2025-07-12 15:11:29
 * @LastEditors: yyf 17786321727@163.com
 * @LastEditTime: 2025-07-17 21:45:26
 * @FilePath: /SignalDecomposition/Users/data_process.h
 * @Description: ����Ĭ������,������`customMade`, ��koroFileHeader�鿴���� ��������: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
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


/* ------------ �û��ɵ����� ---------------- */
//#define FS            600000.0f          /* ������ */
#define CALIBRATION_SAMPLE_FREQ 600000.0f /* ������ */
#define MAVG          40                 /* �� 40 ����ƽ������һ�������ȶ��ԣ� */

/* ������ɢ��������������Ƶ�FLL���� */
#define TARGET_FREQ   50000.0f           /* Ŀ��Ƶ��(Hz) */
#define FLL_BANDWIDTH 50.0f              /* FLL����(Hz) - ��O3�����100Hz����һ�� */
#define DAMPING_RATIO 0.8f               /* ����� - ��0.707�Դ󣬸��ȶ� */
#define UPDATE_PERIOD (MAVG / TARGET_FREQ) /* ��·�������� �� 0.8ms */
#define FTW_SCALE     (268435456.0f/25000000.0f) /* 2^28/ADCLK �� 10.737 */

/* ��������ƺ���ɢ�� */
#define OMEGA_N       (2.0f * 3.14159f * FLL_BANDWIDTH)
#define KP_CONTINUOUS (2.0f * DAMPING_RATIO * OMEGA_N)
#define KI_CONTINUOUS (OMEGA_N * OMEGA_N)

/* ��ɢ�����ʵ�ʿ��������� */
#define KP            (KP_CONTINUOUS * 0.001f)  /* ��һ������ */
#define KI            (KI_CONTINUOUS * UPDATE_PERIOD * 0.001f)  /* ��ɢ��+��һ�� */
#define K_AW          (1.0f / UPDATE_PERIOD)    /* Anti-wind-upϵ�� */

/* ͳһ���޷����� */
#define MAX_FREQ_STEP 15.0f              /* ���Ƶ�ʲ���(Hz) */
#define FREQ_DEAD_ZONE 1.0f              /* Ƶ������(Hz) */
#define TOL_HZ        2.0f               /* �ж�������ֵ */

/* �˲����� */
#define FREQ_FILTER_ALPHA_EXTERNAL 0.92f /* �ⲿ�ź�Դ�˲�ϵ�� */
#define FREQ_FILTER_ALPHA_DDS 0.5f       /* DDS�ź��˲�ϵ�� */
#define ADCLK         25000000.0f        /* AD9833 ʱ�� (Hz) */

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
