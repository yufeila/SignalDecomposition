/*
 * @Author: yyf 17786321727@163.com
 * @Date: 2025-07-12 15:11:29
 * @LastEditors: yyf 17786321727@163.com
 * @LastEditTime: 2025-07-18 15:45:16
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


/* ------------ �û��ɵ����� ---------------- */
#define CALIBRATION_SAMPLE_FREQ 600000.0f /* ADC������ (Ӳ���̶�) */
#define ADCLK                   25000000.0f /* AD9833 ʱ�� (Hz) */
#define MAVG                    20          /* ���������������Ƶ�ʹ���ƽ���� */
#define MAX_ZC                  60         

/* FLL (��Ƶ��) ��������Ʋ��� */
/* 
 * FLL��·����Ƶ��(Hz) - ����ʵ�ʵĲ���������̬����ó�
 * �������� = (��ͨ������ / ADC������) * ƽ��֡��
 * ����Ƶ�� = 1 / ��������
 */
#define FLL_ACTUAL_UPDATE_RATE_HZ (CALIBRATION_SAMPLE_FREQ / (float)(FLL_UPDATE_INTERVAL * CALIBRATION_SIGNAL_CHANNEL_BUFFER_SIZE * FREQ_AVERAGE_FRAMES))
#define FLL_BANDWIDTH_HZ        0.05f    /* ��·����(Hz): �ָ����Կ�ķ�Χ���Ի�ø���ĳ�ʼ��Ӧ */
#define FLL_DAMPING_RATIO       0.9f    /* �����: ����0.9�����ƹ��� */
#define FLL_MAX_STEP_HZ         2.0f    /* ���������ڲ���(Hz): ����5Hz������������� */
#define FLL_DEAD_ZONE_HZ        0.05f   /* ����(Hz): �ʵ���С���û�������������С����̬��� */

/* ��������֡ƽ������ */
#define FREQ_AVERAGE_FRAMES     6       /* ƽ������֡���ݣ��ɸ�����Ҫ����(4-16) */
#define FLL_UPDATE_INTERVAL     FREQ_AVERAGE_FRAMES  /* ÿ������֡����һ��FLL */

/* FLL PI��������ɢ����Ĺ̶����� */
#define FLL_TS                  (1.0f / FLL_ACTUAL_UPDATE_RATE_HZ)
#define FLL_OMEGA_N             (2.0f * 3.1415926f * FLL_BANDWIDTH_HZ)
#define FLL_KP                  (2.0f * FLL_DAMPING_RATIO * FLL_OMEGA_N)
#define FLL_KI                  (FLL_OMEGA_N * FLL_OMEGA_N * FLL_TS)
#define FLL_ANTI_WINDUP_GAIN    (1.0f / FLL_KP)

/* �������������������� */
#define FLL_INTEGRATOR_MAX      20.0f   /* ���������ֵ (Hz) */
#define FLL_INTEGRATOR_MIN      (-20.0f) /* ��������Сֵ (Hz) */
#define FLL_RESET_THRESHOLD_HZ  0.05f    /* ���С�ڴ�ֵʱ���û����� (Hz) - �����ø�С */

/* ������У׼ֹͣ�߼����� */
#define FLL_STABLE_THRESHOLD_HZ 0.02f    /* Ƶ���ȶ��о���ֵ(Hz)��ӦС������ */
#define STABLE_FRAME_COUNT      10       /* ��������֡�ȶ���ֹͣУ׼ */


typedef struct{
    float freq;
    uint16_t wave_form;
}Signal_t;

/* PI������״̬�ṹ�� (����FLL) */
typedef struct {
    float integrator;       /* �����ۼ��� */
    float last_output;      /* ��һ�ε���� (����Anti-windup) */
    uint8_t stable_counter; /* �������ȶ������� */
    uint8_t calibration_stopped; /* ������У׼ֹͣ��־ */
} FLL_PI_Controller_t;

/* ��������λ����Goertzel״̬�ṹ�� */
typedef struct {
    float last_phase;       /* ��һ�ε���λֵ (����) */
    uint8_t phase_valid;    /* ��λ�Ƿ��ѳ�ʼ�� */
    float freq_estimate;    /* ��ǰƵ�ʹ���ֵ */
} Phase_Tracker_t;

/* ��������֡Ƶ��ƽ���������ṹ�� */
typedef struct {
    float freq_buffer[FREQ_AVERAGE_FRAMES];  /* Ƶ�ʲ��������� */
    uint8_t frame_count;                      /* ��ǰ֡���� */
    uint8_t buffer_full;                      /* �������Ƿ����� */
    float averaged_freq;                      /* ƽ�����Ƶ�� */
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

/* ��������ȷƵ�ʲ������� */
float precise_frequency_measurement(const float *signal_data, int N, float target_freq, Phase_Tracker_t *tracker);

/* ��������֡Ƶ��ƽ������ */
uint8_t add_frequency_measurement(Freq_Averager_t *averager, float new_freq);

#endif /* __DATA_PROCESS_H */
