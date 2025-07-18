/*
 * @Author: yyf 17786321727@163.com
 * @Date: 2025-07-12 15:11:29
 * @LastEditors: yyf 17786321727@163.com
 * @LastEditTime: 2025-07-18 09:50:34
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
#define CALIBRATION_SAMPLE_FREQ 600000.0f /* ADC������ (Ӳ���̶�) */
#define ADCLK                   25000000.0f /* AD9833 ʱ�� (Hz) */
#define MAVG                    15          /* ���ٵ�15�����ڣ�ȷ�����´��������㹻���� */
#define MAX_ZC                  200         /* ���ӵ�200����Ӧ���������ݴ��� */

/* FLL (��Ƶ��) ��������Ʋ��� */
#define FLL_BANDWIDTH_HZ        12.0f   /* ��·����(Hz): �ʵ���ߵ�12Hz����ǿ�������� */
#define FLL_DAMPING_RATIO       0.9f    /* �����: ����0.9�����ƹ��� */
#define FLL_MAX_STEP_HZ         5.0f    /* ���������ڲ���(Hz): ����5Hz������������� */
#define FLL_DEAD_ZONE_HZ        1.0f    /* ����(Hz): ��С��1Hz����ϵͳ��С�������� */


typedef struct{
    float freq;
    uint16_t wave_form;
}Signal_t;

/* PI������״̬�ṹ�� (����FLL) */
typedef struct {
    float integrator;       /* �����ۼ��� */
    float last_output;      /* ��һ�ε���� (����Anti-windup) */
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
