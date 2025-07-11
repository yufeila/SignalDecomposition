#include "./x9cxxx/bsp_x9cxxx.h"

// ---- �Ƽ�ʹ�� DWT ��ʱ�� HAL_DelayUs ----
static void delay_us(uint16_t us) {
    // ������ DWT, ����ֻ��������ʱ��������滻
    volatile uint32_t n = us * 20; // 168MHz�´���ÿusԼ20ѭ��
    while(n--) __NOP();
}

// ---- ����GPIO���� ----
#define X9C_INC_HIGH()    HAL_GPIO_WritePin(X9C_INC_GPIO_PORT, X9C_INC_PIN, GPIO_PIN_SET)
#define X9C_INC_LOW()     HAL_GPIO_WritePin(X9C_INC_GPIO_PORT, X9C_INC_PIN, GPIO_PIN_RESET)
#define X9C_UD_HIGH()     HAL_GPIO_WritePin(X9C_UD_GPIO_PORT, X9C_UD_PIN, GPIO_PIN_SET)
#define X9C_UD_LOW()      HAL_GPIO_WritePin(X9C_UD_GPIO_PORT, X9C_UD_PIN, GPIO_PIN_RESET)
#define X9C_CS_HIGH()     HAL_GPIO_WritePin(X9C_CS_GPIO_PORT, X9C_CS_PIN, GPIO_PIN_SET)
#define X9C_CS_LOW()      HAL_GPIO_WritePin(X9C_CS_GPIO_PORT, X9C_CS_PIN, GPIO_PIN_RESET)

// ---- 1. ѡ��/ȡ��оƬ ----
void X9C_Select(void)   { X9C_CS_LOW(); }
void X9C_Deselect(void) { X9C_CS_HIGH(); }

// ---- 2. ���÷��� ----
void X9C_SetDirection(uint8_t up) {
    if(up) X9C_UD_HIGH();
    else   X9C_UD_LOW();
}

// ---- 3. INC ���庯������������Ч�� ----
void X9C_IncPulse(void) {
    // INC �����ȸߣ��ٵͣ����� t_IL��t_IH
    X9C_INC_HIGH();  delay_us(2);  // t_IH
    X9C_INC_LOW();   delay_us(2);  // t_IL
}

// ---- 4. �������ú��� ----
void X9C_SetPos(uint8_t pos) {
    // ����ǰ������㣺�Ƚ�U/D=0, 99������
    X9C_Select();
    X9C_SetDirection(0); // ���£����㣩
    for(int i=0; i<99; i++) X9C_IncPulse();

    // Ȼ�����ϵ�Ŀ��ֵ
    X9C_SetDirection(1);
    for(int i=0; i<pos; i++) X9C_IncPulse();

    // ������ɣ����� INC=�ͣ����ͷ�CS
    X9C_INC_LOW();
    X9C_Deselect();
}

// ---- 5. ����ʧ�Դ洢д�� ----
// ���裺��INC=�ߣ���CS�ӵ����ߣ��ȴ�t_CPH
void X9C_Store(void) {
    X9C_INC_HIGH();
    delay_us(2);
    X9C_Deselect(); // CS ����
    delay_us(10);   // t_CPH
    X9C_INC_LOW();
}

// ---- 6. ��ʼ���������ȳ�ʼ��IO�� ----
void X9C_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = X9C_INC_PIN | X9C_UD_PIN | X9C_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    X9C_Deselect();
    X9C_INC_LOW();
}

// ---- ʹ���� ----
// X9C_Init();
// X9C_SetPos(45);   // ���õ�45���������ٲ�����
// X9C_Store();      // �洢������ʧNVM
