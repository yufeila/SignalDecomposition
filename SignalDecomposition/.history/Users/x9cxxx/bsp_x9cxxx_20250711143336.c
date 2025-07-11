#include "./x9cxxx/bsp_x9cxxx.h"

// ---- 推荐使用 DWT 延时或 HAL_DelayUs ----
static void delay_us(uint16_t us) {
    // 建议用 DWT, 这里只做简单软延时，具体可替换
    volatile uint32_t n = us * 20; // 168MHz下大致每us约20循环
    while(n--) __NOP();
}

// ---- 基本GPIO操作 ----
#define X9C_INC_HIGH()    HAL_GPIO_WritePin(X9C_INC_GPIO_PORT, X9C_INC_PIN, GPIO_PIN_SET)
#define X9C_INC_LOW()     HAL_GPIO_WritePin(X9C_INC_GPIO_PORT, X9C_INC_PIN, GPIO_PIN_RESET)
#define X9C_UD_HIGH()     HAL_GPIO_WritePin(X9C_UD_GPIO_PORT, X9C_UD_PIN, GPIO_PIN_SET)
#define X9C_UD_LOW()      HAL_GPIO_WritePin(X9C_UD_GPIO_PORT, X9C_UD_PIN, GPIO_PIN_RESET)
#define X9C_CS_HIGH()     HAL_GPIO_WritePin(X9C_CS_GPIO_PORT, X9C_CS_PIN, GPIO_PIN_SET)
#define X9C_CS_LOW()      HAL_GPIO_WritePin(X9C_CS_GPIO_PORT, X9C_CS_PIN, GPIO_PIN_RESET)

// ---- 1. 选中/取消芯片 ----
void X9C_Select(void)   { X9C_CS_LOW(); }
void X9C_Deselect(void) { X9C_CS_HIGH(); }

// ---- 2. 设置方向 ----
void X9C_SetDirection(uint8_t up) {
    if(up) X9C_UD_HIGH();
    else   X9C_UD_LOW();
}

// ---- 3. INC 脉冲函数（负边沿有效） ----
void X9C_IncPulse(void) {
    // INC 必须先高，再低，满足 t_IL、t_IH
    X9C_INC_HIGH();  delay_us(2);  // t_IH
    X9C_INC_LOW();   delay_us(2);  // t_IL
}

// ---- 4. 步进设置函数 ----
void X9C_SetPos(uint8_t pos) {
    // 步进前建议归零：先将U/D=0, 99次脉冲
    X9C_Select();
    X9C_SetDirection(0); // 向下（归零）
    for(int i=0; i<99; i++) X9C_IncPulse();

    // 然后向上到目标值
    X9C_SetDirection(1);
    for(int i=0; i<pos; i++) X9C_IncPulse();

    // 步进完成，保持 INC=低，再释放CS
    X9C_INC_LOW();
    X9C_Deselect();
}

// ---- 5. 非易失性存储写入 ----
// 步骤：先INC=高，再CS从低拉高，等待t_CPH
void X9C_Store(void) {
    X9C_INC_HIGH();
    delay_us(2);
    X9C_Deselect(); // CS 拉高
    delay_us(10);   // t_CPH
    X9C_INC_LOW();
}

// ---- 6. 初始化（必须先初始化IO） ----
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

// ---- 使用例 ----
// X9C_Init();
// X9C_SetPos(45);   // 设置第45步（归零再步进）
// X9C_Store();      // 存储到非易失NVM
