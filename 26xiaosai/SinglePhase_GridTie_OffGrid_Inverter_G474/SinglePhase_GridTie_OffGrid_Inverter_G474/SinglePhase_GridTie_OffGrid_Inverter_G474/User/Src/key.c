#include "key.h"
#include "control.h"
#include <string.h>

// ==================== 静态全局变量 (仅本文件可见) ====================
static uint32_t key_last_tick = 0;  // 记录上一次按键检测的时间 (用于消抖)
static uint8_t  key_last = 0;        // 记录上一次的按键状态 (1/2/3/4 或 0)
static uint8_t  key_pending = 0;     // 暂存有效的按键事件 (等待主循环读取)

// ==================== 外部变量声明 (来自其他文件) ====================
extern WorkMode_TypeDef work_mode;       // 当前工作模式
extern uint8_t          relay_state;     // 继电器状态
extern uint8_t          pll_enable;      // 锁相环使能标志
extern float            grid_i_target;    // 并网/整流电流目标值
extern float            offgrid_Uoac_target; // 离网电压目标值
extern uint8_t          adc_isr_busy;    // ADC中断忙标志 (用于互斥)

// ==================== 外部函数声明 ====================
extern void sys_stop(void);
extern void sys_start(void);
extern void Controller_Rect_Init_All(void);
extern void Controller_Rect_Reset_All(void);
extern void Controller_Offgrid_Init_All(void);
extern void Controller_Offgrid_Reset_All(void);
extern void Controller_Ongrid_Init_All(void);
extern void Controller_Ongrid_Reset_All(void);
extern void Controller_SOGITest_Init_All(void);
extern void Controller_SOGITest_Reset_All(void);
extern void Controller_BuckTest_Init_All(void);
extern void Controller_BuckTest_Reset_All(void);
extern void Controller_BoostTest_Init_All(void);
extern void Controller_BoostTest_Reset_All(void);




/**
  * @brief  按键扫描函数 (需在主循环高频调用)
  * @note   实现了消抖逻辑和按键释放检测 (只在松开时才触发事件)
  * @retval 当前暂存的按键值 (如果有)
  */
uint8_t Key_Scan(void)
{
    uint8_t key_now = 0;

    // 1. 消抖检测：如果距离上次检测时间太短，直接返回上次结果
    if(HAL_GetTick() - key_last_tick < KEY_DEBOUNCE_MS) 
        return key_pending;
    
    key_last_tick = HAL_GetTick(); // 更新时间戳

    // 2. 读取当前按键硬件状态 (低电平有效)
    if(HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN) == KEY_PRESSED) key_now = 1;
    else if(HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN) == KEY_PRESSED) key_now = 2;
    else if(HAL_GPIO_ReadPin(KEY3_PORT, KEY3_PIN) == KEY_PRESSED) key_now = 3;
    else if(HAL_GPIO_ReadPin(KEY4_PORT, KEY4_PIN) == KEY_PRESSED) key_now = 4;
    else key_now = 0;

    // 3. 沿触发检测：只有当 "上次按下(非0) && 这次松开(0)" 时，才确认按键有效
    if(key_last != 0 && key_now == 0)
    {
        if(key_pending == 0) // 如果暂存区为空，存入新按键
            key_pending = key_last;
    }

    key_last = key_now; // 更新历史状态
    return key_pending;
}

/**
  * @brief  获取并清除按键事件
  * @note   主循环调用此函数来获取按键，读取后自动清空
  * @retval 按键值 (1-4 或 0)
  */
uint8_t Key_GetAndClear(void)
{
    uint8_t k = key_pending;
    key_pending = 0; // 清空暂存区
    return k;
}

/**
  * @brief  继电器统一控制逻辑
  * @note   根据当前模式决定继电器状态
  */
void Relay_Control(void)
{
    // 待机模式和离网模式强制断开继电器
    if(work_mode == MODE_STANDBY || work_mode == MODE_OFFGRID)
    {
        HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_RESET);
        return;
    }
    // 其他模式根据 relay_state 变量控制
    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, relay_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief  安全停止当前运行的模式
  * @note   关闭使能、停止PWM、复位控制器、断开继电器
  */
static void Safe_Stop_Current_Mode(void)
{
    // 声明外部变量
    extern volatile uint8_t rect_calc_enable;
    extern volatile uint8_t offgrid_calc_enable;
    extern volatile uint8_t ongrid_calc_enable;
    extern volatile uint8_t buck_test_calc_enable;
    extern volatile uint8_t boost_test_calc_enable;
    extern volatile uint8_t sogi_test_calc_enable;

    // 1. 关闭所有计算使能 (中断里检测到使能为0就会停止计算)
    rect_calc_enable = 0;
    offgrid_calc_enable = 0;
    ongrid_calc_enable = 0;
    buck_test_calc_enable = 0;
    boost_test_calc_enable = 0;
    sogi_test_calc_enable = 0;

    // 2. 停止硬件PWM输出
    sys_stop();
    
    // 3. 断开继电器
    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_RESET);
    relay_state = 0;
    
    // 4. 释放互斥标志
    adc_isr_busy = 0;

    // 5. 根据当前模式调用对应的复位函数 (清空算法状态)
    switch(work_mode)
    {
        case MODE_RECTIFIER: Controller_Rect_Reset_All(); break;
        case MODE_OFFGRID:   Controller_Offgrid_Reset_All(); break;
        case MODE_ONGRID:    Controller_Ongrid_Reset_All(); break;
        case MODE_BUCK:      Controller_BuckTest_Reset_All(); break;
        case MODE_BOOST:     Controller_BoostTest_Reset_All(); break;
        case MODE_SOGI:      Controller_SOGITest_Reset_All(); break;
        default: break;
    }
}

/**
  * @brief  K1键功能：模式切换
  * @note   循环切换：待机 -> 整流 -> 离网 -> 并网 -> 待机...
  */
void Mode_Switch_Handle(void)
{
    // 如果ADC中断正在计算，拒绝切换 (防止冲突)
    if(adc_isr_busy == 1) return;

    extern volatile uint8_t rect_calc_enable;
    extern volatile uint8_t offgrid_calc_enable;
    extern volatile uint8_t ongrid_calc_enable;
    extern volatile uint8_t buck_test_calc_enable;
    extern volatile uint8_t boost_test_calc_enable;
    extern volatile uint8_t sogi_test_calc_enable;

    // 1. 如果当前有模式在运行，先安全停止它
    if(rect_calc_enable || offgrid_calc_enable || ongrid_calc_enable || 
       buck_test_calc_enable || boost_test_calc_enable || sogi_test_calc_enable)
    {
        Safe_Stop_Current_Mode();
    }

    // 2. 切换到下一个模式
    work_mode++;
    if(work_mode > MODE_ONGRID) work_mode = MODE_STANDBY; // 超过并网模式则回到待机

    // 3. 初始化新模式的默认状态
    relay_state = 0;
    pll_enable = 0;

    switch(work_mode)
    {
        case MODE_STANDBY: 
            break;
        case MODE_RECTIFIER: 
            pll_enable = 1; 
            grid_i_target = GRID_I_TARGET_DEFAULT; 
            break;
        case MODE_OFFGRID: 
            relay_state = 0; // 离网模式默认断开电网继电器
            offgrid_Uoac_target = OFFGRID_U_TARGET_DEFAULT; 
            break;
        case MODE_ONGRID: 
            pll_enable = 1; 
            grid_i_target = GRID_I_TARGET_DEFAULT; 
            break;
        default: 
            work_mode = MODE_STANDBY; 
            break;
    }
}

/**
  * @brief  K2/K3/K4 键功能处理 (上下文相关)
  * @param  key: 按键值 (2, 3 或 4)
  * @note   已完全对齐 key.h 中的宏定义
  */
void Key_Function_Handle(uint8_t key)
{
    extern volatile uint8_t rect_calc_enable;
    extern volatile uint8_t offgrid_calc_enable;
    extern volatile uint8_t ongrid_calc_enable;
    extern volatile uint8_t buck_test_calc_enable;
    extern volatile uint8_t boost_test_calc_enable;
    extern volatile uint8_t sogi_test_calc_enable;
    extern float buck_test_i_target;
    extern float boost_test_v_target;

    // 如果ADC中断正在计算，忽略按键
    if(adc_isr_busy == 1) return;

    switch(work_mode)
    {
        // ==================== 待机模式：进入测试模式 ====================
        case MODE_STANDBY:
            if(key == 2) // K2: 进入 Buck 测试
            {
                Controller_BuckTest_Init_All();
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // 打开驱动使能
                sys_start();
                buck_test_calc_enable = 1;
                work_mode = MODE_BUCK;
            }
            else if(key == 3) // K3: 进入 Boost 测试
            {
                Controller_BoostTest_Init_All();
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
                sys_start();
                boost_test_calc_enable = 1;
                work_mode = MODE_BOOST;
            }
            else if(key == 4) // K4: 进入 SOGI 锁相环测试
            {
                Controller_SOGITest_Init_All();
                sogi_test_calc_enable = 1;
                work_mode = MODE_SOGI;
            }
            break;

        // ==================== Buck 测试模式 ====================
        case MODE_BUCK:
            switch(key)
            {
                case 2: // K2: 退出测试
                    Controller_BuckTest_Reset_All(); 
                    work_mode = MODE_STANDBY; 
                    break;
                case 3: // K3: 增加电流 (使用宏)
                    buck_test_i_target += GRID_I_TARGET_STEP; 
                    if(buck_test_i_target > GRID_I_TARGET_MAX) 
                        buck_test_i_target = GRID_I_TARGET_MAX; 
                    break;
                case 4: // K4: 减小电流 (使用宏，注意：Buck最小限制为0.2A，宏定义MIN是0.0f，这里保持0.2f的逻辑)
                    buck_test_i_target -= GRID_I_TARGET_STEP; 
                    if(buck_test_i_target < 0.2f) // 这里可以单独加个 BUCK_I_TARGET_MIN 宏，或者保持现状
                        buck_test_i_target = 0.2f; 
                    break;
            }
            break;

        // ==================== Boost 测试模式 ====================
        case MODE_BOOST:
            if(key == 3) // K3: 退出测试
            {
                Controller_BoostTest_Reset_All();
                work_mode = MODE_STANDBY;
            }
            break;

        // ==================== SOGI 测试模式 ====================
        case MODE_SOGI:
            if(key == 4) // K4: 退出测试
            {
                Controller_SOGITest_Reset_All();
                work_mode = MODE_STANDBY;
            }
            break;

        // ==================== 整流模式 (使用宏) ====================
        case MODE_RECTIFIER:
            if(key == 2) // K2: 启动/停止
            {
                if(rect_calc_enable) 
                    Safe_Stop_Current_Mode();
                else { 
                    Controller_Rect_Init_All(); 
                    rect_calc_enable = 1; 
                }
            }
            else if(key == 3){ // K3: 增加电流
                grid_i_target += GRID_I_TARGET_STEP; 
                if(grid_i_target > GRID_I_TARGET_MAX) 
                    grid_i_target = GRID_I_TARGET_MAX; 
            }
            else if(key == 4){ // K4: 减小电流
                grid_i_target -= GRID_I_TARGET_STEP; 
                if(grid_i_target < 0.2f) // 同样，最小限制为0.2A
                    grid_i_target = 0.2f; 
            }
            break;

        // ==================== 离网模式 (完全使用宏) ====================
        case MODE_OFFGRID:
            if(key == 2) // K2: 启动/停止
            {
                if(offgrid_calc_enable) 
                    Safe_Stop_Current_Mode();
                else { 
                    Controller_Offgrid_Init_All(); 
                    offgrid_calc_enable = 1; 
                }
            }
            else if(key == 3){ // K3: 增加电压
                offgrid_Uoac_target += OFFGRID_U_TARGET_STEP; 
                if(offgrid_Uoac_target > OFFGRID_U_TARGET_MAX) 
                    offgrid_Uoac_target = OFFGRID_U_TARGET_MAX; 
            }
            else if(key == 4){ // K4: 减小电压
                offgrid_Uoac_target -= OFFGRID_U_TARGET_STEP; 
                if(offgrid_Uoac_target < OFFGRID_U_TARGET_MIN) 
                    offgrid_Uoac_target = OFFGRID_U_TARGET_MIN; 
            }
            break;

        // ==================== 并网模式 (使用宏) ====================
        case MODE_ONGRID:
            if(key == 2) // K2: 启动/停止
            {
                if(ongrid_calc_enable) 
                    Safe_Stop_Current_Mode();
                else { 
                    Controller_Ongrid_Init_All(); 
                    ongrid_calc_enable = 1; 
                }
            }
            else if(key == 3){ // K3: 增加电流
                grid_i_target += GRID_I_TARGET_STEP; 
                if(grid_i_target > GRID_I_TARGET_MAX) 
                    grid_i_target = GRID_I_TARGET_MAX; 
            }
            else if(key == 4){ // K4: 减小电流
                grid_i_target -= GRID_I_TARGET_STEP; 
                if(grid_i_target < 0.2f) 
                    grid_i_target = 0.2f; 
            }
            break;
        default: break;
    }
}
