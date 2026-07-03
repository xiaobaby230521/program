#ifndef __KEY_H
#define __KEY_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  ******************************************************************************
  * @file    key.h
  * @brief   按键驱动头文件：包含工作模式定义、按键/继电器引脚、函数声明
  ******************************************************************************
  */

/* 包含依赖头文件 */
#include "stm32g4xx_hal.h"
#include <stdint.h>
#include "control.h"


/* ==================== 硬件引脚宏定义 ==================== */
// 按键1：模式切换按键 引脚定义
#define KEY1_PIN        GPIO_PIN_14
#define KEY1_PORT       GPIOB
// 按键2：功能按键（继电器/电压调节）引脚定义
#define KEY2_PIN        GPIO_PIN_13
#define KEY2_PORT       GPIOB
// 按键3：参数+ 引脚定义
#define KEY3_PIN        GPIO_PIN_12
#define KEY3_PORT       GPIOB
// 按键4：参数- 引脚定义
#define KEY4_PIN        GPIO_PIN_11
#define KEY4_PORT       GPIOB

// 继电器控制引脚定义
#define RELAY_PIN       GPIO_PIN_7
#define RELAY_PORT      GPIOB

/* ==================== 按键电平定义 ==================== */
#define KEY_PRESSED     GPIO_PIN_RESET   // 按键按下：低电平有效
#define KEY_RELEASED    GPIO_PIN_SET     // 按键松开：高电平

/* ==================== 系统参数定义 ==================== */
#define KEY_DEBOUNCE_MS 20               // 按键消抖时间：20ms

/* ==================== 控制参数宏定义 ==================== */
// 并网/整流模式 电流参数
#define GRID_I_TARGET_MIN       0.0f      // 最小目标电流
#define GRID_I_TARGET_MAX       2.0f      // 最大目标电流
#define GRID_I_TARGET_STEP      0.2f      // 电流调节步长
#define GRID_I_TARGET_DEFAULT   0.2f      // 默认目标电流

// 离网模式 电流参数
#define OFFGRID_I_TARGET_MIN    0.0f
#define OFFGRID_I_TARGET_MAX    2.0f
#define OFFGRID_I_TARGET_STEP   0.2f
#define OFFGRID_I_TARGET_DEFAULT 2.0f

// 离网模式 电压参数
#define OFFGRID_U_TARGET_MIN    30.0f
#define OFFGRID_U_TARGET_MAX    33.0f
#define OFFGRID_U_TARGET_STEP   0.5f
#define OFFGRID_U_TARGET_DEFAULT 32.0f

/* ==================== 外部函数声明 ==================== */
/**
  * @brief  按键扫描函数（带消抖、中断保护）
  * @retval 按键值：0=无按键，1/2/3/4=对应按键
  */
uint8_t Key_Scan(void);

/**
  * @brief  工作模式切换处理函数
  * @note   按键1触发，循环切换4种工作模式
  */
void Mode_Switch_Handle(void);

/**
  * @brief  分模式按键功能处理函数
  * @param  key: 触发的按键编号(2/3/4)
  */
void Key_Function_Handle(uint8_t key);

/**
  * @brief  继电器硬件控制函数
  * @note   根据工作模式自动控制继电器通断
  */
void Relay_Control(void);

uint8_t Key_GetAndClear(void);





#ifdef __cplusplus
}
#endif

#endif /* __KEY_H */