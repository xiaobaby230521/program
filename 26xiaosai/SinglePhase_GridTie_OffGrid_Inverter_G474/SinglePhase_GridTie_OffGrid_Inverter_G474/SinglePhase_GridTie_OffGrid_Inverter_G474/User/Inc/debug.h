#ifndef __DEBUG_H_
#define __DEBUG_H_

/**** 头文件 ****/
#include <stdio.h>
#include <string.h> 
#include <stdlib.h>
#include "main.h"
#include "usart.h"

/**** 宏定义 ****/
#define Debug_Toggle()        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2)

#define Debug_BUF_Len     32
enum Rx_State
{
    rx_Free=0,     // 空闲
    rx_Start,      // 开始
    rx_Stop,       // 停止
    rx_Note1       // 方向一
};

struct Debug_Param
{
    __IO uint8_t tmp_data;
    __IO uint8_t buffer[Debug_BUF_Len];   // 接收数据
    __IO uint16_t index;        // buffer的索引
    __IO uint8_t flag;          // 完成标志位 1
    enum Rx_State state;        // 状态
};


/**** 函数 ****/
void Debug_Init(void);
void VOFA_Init(void);

extern uint8_t vofa_data[4*8];


#endif // __DEBUG_H_
