#ifndef PI_CONTROL_H
#define PI_CONTROL_H

#include "stm32g4xx_hal.h"
#include <math.h>

// 全局变量声明
extern float BUCK_I_CHARGE_REF;
extern float BUCK_V_PROTECT_MAX;
extern float BOOST_UO_REF;

/************************ BUCK充电系统核心参数（匹配仿真） ************************/
#define BUCK_UIN             30.0f        // BUCK输入电压(V)
#define BUCK_FS              100000.0f    // BUCK采样频率(Hz)，100kHz
#define BUCK_T_SAMPLE        (1.0f/BUCK_FS)    // 采样周期(s) = 10μs
#define BUCK_PI              3.1415926f   // 圆周率
// 新增：BUCK硬件参数（用于前馈计算，根据你的实际硬件调整）
#define BUCK_L                1e-3f       // BUCK电感值(H)
#define BUCK_C               470e-6f      // BUCK电容值(F)

/************************ BUCK电流环可调参数（最终调试值） ************************/
#define BUCK_KP_I            0.0500f      // 电流环PI比例系数（调试后最优值）
#define BUCK_KI_I            0.0600f      // 电流环PI积分系数（调试后最优值）
#define BUCK_F_LF            30000.0f     // 电流环低通滤波器截止频率(Hz)，30kHz
#define BUCK_TAU_LAG         1e-5f        // 电流环相位延迟时间常数(s)，10μs
#define BUCK_ALPHA_LAG       20.0f        // 电流环相位延迟衰减系数
// 新增：前馈增益系数（可调试）
#define BUCK_FF_ATTENUATION  1.0f         // 前馈衰减系数，防止补偿过度

/************************ BOOST系统参数（保留原定义，不修改） ************************/
#define BOOST_UIN            18.5f        // BOOST输入电压(V)
#define BOOST_FS             100000.0f    // BOOST采样频率(Hz)
#define BOOST_T_SAMPLE       (1.0f/BOOST_FS)    // BOOST采样周期(s)
#define BOOST_PI             3.1415926f   // BOOST圆周率
#define BOOST_FEEDFORWARD_GAIN 30.0f      // BOOST前馈补偿增益
#define BOOST_L              1e-3f        // BOOST电感值(H)
#define BOOST_C              470e-6f      // BOOST电容值(F)

/************************ BOOST电流环参数（保留原定义） ************************/
#define BOOST_KP_I           0.6000f      // BOOST电流环比例系数
#define BOOST_FC_I           5000.0f      // BOOST电流环低通截止频率(Hz)

/************************ BOOST电压环参数（保留原定义） ************************/
#define BOOST_KP_V           0.0015f      // BOOST电压环PI比例系数
#define BOOST_KI_V           0.4200f      // BOOST电压环PI积分系数
#define BOOST_FC_LEAD        3000.0f      // BOOST相位超前截止频率(Hz)
#define BOOST_ALPHA_LEAD     0.0300f      // BOOST相位超前衰减系数

/************************ 控制变量结构体（精简BUCK，保留BOOST） ************************/
typedef struct
{
    // BUCK恒流充电变量（移除电压环，仅保留电流环）
    float buck_e_i_prev;       // BUCK电流环上一周期误差
    float buck_u_i_prev;       // BUCK电流环上一周期输出
    float buck_x_lf_prev;      // BUCK低通滤波器上一周期输入
    float buck_y_lf_prev;      // BUCK低通滤波器上一周期输出
    float buck_x_lag_prev;     // BUCK相位延迟环节上一周期输入
    float buck_y_lag_prev;     // BUCK相位延迟环节上一周期输出
    float buck_duty;           // BUCK最终PWM占空比(0~1)
    // 新增：BUCK前馈补偿历史值
    float buck_d_ff_prev;      // BUCK前馈补偿上一周期值
    float buck_i_o_prev;       // BUCK负载电流上一周期采样值

    // BOOST变量（保留原定义，不修改）
    float boost_x_lf_prev;     // BOOST电流环低通滤波器上一周期输入
    float boost_y_lf_prev;     // BOOST电流环低通滤波器上一周期输出
    float boost_e_i_prev;      // BOOST电流环上一周期误差
    float boost_u_i_prev;      // BOOST电流环上一周期输出
    float boost_e_v_prev;      // BOOST电压环上一周期误差
    float boost_u_v_prev;      // BOOST电压环上一周期输出
    float boost_x_lead_prev;   // BOOST相位超前环节上一周期输入
    float boost_y_lead_prev;   // BOOST相位超前环节上一周期输出
    float boost_i_ref;         // BOOST电压环输出的电流参考值
    float boost_duty;          // BOOST最终PWM占空比(0~1)

} ControlData;

/************************ BUCK充电控制函数声明 ************************/
// 初始化控制结构体（清零历史值）
void ControlData_Init(ControlData *pdata);

// BUCK电流环低通滤波器（匹配仿真差分方程）
float BUCK_Current_LowPassFilter(ControlData *pdata, float x);

// BUCK电流环相位延迟环节（匹配仿真差分方程）
float BUCK_Current_PhaseLag(ControlData *pdata, float x);

// BUCK电流环PI控制器（恒流核心，匹配最终调试系数）
float BUCK_Current_PIControl(ControlData *pdata, float e_i);

// 新增：BUCK前馈补偿计算函数
float BUCK_Current_FeedForward(ControlData *pdata, float i_o_sample);

// BUCK恒流充电总控制函数（1A恒流+18.5V过压保护+前馈补偿）
// 输入：pdata-控制结构体，v_sample-电压采样值，i_sample-电感电流采样值，i_o_sample-负载电流采样值
// 输出：BUCK最终PWM占空比(0~1)，过压时直接置0
float BUCK_Charge_Control(ControlData *pdata, float v_sample, float i_sample, float i_o_sample);

/************************ BOOST函数声明（保留原定义） ************************/
float BOOST_Current_LowPassFilter(ControlData *pdata, float x);
float BOOST_Current_Control(ControlData *pdata, float e_i);
float BOOST_Voltage_PIControl(ControlData *pdata, float e_v);
float BOOST_Voltage_PhaseLead(ControlData *pdata, float x);
float BOOST_DoubleLoop_Control(ControlData *pdata, float v_sample, float i_sample);

#endif