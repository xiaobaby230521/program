#ifndef __CALCULATE_H
#define __CALCULATE_H

#include "stm32g4xx_hal.h"
#include "arm_math.h"
#include "math.h"
#include <string.h>


// ===================== 重复控制器固定参数（N=400, 20kHz对应50Hz） =====================
#define RC_N_FIXED        400     // 固定采样点数
#define RC_MAX_ORDER      (RC_N_FIXED + 2) // 402
#define RC_ARRAY_LEN      (RC_MAX_ORDER + 1) // 403

// ===================== 整流专用 QPR 控制器 结构体 =====================
typedef struct {
    // 预计算系数
    float b0_1, b1_1, b2_1, a1_1, a2_1, a0_1_inv;
    float b0_3, b1_3, b2_3, a1_3, a2_3, a0_3_inv;
    float b0_5, b1_5, b2_5, a1_5, a2_5, a0_5_inv;
    float b0_7, b1_7, b2_7, a1_7, a2_7, a0_7_inv;
    
    // 状态变量
    float e1_1, e1_2, m1_1, m1_2;
    float e3_1, e3_2, m3_1, m3_2;
    float e5_1, e5_2, m5_1, m5_2;
    float e7_1, e7_2, m7_1, m7_2;
} QPR_Rect_t;



// ===================== 100Hz 陷波滤波器 结构体 (用于滤除二次纹波) =====================
typedef struct
{
    float b0, b1, b2;   // 分子系数
    float a1, a2;       // 分母系数 (a0=1已归一化)
    float x_prev1, x_prev2; // 输入历史
    float y_prev1, y_prev2; // 输出历史
} Notch100Hz_TypeDef;

// ===================== SOGI-FLL 结构体 (二阶广义积分器锁频环) =====================
typedef struct
{
    float x1_prev, x2_prev, x3_prev; // 内部状态变量
    float omega0_prev, U_prev, integral_f; // 频率历史、输入历史、频率积分项
    float alpha, beta, omega0; // 输出：正交分量 & 当前角频率
    float dt, K, K1, gamma_p, gamma_i, w_max, w_min; // 参数
    float two_over_dt; // 【优化】预计算 2.0/dt，消除运行时除法
} SOGI_FLL_TypeDef;



// ===================== 级联陷波结构体 (150/250/350Hz) =====================
typedef struct
{
    // 状态变量
    float x3_1, x3_2, x5_1, x5_2, x7_1, x7_2;
    // 系数
    float b3_0, b3_1, b3_2, a3_1, a3_2;
    float b5_0, b5_1, b5_2, a5_1, a5_2;
    float b7_0, b7_1, b7_2, a7_1, a7_2;
} CascadeNotch_TypeDef;

typedef struct {
    float Ts;           // 采样周期
    float kp_pll;       // PI 比例增益
    float ki_pll;       // PI 积分增益
    float omega_ff;     // 前馈角频率
    float rad2deg;      // 【新增】弧度转角度因子 (180/PI)

    float theta;        // 当前相位
    float theta_prev;   // 上一次相位
    float integral_pll; // 积分器
    float qerror;       // q轴误差

    float sin_theta;    // 缓存 sin
    float cos_theta;    // 缓存 cos
} SRF_PLL_TypeDef;



// ===================== 通用PI结构体 (带抗积分饱和) =====================
typedef struct
{
    float integral_val;  // 积分器值
    float Kp, Ki, Ts;   // 参数
    float OUT_MAX, OUT_MIN, LIMIT; // 限幅
    float out;           // 输出
} PI_TypeDef;



// ===================== RC重复控制器=====================
typedef struct {
    float buffer[402];    // 环形缓冲区 (L=402)
    uint32_t ptr;         // 环形指针
    uint8_t is_initialized;
} RC_TypeDef;



// ===================== 逆变并网准谐振PR控制器 结构体 =====================
typedef struct
{
    // 参数
    float Kp, Kr, w0, wi, Ts;
    // 预计算系数
    float b0, b1, b2, a1, a2, a0_inv;
    // 状态变量
    float x_prev1, x_prev2, y_prev1, y_prev2;
    uint8_t first_run;
} PR_Inverter_t;  // 逆变PR控制器


// ====================================================================================
// 【拆分1/3】电网电压比例前馈
// ====================================================================================
typedef struct {
    float gain_ff;          // 前馈增益：1/Vdc
    float Vdc_nominal;      // 额定母线电压
} FF_Proportional_t;

// ====================================================================================
// 【拆分2/3】电容电流有源阻尼 (基于电网电压微分)
// ====================================================================================
typedef struct {
    float b0, b1;          // 分子系数 (预计算)
    float a1;              // 分母系数 (预计算)
    float u_prev_damp;     // 上一拍输入 u(k-1)
    float y_prev_damp;     // 上一拍输出 y(k-1)
    
    // 配置参数 (保留用于初始化)
    float Ts;              // 采样周期
    float C;               // 滤波电容
    float Hi1;             // 阻尼系数
    float f_cutoff;        // 低通截止频率 (1kHz)
} FF_ActiveDamping_t;


// ====================================================================================
// 【拆分3/3】二阶微分前馈 (含频率预畸变)
// ====================================================================================
typedef struct {
    float num2, num1, num0; // 分子系数
    float den2, den1, den0; // 分母系数
    float den2_inv;          // 分母倒数 (优化除法)
    float u_prev1_ff, u_prev2_ff; // 输入历史
    float y_prev1_ff, y_prev2_ff; // 输出历史
    float Ts;               // 采样周期
    float L1, C;            // 滤波电感、电容
    float k_pwm;            // 逆变器增益
    float f_cutoff;         // 截止频率
    float zeta;             // 阻尼比
} FF_SecondOrder_t;



typedef struct
{
    // ===================== 原有变量 (完全不动) =====================
    uint8_t AC_State;
    float derta_VAC;
    uint8_t Covernt_EN;
    uint16_t PLL_Discern_Counter;
    uint16_t PLL_Discern_Counter_MAX;
    
    // ===================== 【新增】过零预判变量 =====================
    float Vac_prev1;                // 上一个采样点电压
    float Vac_prev2;                // 上上个采样点电压
    uint8_t Relay_Pretrigger_Flag;  // 继电器预触发标志
    float ZCD_Pretrigger_Level;     // 过零预判电压阈值 (32V)
    uint16_t Relay_Delay_Samples;   // 继电器延迟补偿采样点数 (5ms @20kHz = 100)
    uint16_t Trend_Confirm_Cnt;     // 趋势连续确认计数 (防止抖动)
    uint16_t Trend_Confirm_Max;     // 趋势连续确认最大值 (设为10)
    
    // ===================== 新周期标志 =====================
    uint8_t New_Cycle_Flag;         // 过零检测完成一个周期的标志
    
} Rect_Lock_ZCD_TypeDef;



// ====================================================================================
// ===================== 【适配 -π~π 版】闭环切入专用过零判断 =====================
// 对接：你的SRF-PLL（theta范围 -PI ~ PI）
// 功能：在负->正过零点（theta=0）精准触发闭环切入
// ====================================================================================
typedef struct {
    // 配置参数
    float PLL_QError_Thresh;        // PLL锁定误差阈值（建议0.15）
    uint16_t PLL_Lock_Cycles_Max;   // PLL锁定需要的连续周期数（建议15）

    // 状态机变量
    uint8_t AC_State;                // 相位状态机（0~2）
    uint8_t last_cycle_flag;         // 去抖动标志

    // 输出标志
    uint8_t PLL_Locked;              // PLL是否已锁定
    uint8_t New_Cycle_Flag;          // 【用户用】新周期开始标志（在过零点）
    uint8_t ZCD_Trigger_Flag;        // 【用户用】过零触发标志（立即切入）
    
    // 内部变量
    uint16_t pll_lock_counter;       // PLL锁定计数器
} ZCD_StateMachine_t;



// ===================== 函数声明 =====================
// 电网电压全前馈
// 比例前馈函数声明
void FF_Proportional_Init(FF_Proportional_t *ff);
float FF_Proportional_Update(FF_Proportional_t *ff, float u_grid);
void FF_Proportional_Reset(FF_Proportional_t *ff);
// 有源阻尼函数声明
void FF_ActiveDamping_Init(FF_ActiveDamping_t *ff);
float FF_ActiveDamping_Update(FF_ActiveDamping_t *ff, float u_grid);
void FF_ActiveDamping_Reset(FF_ActiveDamping_t *ff);
// 二阶微分前馈函数声明
void FF_SecondOrder_Init(FF_SecondOrder_t *ff);
float FF_SecondOrder_Update(FF_SecondOrder_t *ff, float u_grid);
void FF_SecondOrder_Reset(FF_SecondOrder_t *ff);




// 逆变PR控制器
void PR_Inverter_Init(PR_Inverter_t *pr);
float PR_Inverter_Update(PR_Inverter_t *pr, float u);
void PR_Inverter_Reset(PR_Inverter_t *pr);


// 整流QPR控制器
void QPR_Rect_Init(QPR_Rect_t *qpr);
void QPR_Rect_Reset(QPR_Rect_t *qpr);
float QPR_Rect_Update(QPR_Rect_t *qpr, float i_error);

// 100Hz陷波
void Notch100Hz_Init(Notch100Hz_TypeDef *hnotch);
float Notch100Hz_Update(Notch100Hz_TypeDef *hnotch, float IGm_total);
void Notch100Hz_Reset(Notch100Hz_TypeDef *hnotch);

// SOGI-FLL (原版：矩阵库版本)
void SOGI_FLL_Init(SOGI_FLL_TypeDef *sogi);
void SOGI_FLL_Update(SOGI_FLL_TypeDef *sogi, float uin);
void SOGI_FLL_Reset(SOGI_FLL_TypeDef *sogi);

// SOGI-FLL (优化版：纯标量运算，性能提升8-10倍)
void SOGI_FLL_Update_Optimized(SOGI_FLL_TypeDef *sogi, float uin);

// 级联陷波
void CascadeNotch_Init(CascadeNotch_TypeDef *filter);
float CascadeNotch_Update(CascadeNotch_TypeDef *filter, float u);
void CascadeNotch_Reset(CascadeNotch_TypeDef *filter);

// SRF-PLL
void SRF_PLL_Init(SRF_PLL_TypeDef *pll);
void SRF_PLL_Update(SRF_PLL_TypeDef *pll, float u_alpha, float u_beta);
void SRF_PLL_Reset(SRF_PLL_TypeDef *pll);

// 通用PI
void PI_Init(PI_TypeDef *pi);
float PI_Update(PI_TypeDef *pi, float ref, float fb);
void PI_Reset(PI_TypeDef *pi);

// 重复控制器RC (新版：无参数传入，风格统一)
void RC_Init(RC_TypeDef *rc);
void RC_Reset(RC_TypeDef *rc);
float RC_Update(RC_TypeDef *rc, float e_in);


// 并网锁相及过零检测
void Rect_Lock_ZCD_Init(Rect_Lock_ZCD_TypeDef *zcd);
void Rect_Lock_ZCD_Update(Rect_Lock_ZCD_TypeDef *zcd, float Vac, float PLL_UQ, float theta);
void Rect_Lock_ZCD_Reset(Rect_Lock_ZCD_TypeDef *zcd);


void sys_start(void); // 系统启动函数
void sys_stop(void);  // 系统停止函数


// 并网锁相及过零检测
void ZCD_StateMachine_Init(ZCD_StateMachine_t *zcd);
void ZCD_StateMachine_Reset(ZCD_StateMachine_t *zcd);
void ZCD_StateMachine_Update(ZCD_StateMachine_t *zcd, float pll_qerror, float pll_theta);

// ==========================================
// 【新增】并网模式硬件启动函数声明
// ==========================================
void GridTie_Boost_Start(void);
void GridTie_Inverter_Start(void);


int32_t SPWM_SinglePolarity_CCR(float modulate);

#endif
