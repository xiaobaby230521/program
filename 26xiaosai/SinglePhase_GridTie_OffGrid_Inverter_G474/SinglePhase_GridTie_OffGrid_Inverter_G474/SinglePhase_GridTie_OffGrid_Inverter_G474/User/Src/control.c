#include "control.h"
#include "usart.h"
#include "debug.h"
#include "key.h"
#include "tim.h"
#include "gpio.h"
#include "calculate.h"
#include "arm_math.h"
#include <string.h>

/* ==================== 全局变量 ==================== */
// 【唯一定义】各模式计算使能标志（中断/主循环共享）
// 设计意图：使用 volatile 防止编译器优化，确保主循环和中断看到的是同一个值
volatile uint8_t rect_calc_enable = 0;     // 整流模式使能：1=运行计算，0=停止
volatile uint8_t offgrid_calc_enable = 0;  // 离网模式使能：1=运行计算，0=停止
volatile uint8_t ongrid_calc_enable = 0;   // 并网模式使能：1=运行计算，0=停止
volatile uint8_t buck_calc_enable = 0;     // 【新增】BUCK标准使能（和其他模式统一）



// --- 故障标志变量 ---
// 设计意图：故障发生后置1，由主循环锁存并处理，中断仅负责置位
uint8_t OVP_DC = 0;         // 直流母线过压故障 (Over Voltage Protection)
uint8_t UVP_DC = 0;         // 直流母线欠压故障 (Under Voltage Protection)
uint8_t OCP_BOOST = 0;      // Boost侧过流故障 (Over Current Protection)
uint8_t OCP_CONV = 0;       // 变换器侧(H桥)过流故障
uint8_t OVP_OFFGRID = 0;      // 【新增】离网输出过压故障
uint8_t OCP_GRID = 0;          // 【新增】网侧电流过流故障
uint8_t PLL_FAULT = 0;      // 锁相环失锁故障 (Phase Locked Loop)
uint8_t OCP_BUCK_RMS = 0;    // 【新增】Buck侧电流有效值过流故障

uint8_t Fault_Flag = 0;     // 总故障标志：任意故障发生后置1，锁存状态供主循环查询

// --- 故障记忆变量 --- (唯一一份定义)
FaultCode_t last_fault_code = FAULT_NONE;

// --- 全局状态变量 ---
uint8_t adc_isr_busy = 0;    // ADC中断忙标志：用于按键扫描互锁，防止按键在中断计算时修改参数
uint16_t ADC_Sample[7];      // ADC原始数据缓冲区：DMA自动搬运数据到这里，7个通道对应7路采样


// ==========================================
// 【通用配置】启动与保护参数
// ==========================================
uint32_t g_fault_mask_cnt = 0;           // 故障屏蔽/软启动计数器：启动初期屏蔽非致命故障，防止误触发
#define FAULT_MASK_TIME           20000  // 软启动屏蔽时间：1秒 (10000个中断点 @ 20kHz)
#define FATAL_BUS_UV_THRESHOLD    18.0f  // 致命低压阈值：低于此值视为母线短路或严重故障，立即停机
#define NORMAL_BUS_UV_THRESHOLD   50.0f  // 正常运行欠压阈值：软启动结束后检测，低于此值报警
#define BUS_OV_THRESHOLD          65.0f  // 母线过压阈值：高于此值视为过压，立即停机
#define FAULT_TRIGGER_CNT          5      // 故障连续检测次数：需连续检测到5次才触发，防止干扰误触发



// ==========================================
// 【整流模式】专用变量定义
// ==========================================
//整流预充电占空比
const uint16_t CCR_SinTable[400] = {
4250,4307,4363,4420,4477,4533,4590,4646,4703,4759,4815,4871,4927,4983,5038,5093,5148,5203,5258,5312,
5366,5420,5474,5527,5580,5632,5685,5737,5788,5839,5890,5940,5990,6040,6089,6138,6186,6233,6281,6327,
6373,6419,6464,6509,6553,6596,6639,6681,6723,6764,6804,6844,6883,6922,6960,6997,7033,7069,7104,7139,
7173,7206,7238,7269,7300,7330,7359,7388,7416,7443,7469,7494,7519,7542,7565,7588,7609,7629,7649,7668,
7686,7703,7719,7734,7749,7763,7775,7787,7799,7809,7818,7826,7834,7841,7846,7851,7855,7858,7861,7862,
7863,7862,7861,7858,7855,7851,7846,7841,7834,7826,7818,7809,7799,7787,7775,7763,7749,7734,7719,7703,
7686,7668,7649,7629,7609,7588,7565,7542,7519,7494,7469,7443,7416,7388,7359,7330,7300,7269,7238,7206,
7173,7139,7104,7069,7033,6997,6960,6922,6883,6844,6804,6764,6723,6681,6639,6596,6553,6509,6464,6419,
6373,6327,6281,6233,6186,6138,6089,6040,5990,5940,5890,5839,5788,5737,5685,5632,5580,5527,5474,5420,
5366,5312,5258,5203,5148,5093,5038,4983,4927,4871,4815,4759,4703,4646,4590,4533,4477,4420,4363,4307,
4250,4193,4137,4080,4023,3967,3910,3854,3797,3741,3685,3629,3573,3517,3462,3407,3352,3297,3242,3188,
3134,3080,3026,2973,2920,2868,2815,2763,2712,2661,2610,2560,2510,2460,2411,2362,2314,2267,2219,2173,
2127,2081,2036,1991,1947,1904,1861,1819,1777,1736,1696,1656,1617,1578,1540,1503,1467,1431,1396,1361,
1327,1294,1262,1231,1200,1170,1141,1112,1084,1057,1031,1006,981,958,935,912,891,871,851,832,
814,797,781,766,751,737,725,713,701,691,682,674,666,659,654,649,645,642,639,638,
638,638,639,642,645,649,654,659,666,674,682,691,701,713,725,737,751,766,781,797,
814,832,851,871,891,912,935,958,981,1006,1031,1057,1084,1112,1141,1170,1200,1231,1262,1294,
1327,1361,1396,1431,1467,1503,1540,1578,1617,1656,1696,1736,1777,1819,1861,1904,1947,1991,2036,2081,
2127,2173,2219,2267,2314,2362,2411,2460,2510,2560,2610,2661,2712,2763,2815,2868,2920,2973,3026,3080,
3134,3188,3242,3297,3352,3407,3462,3517,3573,3629,3685,3741,3797,3854,3910,3967,4023,4080,4137,4193};

// 【新增】整流开环查表索引
static uint16_t g_rect_open_idx = 0;

// 设计意图：使用 static 限定作用域仅在本文件(c)内，防止命名污染
static uint8_t g_rect_hw_started = 0;    // 整流硬件启动完成标志：1=MOE已开，PWM已输出

// ==========================================
// 【修改】整流状态机枚举 (全新逻辑)
// ==========================================
typedef enum {
    RECT_STATE_PRECHARGE_ONLY = 0,  // 阶段1：仅预充电 (闭合继电器+计时)
    RECT_STATE_ZERO_DETECT,          // 阶段2：过零判断
    RECT_STATE_CLOSED_LOOP           // 阶段3：闭环运行
} Rect_State_TypeDef;



Rect_State_TypeDef g_rect_state = RECT_STATE_PRECHARGE_ONLY; // 整流状态机当前状态
uint8_t  g_relay_triggered = 0;                       // 继电器触发标志：1=继电器已闭合
uint32_t g_precharge_counter = 0;                       // 预充电计数器：记录预充电时间
uint32_t g_zero_detect_timer = 0;   // 【新增】阶段2计时器
// 在模式切换时或需要复位时，将其置 1
uint8_t g_rect_reset_req = 1; // 上电初始化为 1，确保第一次运行时复位

#define PRECHARGE_POINTS  20000                        // 预充电时间：1s (20000个中断点 @ 20kHz)

// 【修改点 3】故障计数器改为全局变量 (原中断内 static)
// 设计意图：移到全局是为了能在 Controller_Rect_Reset_All() 中清零，避免重启后残留值导致误触发
uint8_t rect_pll_fault_cnt = 0;     // 整流锁相故障计数
uint8_t rect_bus_fault_cnt = 0;     // 整流母线故障计数
uint8_t rect_boost_ocp_cnt = 0;     // 整流Boost过流计数
uint8_t rect_conv_ocp_cnt = 0;      // 整流H桥过流计数
uint16_t g_buck_rms_fault_cnt = 0; 

// -----------------------------------------------------------------------------
// 【全局变量】V_I_ConvSide 有效值计算 (50Hz工频, 20kHz中断 → 400点/周期)
// -----------------------------------------------------------------------------
float g_i_conv_sq_sum = 0.0f;       // 电流平方累加和
uint16_t g_i_conv_sample_cnt = 0;    // 采样点计数器
float g_i_conv_rms = 0.0f;           // 最终计算的电流有效值



// 控制器结构体实例
Rect_Lock_ZCD_TypeDef g_rect_zcd;   // 整流过零检测结构体
PI_TypeDef          pi_vdc;          // 母线电压PI控制器
Notch100Hz_TypeDef  notch100;        // 100Hz陷波滤波器
QPR_Rect_t          qpr_i;           // 电流准谐振控制器
PI_TypeDef          pi_buck;         // Buck电流PI控制器
// 【新增】新的状态机过零检测全局变量
ZCD_StateMachine_t g_zcd_state; 

// ==========================================
// 【离网逆变模式】专用变量定义
// ==========================================
// 50Hz标准正弦波表（20kHz中断，400点）
// 设计意图：查表法生成正弦波，比计算sin()快10倍以上，适合20kHz中断
const float sin_wave[400] = {
    0.000, 0.016, 0.031, 0.047, 0.063, 0.078, 0.094, 0.110, 0.125, 0.141,
    0.156, 0.172, 0.187, 0.203, 0.218, 0.233, 0.249, 0.264, 0.279, 0.294,
    0.309, 0.324, 0.339, 0.353, 0.368, 0.383, 0.397, 0.412, 0.426, 0.440,
    0.454, 0.468, 0.482, 0.495, 0.509, 0.522, 0.536, 0.549, 0.562, 0.575,
    0.588, 0.600, 0.613, 0.625, 0.637, 0.649, 0.661, 0.673, 0.685, 0.696,
    0.707, 0.718, 0.729, 0.740, 0.750, 0.760, 0.771, 0.780, 0.790, 0.800,
    0.809, 0.818, 0.827, 0.836, 0.844, 0.853, 0.861, 0.869, 0.876, 0.884,
    0.891, 0.898, 0.905, 0.911, 0.918, 0.924, 0.930, 0.935, 0.941, 0.946,
    0.951, 0.956, 0.960, 0.965, 0.969, 0.972, 0.976, 0.979, 0.982, 0.985,
    0.988, 0.990, 0.992, 0.994, 0.996, 0.997, 0.998, 0.999, 1.000, 1.000,
    1.000, 1.000, 1.000, 0.999, 0.998, 0.997, 0.996, 0.994, 0.992, 0.990,
    0.988, 0.985, 0.982, 0.979, 0.976, 0.972, 0.969, 0.965, 0.960, 0.956,
    0.951, 0.946, 0.941, 0.935, 0.930, 0.924, 0.918, 0.911, 0.905, 0.898,
    0.891, 0.884, 0.876, 0.869, 0.861, 0.853, 0.844, 0.836, 0.827, 0.818,
    0.809, 0.800, 0.790, 0.780, 0.771, 0.760, 0.750, 0.740, 0.729, 0.718,
    0.707, 0.696, 0.685, 0.673, 0.661, 0.649, 0.637, 0.625, 0.613, 0.600,
    0.588, 0.575, 0.562, 0.549, 0.536, 0.522, 0.509, 0.495, 0.482, 0.468,
    0.454, 0.440, 0.426, 0.412, 0.397, 0.383, 0.368, 0.353, 0.339, 0.324,
    0.309, 0.294, 0.279, 0.264, 0.249, 0.233, 0.218, 0.203, 0.187, 0.172,
    0.156, 0.141, 0.125, 0.110, 0.094, 0.078, 0.063, 0.047, 0.031, 0.016,
    0.000, -0.016, -0.031, -0.047, -0.063, -0.078, -0.094, -0.110, -0.125, -0.141,
    -0.156, -0.172, -0.187, -0.203, -0.218, -0.233, -0.249, -0.264, -0.279, -0.294,
    -0.309, -0.324, -0.339, -0.353, -0.368, -0.383, -0.397, -0.412, -0.426, -0.440,
    -0.454, -0.468, -0.482, -0.495, -0.509, -0.522, -0.536, -0.549, -0.562, -0.575,
    -0.588, -0.600, -0.613, -0.625, -0.637, -0.649, -0.661, -0.673, -0.685, -0.696,
    -0.707, -0.718, -0.729, -0.740, -0.750, -0.760, -0.771, -0.780, -0.790, -0.800,
    -0.809, -0.818, -0.827, -0.836, -0.844, -0.853, -0.861, -0.869, -0.876, -0.884,
    -0.891, -0.898, -0.905, -0.911, -0.918, -0.924, -0.930, -0.935, -0.941, -0.946,
    -0.951, -0.956, -0.960, -0.965, -0.969, -0.972, -0.976, -0.979, -0.982, -0.985,
    -0.988, -0.990, -0.992, -0.994, -0.996, -0.997, -0.998, -0.999, -1.000, -1.000,
    -1.000, -1.000, -1.000, -0.999, -0.998, -0.997, -0.996, -0.994, -0.992, -0.990,
    -0.988, -0.985, -0.982, -0.979, -0.976, -0.972, -0.969, -0.965, -0.960, -0.956,
    -0.951, -0.946, -0.941, -0.935, -0.930, -0.924, -0.918, -0.911, -0.905, -0.898,
    -0.891, -0.884, -0.876, -0.869, -0.861, -0.853, -0.844, -0.836, -0.827, -0.818,
    -0.809, -0.800, -0.790, -0.780, -0.771, -0.760, -0.750, -0.740, -0.729, -0.718,
    -0.707, -0.696, -0.685, -0.673, -0.661, -0.649, -0.637, -0.625, -0.613, -0.600,
    -0.588, -0.575, -0.562, -0.549, -0.536, -0.522, -0.509, -0.495, -0.482, -0.468,
    -0.454, -0.440, -0.426, -0.412, -0.397, -0.383, -0.368, -0.353, -0.339, -0.324,
    -0.309, -0.294, -0.279, -0.264, -0.249, -0.233, -0.218, -0.203, -0.187, -0.172,
    -0.156, -0.141, -0.125, -0.110, -0.094, -0.078, -0.063, -0.047, -0.031, -0.016
};

// 离网模式变量
// 离网软启动计时器（20kHz中断，1秒 = 20000次）
uint32_t g_offgrid_soft_start_cnt = 0;
static uint8_t g_offgrid_hw_started = 0; // 离网硬件启动完成标志
static uint16_t s_idx = 0;                // 正弦波表索引：0~399循环
// 在 control.c 的全局变量区

RC_TypeDef          rc_offgrid_v;         // 离网电压重复控制器
PI_TypeDef          pi_offgrid_v;         // 离网电压PI控制器
PI_TypeDef          pi_offgrid_i;         // 离网电流PI控制器
PI_TypeDef          pi_boost_v;           // 离网Boost电压PI
PI_TypeDef          pi_boost_i;           // 离网Boost电流PI

// 【修改点 3】离网故障计数器改为全局变量
uint8_t offgrid_fatal_fault_cnt = 0; // 离网致命故障计数
uint8_t offgrid_ov_fault_cnt = 0;     // 离网过压故障计数
uint8_t offgrid_ocp_fault_cnt = 0;    // 离网过流故障计数

// ==========================================
// 【并网逆变模式】专用变量定义
// ==========================================
static uint8_t g_ongrid_hw_started = 0; // 并网硬件启动完成标志
// 【新增】并网软启动计数器 (20kHz中断，1秒 = 20000次)
uint32_t g_ongrid_soft_start_cnt = 0;

float modulation = 0.0f; // 调制波全局变量，用于调试观察
// 【新增】三个前馈项全局变量，用于调试观察
float ff_out_prop = 0.0f;      // 比例前馈输出
float ff_out_damp = 0.0f;      // 有源阻尼输出
float ff_out_2nd  = 0.0f;      // 二阶微分前馈输出

// 并网状态机
typedef enum {
    ONGRID_STATE_IDLE = 0,        // 状态0：空闲，等待启动命令
    ONGRID_STATE_PREBOOST,        // 状态1：Boost建压，先抬升母线电压
    ONGRID_STATE_SYNCING,         // 状态2：同步锁相，检测电网过零点
    ONGRID_STATE_RELAY_WAIT,      // 状态3：继电器等待
    ONGRID_STATE_CLOSED_LOOP      // 状态4：闭环运行，并网发电
} OnGrid_State_TypeDef;

OnGrid_State_TypeDef g_ongrid_state = ONGRID_STATE_IDLE; // 并网状态机当前状态
uint8_t  g_ongrid_relay_triggered = 0;                   // 并网继电器触发标志
uint8_t  g_ongrid_boost_en = 0;                          // 并网Boost使能标志
uint8_t  g_ongrid_inverter_en = 0;                       // 并网H桥使能标志


// 母线电压滑动平均滤波
// 设计意图：用于判断母线电压是否稳定，避免因为噪声导致状态机乱跳
#define VDC_STABLE_WINDOW_SIZE  200      // 滑动窗口大小：200点 = 10ms
#define VDC_STABLE_TARGET       55.0f    // 母线稳定目标值
#define VDC_STABLE_TOLERANCE     3.0f     // 母线稳定容差：±4V以内视为稳定
float g_vdc_window_buffer[VDC_STABLE_WINDOW_SIZE]; // 窗口数据缓冲区
uint16_t g_vdc_window_idx = 0;             // 窗口索引
float g_vdc_running_sum = 0.0f;            // 窗口累加和（用于快速计算平均值）
uint8_t g_vdc_stable_flag = 0;              // 母线稳定标志
uint8_t g_window_filled_flag = 0;           // 窗口填满标志：第一次启动时需等窗口填满


// 并网控制器
PI_TypeDef          pi_boost_v_ongrid;     // 并网Boost电压PI
PI_TypeDef          pi_boost_i_ongrid;     // 并网Boost电流PI
PR_Inverter_t       pr_ongrid_i;           // 并网电流准谐振控制器
FF_Proportional_t          ff_prop;               // 比例前馈
FF_ActiveDamping_t         ff_damp;               // 有源阻尼
FF_SecondOrder_t           ff_2nd;                // 二阶微分前馈



// 【修改点 3】并网故障计数器改为全局变量 (原中断内 static)
uint8_t preboost_fatal_cnt = 0;      // 预启动阶段致命故障计数
uint8_t preboost_ocp_cnt = 0;        // 预启动阶段过流计数
uint8_t ongrid_fatal_fault_cnt = 0;  // 并网致命故障计数
uint8_t ongrid_ov_fault_cnt = 0;      // 并网过压计数
uint8_t ongrid_uv_fault_cnt = 0;      // 并网欠压计数
uint8_t ongrid_ocp_fault_cnt = 0;     // 并网过流计数
uint8_t ongrid_pll_fault_cnt = 0;     // 并网锁相故障计数


// ADC 比例系数
#define V_REF_MCU              3.306f   // MCU参考电压：3.3V (实际校准值)
#define ADC_FULL_SCALE         4095.0f   // 12位ADC满量程：2^12 - 1
#define ADC_SCALE_COEFF          0.000807326f   //(V_REF_MCU / ADC_FULL_SCALE) // ADC原始值转物理值的系数


// ADC采样变量
// 设计意图：Raw是DMA直接搬运的原始值，V_/I_是转换后的物理值(浮点数)
uint16_t VAC_Grid_Raw, VDC_Bus_Raw, V_OffGrid_Raw; // 电压采样ADC原始值电压
uint16_t I_GridSide_Raw, I_ConvSide_Raw, I_DCDC_Raw, I_Cap_Raw; // 电流采样ADC原始值电压
float V_VAC_Grid, V_VDC_Bus, V_V_OffGrid;          // 电压物理值
float V_I_GridSide, V_I_ConvSide, V_I_DCDC, V_I_Cap; // 电流物理值

// 算法库结构体实例
CascadeNotch_TypeDef g_cascade_notch; // 用于滤除电网电压的3/5/7次谐波
SOGI_FLL_TypeDef g_sogi;   // 二阶广义积分器 (用于电网滤波)
SRF_PLL_TypeDef  g_pll;    // 同步旋转坐标系锁相环



// ==========================================
// 【VOFA上位机调试】分模式数据缓冲区
// ==========================================
// Just Float协议：每个float占4字节，最后必须加4字节结束帧 0x00,0x00,0x80,0x7F
// 整流模式：4个变量 → 4*4 + 4 = 20字节
uint8_t vofa_buf_rect[20] = {
    0,0,0,0,  // 变量1
    0,0,0,0,  // 变量2
    0,0,0,0,  // 变量3
    0,0,0,0,  // 变量4
    0x00,0x00,0x80,0x7F  // 固定结束帧
};

// 离网模式：4个变量 → 4*4 + 4 = 20字节
uint8_t vofa_buf_offgrid[20] = {
    0,0,0,0,
    0,0,0,0,
    0,0,0,0,
    0,0,0,0,
    0x00,0x00,0x80,0x7F
};

// 并网模式：5个变量 → 5*4 + 4 = 24字节
uint8_t vofa_buf_ongrid[24] = {
    0,0,0,0,  // 变量1
    0,0,0,0,  // 变量2
    0,0,0,0,  // 变量3
    0,0,0,0,  // 变量4
    0,0,0,0,  // 变量5
    0x00,0x00,0x80,0x7F  // 固定结束帧
};



// 校准模式：8个变量 → 8*4 + 4 = 36字节
uint8_t vofa_buf_test[36] = {
    0,0,0,0,  // 变量1
    0,0,0,0,  // 变量2
    0,0,0,0,  // 变量3
    0,0,0,0,  // 变量4
    0,0,0,0,  // 变量5
		0,0,0,0,  // 变量6
	  0,0,0,0,  // 变量7
	  0,0,0,0,  // 变量8
    0x00,0x00,0x80,0x7F  // 固定结束帧
};



// 锁相环测试模式：6个变量 → 6*4 + 4 = 28字节  
uint8_t vofa_buf_SOGI2[8] = {
    0,0,0,0,  // 变量1
    0x00,0x00,0x80,0x7F  // 固定结束帧
};


// ===================== 【7 通道 300 点 自适应滑动滤波】 =====================
#define FILTER_WIN_SIZE 300

static float filter_buf[7][FILTER_WIN_SIZE] = {0.0f};
static uint16_t filter_idx[7]  = {0};
static float    filter_sum[7] = {0.0f};
static uint16_t filter_cnt[7] = {0};

// 必须放在调用前！！！
static float sliding_filter(uint8_t ch, float new_val)
{
    filter_sum[ch] -= filter_buf[ch][filter_idx[ch]];
    filter_buf[ch][filter_idx[ch]] = new_val;
    filter_sum[ch] += new_val;

    if(filter_cnt[ch] < FILTER_WIN_SIZE){
        filter_cnt[ch]++;
    }

    float avg = filter_sum[ch] / filter_cnt[ch];
    filter_idx[ch] = (filter_idx[ch] + 1) % FILTER_WIN_SIZE;
    return avg;
}
// ==========================================================================

// ==========================================
// 【ADC校准参数】7通道Vadc→实际值转换公式
// 所有系数均为强制过原点最小二乘法拟合结果，零点固定为硬件测量值
// ==========================================
// 电流通道公式：I = K * (Vadc - Vzero)  电流正→Vadc>零点，电流负→Vadc<零点
// 电压通道公式：V = K * (Vadc - Vzero)  电压正→Vadc<零点，电压负→Vadc>零点

// 通道0：ADC1_1 网侧电感电流
#define CH0_VZERO   1.6577f
#define CH0_GAIN    8.2916f
// 通道1：ADC1_2 离网侧输出电压
#define CH1_VZERO   1.6393f
#define CH1_GAIN   -46.9690f
// 通道2：ADC1_3 电网侧输入电压
#define CH2_VZERO   1.7268f
#define CH2_GAIN   -46.8883f
// 通道3：ADC1_6 直流母线电压
#define CH3_VZERO   1.6413f
#define CH3_GAIN   -47.0519f
// 通道4：ADC1_7 Boost级电感电流
#define CH4_VZERO   1.6611f
#define CH4_GAIN    8.7773f
// 通道5：ADC1_8 变换器侧(H桥)电感电流
#define CH5_VZERO   1.6642f
#define CH5_GAIN    7.3388f
// 通道6：ADC1_9 LCL滤波电容电流
#define CH6_VZERO   1.6528f
#define CH6_GAIN    8.2087f

// ADC原始值转引脚电压系数
#define ADC_TO_VADC  0.000807326f


// 【新增】待机 BUCK 测试独立变量
volatile uint8_t buck_test_calc_enable = 0;
float buck_test_i_target = 1.0f;
PI_TypeDef pi_buck_test;
// 【新增】Buck测试专用的故障计数器
uint8_t bt_bus_ov_cnt = 0;    // 过压计数
uint8_t bt_bus_uv_cnt = 0;    // 欠压计数
uint8_t bt_ocp_cnt = 0;       // 过流计数


volatile uint8_t boost_test_calc_enable = 0;
float boost_test_v_target = 55.0f;
PI_TypeDef pi_bt_v, pi_bt_i; 

// 【补上这一行】定义硬件启动标志位
static uint8_t g_boost_test_hw_started = 0;


// 【新增】待机 SOGI 测试独立变量
volatile uint8_t sogi_test_calc_enable = 0;



// ==========================================
// @brief  ADC中断回调函数（20kHz，核心控制中断）
// ==========================================
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // 仅处理 ADC1 的中断
    if(hadc->Instance == ADC1)
    {
        // 在 HAL_ADC_ConvCpltCallback 函数中修改 LED 阈值判断
        static uint32_t led_cnt = 0;
        // 【修复】判断所有正式模式的使能标志：整流/离网/并网任一使能则快闪
        uint8_t is_any_mode_running = rect_calc_enable || offgrid_calc_enable || ongrid_calc_enable;
        uint32_t threshold = (work_mode == MODE_STANDBY || !is_any_mode_running) ? 50000 : 10000;

        if(led_cnt++ >= threshold) {
            HAL_GPIO_TogglePin(GPIOA, LED_Pin);
            led_cnt = 0;
        }
				
        adc_isr_busy = 1;
				
        // ==========================================   
        /* 零点校正情况   12.5个采样周期（每个通道）    重要信息，请勿删除
            adc通道1 对应adc1_1   网侧电感电流         1.6577V    
            adc通道2 对应adc1_2   离网侧电压           1.6393V
            adc通道3 对应adc1_3   电网侧电压           1.7268V
            adc通道4 对应adc1_6   母线电压             1.6413V
            adc通道5 对应adc1_7   Boost电流            1.6611V
            adc通道6 对应adc1_8   变换器侧电感电流     1.6642V
            adc通道7 对应adc1_9   LCL滤波电电容电流    1.6528V
         ========================================== */
				  
        // ==========================================  
        // 理论转换公式   ADC引脚电压→实际采样电压/电流   重要信息，请勿删除
        // 电压转换公式  ( V+ -  V-)=-47*（Vadc-Vzero） 输入电压为正时，Vadc小于零点电压  输入电压为负时，Vadc大于零点电压 
        // 电流转换公式  Iin=7.5758*（Vadc-Vzero）   输入电流为正时，Vadc大于零点电压  输入电流为负时，Vadc小于零点电压		      
    
        /* ========================================== 
            最终转换公式   ADC引脚电压→实际采样电压/电流    重要信息，请勿删除
            说明：所有系数均为强制过原点最小二乘法拟合结果，零点固定为硬件测量值
         ==========================================
            电流方向：输入电流为正时，Vadc > 零点电压；输入电流为负时，Vadc < 零点电压
            电压方向：输入电压为正时，Vadc < 零点电压；输入电压为负时，Vadc > 零点电压
            adc通道1 对应adc1_1  网侧电感电流         实际值 = 8.2916 * (Vadc - 1.6577V)
            adc通道2 对应adc1_2  离网侧电压           实际值 = -46.9690 * (Vadc - 1.6393V)
            adc通道3 对应adc1_3  电网侧电压           实际值 = -46.8883 * (Vadc - 1.6484V)
            adc通道4 对应adc1_6  母线电压             实际值 = -47.0519 * (Vadc - 1.6413V)
            adc通道5 对应adc1_7  Boost电流            实际值 = 8.7773 * (Vadc - 1.6611V)
            adc通道6 对应adc1_8  变换器侧电感电流     实际值 = 7.3388 * (Vadc - 1.6642V)
            adc通道7 对应adc1_9  LCL滤波电容电流      实际值 = 8.2087 * (Vadc - 1.6528V)
         ==========================================*/
		 
        // ==========================================
        // 步骤 1：读取 ADC 原始值
        // ==========================================
        float I_GridSide_Raw = ADC_Sample[0];
        float V_OffGrid_Raw  = ADC_Sample[1];
        float VAC_Grid_Raw   = ADC_Sample[2];
        float VDC_Bus_Raw    = ADC_Sample[3];
        float I_DCDC_Raw     = ADC_Sample[4];
        float I_ConvSide_Raw = ADC_Sample[5];
        float I_Cap_Raw      = ADC_Sample[6];

        // ==========================================
        // 步骤 2：ADC原始值 -> ADC输入引脚电压
        // ==========================================
        float adc_phy[7] = {0.0f};
        adc_phy[0] = I_GridSide_Raw   * 0.000807326f;
        adc_phy[1] = V_OffGrid_Raw    * 0.000807326f;
        adc_phy[2] = VAC_Grid_Raw     * 0.000807326f;
        adc_phy[3] = VDC_Bus_Raw      * 0.000807326f;
        adc_phy[4] = I_DCDC_Raw       * 0.000807326f;
        adc_phy[5] = I_ConvSide_Raw   * 0.000807326f;
        adc_phy[6] = I_Cap_Raw        * 0.000807326f;

        // ==========================================
        // 步骤 3：7路滑动滤波（注释保留，需要时取消注释）
        // ==========================================
         float adc_filtered[7];
        // adc_filtered[0] = sliding_filter(0, adc_phy[0]);
        // adc_filtered[1] = sliding_filter(1, adc_phy[1]);
        // adc_filtered[2] = sliding_filter(2, adc_phy[2]);
           adc_filtered[3] = sliding_filter(3, adc_phy[3]);
        // adc_filtered[4] = sliding_filter(4, adc_phy[4]);
        // adc_filtered[5] = sliding_filter(5, adc_phy[5]);
        // adc_filtered[6] = sliding_filter(6, adc_phy[6]);

        // ==========================================
        // 步骤 4：滤波后Vadc -> 实际电压/电流值
        // ==========================================
        float adc_actual[8] = {0.0f};
        adc_actual[0] = CH0_GAIN * (adc_phy[0] - CH0_VZERO);
        adc_actual[1] = CH1_GAIN * (adc_phy[1] - CH1_VZERO);
        adc_actual[2] = CH2_GAIN * (adc_phy[2] - CH2_VZERO);
//		    	adc_actual[2] = 	adc_phy[2] ;
        adc_actual[3] = CH3_GAIN * (adc_filtered[3] - CH3_VZERO);   //母线电压用滑动滤波后的值
        adc_actual[4] = CH4_GAIN * (adc_phy[4] - CH4_VZERO); 
        adc_actual[5] = CH5_GAIN * (adc_phy[5] - CH5_VZERO);
        adc_actual[6] = CH6_GAIN * (adc_phy[6] - CH6_VZERO);
//          adc_actual[7]=TIM1->CCR4;          //调制波ccr值
//			  	adc_actual[7]=modulation;          //调制波-1~1初始值
//			    adc_actual[7] = ff_out_damp;       //电网电压全前馈第二项
//					adc_actual[7]=g_pll.qerror;          //锁相环误差 
				     adc_actual[7]=g_i_conv_rms;
				
        // 更新全局变量供其他地方使用
        V_VAC_Grid = adc_actual[2];
        V_VDC_Bus  = adc_actual[3];
        V_I_DCDC   = adc_actual[4];
        V_I_ConvSide = adc_actual[5];
        V_I_GridSide = adc_actual[0];
        V_V_OffGrid  = adc_actual[1];
        V_I_Cap = adc_actual[6];
				
        // ==========================================
        // VOFA 数据传输
        // ==========================================
        memcpy(vofa_buf_test, (uint8_t*)adc_actual, sizeof(adc_actual));
        HAL_UART_Transmit_DMA(&huart3, (uint8_t*)vofa_buf_test, sizeof(vofa_buf_test));
				
        // ======================== 【修复/增强】待机模式 BUCK 测试 ========================
        if(work_mode == MODE_BUCK && buck_test_calc_enable == 1)
        {
            uint8_t bt_fault_happened = 0;

            // --- 1. 母线过压保护 (OVP) ---
            if(V_VDC_Bus > BUS_OV_THRESHOLD) {
                if(++bt_bus_ov_cnt >= FAULT_TRIGGER_CNT) {
                    OVP_DC = 1;
                    last_fault_code = FAULT_BUS_OVERVOLT;
                    bt_fault_happened = 1;
                }
            } else { bt_bus_ov_cnt = 0; }

            // --- 2. 母线致命欠压保护 (Fatal UVP) ---
            if(V_VDC_Bus < FATAL_BUS_UV_THRESHOLD) {
                if(++bt_bus_uv_cnt >= FAULT_TRIGGER_CNT) {
                    UVP_DC = 1;
                    last_fault_code = FAULT_BUS_UNDERVOLT;
                    bt_fault_happened = 1;
                }
            } else { bt_bus_uv_cnt = 0; }

            // --- 3. 电感过流保护 (OCP) ---
            // 判定 Buck 电感电流（V_I_DCDC）是否超过安全限制（如 4.0A）
            if(fabsf(V_I_DCDC) > 4.0f) {
                if(++bt_ocp_cnt >= FAULT_TRIGGER_CNT) {
                    OCP_BOOST = 1;
                    last_fault_code = FAULT_BOOST_OVERCUR;
                    bt_fault_happened = 1;
                }
            } else { bt_ocp_cnt = 0; }

            // --- 故障统一执行动作 ---
            if(bt_fault_happened)
            {
                Fault_Flag = 1;                 // 触发总故障，通知主循环锁死
                Controller_BuckTest_Reset_All(); // 立即停止硬件、拉高引脚、清空使能
                adc_isr_busy = 0;               // 释放 busy 标志
                return;                         // 退出中断，不执行后续 PI 计算
            }

            // --- 4. 正常闭环控制逻辑 ---
            float duty = PI_Update(&pi_buck_test, buck_test_i_target, V_I_DCDC);  //有偏差可以直接在这里减，不要动pi参数
                                                                                        //调整率待优化
            // 安全限幅
            if(duty > 0.93f) duty = 0.93f;
            if(duty < 0.07f) duty = 0.07f;

            TIM1->CCR1 = (int32_t)(duty * 8500.0f);
        }

        // ======================== 【新增】待机模式 BOOST 直接测试 ========================
        if(work_mode == MODE_BOOST && boost_test_calc_enable == 1)
        {
            // 1. 故障判定逻辑（与其它模式对齐）
            uint8_t fault = 0;
            if(V_VDC_Bus > 65.0f) { if(++bt_bus_ov_cnt > 5) { last_fault_code = FAULT_BUS_OVERVOLT; fault = 1; } }
            else if(V_VDC_Bus < 15.0f) { if(++bt_bus_uv_cnt > 5) { last_fault_code = FAULT_BUS_UNDERVOLT; fault = 1; } }
            // else if( -V_I_DCDC > 6.0f) { if(++bt_ocp_cnt > 5) { last_fault_code = FAULT_BOOST_OVERCUR; fault = 1; } }   //这里也要修改电流正方向
                                                                                                                        //上电瞬间有浪涌，得注释掉或者计时
            else { bt_bus_ov_cnt = 0; bt_bus_uv_cnt = 0; bt_ocp_cnt = 0; }                                                       //因为buck发现才是正方向

            if(fault) {
                Fault_Flag = 1; 
                Controller_BoostTest_Reset_All(); 
                adc_isr_busy = 0;
                return; 
            }

            // 2. 双闭环计算
            // 电压环反馈是 V_VDC_Bus (Vdc)，电流环反馈是 V_I_DCDC (Boost电流)
            float i_ref = PI_Update(&pi_bt_v, boost_test_v_target, V_VDC_Bus);
            float duty = PI_Update(&pi_bt_i, i_ref, -V_I_DCDC);   //boost方向采样是和buck反过来的
            // 3. 硬件直接输出 (假设 CCR1 对应下管)
            // 注意：Boost 的占空比与 CCR 的逻辑取决于硬件，通常是 TIM1->CCR1 = duty * 8500
            TIM1->CCR1 = (int32_t)((1.0f-duty) * 8500.0f); 
        }

        // ======================== 【新增】待机模式 SOGI 锁相环测试 ========================
        if(work_mode == MODE_SOGI && sogi_test_calc_enable == 1)
        {
            // 1. SOGI+PLL算法运行
          float V_VAC_Grid_Filtered = CascadeNotch_Update(&g_cascade_notch, V_VAC_Grid);
            SOGI_FLL_Update_Optimized(&g_sogi, V_VAC_Grid);
            SRF_PLL_Update(&g_pll, g_sogi.alpha, g_sogi.beta);

            // 2. VOFA数据传输：发送电网电压、SOGI输出、PLL相位等
            float sogi_debug1_data[4];
            sogi_debug1_data[0] = V_VAC_Grid;           // 原始电网电压
            sogi_debug1_data[1] = g_sogi.alpha;          // SOGI α轴输出
            sogi_debug1_data[2] = g_sogi.beta;           // SOGI β轴输出
            sogi_debug1_data[3] = g_pll.qerror;          // PLL q轴

            float sogi_debug2_data[1];
            sogi_debug2_data[0] = g_pll.qerror;          // PLL q轴

            // 打包发送（使用vofa_buf_ongrid，它有24字节空间，足够6个float）
            memcpy(vofa_buf_SOGI2, (uint8_t*)sogi_debug2_data, sizeof(sogi_debug2_data));
            HAL_UART_Transmit_DMA(&huart3, (uint8_t*)vofa_buf_SOGI2, sizeof(vofa_buf_SOGI2));
        }

				
				
				
// ====================================================================================
// 模式 2：整流模式 (全新逻辑：预充 -> 过零 -> 闭环)
// 
// 【新状态机流程】
// RECT_STATE_PRECHARGE_ONLY (仅继电器闭合+计时) 
//   -> RECT_STATE_ZERO_DETECT (过零判断) 
//     -> RECT_STATE_CLOSED_LOOP (闭环运行)
// 
// 【关键设计点】
// 1. sys_start() 仅在进入闭环前调用
// 2. 前两个阶段硬件不工作，只有继电器闭合
// 3. 前两个阶段仅保留母线过压保护
// ====================================================================================
if(work_mode == MODE_RECTIFIER)
{
      // --------------------------------------------------------------------------------
    // 【层1】检测复位请求 (全局变量方案)
    // --------------------------------------------------------------------------------
    if(g_rect_reset_req == 1)
    {
        g_rect_reset_req = 0; // 清零标志
        
        // 执行完整复位
        Controller_Rect_Reset_All();
    }
		
		
		
    // --------------------------------------------------------------------------------
    // 【层2】核心计算逻辑
    // --------------------------------------------------------------------------------
    if(rect_calc_enable == 1)
    {		
        // ==========================================
        // 【前级算法】始终运行锁相
        // ==========================================
        float V_VAC_Grid_Filtered = CascadeNotch_Update(&g_cascade_notch, V_VAC_Grid);
        SOGI_FLL_Update_Optimized(&g_sogi, V_VAC_Grid_Filtered);
        SRF_PLL_Update(&g_pll, g_sogi.alpha, g_sogi.beta);

        // ==========================================
        // 主状态机
        // ==========================================
           switch(g_rect_state)
        {
            // ========================================================================
            // 阶段 1/3：RECT_STATE_PRECHARGE_ONLY (仅预充电)
            // 功能：闭合继电器 + 1s计时
            // 【注意】此阶段不调用过零检测函数
            // ========================================================================
            case RECT_STATE_PRECHARGE_ONLY:
            {
                g_precharge_counter++; // 仅阶段1的1s专用计时器
                
                // ==========================================
                // 动作1：立刻闭合继电器
                // ==========================================
                if(g_relay_triggered == 0)
                {
                    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_SET);
                    g_relay_triggered = 1;
                }

                // ==========================================
                // 保护：仅母线过压保护
                // ==========================================
                uint8_t fault_flag = 0;
                if(V_VDC_Bus > BUS_OV_THRESHOLD) {
                    if(++rect_bus_fault_cnt >= FAULT_TRIGGER_CNT) {
                        OVP_DC = 1;
                        fault_flag = 1;
                    }
                } else {
                    rect_bus_fault_cnt = 0;
                }

                if(fault_flag) {
                    last_fault_code = FAULT_BUS_OVERVOLT;
                    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_RESET);
                    rect_calc_enable = 0; Fault_Flag = 1;
                    return;
                }

                // ==========================================
                // 状态切换：阶段1计时到 (1s = 20000个中断)
                // ==========================================
                if(g_precharge_counter >= 20000)
                {
                    // 切换到阶段2
                    g_rect_state = RECT_STATE_ZERO_DETECT;
                }
                break;
            }

            // ========================================================================
            // 阶段 2/3：RECT_STATE_ZERO_DETECT (过零判断)
            // 功能：调用过零检测函数，不设超时
            // 【注意】此阶段才开始调用 ZCD_StateMachine_Update
            // ========================================================================
            case RECT_STATE_ZERO_DETECT:
            {
                // ==========================================
                // 【核心】从此刻开始调用过零检测函数
                // ==========================================
                ZCD_StateMachine_Update(&g_zcd_state, g_pll.qerror, g_pll.theta);

                // ==========================================
                // 保护：仅母线过压保护
                // ==========================================
                uint8_t fault_flag = 0;
                if(V_VDC_Bus > BUS_OV_THRESHOLD) {
                    if(++rect_bus_fault_cnt >= FAULT_TRIGGER_CNT) {
                        OVP_DC = 1;
                        fault_flag = 1;
                    }
                } else {
                    rect_bus_fault_cnt = 0;
                }

                if(fault_flag) {
                    last_fault_code = FAULT_BUS_OVERVOLT;
                    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_RESET);
                    rect_calc_enable = 0; 
                    Fault_Flag = 1;
                    return;
                }

                // ==========================================
                // 【核心】检测过零标志 (不设超时，无限等待)
                // ==========================================
                if(g_zcd_state.ZCD_Trigger_Flag == 1)
                {
                    // ==========================================
                    // 动作1：复位控制器 
                    // ==========================================
                    QPR_Rect_Reset(&qpr_i);
                    PI_Reset(&pi_vdc);
                    PI_Reset(&pi_buck);
                    
                    // ==========================================
                    // 动作2：预填PWM寄存器
                    // ==========================================
                    TIM1->CCR4 = 4250;
                    TIM1->CCR2 = 4250;
                    TIM1->CCR1 = 2000;
                    
                    // ==========================================
                    // 动作3：开启硬件PWM
                    // ==========================================
                    sys_start();
                    g_rect_hw_started = 1;
                    g_fault_mask_cnt = 0;
                    
                    // ==========================================
                    // 动作4：切换到闭环状态
                    // ==========================================
                    g_rect_state = RECT_STATE_CLOSED_LOOP;
                }
                break;
            }
						
						
						
						
						
        // ========================================================================
            // 阶段 3/3：RECT_STATE_CLOSED_LOOP
            // ========================================================================
            case RECT_STATE_CLOSED_LOOP:
            {
                if(g_fault_mask_cnt < FAULT_MASK_TIME + 10) g_fault_mask_cnt++;
                uint8_t rect_fault = 0;

                // ==========================================
                // 闭环阶段全功能故障检测
                // ==========================================
                if(fabsf(g_pll.qerror) > 0.15f)
                    rect_pll_fault_cnt++;
                else
                    rect_pll_fault_cnt = 0;

                uint8_t bus_fault_condition = 0;
                if(V_VDC_Bus < FATAL_BUS_UV_THRESHOLD) bus_fault_condition = 1;
                if(V_VDC_Bus > BUS_OV_THRESHOLD) bus_fault_condition = 1;
                if(g_fault_mask_cnt > FAULT_MASK_TIME && V_VDC_Bus < NORMAL_BUS_UV_THRESHOLD-20.0f) bus_fault_condition = 1;
                if(bus_fault_condition) rect_bus_fault_cnt++; else rect_bus_fault_cnt = 0;

                if(V_I_DCDC > 6.5f) rect_boost_ocp_cnt++; else rect_boost_ocp_cnt = 0;
                if(fabsf(V_I_ConvSide) > 6.5f) rect_conv_ocp_cnt++; else rect_conv_ocp_cnt = 0;

                // ==========================================
                // 【新增】Buck侧电流有效值过流检测 (阈值 > 1.71A)
                // ==========================================
                if(g_fault_mask_cnt > FAULT_MASK_TIME && g_i_conv_rms > 1.71f)
                    g_buck_rms_fault_cnt++;
                else
                    g_buck_rms_fault_cnt = 0;

                // ==========================================
                // 故障触发判断
                // ==========================================
                if(rect_pll_fault_cnt >= FAULT_TRIGGER_CNT+5)  { PLL_FAULT = 1; rect_fault = 1; }
                if(rect_bus_fault_cnt >= FAULT_TRIGGER_CNT)  { OVP_DC=1;UVP_DC=1; rect_fault=1; }
                if(rect_boost_ocp_cnt >= FAULT_TRIGGER_CNT)  { OCP_BOOST = 1; rect_fault = 1; }
                if(rect_conv_ocp_cnt >= FAULT_TRIGGER_CNT)   { OCP_CONV = 1; rect_fault = 1; }
                if(g_buck_rms_fault_cnt >= FAULT_TRIGGER_CNT) { OCP_BUCK_RMS = 1; rect_fault = 1; } // 【新增】

                if(rect_fault) {
                    if(PLL_FAULT) last_fault_code = FAULT_PLL_LOST;
                    else if(OCP_CONV) last_fault_code = FAULT_CONV_OVERCUR;
                    else if(OCP_BOOST) last_fault_code = FAULT_BOOST_OVERCUR;
                    else if(OVP_DC) last_fault_code = FAULT_BUS_OVERVOLT;
                    else if(UVP_DC) last_fault_code = FAULT_BUS_UNDERVOLT;
                    else if(OCP_BUCK_RMS) last_fault_code = FAULT_BUCK_RMS_OVERCUR; // 【新增】

                    sys_stop();
                    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_RESET);
                    rect_calc_enable = 0;
                    Fault_Flag = 1;
                    adc_isr_busy = 0;

                    // 复位有效值计算状态
                    g_i_conv_sq_sum = 0.0f;
                    g_i_conv_sample_cnt = 0;
                    g_i_conv_rms = 0.0f;
                    g_buck_rms_fault_cnt = 0;

                    return;
                }

                // ==========================================
                // V_I_ConvSide 有效值计算
                // ==========================================
                g_i_conv_sq_sum += V_I_ConvSide * V_I_ConvSide;
                g_i_conv_sample_cnt++;

                if(g_i_conv_sample_cnt >= 400)
                {
                    g_i_conv_rms = sqrtf(g_i_conv_sq_sum / 400.0f);
                    g_i_conv_sq_sum = 0.0f;
                    g_i_conv_sample_cnt = 0;
                }

                // ==========================================
                // 正常闭环控制逻辑
                // ==========================================
                if(g_rect_hw_started == 1 && rect_calc_enable == 1)
                {
                    float vdc_ref_loop = 55.0f;
                    float pi_out_loop = PI_Update(&pi_vdc, vdc_ref_loop, V_VDC_Bus);

                    float cos_theta_loop = g_pll.cos_theta;
                    float sin_theta_loop = g_pll.sin_theta;

                    float i_err_loop = pi_out_loop * cos_theta_loop - (-V_I_ConvSide);
                    float qpr_out_loop = QPR_Rect_Update(&qpr_i, i_err_loop);

                    float duty_12_loop = -qpr_out_loop + 0.471f * sin_theta_loop + V_VAC_Grid*0.022097f;

                    float buck_duty_loop = PI_Update(&pi_buck, grid_i_target+0.1f, V_I_DCDC);

                    TIM1->CCR1 = (int32_t)(buck_duty_loop * 8500.0f);
                    TIM1->CCR2 = SPWM_SinglePolarity_CCR(-duty_12_loop);
                    TIM1->CCR4 = SPWM_SinglePolarity_CCR(duty_12_loop);
                }
                break;
            }

            default: break;
        }
    }
}









        // ==========================================
        // 模式 3：离网模式 (Key宏联动版)
        // ==========================================
        else if(work_mode == MODE_OFFGRID)
        {
            if(offgrid_calc_enable == 1)
            {
                // ===================== 1. 变量安全限幅 (防御性编程) =====================
                // 确保变量不会因为Bug或意外超出 key.h 定义的合法范围
                if(offgrid_Uoac_target > OFFGRID_U_TARGET_MAX) offgrid_Uoac_target = OFFGRID_U_TARGET_MAX;
                if(offgrid_Uoac_target < OFFGRID_U_TARGET_MIN) offgrid_Uoac_target = OFFGRID_U_TARGET_MIN;

                // ===================== 2. 计算目标峰值 (关键联动) =====================
                // 公式：峰值 = 有效值 * 1.4142 (sqrt(2))
                // 这里 offgrid_Uoac_target 就是 key.c 里按键修改的那个变量
                float target_amp_peek = (offgrid_Uoac_target * 1.4142f); 

                // ===================== 3. 软启动 =====================
                const uint32_t soft_start_total = 80000; // 1s @20kHz (4秒)
                float amp;
                
                if (g_offgrid_soft_start_cnt >= soft_start_total)
                {
                    // 软启动结束：直接使用动态目标
                    amp = target_amp_peek;
                }
                else
                {
                    // 软启动进行中：从 0 上升到 目标峰值
                    amp = target_amp_peek * ((float)g_offgrid_soft_start_cnt / soft_start_total);
                    g_offgrid_soft_start_cnt++;
                }

                // 正弦参考生成
                float v_ref = amp * sin_wave[s_idx];
                s_idx++; if(s_idx >= 400) s_idx = 0;

                // ===================== 4. 控制算法 (双环 + RC) =====================
                float v_err = v_ref - V_V_OffGrid;
                float rc_out = RC_Update(&rc_offgrid_v, v_err);
                float pi_v_out = PI_Update(&pi_offgrid_v, rc_out, 0.0f);
                float pi_i_out = PI_Update(&pi_offgrid_i, pi_v_out, V_I_ConvSide);
                int32_t ccr_12 = SPWM_SinglePolarity_CCR(pi_i_out);
                int32_t ccr_34 = SPWM_SinglePolarity_CCR(-pi_i_out);
								

                // ===================== 5. Boost 控制 =====================
                float boost_v_ref = 55.0f;
                float boost_i_ref = PI_Update(&pi_boost_v, boost_v_ref, V_VDC_Bus);
                float boost_duty = PI_Update(&pi_boost_i, boost_i_ref, -V_I_DCDC);
                if(boost_duty > 0.9f) boost_duty = 0.9f;
                if(boost_duty < 0.1f) boost_duty = 0.1f;
                int32_t ccr_boost = (int32_t)((1.0f - boost_duty) * 8500.0f);

                // ===================== 6. 硬件启动 =====================
                if(g_offgrid_hw_started == 0)
                {
                    TIM1->CCR1 = ccr_boost;
                    TIM1->CCR2 = ccr_34;
                    TIM1->CCR4 = ccr_12;
                    sys_start();
                    g_offgrid_hw_started = 1;
                    g_fault_mask_cnt = 0;
                }

                if(g_offgrid_hw_started == 1)
                {
                    static uint8_t cnt_conv = 0;
                    static uint8_t cnt_vo = 0;

                    if(g_fault_mask_cnt < FAULT_MASK_TIME + 10)
                        g_fault_mask_cnt++;

                    uint8_t offgrid_fault = 0;

                    // ===================== 保护1: 母线过压 >60V =====================
                    if(V_VDC_Bus > 60.0f)
                    {
                        if(++offgrid_ov_fault_cnt >= 5){
                            OVP_DC = 1;
                            last_fault_code = FAULT_BUS_OVERVOLT;
                            offgrid_fault = 1;
                        }
                    }
                    else offgrid_ov_fault_cnt = 0;

                    // ===================== 保护2: 母线欠压 <18V (屏蔽) =====================
                    if(g_fault_mask_cnt >= FAULT_MASK_TIME + 10)
                    {
                        if(V_VDC_Bus < 18.0f)
                        {
                            if(++offgrid_fatal_fault_cnt >=5){
                                UVP_DC =1;
                                last_fault_code = FAULT_BUS_UNDERVOLT;
                                offgrid_fault=1;
                            }
                        }
                        else offgrid_fatal_fault_cnt =0;
                    }

                    // ===================== 保护3: Boost 电流 >6A (屏蔽) =====================
                    if(g_fault_mask_cnt >= FAULT_MASK_TIME +10)
                    {
                        if(fabsf(V_I_DCDC) > 6.0f)
                        {
                            if(++offgrid_ocp_fault_cnt >=5){ // 修正：这里原来是6，统一为5
                                OCP_BOOST=1;
                                last_fault_code = FAULT_BOOST_OVERCUR;
                                offgrid_fault=1;
                            }
                        }
                        else offgrid_ocp_fault_cnt =0;
                    }

                    // ===================== 保护4: 逆变侧电感 >5A =====================
                    if(fabsf(V_I_ConvSide) > 5.0f)
                    {
                        if(++cnt_conv >=5){
                            OCP_CONV =1;
                            last_fault_code = FAULT_CONV_OVERCUR;
                            offgrid_fault=1;
                        }
                    }
                    else cnt_conv =0;

               // ===================== 保护5: 离网输出过压 (专属保护) =====================
									float ov_prot_limit = target_amp_peek + 7.5f; // 动态裕量7.5V
									if(fabsf(V_V_OffGrid) > ov_prot_limit) 
									{
											if(++cnt_vo >= 5) {
													OVP_OFFGRID = 1;                    // 【修改】使用专属标志位
													last_fault_code = FAULT_OFFGRID_OVERVOLT; // 【修改】使用专属故障码
													offgrid_fault = 1;
											}
									}
									else cnt_vo = 0;
                    // ===================== 故障触发 =====================
                    if(offgrid_fault)
                    {
                        sys_stop();
                        offgrid_calc_enable = 0;
                        Fault_Flag = 1;
                        adc_isr_busy = 0;
                        return;
                    }

                    // 更新PWM
                    TIM1->CCR1 = ccr_boost;
                    TIM1->CCR2 = ccr_34;
                    TIM1->CCR4 = ccr_12;
                }
            }
        }

        // ==========================================
        // 模式 4：并网逆变模式 (最终屏蔽逻辑版)
        // 屏蔽规则：
        // 1. PREBOOST (预升压): 仅屏蔽 Boost 过流 (防止启动冲击)，致命欠压立刻保护
        // 2. CLOSED_LOOP (闭环): 完全取消屏蔽，所有故障立刻触发
        // ==========================================
        else if(work_mode == MODE_ONGRID)
        {
            if(ongrid_calc_enable == 1)
            {
                // ==========================================
                // 【前级算法】无论在哪个状态，只要使能就运行
                // 目的：保持锁相环始终跟踪电网
                // ==========================================
                float V_VAC_Grid_Filtered = CascadeNotch_Update(&g_cascade_notch, V_VAC_Grid);  // 滤除3/5/7次谐波
                SOGI_FLL_Update_Optimized(&g_sogi, V_VAC_Grid_Filtered);   // 提取洁净基波
                SRF_PLL_Update(&g_pll, g_sogi.alpha, g_sogi.beta);      // 获取电网相位 θ

             
                int32_t ccr_boost = 0;   // Boost占空比暂存
                
                // ==========================================
                // Boost 双闭环控制 (电压外环 -> 电流内环)
                // 功能：稳定直流母线电压为 55V
                // ========================================== 
                float boost_v_ref = 55.0f; // 母线目标电压
                float boost_i_ref = PI_Update(&pi_boost_v_ongrid, boost_v_ref, V_VDC_Bus); // 电压环输出
                float boost_duty = PI_Update(&pi_boost_i_ongrid, boost_i_ref, -V_I_DCDC);  // 电流环输出
                
                // 占空比限幅 (防止上下管直通)
                if(boost_duty > 0.85f)  boost_duty = 0.85f; 
                if(boost_duty < 0.15f)  boost_duty = 0.15f;
                
                // 转换为定时器寄存器值 (Boost为反向计数)
                ccr_boost = (int32_t)((1.0f - boost_duty) * 8500.0f);

                // ==========================================
                // 并网状态机主逻辑
                // ==========================================
                switch(g_ongrid_state)
                {
                    // ==========================================
                    // 状态0：IDLE (空闲初始化)
                    // ==========================================
                    case ONGRID_STATE_IDLE:
                        GridTie_Boost_Start();       // 开启Boost PWM外设
                        g_ongrid_boost_en = 1;        // 置位Boost使能
                        g_fault_mask_cnt = 0;         // 清零屏蔽计数器
                        g_ongrid_state = ONGRID_STATE_PREBOOST; // 切换到预升压
                        break;

                    // ==========================================
                    // 状态1：PREBOOST (预升压)
                    // 【核心逻辑】仅屏蔽 Boost 过流，致命欠压不屏蔽
                    // ==========================================
                    case ONGRID_STATE_PREBOOST:
                    {
                        g_fault_mask_cnt++; // 屏蔽计数器自增
                        
                        // 更新 Boost PWM 输出
                        if(g_ongrid_boost_en == 1 && ongrid_calc_enable == 1)
                        {
                            TIM1->CCR1 = ccr_boost;
                        }

                        // ==========================================
                        // 母线电压稳定判断 (52V~58V，连续200拍)
                        // ==========================================
                        static uint16_t vdc_stable_cnt = 0;
                        uint8_t is_vdc_stable = 0;
                        
                        if(V_VDC_Bus > (VDC_STABLE_TARGET - VDC_STABLE_TOLERANCE) && 
                           V_VDC_Bus < (VDC_STABLE_TARGET + VDC_STABLE_TOLERANCE))
                        {
                            if(vdc_stable_cnt < 200) vdc_stable_cnt++;
                        } 
                        else 
                        {
                            vdc_stable_cnt = 0; // 电压波动，清零重来
                        }
                        
                        if(vdc_stable_cnt >= 200) is_vdc_stable = 1;

                        // ==========================================
                        // 预升压阶段故障检测
                        // ==========================================
                        uint8_t preboost_fault = 0;
                        
                        // --------------------
                        // 故障1：母线致命欠压 (<18V) 【不屏蔽！立刻检测】
                        // --------------------
                        // 注意：这里没有 if(g_fault_mask_cnt...)，只要低于18V立刻计数
                        if(V_VDC_Bus < FATAL_BUS_UV_THRESHOLD) 
                            preboost_fatal_cnt++; 
                        else 
                            preboost_fatal_cnt = 0;
                            
                        // --------------------
                        // 故障2：Boost 过流 (>6.0A) 【仅这个屏蔽】
                        // --------------------
                        if(g_fault_mask_cnt >= FAULT_MASK_TIME + 10)
                        {
                            if(fabsf(V_I_DCDC) > 6.0f) 
                                preboost_ocp_cnt++; 
                            else 
                                preboost_ocp_cnt = 0;
                        }
                            
                        // --------------------
                        // 故障触发逻辑
                        // --------------------
                        // 欠压：不需要等屏蔽时间，只要满足就触发
                        // 过流：必须等屏蔽时间过后才触发
                        if( (preboost_fatal_cnt >= FAULT_TRIGGER_CNT) || 
                            ((g_fault_mask_cnt >= FAULT_MASK_TIME + 10) && (preboost_ocp_cnt >= FAULT_TRIGGER_CNT)) )
                        {
                            preboost_fault = 1;
                        }

                        // ==========================================
                        // 故障处理
                        // ==========================================
                        if(preboost_fault) {
                            // 1. 置位标志位
                            if(preboost_fatal_cnt >= FAULT_TRIGGER_CNT) UVP_DC = 1;
                            if(preboost_ocp_cnt >= FAULT_TRIGGER_CNT)   OCP_BOOST = 1;
                            
                            // 2. 快照故障码
                            if(UVP_DC)     last_fault_code = FAULT_BUS_UNDERVOLT;
                            else if(OCP_BOOST) last_fault_code = FAULT_BOOST_OVERCUR;
                            
                            vdc_stable_cnt = 0;
                            
                            // 3. 执行停机
                            sys_stop();
                            ongrid_calc_enable = 0;
                            Fault_Flag = 1;
                            adc_isr_busy = 0;
                            return;
                        }
                        
                        // ==========================================
                        // 状态切换条件：母线稳定 + 锁相稳定，保持100ms
                        // ==========================================
                        #define PREBOOST_STABLE_TIME 40000 // 2s
                        static uint16_t ready_cnt = 0;
                        
                        if(is_vdc_stable && fabsf(g_pll.qerror) < 0.15f) {
                            ready_cnt++; 
                            if(ready_cnt > PREBOOST_STABLE_TIME) {
                                g_ongrid_state = ONGRID_STATE_SYNCING;
                                ready_cnt = 0;
                            }
                        } else {
                            ready_cnt = 0;
                        }
                        break;
                    }

                    // ==========================================
                    // 状态2：SYNCING (同步锁相，等待合闸)
                    // ==========================================
                    case ONGRID_STATE_SYNCING:
                    {
                        // Boost 维持升压运行
                        if(g_ongrid_boost_en == 1 && ongrid_calc_enable == 1)
                        {
                            TIM1->CCR1 = ccr_boost;
                        }
                        
                        // 更新过零检测算法
                        Rect_Lock_ZCD_Update(&g_rect_zcd, V_VAC_Grid, g_pll.qerror, g_pll.theta);
                        
                        // 合闸条件：检测到预判过零点 + 继电器未合闸
                        if(g_rect_zcd.Relay_Pretrigger_Flag == 1 && g_ongrid_relay_triggered == 0)
                        {
                            // 并网电流 PR 控制预计算
                            float cos_theta = g_pll.cos_theta;
                            float i_ref = grid_i_target * cos_theta; // 与电网同相
                            float i_err = i_ref - V_I_GridSide;      
                            float pr_out = PR_Inverter_Update(&pr_ongrid_i, i_err); 
                 
                            
                          // 合成调制波           
												ff_out_prop = FF_Proportional_Update(&ff_prop, V_VAC_Grid);
												ff_out_damp = FF_ActiveDamping_Update(&ff_damp, V_VAC_Grid); // 你说暂时不能加，可以保留计算但在调制波里不加
												ff_out_2nd  = FF_SecondOrder_Update(&ff_2nd, V_VAC_Grid);
						
											   modulation = pr_out +  (-0.07f * V_I_Cap)+ff_out_prop+ff_out_2nd+ff_out_damp;
                            
                            // 调制波限幅
                            if(modulation >  0.90f) modulation =  0.90f;
                            if(modulation < -0.90f) modulation = -0.90f;

                            // 预写 PWM 寄存器
                            int32_t ccr_inv_4 = SPWM_SinglePolarity_CCR(modulation);
                            int32_t ccr_inv_2 = SPWM_SinglePolarity_CCR(-modulation);
                            TIM1->CCR4 = ccr_inv_4;
                            TIM1->CCR2 = ccr_inv_2;

                            // 开启 H桥 + 合闸继电器
                            GridTie_Inverter_Start();
                          HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_SET);
                            
                            g_ongrid_relay_triggered = 1;
                            g_fault_mask_cnt = 0; // 清零，闭环反正不用
                            g_ongrid_hw_started = 1;
                            g_ongrid_state = ONGRID_STATE_CLOSED_LOOP; // 进入闭环
                        }
                        break;
                    }

                    // ==========================================
                    // 状态3：预留
                    // ==========================================
                    case ONGRID_STATE_RELAY_WAIT:
                        break;

                       // ==========================================
                    // 状态4：CLOSED_LOOP (闭环运行)
                    // 屏蔽规则：仅锁相环失锁屏蔽200ms，其他故障全开放
                    // ==========================================
                    case ONGRID_STATE_CLOSED_LOOP:
                    {
                        // 屏蔽计数器自增 (虽然其他故障不用，但给失锁屏蔽用)
                        if(g_fault_mask_cnt < FAULT_MASK_TIME + 10) g_fault_mask_cnt++; 
                        
                        uint8_t ongrid_fault = 0;

                        // ==========================================
                        // 故障1：锁相环失锁 【仅这个屏蔽！扛合闸冲击】
                        // ==========================================
                        #define PLL_SHOCK_MASK_TIME 4000 // 定义屏蔽时间：4000个中断 = 200ms @20kHz
                        
                        if(g_fault_mask_cnt >= PLL_SHOCK_MASK_TIME)
                        {
                            // 只有过了 200ms 抗冲击期，才开始检测失锁
                            if(fabsf(g_pll.qerror) > 0.3f) 
                                ongrid_pll_fault_cnt++; 
                            else 
                                ongrid_pll_fault_cnt = 0;
                        }
                        else
                        {
                            // 屏蔽期内：强制清零计数器，防止合闸尖峰误触发
                            ongrid_pll_fault_cnt = 0;
                        }

                        // ==========================================
                        // 故障2：母线致命欠压 (<18V) 【无屏蔽】
                        // ==========================================
                        if(V_VDC_Bus < FATAL_BUS_UV_THRESHOLD) 
                            ongrid_fatal_fault_cnt++; 
                        else 
                            ongrid_fatal_fault_cnt = 0;
                            
                        // ==========================================
                        // 故障3：母线过压 (>65V) 【无屏蔽】
                        // ==========================================
                        if(V_VDC_Bus > BUS_OV_THRESHOLD) 
                            ongrid_ov_fault_cnt++; 
                        else 
                            ongrid_ov_fault_cnt = 0;
                            
                        // ==========================================
                        // 故障4：母线欠压 (<52.0V) 【无屏蔽】
                        // ==========================================
                        if(V_VDC_Bus < 52.0f) 
                            ongrid_uv_fault_cnt++; 
                        else 
                            ongrid_uv_fault_cnt = 0;
                            
                       // ==========================================
                        // 故障5：网侧电流过流 (>6.0A) 【也加屏蔽】
                        // ==========================================
                        if(g_fault_mask_cnt >= PLL_SHOCK_MASK_TIME) // 使用和失锁一样的屏蔽时间
                        {
                            if(fabsf(V_I_GridSide) > 6.0f) 
                                ongrid_ocp_fault_cnt++; 
                            else 
                                ongrid_ocp_fault_cnt = 0;
                        }
                        else
                        {
                            // 屏蔽期内强制清零
                            ongrid_ocp_fault_cnt = 0;
                        }
												

                        // ==========================================
                        // 故障触发判断
                        // ==========================================
                        if(ongrid_pll_fault_cnt >= FAULT_TRIGGER_CNT)    { PLL_FAULT=1;   ongrid_fault=1; }
                        if(ongrid_fatal_fault_cnt >= FAULT_TRIGGER_CNT)  { UVP_DC=1;      ongrid_fault=1; }
                        if(ongrid_ov_fault_cnt >= FAULT_TRIGGER_CNT)     { OVP_DC=1;      ongrid_fault=1; }
                        if(ongrid_uv_fault_cnt >= FAULT_TRIGGER_CNT)     { UVP_DC=1;      ongrid_fault=1; }
                        if(ongrid_ocp_fault_cnt >= FAULT_TRIGGER_CNT)    { OCP_GRID=1;    ongrid_fault=1; }

                        // ==========================================
                        // 故障处理
                        // ==========================================
                        if(ongrid_fault) {
                            if(PLL_FAULT)      last_fault_code = FAULT_PLL_LOST;
                            else if(OCP_GRID)  last_fault_code = FAULT_GRID_OVERCUR;
                            else if(OCP_BOOST) last_fault_code = FAULT_BOOST_OVERCUR;
                            else if(OVP_DC)    last_fault_code = FAULT_BUS_OVERVOLT;
                            else if(UVP_DC)    last_fault_code = FAULT_BUS_UNDERVOLT;

                            sys_stop();
                            ongrid_calc_enable = 0;
                            Fault_Flag = 1;
                            adc_isr_busy = 0;
                            return;
                        }
                        
                        // ==========================================
                        // 深度过调制保护 (保持原样)
                        // ==========================================
                        if(fabsf(modulation) > 0.98f && V_VDC_Bus < 50.0f)
                        {
                            UVP_DC = 1;
                            last_fault_code = FAULT_BUS_UNDERVOLT;
                            sys_stop();
                            ongrid_calc_enable = 0;
                            Fault_Flag = 1;
                            adc_isr_busy = 0;
                            return;
                        }

                        // ==========================================
                        // 正常闭环控制 (含软启动)
                        // ==========================================
                        if(g_ongrid_hw_started == 1 && ongrid_calc_enable == 1)
                        {
                            float cos_theta = g_pll.cos_theta;
                            
                            // ===================== 并网电流软启动 (2秒线性上升) =====================
                            #define ONGRID_SOFT_START_TOTAL 40000
                            float target_amp_peek;
                            float amp;

                            // 有效值转峰值
                            target_amp_peek = 1.4142f * grid_i_target; 

                            // 软启动逻辑
                            if (g_ongrid_soft_start_cnt >= ONGRID_SOFT_START_TOTAL)
                            {
                                amp = target_amp_peek;
                            }
                            else
                            {
                                amp = target_amp_peek * ((float)g_ongrid_soft_start_cnt / ONGRID_SOFT_START_TOTAL);
                                g_ongrid_soft_start_cnt++;
                            }

                            // 生成电流参考
                            float i_ref = amp * cos_theta;
                            // =========================================================================
                            
                            float i_err = i_ref - V_I_GridSide;
                            float pr_out = PR_Inverter_Update(&pr_ongrid_i, i_err);
														
															
														// 1. 比例前馈			 // 2. 有源阻尼					 	// 3. 二阶微分前馈	
												ff_out_prop = FF_Proportional_Update(&ff_prop, V_VAC_Grid);
												ff_out_damp = FF_ActiveDamping_Update(&ff_damp, V_VAC_Grid); // 你说暂时不能加，可以保留计算但在调制波里不加
												ff_out_2nd  = FF_SecondOrder_Update(&ff_2nd, V_VAC_Grid);
														
                             modulation = pr_out +  (-0.07f * V_I_Cap)+ff_out_prop+ff_out_2nd+ff_out_damp;
                            // 调制波限幅
                            if(modulation >  0.98f) modulation =  0.98f;
                            if(modulation < -0.98f) modulation = -0.98f;

                            // 更新 PWM
                            TIM1->CCR1 = ccr_boost;
                            TIM1->CCR4 = SPWM_SinglePolarity_CCR(modulation);
                            TIM1->CCR2 = SPWM_SinglePolarity_CCR(-modulation);
                        }
                        break;
                    }
										
                    default: break;
                }
            }
        }

        adc_isr_busy = 0; // 清除忙标志
    }
}




// ==========================================
// 初始化与复位函数 
// ==========================================


/**
  * @brief  整流模式控制器初始化 (上电调用一次)
  * @note   仅配置参数，不清零运行状态
  */
void Controller_Rect_Init_All(void)
{
    // ==========================================
    // 0. 【新增】初始化全局复位标志
    // ==========================================
    g_rect_reset_req = 1; // 上电初始化为1，确保第一次进入模式时会复位

    // ==========================================
    // 1. 配置母线电压外环 PI 控制器
    // ==========================================
    pi_vdc.Kp = 0.5f;
    pi_vdc.Ki = 10.0f;
    pi_vdc.Ts = 0.00005f;       // 20kHz
    pi_vdc.OUT_MAX = 100.0f;
    pi_vdc.OUT_MIN = -100.0f;
    pi_vdc.LIMIT = 60.0f;
    pi_vdc.integral_val = 0.0f;
    pi_vdc.out = 0.0f;

    // ==========================================
    // 2. 配置 Buck 电流内环 PI 控制器
    // ==========================================
    pi_buck.Kp = 0.5f;
    pi_buck.Ki = 10.0f;
    pi_buck.Ts = 0.00005f;       // 20kHz
    pi_buck.OUT_MAX = 0.90f;
    pi_buck.OUT_MIN = 0.10f;
    pi_buck.LIMIT = 7.0f;
    pi_buck.integral_val = 0.0f;
    pi_buck.out = 0.0f;

    // ==========================================
    // 3. 初始化各类算法模块
    // ==========================================
    QPR_Rect_Init(&qpr_i);
    CascadeNotch_Init(&g_cascade_notch);
    SOGI_FLL_Init(&g_sogi);
    SRF_PLL_Init(&g_pll);
    
    // 【更新】使用新的状态机过零检测
    ZCD_StateMachine_Init(&g_zcd_state);
}

/**
  * @brief  整流模式控制器复位 (模式切换/故障清除时调用)
  * @note   彻底清零所有运行状态，确保干净重启
  */
void Controller_Rect_Reset_All(void)
{
    // ==========================================
    // 0. 确保硬件完全关断
    // ==========================================
    sys_stop(); // 关闭PWM输出
    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_RESET); // 确保继电器断开

    // ==========================================
    // 1. 复位所有控制器
    // ==========================================
    PI_Reset(&pi_vdc);
    PI_Reset(&pi_buck);
    QPR_Rect_Reset(&qpr_i);
    SOGI_FLL_Reset(&g_sogi);
    SRF_PLL_Reset(&g_pll);
    ZCD_StateMachine_Reset(&g_zcd_state);
    CascadeNotch_Reset(&g_cascade_notch);
    
    // ==========================================
    // 2. 清零故障计数器
    // ==========================================
    rect_pll_fault_cnt = 0;
    rect_bus_fault_cnt = 0;
    rect_boost_ocp_cnt = 0;
    rect_conv_ocp_cnt = 0;

    // ==========================================
    // 3. 清零故障标志位
    // ==========================================
    OVP_DC = 0;
    UVP_DC = 0;
    OCP_BOOST = 0;
    OCP_CONV = 0;
    PLL_FAULT = 0;

    // ==========================================
    // 4. 复位状态机和内部标志
    // ==========================================
    g_rect_state = RECT_STATE_PRECHARGE_ONLY;
    g_relay_triggered = 0;
    g_precharge_counter = 0;
    g_zero_detect_timer = 0;
    g_fault_mask_cnt = 0;
    g_rect_hw_started = 0;
    g_rect_open_idx = 0;

    // ==========================================
    // 5. 【新增】复位完成后，清零复位请求标志
    // ==========================================
    g_rect_reset_req = 0;
}


// ==========================================
// @brief  离网模式控制器初始化 (最终无卡死版)
// @note   1. 无关闭中断，不影响20kHz控制性能
//         2. RC状态完全清零，无记忆残留
//         3. 所有PI参数、状态、索引彻底初始化
// ==========================================
void Controller_Offgrid_Init_All(void)
{
    // 1. 【核心】重复控制器 RC 彻底初始化 + 清零
    RC_Init(&rc_offgrid_v);
    RC_Reset(&rc_offgrid_v);

    // ==================== 2. 离网电压外环 PI ====================
    pi_offgrid_v.Kp = 0.1f;
    pi_offgrid_v.Ki = 10000.0f;
    pi_offgrid_v.Ts = 5.0e-5f;
    pi_offgrid_v.OUT_MAX = 1000.0f;
    pi_offgrid_v.OUT_MIN = -1000.0f;
    pi_offgrid_v.LIMIT = 60.0f;
    pi_offgrid_v.integral_val = 0.0f;
    pi_offgrid_v.out = 0.0f;

    // ==================== 3. 离网电流内环 PI ====================
    pi_offgrid_i.Kp = 0.1f;
    pi_offgrid_i.Ki = 10.5f;
    pi_offgrid_i.Ts = 5.0e-5f;
    pi_offgrid_i.OUT_MAX = 0.98f;
    pi_offgrid_i.OUT_MIN = -0.98f;
    pi_offgrid_i.LIMIT = 7.0f;
    pi_offgrid_i.integral_val = 0.0f;
    pi_offgrid_i.out = 0.0f;

    // ==================== 4. Boost 电压环 PI ====================
    pi_boost_v.Kp = 0.5f;
    pi_boost_v.Ki = 10.0f;
    pi_boost_v.Ts = 5.0e-5f;
    pi_boost_v.OUT_MAX = 100.0f;
    pi_boost_v.OUT_MIN = -100.0f;
    pi_boost_v.LIMIT = 60.0f;
    pi_boost_v.integral_val = 0.0f;
    pi_boost_v.out = 0.0f;

    // ==================== 5. Boost 电流环 PI ====================
    pi_boost_i.Kp = 0.05f;
    pi_boost_i.Ki = 10.0f;
    pi_boost_i.Ts = 5.0e-5f;
    pi_boost_i.OUT_MAX = 0.57f;
    pi_boost_i.OUT_MIN = 0.15f;
    pi_boost_i.LIMIT = 5.0f;
    pi_boost_i.integral_val = 0.0f;
    pi_boost_i.out = 0.0f;

    // ==================== 6. 内部运行状态清零 ====================
    g_offgrid_hw_started = 0;
    g_fault_mask_cnt = 0;
    s_idx = 0;
    g_offgrid_soft_start_cnt = 0;   // 软启动计时器归零

    // ==================== 7. 故障计数器全部清零 ====================
    offgrid_fatal_fault_cnt = 0;
    offgrid_ov_fault_cnt = 0;
    offgrid_ocp_fault_cnt = 0;
    
    // ==================== 8. 【新增】离网专属故障标志清零 ====================
    OVP_OFFGRID = 0;
}

// ==========================================
// @brief  离网模式控制器彻底复位（停机/故障/模式切换专用）
// @note   1. 算法状态全清
//         2. 硬件启动标志强制清零
//         3. 故障状态全清
//         4. 绝对无残留状态，杜绝卡死/冲击
// ==========================================
void Controller_Offgrid_Reset_All(void)
{
    // 1. RC 状态清零
    RC_Reset(&rc_offgrid_v);

    // 2. 所有 PI 控制器复位
    PI_Reset(&pi_offgrid_v);
    PI_Reset(&pi_offgrid_i);
    PI_Reset(&pi_boost_v);
    PI_Reset(&pi_boost_i);

    // 3. 正弦波索引 & 软启动归零
    s_idx = 0;
    g_offgrid_soft_start_cnt = 0;

    // 4. 本模式专用故障计数清零
    offgrid_fatal_fault_cnt = 0;
    offgrid_ov_fault_cnt = 0;
    offgrid_ocp_fault_cnt = 0;

    // 5. 运行状态标志清零
    g_fault_mask_cnt = 0;
    g_offgrid_hw_started = 0;

    // 6. 【修复】仅清零硬件相关标志位
    // 绝对不要清零 Fault_Flag 和 last_fault_code！！
    OVP_DC = 0;
    UVP_DC = 0;
    OCP_BOOST = 0;
    OCP_CONV = 0;
    PLL_FAULT = 0;
    OVP_OFFGRID = 0;  // 【新增】清零离网过压标志
}

/**
  * @brief  并网模式控制器初始化
  * @note   可根据调试阶段，选择性注释掉不需要的前馈模块
  */
void Controller_Ongrid_Init_All(void)
{
    // 初始化并网状态机和标志
    g_ongrid_state = ONGRID_STATE_IDLE;
    g_ongrid_relay_triggered = 0;
    g_ongrid_boost_en = 0;
    g_ongrid_inverter_en = 0;
    g_vdc_window_idx = 0;
    g_vdc_running_sum = 0.0f;
    g_window_filled_flag = 0;
    g_ongrid_soft_start_cnt = 0;

    // ==================== 配置并网 Boost 电压 PI ====================
    pi_boost_v_ongrid.Kp = 0.5f;
    pi_boost_v_ongrid.Ki = 10.0f;
    pi_boost_v_ongrid.Ts = 5.0e-5f;
    pi_boost_v_ongrid.OUT_MAX = 100.0f;
    pi_boost_v_ongrid.OUT_MIN = -100.0f;
    pi_boost_v_ongrid.LIMIT = 60.0f;
    pi_boost_v_ongrid.integral_val = 0.0f;
    pi_boost_v_ongrid.out = 0.0f;

    // ==================== 配置并网 Boost 电流 PI ====================
    pi_boost_i_ongrid.Kp = 0.05f;
    pi_boost_i_ongrid.Ki = 10.0f;
    pi_boost_i_ongrid.Ts = 5.0e-5f;
    pi_boost_i_ongrid.OUT_MAX = 0.6f;
    pi_boost_i_ongrid.OUT_MIN = 0.15f;
    pi_boost_i_ongrid.LIMIT = 6.0f;
    pi_boost_i_ongrid.integral_val = 0.0f;
    pi_boost_i_ongrid.out = 0.0f;

    // 初始化算法模块
    SOGI_FLL_Init(&g_sogi);
    SRF_PLL_Init(&g_pll);
    Rect_Lock_ZCD_Init(&g_rect_zcd);
    PR_Inverter_Init(&pr_ongrid_i);
    CascadeNotch_Init(&g_cascade_notch);
    
    // =========================================================================
    // 【前馈模块初始化】按调试阶段选择性开启
    // =========================================================================
    
    // 【阶段1】必须初始化：比例前馈
    FF_Proportional_Init(&ff_prop);
    
    // 【阶段2】可选初始化：有源阻尼 (阶段1稳定后再加)
    FF_ActiveDamping_Init(&ff_damp);
    
    // 【阶段3】最后初始化：二阶微分前馈 (前两个都稳定后再加)
    FF_SecondOrder_Init(&ff_2nd);
    
    // =========================================================================
    
    // 初始化故障标志
    OCP_GRID = 0;
}

/**
  * @brief  并网模式控制器复位
  */
void Controller_Ongrid_Reset_All(void)
{
    // 复位所有控制器
    PI_Reset(&pi_boost_v_ongrid);
    PI_Reset(&pi_boost_i_ongrid);
    SOGI_FLL_Reset(&g_sogi);
    SRF_PLL_Reset(&g_pll);
    Rect_Lock_ZCD_Reset(&g_rect_zcd);
    PR_Inverter_Reset(&pr_ongrid_i);
    CascadeNotch_Reset(&g_cascade_notch);

    // =========================================================================
    // 【前馈模块复位】与初始化对应
    // =========================================================================
    FF_Proportional_Reset(&ff_prop);
    FF_ActiveDamping_Reset(&ff_damp);
    FF_SecondOrder_Reset(&ff_2nd);
    // =========================================================================

    // 显式清零所有阶段的故障计数器
    preboost_fatal_cnt = 0;
    preboost_ocp_cnt = 0;
    ongrid_fatal_fault_cnt = 0;
    ongrid_ov_fault_cnt = 0;
    ongrid_uv_fault_cnt = 0;
    ongrid_ocp_fault_cnt = 0;
    ongrid_pll_fault_cnt = 0;

    // 复位状态机和内部标志
    g_ongrid_state = ONGRID_STATE_IDLE;
    g_ongrid_relay_triggered = 0;
    g_ongrid_boost_en = 0;
    g_ongrid_inverter_en = 0;
    g_ongrid_hw_started = 0;
    g_vdc_window_idx = 0;
    g_vdc_running_sum = 0.0f;
    g_window_filled_flag = 0;
    g_fault_mask_cnt = 0;
    g_ongrid_soft_start_cnt = 0;

    // 清零所有故障标志
    OVP_DC = 0;
    UVP_DC = 0;
    OCP_BOOST = 0;
    OCP_CONV = 0;
    PLL_FAULT = 0;
    OCP_GRID = 0;
}


// ==========================================
// 【新增】待机 BUCK 测试初始化    
// ==========================================
void Controller_BuckTest_Init_All(void)
{
    pi_buck_test.Kp = 7.5f;
    pi_buck_test.Ki = 80.0f;
    pi_buck_test.Ts = 0.00005f;
    pi_buck_test.OUT_MAX = 0.93f;
    pi_buck_test.OUT_MIN = 0.07f;
    pi_buck_test.LIMIT = 6.0f;
    pi_buck_test.integral_val = 0.0f;
    pi_buck_test.out = 0.0f;

    buck_test_calc_enable = 0;
    buck_test_i_target = 1.0f;
}

// ==========================================
// 【新增】待机 BUCK 测试复位
// ==========================================
void Controller_BuckTest_Reset_All(void)
{
    buck_test_calc_enable = 0;
    PI_Reset(&pi_buck_test);
    sys_stop();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
}


// ==========================================
// 【新增】待机 BOOST 测试初始化
// 功能：配置电压外环、电流内环 PI 参数
// ==========================================
void Controller_BoostTest_Init_All(void)
{
    // ==================== 配置 Boost 电压外环 PI ====================
    pi_bt_v.Kp = 0.5f;
    pi_bt_v.Ki = 10.0f;
    pi_bt_v.Ts = 0.00005f;
    pi_bt_v.OUT_MAX = 100.0f;
    pi_bt_v.OUT_MIN = -100.0f;
    pi_bt_v.LIMIT = 60.0f;
    pi_bt_v.integral_val = 0.0f;
    pi_bt_v.out = 0.0f;

    // ==================== 配置 Boost 电流内环 PI ====================
    pi_bt_i.Kp = 0.05f;
    pi_bt_i.Ki = 10.0f;
    pi_bt_i.Ts = 0.00005f;
    pi_bt_i.OUT_MAX = 0.58f;
    pi_bt_i.OUT_MIN = 0.15f;
    pi_bt_i.LIMIT = 5.0f;
    pi_bt_i.integral_val = 0.0f;
    pi_bt_i.out = 0.0f;

    // ==================== 复位状态标志 ====================
    PI_Reset(&pi_bt_v);
    PI_Reset(&pi_bt_i);

    boost_test_calc_enable = 0;
    boost_test_v_target = 55.0f;
    g_boost_test_hw_started = 0;

    // 清零故障计数器
    bt_bus_ov_cnt = 0;
    bt_bus_uv_cnt = 0;
    bt_ocp_cnt = 0;
			
}



// ==========================================
// 【新增】待机 BOOST 测试复位
// 功能：停止计算、复位算法、关闭硬件、清零故障
// ==========================================
void Controller_BoostTest_Reset_All(void)
{
    // 1. 立即关闭计算使能
    boost_test_calc_enable = 0;
	


    // 2. 复位 PI 控制器状态
    PI_Reset(&pi_bt_v);
    PI_Reset(&pi_bt_i);

    // 3. 硬件安全停机
    sys_stop();

    // 4. 关闭驱动芯片（拉高使能引脚）
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

    // 5. 清零硬件启动标志
    g_boost_test_hw_started = 0;

    // 6. 清零所有故障计数器
    bt_bus_ov_cnt = 0;
    bt_bus_uv_cnt = 0;
    bt_ocp_cnt = 0;
}


// ==========================================
// 【新增】待机 SOGI 测试初始化
// ==========================================
void Controller_SOGITest_Init_All(void)
{
    // 初始化SOGI和PLL
    CascadeNotch_Init(&g_cascade_notch);
    SOGI_FLL_Init(&g_sogi);
    SRF_PLL_Init(&g_pll);

    sogi_test_calc_enable = 0;
}

// ==========================================
// 【新增】待机 SOGI 测试复位
// ==========================================
void Controller_SOGITest_Reset_All(void)
{
    sogi_test_calc_enable = 0;
    
    // 复位算法状态
    CascadeNotch_Reset(&g_cascade_notch);
    SOGI_FLL_Reset(&g_sogi);
    SRF_PLL_Reset(&g_pll);
}

