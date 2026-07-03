#ifndef __CONTROL_H
#define __CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "stdint.h"
#include "calculate.h"

// ==========================================
// 【枚举定义】工作模式
// ==========================================
typedef enum {
    MODE_STANDBY = 1,       // 模式1：待机 (安全模式)
    MODE_RECTIFIER,         // 模式2：整流 (AC->DC)
    MODE_OFFGRID,           // 模式3：离网逆变 (DC->AC)
    MODE_ONGRID,            // 模式4：并网逆变 (DC->AC并上网)
    MODE_BUCK,              // 模式5：待机 Buck 测试 (注意这里的逗号)
    MODE_BOOST,             // 模式6：待机 Boost 测试
	  MODE_SOGI
} WorkMode_TypeDef;


// ==========================================
// 【枚举定义】故障记忆码
// 设计意图：用于在待机界面永久显示最后一次故障原因
// ==========================================
typedef enum {
    FAULT_NONE = 0,         // 无故障记录 (上电默认状态)
    FAULT_PLL_LOST,         // 故障1：锁相环失锁
    FAULT_CONV_OVERCUR,     // 故障2：变换器(H桥)过流
    FAULT_BOOST_OVERCUR,    // 故障3：Boost级过流
    FAULT_BUS_OVERVOLT,     // 故障4：直流母线过压
    FAULT_BUS_UNDERVOLT ,    // 故障5：直流母线欠压
	  FAULT_OFFGRID_OVERVOLT,  // 【新增】离网输出过压
    FAULT_GRID_OVERCUR,       // 【新增】网侧电流过流
	  FAULT_BUCK_RMS_OVERCUR, // 【新增】Buck电流有效值过流
} FaultCode_t;


// ==========================================
// 【外部变量声明】 (内存分配在 control.c)
// 使用 extern 告诉编译器变量在其他文件定义，避免重复定义错误
// ==========================================

// --- 核心控制使能标志 ---
// 设计意图：volatile 确保主循环和中断看到的是同一个值
extern volatile uint8_t rect_calc_enable;     // 整流模式计算使能 (1=运行, 0=停止)
extern volatile uint8_t offgrid_calc_enable;  // 离网模式计算使能
extern volatile uint8_t ongrid_calc_enable;   // 并网模式计算使能

// --- 全局状态标志 ---
extern WorkMode_TypeDef work_mode;             // 当前系统工作模式
extern uint8_t adc_isr_busy;                   // ADC中断忙标志 (用于按键互斥)

// --- 故障标志位 (实时) ---
// 注意：这些标志会被 Controller_XXX_Reset_All() 清零
extern uint8_t OVP_DC;                         // 直流母线过压标志 (Over Voltage Protection)
extern uint8_t UVP_DC;                         // 直流母线欠压标志 (Under Voltage Protection)
extern uint8_t OCP_BOOST;                      // Boost侧过流标志 (Over Current Protection)
extern uint8_t OCP_CONV;                       // 变换器(H桥)侧过流标志
extern uint8_t PLL_FAULT;                      // 锁相环失锁标志
extern uint8_t OVP_OFFGRID ;                   // 【新增】离网输出过压故障
extern uint8_t OCP_GRID ;                      // 【新增】网侧电流过流故障
extern uint8_t Fault_Flag;                     // 总故障标志 (任意故障置1，锁存用)

// --- 故障记忆 (非易失性逻辑) ---
// 注意：这个变量不会被复位函数清零，只有硬件Reset才会清空
extern FaultCode_t last_fault_code;            // 最后一次故障记忆码

// --- ADC数据缓冲区 ---
extern uint16_t ADC_Sample[7];                 // ADC原始数据 (DMA自动写入)

// --- 物理量变量 (浮点数) ---
extern float V_VAC_Grid;                       // 电网电压 (物理值)
extern float V_VDC_Bus;                        // 直流母线电压 (物理值)
extern float V_V_OffGrid;                      // 离网输出电压 (物理值)
extern float V_I_GridSide;                     // 网侧电流 (物理值)
extern float V_I_ConvSide;                     // 变换器侧电流 (物理值)
extern float V_I_DCDC;                         // DCDC(Boost)侧电流 (物理值)
extern float V_I_Cap;                          // 电容电流 (物理值)

// --- 查表法资源 ---
extern const float sin_wave[400];              // 50Hz正弦波表 (20kHz中断，400点)

// --- VOFA调试缓冲区 ---
// 设计意图：分模式独立缓冲区，防止数据污染
extern uint8_t vofa_buf_rect[20];              // 整流模式VOFA数据 (4个float + 尾帧)
extern uint8_t vofa_buf_offgrid[20];           // 离网模式VOFA数据 (4个float + 尾帧)
extern uint8_t vofa_buf_ongrid[24];            // 并网模式VOFA数据 (5个float + 尾帧)


// 外部引用 (声明在其他文件定义的变量，避免重复定义)
extern float grid_i_target;         // 网侧电流目标值 (来自 key.c 或 main.c)
extern float offgrid_Uoac_target;    // 离网电压目标值 (来自 key.c 或 main.c)
extern WorkMode_TypeDef work_mode;   // 系统工作模式枚举 


// ==========================================
// 【函数声明】
// ==========================================

// --- STM32 HAL 回调函数 ---
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc); // 20kHz核心控制中断

// --- 系统硬件控制 (在 control.c 或 calculate.c 实现) ---
void sys_start(void);                           // 开启硬件 (开MOE/PWM)
void sys_stop(void);                            // 关闭硬件 (关MOE/PWM，安全停机)
void GridTie_Boost_Start(void);                // 并网模式Boost启动
void GridTie_Inverter_Start(void);             // 并网模式H桥启动
int32_t SPWM_SinglePolarity_CCR(float duty);  // 单极性SPWM调制波转CCR值

// --- 【整流模式】控制器 ---
void Controller_Rect_Init_All(void);           // 整流模式初始化 (上电调用一次)
void Controller_Rect_Reset_All(void);          // 整流模式复位 (故障/切模式调用)

// --- 【离网模式】控制器 ---
void Controller_Offgrid_Init_All(void);        // 离网模式初始化
void Controller_Offgrid_Reset_All(void);       // 离网模式复位

// --- 【并网模式】控制器 ---
void Controller_Ongrid_Init_All(void);         // 并网模式初始化
void Controller_Ongrid_Reset_All(void);        // 并网模式复位


// ==================== 待机模式 BUCK 测试 ====================
extern volatile uint8_t buck_test_calc_enable;
extern float buck_test_i_target;

void Controller_BuckTest_Init_All(void);
void Controller_BuckTest_Reset_All(void);



// --- Boost 测试变量 ---
extern volatile uint8_t boost_test_calc_enable;
extern float boost_test_v_target; // 目标 55V
extern PI_TypeDef pi_boost_v;    // 电压环
extern PI_TypeDef pi_boost_i;    // 电流环

// --- 函数声明 ---
void Controller_BoostTest_Init_All(void);
void Controller_BoostTest_Reset_All(void);



// ==================== 待机模式 SOGI 测试 ====================
extern volatile uint8_t sogi_test_calc_enable;

void Controller_SOGITest_Init_All(void);
void Controller_SOGITest_Reset_All(void);

void Controller_Rect_ForceEnterClosedLoop(void);   //整流算力测试



#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_H */