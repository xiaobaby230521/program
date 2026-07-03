/* USER CODE BEGIN Header */
/**
  * @file           : main.c
  * @brief          : 最终版：OLED仅按键刷新 · 故障定格不自动刷新
  * 
  * 核心功能：
  * 1. 四模式工作：待机(STANDBY)、整流(RECTIFIER)、离网(OFFGRID)、并网(ONGRID)
  * 2. 三级测试：Buck、Boost、SOGI锁相环
  * 3. 故障保护：过压/欠压/过流/失锁，故障后屏幕锁定显示
  * 4. 人机交互：OLED显示 + 4按键操作
  * 
  * 故障列表 (FaultCode_t):
  * - FAULT_PLL_LOST:          锁相环失锁
  * - FAULT_CONV_OVERCUR:      逆变侧(H桥)过流
  * - FAULT_BOOST_OVERCUR:     Boost级过流
  * - FAULT_BUS_OVERVOLT:      直流母线过压
  * - FAULT_BUS_UNDERVOLT:     直流母线欠压
  * - FAULT_OFFGRID_OVERVOLT:  离网输出过压 (新增)
  * - FAULT_GRID_OVERCUR:      网侧电流过流 (新增)
  */
/* USER CODE END Header */

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "oled.h"
#include "control.h"
#include "calculate.h"
#include "key.h"
#include "debug.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

// ==================== 系统状态变量 ====================
WorkMode_TypeDef work_mode = MODE_STANDBY; // 当前工作模式枚举
uint8_t relay_state = 0;                    // 继电器状态 (仅并网/整流用)
uint8_t pll_enable = 0;                     // 锁相环使能标志

// ==================== 控制目标值 ====================
float grid_i_target = 2.0f;         // 并网/整流电流目标 (A) 
float offgrid_Uoac_target = 32.0f;  // 离网输出电压目标 (V)

// ==================== 外部控制使能 (来自 control.c) ====================
extern volatile uint8_t rect_calc_enable;
extern volatile uint8_t offgrid_calc_enable;
extern volatile uint8_t ongrid_calc_enable;
extern volatile uint8_t buck_test_calc_enable;
extern volatile uint8_t boost_test_calc_enable;
extern volatile uint8_t sogi_test_calc_enable;

// ==================== 外部测试目标值 ====================
extern float buck_test_i_target;
extern float boost_test_v_target;

// ==================== 外部采样值 (来自 control.c) ====================
extern float V_VDC_Bus; // 母线电压
extern float V_I_DCDC;  // DCDC电感电流

// ==================== 外部故障标志 (来自 control.c) ====================
extern uint8_t OVP_DC;         // 直流母线过压
extern uint8_t UVP_DC;         // 直流母线欠压
extern uint8_t OCP_BOOST;      // Boost过流
extern uint8_t OCP_CONV;       // 变换器(H桥)过流
extern uint8_t PLL_FAULT;      // 锁相环失锁
extern uint8_t OVP_OFFGRID;    // 【新增】离网输出过压
extern uint8_t OCP_GRID;        // 【新增】网侧电流过流
extern uint8_t Fault_Flag;     // 总故障标志 (逻辑或)
extern FaultCode_t last_fault_code; // 最后一次故障代码 (用于显示)

// ==================== 外部同步标志 ====================
extern uint8_t adc_isr_busy; // ADC中断忙标志 (1=正在计算，勿扰)

// ==================== 本机状态锁存 ====================
uint8_t fault_locked = 0; // 故障锁定标志 (1=屏幕定格，按键仅解锁)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void OLED_Status_Update(void);
void Fault_Recovery_Process(void);

/* USER CODE BEGIN 0 */

/**
  * @brief  OLED状态刷新函数
  * @note   仅在按键按下或故障触发时调用，不自动刷屏
  */
void OLED_Status_Update(void)
{
    char buf[32]; // 用于格式化字符串的缓冲区
    OLED_Clear(); // 清屏

    // ==========================================
    // 1. 第一行：永远显示当前模式
    // ==========================================
    switch(work_mode)
    {
        case MODE_STANDBY:   OLED_ShowString(0, 0, (u8*)"MODE: STANDBY   ", 16, 1); break;
        case MODE_RECTIFIER: OLED_ShowString(0, 0, (u8*)"MODE: RECTIFIER ", 16, 1); break;
        case MODE_OFFGRID:   OLED_ShowString(0, 0, (u8*)"MODE: OFFGRID   ", 16, 1); break;
        case MODE_ONGRID:    OLED_ShowString(0, 0, (u8*)"MODE: ONGRID    ", 16, 1); break;
        case MODE_BUCK:      OLED_ShowString(0, 0, (u8*)"TEST: BUCK MODE ", 16, 1); break;
        case MODE_BOOST:     OLED_ShowString(0, 0, (u8*)"TEST: BOOST MODE", 16, 1); break;
        case MODE_SOGI:      OLED_ShowString(0, 0, (u8*)"TEST: SOGI PLL  ", 16, 1); break;
        default:             OLED_ShowString(0, 0, (u8*)"MODE: ERROR     ", 16, 1); break;
    }

    // ==========================================
    // 2. 待机模式下的详细显示
    // ==========================================
    if(work_mode == MODE_STANDBY)
    {
        // 情况A：有故障记录 (锁定状态)
        if(last_fault_code != FAULT_NONE)
        {
            OLED_ShowString(0, 16, (u8*)"STATUS: LOCKED!!", 16, 1);
            OLED_ShowString(0, 32, (u8*)"--- LAST ERR ---", 16, 1);
            
            // 根据故障码显示具体原因
            switch(last_fault_code)
            {
                case FAULT_PLL_LOST:          OLED_ShowString(0, 48, (u8*)"PLL LOST LOCK  ", 16, 1); break;
                case FAULT_CONV_OVERCUR:      OLED_ShowString(0, 48, (u8*)"INV OVERCURRENT", 16, 1); break;
                case FAULT_BOOST_OVERCUR:     OLED_ShowString(0, 48, (u8*)"BOOST OVERCUR  ", 16, 1); break;
                case FAULT_BUS_OVERVOLT:      OLED_ShowString(0, 48, (u8*)"BUS OVERVOLTAGE", 16, 1); break;
                case FAULT_BUS_UNDERVOLT:     OLED_ShowString(0, 48, (u8*)"BUS UNDERVOLT  ", 16, 1); break;
                case FAULT_OFFGRID_OVERVOLT:  OLED_ShowString(0, 48, (u8*)"OFFGRID OV     ", 16, 1); break; // 【新增】离网过压
                case FAULT_GRID_OVERCUR:      OLED_ShowString(0, 48, (u8*)"GRID OVERCUR   ", 16, 1); break; // 【新增】网侧过流
							  case FAULT_BUCK_RMS_OVERCUR: OLED_ShowString(0, 48, (u8*)"BUCK RMS OCP   ", 16, 1); break;
                default:                      OLED_ShowString(0, 48, (u8*)"UNKNOWN ERROR  ", 16, 1); break;

							
            }
        }
        // 情况B：无故障 (正常待机)
        else
        {
            OLED_ShowString(0, 16, (u8*)"STATUS: READY   ", 16, 1);
            OLED_ShowString(0, 32, (u8*)"K1:Switch Mode  ", 12, 1); // 提示K1切换模式
            OLED_ShowString(0, 48, (u8*)"K2:Buck K3:Boost K4:SOGI", 12, 1); // 提示测试模式入口
        }
    }
    // ==========================================
    // 3. Buck测试模式显示
    // ==========================================
    else if(work_mode == MODE_BUCK)
    {
        OLED_ShowString(0, 16, buck_test_calc_enable ? (u8*)"STATE: RUNNING " : (u8*)"STATE: STOPPED ", 16, 1);
        sprintf(buf, "I_Targ: %.1f A", buck_test_i_target); // 显示电流目标值
        OLED_ShowString(0, 32, (u8*)buf, 16, 1);
    }
    // ==========================================
    // 4. Boost测试模式显示
    // ==========================================
    else if(work_mode == MODE_BOOST)
    {
        OLED_ShowString(0, 16, boost_test_calc_enable ? (u8*)"STATE: RUNNING " : (u8*)"STATE: STOPPED ", 16, 1);
        sprintf(buf, "V_Targ: %.1f V", boost_test_v_target); // 显示电压目标值
        OLED_ShowString(0, 32, (u8*)buf, 16, 1);
    }
    // ==========================================
    // 5. SOGI测试模式显示
    // ==========================================
    else if(work_mode == MODE_SOGI)
    {
        OLED_ShowString(0, 16, (u8*)"STATE: RUNNING ", 16, 1);
        OLED_ShowString(0, 32, (u8*)"VOFA: Grid,SOGI", 16, 1); // 提示看上位机
        OLED_ShowString(0, 48, (u8*)"K4: Exit to STBY", 16, 1);
    }
    // ==========================================
    // 6. 三大主模式显示 (整流/离网/并网)
    // ==========================================
    else
    {
        // 判断是否正在运行 (看对应使能位)
        uint8_t is_run = 0;
        if(work_mode == MODE_RECTIFIER) is_run = rect_calc_enable;
        else if(work_mode == MODE_OFFGRID) is_run = offgrid_calc_enable;
        else if(work_mode == MODE_ONGRID) is_run = ongrid_calc_enable;

        OLED_ShowString(0, 16, is_run ? (u8*)"STATUS: RUNNING" : (u8*)"STATUS: STOPPED", 16, 1);
        
        // 显示目标值 (离网显示电压，其他显示电流)
        if(work_mode == MODE_OFFGRID)
            sprintf(buf, "Uo_Tar: %.1f V", offgrid_Uoac_target);
        else
            sprintf(buf, "Io_Tar: %.1f A", grid_i_target);
        
        OLED_ShowString(0, 32, (u8*)buf, 16, 1);
    }

    OLED_Refresh(); // 最后调用一次刷新，将显存内容写入屏幕
}

/**
  * @brief  故障恢复与处理流程 (主循环轮询)
  * @note   核心逻辑：检测故障 -> 锁存故障码 -> 停机 -> 定格显示
  */
void Fault_Recovery_Process(void)
{
    // 如果已经锁定，直接跳过 (防止重复触发)
    if(fault_locked == 1)
        return;

    // 检测到总故障标志为1
    if(Fault_Flag == 1)
    {
        fault_locked = 1; // 立即锁定系统

        // ==========================================
        // 【关键 1】必须第一时间快照故障码！
        // 绝对不能先调用 Controller_XXX_Reset_All()，否则标志位会被清零！
        // 注意：判断顺序很重要，优先级高的放前面
        // ==========================================
        if(PLL_FAULT)          last_fault_code = FAULT_PLL_LOST;
        else if(OCP_GRID)      last_fault_code = FAULT_GRID_OVERCUR;      // 【新增】网侧过流 (高优先级)
        else if(OCP_CONV)      last_fault_code = FAULT_CONV_OVERCUR;
        else if(OCP_BOOST)     last_fault_code = FAULT_BOOST_OVERCUR;
        else if(OVP_OFFGRID)   last_fault_code = FAULT_OFFGRID_OVERVOLT;  // 【新增】离网过压
        else if(OVP_DC)        last_fault_code = FAULT_BUS_OVERVOLT;
        else if(UVP_DC)        last_fault_code = FAULT_BUS_UNDERVOLT;

        // ==========================================
        // 【关键 2】执行安全停机序列
        // ==========================================
        // 1. 关闭所有软件使能
        rect_calc_enable = 0;
        offgrid_calc_enable = 0;
        ongrid_calc_enable = 0;
        buck_test_calc_enable = 0;
        boost_test_calc_enable = 0;
        sogi_test_calc_enable = 0;

        // 2. 复位所有控制器 (清零算法积分项等状态)
        Controller_BuckTest_Reset_All();
        Controller_BoostTest_Reset_All();
        Controller_SOGITest_Reset_All();
        Controller_Rect_Reset_All();
        Controller_Offgrid_Reset_All(); 
        Controller_Ongrid_Reset_All();

        // 3. 硬件级停机 (关闭PWM输出)
        sys_stop();
        
        // 4. 断开继电器和关闭驱动使能 (拉高引脚为关闭)
        HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

        // 5. 切回待机模式
        work_mode = MODE_STANDBY;

        // ==========================================
        // 最后刷新OLED，显示故障信息并定格
        // ==========================================
        OLED_Status_Update();
    }
}

/* USER CODE END 0 */

/**
  * @brief  主函数入口
  */
int main(void)
{
    // 1. HAL库初始化
    HAL_Init();
    
    // 2. 系统时钟配置 (170MHz)
    SystemClock_Config();

    // 3. 外设初始化 (GPIO, DMA, ADC, Timer, UART)
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM1_Init();
    MX_USART3_UART_Init();

    // 4. OLED初始化与开机画面
    HAL_Delay(100);
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, (u8*)"System Core Init", 16, 1);
    OLED_Refresh();
    HAL_Delay(400); // 延时让用户看到启动画面

    // 5. 所有控制算法初始化 (仅上电执行一次)
    Controller_Rect_Init_All();
    Controller_Offgrid_Init_All();
    Controller_Ongrid_Init_All();
    Controller_BuckTest_Init_All();
    Controller_BoostTest_Init_All();
    Controller_SOGITest_Init_All();

    // 6. 初始状态设置
    sys_stop();       // 确保硬件PWM停止
    fault_locked = 0; // 解锁状态
    OLED_Status_Update(); // 刷新到待机界面

    // 7. 启动模拟与数字外设
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // ADC校准
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Sample, 7);   // 启动ADC DMA搬运 (7通道)
    HAL_TIM_Base_Start(&htim1);                               // 启动定时器 (触发ADC)

    // ==================== 主循环 (Super Loop) ====================
    while (1)
    {
        // 任务1：故障检测与处理 (最高优先级)
        Fault_Recovery_Process();

        // 如果处于故障锁定状态...
        if(fault_locked == 1)
        {
            Key_Scan(); // 仅扫描按键
            // 检测K1键：解锁系统
            if(Key_GetAndClear() == 1)
            {
                fault_locked = 0;                    // 解锁
                Fault_Flag = 0;                      // 清除总故障标志
                last_fault_code = FAULT_NONE;        // 清除故障记忆
                
                // 清除所有具体故障标志
                OVP_DC = 0; 
                UVP_DC = 0; 
                OCP_BOOST = 0; 
                OCP_CONV = 0; 
                PLL_FAULT = 0;
                OVP_OFFGRID = 0;                     // 【新增】清零离网过压标志
                OCP_GRID = 0;                         // 【新增】清零网侧过流标志
                
                OLED_Status_Update();                // 刷新回正常待机界面
            }
            HAL_Delay(10);
            continue; // 锁定状态下跳过后面的逻辑
        }

        // 任务2：正常状态下的按键扫描
        Key_Scan();

        // 任务3：如果ADC不在中断中，处理按键事件
        if(adc_isr_busy == 0)
        {
            uint8_t key = Key_GetAndClear();
            if(key != 0)
            {
                if(key == 1)
                    Mode_Switch_Handle();    // K1: 切换模式
                else
                    Key_Function_Handle(key); // K2/K3/K4: 上下文相关功能
                
                OLED_Status_Update(); // 按键操作后刷新一次屏幕
            }
        }
        HAL_Delay(5); // 简单的延时，防止CPU占用率100%
    }
}

/**
  * @brief  System Clock Configuration (STM32G474 170MHz)
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // 配置电压缩放范围 (Boost模式以支持170MHz)
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    // 配置HSE外部晶振 + PLL
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    // 配置AHB/APB总线分频
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

/**
  * @brief  错误处理函数 (HAL库报错时调用)
  */
void Error_Handler(void)
{
    __disable_irq(); // 关中断
    while (1) {}    // 死循环死机
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  断言失败回调
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number */
}
#endif /* USE_FULL_ASSERT */