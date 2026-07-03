/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> 
#include <string.h>
#include "ADC_Filter.h"
#include "pi_control.h" 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_FLASH_INTERVAL 200   // LED闪烁间隔200ms
#define ADC_CHANNELS       4     // 4个ADC通道
#define SAMPLES_PER_PWM    2     // 中央对齐1个PWM周期采样2次
#define PWM_CYCLES         1     // 缓存1个PWM周期数据
#define BUFFER_LENGTH      (ADC_CHANNELS * SAMPLES_PER_PWM * PWM_CYCLES) // 8个数据

// KEY1按键配置（匹配你的硬件：PC1、上拉输入）
#define KEY1_PIN           GPIO_PIN_13
#define KEY1_PORT          GPIOB
#define KEY1_PRESSED       0           // 按键按下为低电平（上拉输入）
#define KEY_DEBOUNCE_MS    50          // 增大消抖时间到50ms，抗干扰
#define CURRENT_STEP       0.1f        // 电流步进值（宏定义，避免浮点魔法数）
#define CURRENT_MAX        1.5f        // 最大电流
#define CURRENT_MIN        1.0f        // 最小电流
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// 浮点值比较宏（解决精度问题）
#define FLOAT_EQ(a, b) (fabs((a) - (b)) < 0.001f)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t sys_tick_ms = 0;
uint32_t last_led_tick = 0;
uint8_t led_state = 0;
float buck_duty = 0.5f;  // 占空比0~1
ControlData buck_control_data;
// ADC采样缓冲区
uint16_t adc_sample_buffer[BUFFER_LENGTH] = {0};
// DMA传输完成标志
volatile uint8_t adc_dma_complete_flag = 0;

// 按键相关变量
uint32_t last_key1_tick = 0;          // 按键消抖计时
uint8_t key1_state = 0;               // 按键状态（0：未按下，1：已按下）
// 核心修复：显式定义并初始化期望电流（避免内存随机值）
float BUCK_I_CHARGE_REF = CURRENT_MIN;





volatile uint16_t ch1_sample1 = 0;
volatile uint16_t ch1_sample2 = 0;
volatile uint16_t ch2_sample1 = 0;
volatile uint16_t ch2_sample2 = 0;
volatile uint16_t ch4_sample1 = 0;
volatile uint16_t ch4_sample2 = 0;
volatile uint16_t ch6_sample1 = 0;
volatile uint16_t ch6_sample2 = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// 新增：按键扫描函数（封装逻辑，便于调试）
uint8_t KEY1_Scan(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// TIM3中断回调函数（1ms系统滴答）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        sys_tick_ms++;  // 仅保留1ms系统滴答
    }
}

// ADC DMA传输完成回调函数
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        // 步骤1：快速解析缓冲区数据（纯内存操作，耗时<1μs）
        ch1_sample1 = adc_sample_buffer[0];
        ch1_sample2 = adc_sample_buffer[4];
        ch2_sample1 = adc_sample_buffer[1];
        ch2_sample2 = adc_sample_buffer[5];
        ch4_sample1 = adc_sample_buffer[2];
        ch4_sample2 = adc_sample_buffer[6];
        ch6_sample1 = adc_sample_buffer[3];
        ch6_sample2 = adc_sample_buffer[7];

        float adc_1=(ch1_sample1+ch1_sample2)/2.0f;
       

        ADC_Calculate_Current1 (adc_1);
       
        filtered_current1= smooth_sum_filter1(current1);  //电感电流
       
        buck_duty = BUCK_Charge_Control(&buck_control_data, 0.0f, filtered_current1, 0.0f);
        
        // 2. 适配PWM Mode 1的CCR计算（TIM1 ARR=8499）
        uint16_t arr = htim1.Init.Period; // 获取TIM1的ARR值（8499）
        uint16_t ccr_value = (uint16_t)(buck_duty * (arr + 1)); // 核心映射公式

        // 3. 安全防护：确保CCR值不超出定时器范围
        if (ccr_value > arr) ccr_value = arr;

        // 4. 更新TIM1通道3的PWM占空比
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr_value);
    }
}

// 按键扫描函数（带消抖、防重复触发）
uint8_t KEY1_Scan(void)
{
    static uint8_t key_flag = 0; // 静态变量，记录按键触发状态
    uint8_t ret = 0;
    
    // 1. 检测按键硬件状态
    if (HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN) == KEY1_PRESSED)
    {
        // 2. 消抖延时
        HAL_Delay(10); // 硬件消抖（短延时）
        if (HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN) == KEY1_PRESSED)
        {
            // 3. 防重复触发：仅第一次按下时返回1
            if (key_flag == 0)
            {
                key_flag = 1;
                ret = 1;
            }
        }
    }
    else
    {
        // 按键松开，重置标志
        key_flag = 0;
    }
    return ret;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  // 初始化ADC滤波
  ADC_Filter_Init();
  
  // 显式初始化期望输出电流（双重保障）
  BUCK_I_CHARGE_REF = CURRENT_MIN;
  
  // 初始化BUCK控制结构体
  ControlData_Init(&buck_control_data);

  // 1. 启动TIM3（1ms系统滴答）
  HAL_TIM_Base_Start_IT(&htim3);

  // 2. ADC校准（提升采样精度）
  if (HAL_ADCEx_Calibration_Start(&hadc1,  ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  // 3. 启动ADC+DMA采样
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_sample_buffer, BUFFER_LENGTH) != HAL_OK)
  {
    Error_Handler();
  }

  // 启动TIM1和PWM输出，并设置初始50%占空比
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  uint16_t init_ccr = (uint16_t)(0.5f * (htim1.Init.Period + 1));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, init_ccr);
	
	
	
	//初始化gpioc1
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // LED翻转逻辑（验证程序运行）
    if (sys_tick_ms - last_led_tick >= LED_FLASH_INTERVAL)
    {
        last_led_tick = sys_tick_ms;
        led_state = !led_state;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    // KEY1按键处理逻辑（重构：更健壮的状态机）
if (KEY1_Scan() == 1) // 仅当按键真的按下时触发
{
    // 期望电流+0.1A
    BUCK_I_CHARGE_REF += CURRENT_STEP;
    
    // 修复：用整数化比较，彻底避免浮点精度问题
    // 把电流值放大10倍转成整数，再比较
    int current_scaled = (int)(BUCK_I_CHARGE_REF * 10 + 0.5f); // 四舍五入
    int max_scaled = (int)(CURRENT_MAX * 10);
    
    if (current_scaled > max_scaled)
    {
        BUCK_I_CHARGE_REF = CURRENT_MIN; // 重置为1A
    }
    

}

    // 串口打印数据（包含期望电流）
    printf("%.3f,%.3f,%.3f,%.3f,%.3f \r\n",adc_volt1,current1,filtered_current1, BUCK_I_CHARGE_REF,buck_duty);
    
   
		
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    // 错误时LED常亮
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
