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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float AD1_Value;
float AD0_Value;
float VOL_Value;
float CUR_Value;  // 去掉重复定义
float Input_V;
float Motor_V;
float Motor_I;
float W_Value;
ADC_ChannelConfTypeDef sConfig = {0};
uint16_t Get_ADC[80];
uint16_t ADValue;
float Voltage;
float W_Input_Value;
float W_Motor_Value;
float VOffset = 0.0f;
float IOffset = 0.0f;
int A = 0;
uint16_t Get_ADC1[2];
 float adc_vol1 =0;
 float adc_cur1 =0;

// 电压均值滤波参数
#define VOL_AVG_FILTER_LEN 40
float vol_avg_buf[VOL_AVG_FILTER_LEN] = {0};
int vol_avg_index = 0;

// 电流均值滤波参数
#define CURR_AVG_FILTER_LEN 20
float curr_avg_buf[CURR_AVG_FILTER_LEN] = {0};
int curr_avg_index = 0;

// 电压中值平均滤波参数
#define VOL_FILTER_LEN 40
#define VOL_USE_LEN 20
#define VOL_SKIP_LEN ((VOL_FILTER_LEN - VOL_USE_LEN) / 2)
float vol_buf[VOL_FILTER_LEN] = {0};
int vol_count = 0;

// 电流中值平均滤波参数
#define CURR_FILTER_LEN 40
#define CURR_USE_LEN 25
#define CURR_SKIP_LEN ((CURR_FILTER_LEN - CURR_USE_LEN) / 2)
float curr_buf[CURR_FILTER_LEN] = {0};
int curr_count = 0;

// 电压滑动窗口均值滤波（保持不变，简单高效）
float vol_avg_filter(float new_val) {
    vol_avg_buf[vol_avg_index] = new_val;
    vol_avg_index = (vol_avg_index + 1) % VOL_AVG_FILTER_LEN;
    float sum = 0;
    for (int i = 0; i < VOL_AVG_FILTER_LEN; i++) {
        sum += vol_avg_buf[i];
    }
    return sum / VOL_AVG_FILTER_LEN;
}

// 电流滑动窗口均值滤波（保持不变）
float curr_avg_filter(float new_val) {
    curr_avg_buf[curr_avg_index] = new_val;
    curr_avg_index = (curr_avg_index + 1) % CURR_AVG_FILTER_LEN;
    float sum = 0;
    for (int i = 0; i < CURR_AVG_FILTER_LEN; i++) {
        sum += curr_avg_buf[i];
    }
    return sum / CURR_AVG_FILTER_LEN;
}

// 选择排序（替代冒泡排序，交换次数更少，适合小样本）
static void select_sort(float arr[], int len) {
    for (int i = 0; i < len - 1; i++) {
        int min_idx = i;
        for (int j = i + 1; j < len; j++) {
            if (arr[j] < arr[min_idx]) {
                min_idx = j;  // 记录最小值索引
            }
        }
        // 只交换一次
        float temp = arr[i];
        arr[i] = arr[min_idx];
        arr[min_idx] = temp;
    }
}

// 电压中值平均滤波（用选择排序优化）
float vol_median_avg_filter(float new_val) {
    vol_buf[vol_count % VOL_FILTER_LEN] = new_val;
    vol_count++;
    if (vol_count < VOL_FILTER_LEN) {
        return new_val;
    }
    float temp[VOL_FILTER_LEN];
    for (int i = 0; i < VOL_FILTER_LEN; i++) {
        temp[i] = vol_buf[i];
    }
    select_sort(temp, VOL_FILTER_LEN);  // 替换为选择排序
    float sum = 0;
    for (int i = VOL_SKIP_LEN; i < VOL_SKIP_LEN + VOL_USE_LEN; i++) {
        sum += temp[i];
    }
    return sum / VOL_USE_LEN;
}

// 电流中值平均滤波（用选择排序优化）
float curr_median_avg_filter(float new_val) {
    curr_buf[curr_count % CURR_FILTER_LEN] = new_val;
    curr_count++;
    if (curr_count < CURR_FILTER_LEN) {
        return new_val;
    }
    float temp[CURR_FILTER_LEN];
    for (int i = 0; i < CURR_FILTER_LEN; i++) {
        temp[i] = curr_buf[i];
    }
    select_sort(temp, CURR_FILTER_LEN);  // 替换为选择排序
    float sum = 0;
    for (int i = CURR_SKIP_LEN; i < CURR_SKIP_LEN + CURR_USE_LEN; i++) {
        sum += temp[i];
    }
    return sum / CURR_USE_LEN;
}

// -------------------------- 零点校准滤波专属参数 --------------------------
// 电压零点滤波：滑动窗口长度（可根据实际噪声调整）
#define V_ZERO_AVG_FILTER_LEN 40
// 电压零点滤波缓冲区（存储历史零点样本）
float v_zero_avg_buf[V_ZERO_AVG_FILTER_LEN] = {0};
// 电压零点滤波缓冲区索引（循环更新）
int v_zero_avg_index = 0;

// 电流零点滤波：滑动窗口长度（可独立调整）
#define I_ZERO_AVG_FILTER_LEN 30
// 电流零点滤波缓冲区（存储历史零点样本）
float i_zero_avg_buf[I_ZERO_AVG_FILTER_LEN] = {0};
// 电流零点滤波缓冲区索引（循环更新）
int i_zero_avg_index = 0;
// --------------------------------------------------------------------------
// 电压零点校准均值滤波函数（滑动窗口平均）
// 功能：对零点校准过程中的电压值进行平滑处理，输出稳定的零点参考值
float V_zero_calib_avg_filter(float new_zero_val) {
    // 1. 新零点样本存入缓冲区（环形覆盖旧数据）
    v_zero_avg_buf[v_zero_avg_index] = new_zero_val;
    // 2. 循环更新索引（超出窗口长度后从头开始）
    v_zero_avg_index = (v_zero_avg_index + 1) % V_ZERO_AVG_FILTER_LEN;
    
    // 3. 计算窗口内所有零点样本的平均值
    float zero_vol_sum = 0.0f;
    for (int i = 0; i < V_ZERO_AVG_FILTER_LEN; i++) {
	
        zero_vol_sum += v_zero_avg_buf[i];
    }
    return zero_vol_sum / V_ZERO_AVG_FILTER_LEN;
}

// 电流零点校准均值滤波函数（滑动窗口平均）
// 功能：对零点校准过程中的电流值进行平滑处理，输出稳定的零点参考值
float I_zero_calib_avg_filter(float new_zero_val) {
    // 1. 新零点样本存入缓冲区（环形覆盖旧数据）
    i_zero_avg_buf[i_zero_avg_index] = new_zero_val;
    // 2. 循环更新索引（超出窗口长度后从头开始）
    i_zero_avg_index = (i_zero_avg_index + 1) % I_ZERO_AVG_FILTER_LEN;
    
    // 3. 计算窗口内所有零点样本的平均值
    float zero_cur_sum = 0.0f;
    for (int i = 0; i < I_ZERO_AVG_FILTER_LEN; i++) {
        zero_cur_sum += i_zero_avg_buf[i];
    }
    return zero_cur_sum / I_ZERO_AVG_FILTER_LEN;
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1);
//	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&Get_ADC,80);
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&Get_ADC1,2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE BEGIN 3 */
			A++;
		// 引脚改为GPIOA和GPIO_PIN_2，其余逻辑保持不变（亮15循环，灭15循环）
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, (A % 30) < 15 ? GPIO_PIN_SET : GPIO_PIN_RESET);
		if (A >= 30) A = 0;

    // ADC数据累加（用预计算常数减少运算）
    AD0_Value = 0;  // 提前清零，避免累加残留
    AD1_Value = 0;
		
		 float adc_vol1 = Get_ADC1[0] * (3.3f / 4095.0f); 
		 float adc_cur1 = Get_ADC1[1] * (3.3f / 4095.0f);
  
		for(int i = 0; i < 80; i += 2) {
        // 电压计算：合并常数，减少浮点运算步骤
        float adc_vol = Get_ADC[i] * (3.3f / 4095.0f);  // 先算ADC对应的电压
        AD0_Value += (adc_vol - 2.43717f) * 20.43f;
        
        // 电流计算：同理优化
        float adc_cur = Get_ADC[i+1] * (3.3f / 4095.0f) ;
        AD1_Value += (adc_cur - 2.436) * (1.0f / 2.6f);
   }

    // 原始平均值计算
    float raw_vol = AD0_Value / 40.0f;  // 80个样本，每2个一组，共40组
    float raw_cur = AD1_Value / 40.0f;

    // 中值平均滤波
    VOL_Value = vol_median_avg_filter(raw_vol);
    CUR_Value = curr_median_avg_filter(raw_cur);

    // 零点校准（优化：标定时取10次平均，而非滑动窗口）
    if  (VOL_Value > -0.04f && VOL_Value < 0.04f) {
				VOffset=V_zero_calib_avg_filter(VOL_Value);

			 }
//			     if( VOL_Value > -0.0003f && VOL_Value < 0.0003f) {
//				IOffset=I_zero_calib_avg_filter(CUR_Value);
//			 }

    // 计算实际值（简化条件判断）
			Motor_V = (VOL_Value - VOffset);
			 Motor_I=CUR_Value;
//    Motor_I = CUR_Value - IOffset;
//		float I_Err=0.0f;
//		
//		if(Motor_I<-0.120){		
//		I_Err=Motor_I+0.120;
//		Motor_I+=I_Err*1.5;

//		}
    // 功率计算
    W_Motor_Value = Motor_V * Motor_I;
			 
//		float T_I=Motor_I+Motor_V*0.01;
// 输出数据（保持原格式）
//    printf("simples:%f,%f,%f,%f,%f\n", Motor_V, Motor_I, W_Motor_Value, Input_V, W_Input_Value);
//			 printf("%f,%f,%f,%f\n\r", Motor_V,Motor_I ,W_Motor_Value,raw_cur );         // A
			 	 printf("%d,%d,%f,%f\n\r",   Get_ADC1[0], Get_ADC1[1], adc_vol1,adc_cur1);         // A
  }
}
  /* USER CODE END 3 */

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV3;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
