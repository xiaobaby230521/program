/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "comp.h"
#include "cordic.h"
#include "dac.h"
#include "dma.h"
#include "hrtim.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "My_Digital_Control_LL.h"
#include "My_Cordic_LL.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
Type2_TypeDef LLC_CVPamer;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ADC1_samples[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
  */
  LL_PWR_DisableUCPDDeadBattery();

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
  MX_HRTIM1_Init();
  MX_COMP2_Init();
  MX_COMP3_Init();
  MX_DAC1_Init();
  MX_DAC2_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_DAC3_Init();
  MX_CORDIC_Init();
  /* USER CODE BEGIN 2 */
    //配置Cordic
  
  LL_CORDIC_Config(CORDIC, CORDIC_FUNCTION_COSINE, CORDIC_PRECISION_6CYCLES, CORDIC_SCALE_0, 
						CORDIC_NBWRITE_1,CORDIC_NBREAD_2, CORDIC_INSIZE_32BITS, CORDIC_OUTSIZE_32BITS);

  //DMA和ADC的初始化
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 2);
  LL_DMA_SetPeriphAddress(DMA1,LL_DMA_CHANNEL_1,LL_ADC_DMA_GetRegAddr(ADC1,LL_ADC_DMA_REG_REGULAR_DATA));
  LL_DMA_SetMemoryAddress(DMA1,LL_DMA_CHANNEL_1,(uint32_t)ADC1_samples);   //用DMA传输ADC采样数据   这样不占用系统总线 就是牛马搬运工来的
  LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_1);
  
  LL_ADC_DisableDeepPowerDown(ADC1); 
  LL_ADC_EnableInternalRegulator(ADC1);
  LL_mDelay(1);
  LL_ADC_StartCalibration(ADC1,LL_ADC_SINGLE_ENDED);
  LL_mDelay(1);
  LL_ADC_Enable(ADC1);
  LL_mDelay(1);
  LL_ADC_REG_SetDMATransfer(ADC1,LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
  LL_ADC_REG_StartConversion(ADC1);

  LL_DAC_Enable(DAC3, LL_DAC_CHANNEL_1);
  LL_DAC_ConvertData12RightAligned(DAC3, LL_DAC_CHANNEL_1, 2048);    //用于给过零比较器做参考电压
  
  LL_DAC_Enable(DAC3, LL_DAC_CHANNEL_2);
  LL_DAC_ConvertData12RightAligned(DAC3, LL_DAC_CHANNEL_2, 2048);    //用于给过零比较器做参考电压
  
  LL_DAC_SetWaveSawtoothStepData(DAC3, LL_DAC_CHANNEL_1, 0);          //可用于使DAC产生斜坡   现在不需要
  LL_DAC_SetWaveSawtoothStepData(DAC3, LL_DAC_CHANNEL_2,0);           //可用于使DAC产生斜坡   现在不需要
  
  LL_DAC_SetWaveSawtoothResetData(DAC3, LL_DAC_CHANNEL_1, 2048);
  LL_DAC_SetWaveSawtoothResetData(DAC3, LL_DAC_CHANNEL_2, 2048);
  
//  LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);                             //加油 这些没用的DAC  我关了哈
//  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, 2048);
//  
//  LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_2);
//  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_2, 2048);
//  
//  LL_DAC_SetWaveSawtoothStepData(DAC1, LL_DAC_CHANNEL_1, 0);
//  LL_DAC_SetWaveSawtoothStepData(DAC1, LL_DAC_CHANNEL_2,0);
  
  
  LL_COMP_Enable(COMP2);                  //两个过零比较器使能
  LL_COMP_Enable(COMP3);

  LL_mDelay(100);
  FLASH->ACR |= (1<<9);//将第9位置1
  FLASH->ACR |= (1<<10);//将第10位置1

  LL_HRTIM_TIM_CounterEnable(HRTIM1, LL_HRTIM_TIMER_A|LL_HRTIM_TIMER_E|LL_HRTIM_TIMER_F);  //开启定时器
  LL_HRTIM_EnableIT_REP(HRTIM1, LL_HRTIM_TIMER_F);                          //开启定时器F中断
  LL_HRTIM_EnableOutput(HRTIM1, LL_HRTIM_OUTPUT_TE2);                       //无用  别理会
//  float LLC_CV_Afilter[2] = {1.4795635f, -0.47956353f};                     //电压环系数
//  float LLC_CV_Bfilter[3] = {0.06530368f, 0.003216941f, -0.062086739f};
  float LLC_CV_Afilter[2] = {0.69325157f, 0.30674843f};
  float LLC_CV_Bfilter[3] = {0.081853334f, 0.0068968557f, -0.074956478f};
  f32_Type2_Init(&LLC_CVPamer, LLC_CV_Afilter, LLC_CV_Bfilter, 1, 0.5f, 0.05f);  //4.2k
//  LL_HRTIM_EnableOutput(HRTIM1, LL_HRTIM_OUTPUT_TA1|LL_HRTIM_OUTPUT_TA2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_12);   //LL库控制器GPIO的函数   LED灯
	  LL_mDelay(500);
	  LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_12);
	  LL_mDelay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSE_Enable();
   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
  }

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 85, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1us transition state at intermediate medium speed clock*/
  for (__IO uint32_t i = (170 >> 1); i !=0; i--);

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_Init1msTick(170000000);

  LL_SetSystemCoreClock(170000000);
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
