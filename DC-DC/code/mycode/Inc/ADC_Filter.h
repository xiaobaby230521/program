#ifndef ADC_FILTER_H
#define ADC_FILTER_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/**
 * @brief 滤波缓存长度宏定义
 * @note 针对DC-DC低突变场景，设置为100组，兼顾平滑性和实时性
 */
#define FILTER_BUF_LEN 100

/**
 * @brief 全局变量声明（区分四路采样：两路电流+两路电压）
 * @note 仅保留必需的核心变量，减少内存占用
 */
// ADC原始数据缓存（DMA搬运的4通道原始值，必须保留）
extern volatile uint16_t adc_ch_buf[4];

// ---------------------- 第一路电流检测（3.3V） ----------------------
extern volatile float adc_volt1;        // 第一路电压值
extern volatile float current1;         // 第一路实时电流值
extern volatile float filtered_current1;// 第一路滤波后电流值
extern volatile float current_buf1[FILTER_BUF_LEN]; // 第一路滤波缓存
extern volatile uint16_t buf_count1;    // 第一路缓存计数
extern volatile uint16_t buf_index1;    // 第一路写入索引

// ---------------------- 第二路电流检测 ----------------------
extern volatile float adc_volt2;        // 第二路电压值
extern volatile float current2;         // 第二路实时电流值
extern volatile float filtered_current2;// 第二路滤波后电流值
extern volatile float current_buf2[FILTER_BUF_LEN]; // 第二路滤波缓存
extern volatile uint16_t buf_count2;    // 第二路缓存计数
extern volatile uint16_t buf_index2;    // 第二路写入索引

// ---------------------- 第三路电压检测（红色模块） ----------------------
extern volatile float adc_volt3;        // 第三路原始电压值
extern volatile float delta_u3;         // 第三路△u（转换公式：△u = adc_volt3 / 0.02168）
extern volatile float filtered_delta_u3;// 第三路滤波后△u值
extern volatile float volt_buf3[FILTER_BUF_LEN];    // 第三路电压滤波缓存
extern volatile uint16_t buf_count3;    // 第三路缓存计数
extern volatile uint16_t buf_index3;    // 第三路写入索引

// ---------------------- 第四路电压检测（同第三路系数） ----------------------
extern volatile float adc_volt4;        // 第四路原始电压值
extern volatile float delta_u4;         // 第四路△u（转换公式：△u = adc_volt4 / 0.02168）
extern volatile float filtered_delta_u4;// 第四路滤波后△u值
extern volatile float volt_buf4[FILTER_BUF_LEN];    // 第四路电压滤波缓存
extern volatile uint16_t buf_count4;    // 第四路缓存计数
extern volatile uint16_t buf_index4;    // 第四路写入索引

/**
 * @brief 函数声明（区分四路）
 */
// 滤波模块初始化（初始化四路）
void ADC_Filter_Init(void);

// 第一路滤波函数
float smooth_sum_filter1(float real_time_value);
float ADC_Calculate_Current1(float adc_avg);

// 第二路滤波函数
float smooth_sum_filter2(float real_time_value);
float ADC_Calculate_Current2(float adc_avg);

// 第三路电压检测滤波函数
float smooth_sum_filter3(float real_time_value);
void ADC_Calculate_Voltage3(void);

// 第四路电压检测滤波函数
float smooth_sum_filter4(float real_time_value);
void ADC_Calculate_Voltage4(void);

#endif