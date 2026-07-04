#include "ADC_Filter.h"
#include "main.h"

/**
 * @brief 核心全局变量定义（区分四路采样：两路电流+两路电压）
 * @note 仅保留必需变量，删除冗余的电压缓存/采样缓存
 */
// ADC原始数据缓存（DMA搬运的4通道原始值，必须保留）
volatile uint16_t adc_ch_buf[4] = {0};

// ---------------------- 第一路电流检测（3.3V） ----------------------
volatile float adc_volt1 = 0.0f;        // 第一路电压值（3.3V检测）
volatile float current1 = 0.0f;         // 第一路实时电流值
volatile float filtered_current1 = 0.0f;// 第一路滤波后电流值
volatile float current_buf1[FILTER_BUF_LEN] = {0.0f}; // 第一路滤波缓存
volatile uint16_t buf_count1 = 0;       // 第一路缓存计数
volatile uint16_t buf_index1 = 0;       // 第一路写入索引

// ---------------------- 第二路电流检测 ----------------------
volatile float adc_volt2 = 0.0f;        // 第二路电压值
volatile float current2 = 0.0f;         // 第二路实时电流值
volatile float filtered_current2 = 0.0f;// 第二路滤波后电流值
volatile float current_buf2[FILTER_BUF_LEN] = {0.0f}; // 第二路滤波缓存
volatile uint16_t buf_count2 = 0;       // 第二路缓存计数
volatile uint16_t buf_index2 = 0;       // 第二路写入索引

// ---------------------- 第三路电压检测（红色模块） ----------------------
volatile float adc_volt3 = 0.0f;        // 第三路原始电压值
volatile float delta_u3 = 0.0f;         // 第三路△u（转换公式：△u = adc_volt3 / 0.02168）
volatile float filtered_delta_u3 = 0.0f;// 第三路滤波后△u值
volatile float volt_buf3[FILTER_BUF_LEN] = {0.0f};    // 第三路电压滤波缓存
volatile uint16_t buf_count3 = 0;       // 第三路缓存计数
volatile uint16_t buf_index3 = 0;       // 第三路写入索引

// ---------------------- 第四路电压检测（同第三路系数） ----------------------
volatile float adc_volt4 = 0.0f;        // 第四路原始电压值
volatile float delta_u4 = 0.0f;         // 第四路△u（转换公式：△u = adc_volt4 / 0.02168）
volatile float filtered_delta_u4 = 0.0f;// 第四路滤波后△u值
volatile float volt_buf4[FILTER_BUF_LEN] = {0.0f};    // 第四路电压滤波缓存
volatile uint16_t buf_count4 = 0;       // 第四路缓存计数
volatile uint16_t buf_index4 = 0;       // 第四路写入索引

/**
 * @brief 滤波模块初始化函数（初始化四路）
 * @note 初始化缓存和索引，避免随机值影响滤波效果
 */
void ADC_Filter_Init(void)
{
    // 初始化第一路滤波缓存
    for (uint16_t i = 0; i < FILTER_BUF_LEN; i++)
    {
        current_buf1[i] = 0.0f;
    }
    buf_count1 = 0;
    buf_index1 = 0;
    current1 = 0.0f;
    filtered_current1 = 0.0f;
    adc_volt1 = 0.0f;

    // 初始化第二路滤波缓存
    for (uint16_t i = 0; i < FILTER_BUF_LEN; i++)
    {
        current_buf2[i] = 0.0f;
    }
    buf_count2 = 0;
    buf_index2 = 0;
    current2 = 0.0f;
    filtered_current2 = 0.0f;
    adc_volt2 = 0.0f;

    // 初始化第三路电压滤波缓存
    for (uint16_t i = 0; i < FILTER_BUF_LEN; i++)
    {
        volt_buf3[i] = 0.0f;
    }
    buf_count3 = 0;
    buf_index3 = 0;
    delta_u3 = 0.0f;
    filtered_delta_u3 = 0.0f;
    adc_volt3 = 0.0f;

    // 初始化第四路电压滤波缓存
    for (uint16_t i = 0; i < FILTER_BUF_LEN; i++)
    {
        volt_buf4[i] = 0.0f;
    }
    buf_count4 = 0;
    buf_index4 = 0;
    delta_u4 = 0.0f;
    filtered_delta_u4 = 0.0f;
    adc_volt4 = 0.0f;
}

/**
 * @brief 第一路平滑求和滤波函数（核心算法）
 * @param real_time_value 第一路实时电流值（中断计算的原始值）
 * @return float 滤波后的值（未满用实时值，满了用100组平均值）
 * @note 针对DC-DC低突变场景，全量求和滤波波形更平滑
 */
float smooth_sum_filter1(float real_time_value)
{
    current_buf1[buf_index1] = real_time_value;
    buf_index1 = (buf_index1 + 1) % FILTER_BUF_LEN;
    if (buf_count1 < FILTER_BUF_LEN) buf_count1++;

    if (buf_count1 < FILTER_BUF_LEN) return real_time_value;
    else
    {
        float sum = 0.0f;
        for (uint16_t i = 0; i < FILTER_BUF_LEN; i++) sum += current_buf1[i];
        return sum / FILTER_BUF_LEN;
    }
}

/**
 * @brief 第二路平滑求和滤波函数（核心算法）
 * @param real_time_value 第二路实时电流值（中断计算的原始值）
 * @return float 滤波后的值（未满用实时值，满了用100组平均值）
 */
float smooth_sum_filter2(float real_time_value)
{
    current_buf2[buf_index2] = real_time_value;
    buf_index2 = (buf_index2 + 1) % FILTER_BUF_LEN;
    if (buf_count2 < FILTER_BUF_LEN) buf_count2++;

    if (buf_count2 < FILTER_BUF_LEN) return real_time_value;
    else
    {
        float sum = 0.0f;
        for (uint16_t i = 0; i < FILTER_BUF_LEN; i++) sum += current_buf2[i];
        return sum / FILTER_BUF_LEN;
    }
}

/**
 * @brief 第三路电压平滑求和滤波函数（核心算法）
 * @param real_time_value 第三路实时△u值（中断计算的原始值）
 * @return float 滤波后的值（未满用实时值，满了用100组平均值）
 */
float smooth_sum_filter3(float real_time_value)
{
    volt_buf3[buf_index3] = real_time_value;
    buf_index3 = (buf_index3 + 1) % FILTER_BUF_LEN;
    if (buf_count3 < FILTER_BUF_LEN) buf_count3++;

    if (buf_count3 < FILTER_BUF_LEN) return real_time_value;
    else
    {
        float sum = 0.0f;
        for (uint16_t i = 0; i < FILTER_BUF_LEN; i++) sum += volt_buf3[i];
        return sum / FILTER_BUF_LEN;
    }
}

/**
 * @brief 第四路电压平滑求和滤波函数（核心算法）
 * @param real_time_value 第四路实时△u值（中断计算的原始值）
 * @return float 滤波后的值（未满用实时值，满了用100组平均值）
 */
float smooth_sum_filter4(float real_time_value)
{
    volt_buf4[buf_index4] = real_time_value;
    buf_index4 = (buf_index4 + 1) % FILTER_BUF_LEN;
    if (buf_count4 < FILTER_BUF_LEN) buf_count4++;

    if (buf_count4 < FILTER_BUF_LEN) return real_time_value;
    else
    {
        float sum = 0.0f;
        for (uint16_t i = 0; i < FILTER_BUF_LEN; i++) sum += volt_buf4[i];
        return sum / FILTER_BUF_LEN;
    }
}

/**
 * @brief 从ADC原始值计算第一路实时电流
 * @note 公式：电流 = (采样电压 - 中点电压) / 电流系数（0.254）
 */
float ADC_Calculate_Current1(float adc_avg)
{
  
    
    // 电压转换：ADC值 → 实际电压（3.3V参考，12位ADC）
    adc_volt1 = adc_avg * 3.3f / 4095.0f;
    // 电流计算：扣除偏置电压，除以电流采样系数
    current1 = (adc_volt1 - (3.33f / 2)) / 0.254f;
    
    return current1; // 返回计算结果
}

/**
 * @brief 从ADC原始值计算第二路实时电流
 * @note 公式：电流 = (采样电压 - 5V中点) / 电流系数（0.0386）
 */
float ADC_Calculate_Current2(float adc_avg)
{
   
    // 电压转换：ADC值 → 实际电压（3.3V参考，12位ADC）
    adc_volt2 = adc_avg * 3.3f / 4095.0f;
    // 电流计算：扣除偏置电压，除以电流采样系数
    current2 = (adc_volt2 - (5.0f / 2)) / 0.0386f;
    
    return current2; // 返回计算结果
}

/**
 * @brief 从ADC原始值计算第三路△u（电压检测-红色模块）
 * @note 转换公式：△u = adc_volt3 / 0.02168
 *       ADC通道：默认使用通道2（adc_ch_buf[2]）
 */
void ADC_Calculate_Voltage3(void)
{
    // 步骤1：ADC原始值转电压（3.3V参考，12位ADC）
    adc_volt3 = adc_ch_buf[2] * 3.3f / 4095.0f;
    
    // 步骤2：计算△u（核心公式：△u = 原始电压 / 0.02168）
    delta_u3 = adc_volt3 / 0.02168f;
}

/**
 * @brief 从ADC原始值计算第四路△u（电压检测，同第三路系数）
 * @note 转换公式：△u = adc_volt4 / 0.02168
 *       ADC通道：默认使用通道3（adc_ch_buf[3]）
 */
void ADC_Calculate_Voltage4(void)
{
    // 步骤1：ADC原始值转电压（3.3V参考，12位ADC）
    adc_volt4 = adc_ch_buf[3] * 3.3f / 4095.0f;
    
    // 步骤2：计算△u（与第三路相同系数：0.02168）
    delta_u4 = adc_volt4 / 0.02164f;
}