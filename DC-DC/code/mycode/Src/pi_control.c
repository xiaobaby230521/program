#include "pi_control.h"

/************************ 初始化期望值定义 ************************/
float BUCK_V_PROTECT_MAX = 18.5f; // 过压保护初始18.5V
float BOOST_UO_REF=30.0f;     // BOOST输出参考电压(V)

/************************ 初始化控制结构体 ************************/
void ControlData_Init(ControlData *pdata)
{
    if(pdata == NULL) return;
    
    // BUCK充电变量清零
    pdata->buck_e_i_prev = 0.0f;
    pdata->buck_u_i_prev = 0.0f;
    pdata->buck_x_lf_prev = 0.0f;
    pdata->buck_y_lf_prev = 0.0f;
    pdata->buck_x_lag_prev = 0.0f;
    pdata->buck_y_lag_prev = 0.0f;
    pdata->buck_duty = 0.0f;
    // 新增：前馈补偿历史值清零
    pdata->buck_d_ff_prev = 0.0f;
    pdata->buck_i_o_prev = 0.0f;

    // BOOST变量清零
    pdata->boost_x_lf_prev = 0.0f;
    pdata->boost_y_lf_prev = 0.0f;
    pdata->boost_e_i_prev = 0.0f;
    pdata->boost_u_i_prev = 0.0f;
    pdata->boost_e_v_prev = 0.0f;
    pdata->boost_u_v_prev = 0.0f;
    pdata->boost_x_lead_prev = 0.0f;
    pdata->boost_y_lead_prev = 0.0f;
    pdata->boost_i_ref = 0.0f;
    pdata->boost_duty = 0.0f;
}

/************************ BUCK恒流充电核心函数 ************************/
// BUCK电流环低通滤波器（匹配仿真差分方程）
float BUCK_Current_LowPassFilter(ControlData *pdata, float x)
{
    if(pdata == NULL) return 0.0f;
    
    // 计算离散系数（Tustin变换，匹配仿真）
    float omega_lf = 2 * BUCK_PI * BUCK_F_LF;
    float k_lf = omega_lf * BUCK_T_SAMPLE;
    float b0 = k_lf / (2 + k_lf);
    float b1 = k_lf / (2 + k_lf);
    float a1 = (k_lf - 2) / (2 + k_lf);
    
    // 差分方程计算
    float y = b0 * x + b1 * pdata->buck_x_lf_prev - a1 * pdata->buck_y_lf_prev;
    
    // 更新历史值
    pdata->buck_x_lf_prev = x;
    pdata->buck_y_lf_prev = y;
    
    return y;
}

// BUCK电流环相位延迟环节（匹配仿真差分方程）
float BUCK_Current_PhaseLag(ControlData *pdata, float x)
{
    if(pdata == NULL) return 0.0f;
    
    // 计算离散系数（Tustin变换，匹配仿真）
    float tau = BUCK_TAU_LAG;
    float alpha = BUCK_ALPHA_LAG;
    float b0 = (BUCK_T_SAMPLE + 2 * tau) / (BUCK_T_SAMPLE + 2 * alpha * tau);
    float b1 = (BUCK_T_SAMPLE - 2 * tau) / (BUCK_T_SAMPLE + 2 * alpha * tau);
    float a1 = (BUCK_T_SAMPLE - 2 * alpha * tau) / (BUCK_T_SAMPLE + 2 * alpha * tau);
    
    // 差分方程计算
    float y = b0 * x + b1 * pdata->buck_x_lag_prev - a1 * pdata->buck_y_lag_prev;
    
    // 更新历史值
    pdata->buck_x_lag_prev = x;
    pdata->buck_y_lag_prev = y;
    
    return y;
}

// BUCK电流环PI控制器（最终调试系数，低超调）
float BUCK_Current_PIControl(ControlData *pdata, float e_i)
{
    if(pdata == NULL) return 0.0f;
    
    // 计算PI离散系数（Tustin变换，匹配最终调试值）
    float a0 = BUCK_KP_I + (BUCK_KI_I * BUCK_T_SAMPLE) / 2;
    float a1 = (BUCK_KI_I * BUCK_T_SAMPLE) / 2 - BUCK_KP_I;
    
    // 差分方程计算
    float u_i = a0 * e_i + a1 * pdata->buck_e_i_prev + pdata->buck_u_i_prev;
    
    // 限幅（防止积分饱和，占空比0~1）
    if(u_i > 1.0f) u_i = 1.0f;
    if(u_i < 0.0f) u_i = 0.0f;
    
    // 更新历史值
    pdata->buck_e_i_prev = e_i;
    pdata->buck_u_i_prev = u_i;
    
    return u_i;
}

// 修正后：BUCK前馈补偿计算函数（基于ΔI = 参考值 - 实际采样值）
float BUCK_Current_FeedForward(ControlData *pdata, float i_o_sample)
{
    if(pdata == NULL) return 0.0f;
    
    // 1. 计算前馈核心增益K_ff
    float K_ff = BUCK_T_SAMPLE / (2 * BUCK_C * BUCK_UIN);
    // 2. 乘以衰减系数，防止补偿过度（可调试）
    K_ff *= BUCK_FF_ATTENUATION;
    
    // 3. 核心修正：计算扰动电流ΔI（参考值 - 实际采样值）
    float delta_i_o = BUCK_I_CHARGE_REF - i_o_sample;       // 当前周期扰动
    float delta_i_o_prev = BUCK_I_CHARGE_REF - pdata->buck_i_o_prev; // 上一周期扰动
    
    // 4. 离散差分方程计算前馈补偿值（使用扰动电流，而非原始采样值）
    float D_ff = pdata->buck_d_ff_prev - K_ff * (delta_i_o + delta_i_o_prev);
    
    // 5. 前馈补偿值限幅（避免补偿值过大）
    if(D_ff > 0.3f) D_ff = 0.3f;
    if(D_ff < 0.0f) D_ff = 0.0f;
    
    // 6. 更新前馈历史值（仍保存实际负载电流采样值，用于下一周期计算ΔI_prev）
    pdata->buck_d_ff_prev = D_ff;
    pdata->buck_i_o_prev = i_o_sample;
    
    return D_ff;
}

// BUCK恒流充电总控制函数（核心：1A恒流+18.5V过压保护+前馈补偿）
float BUCK_Charge_Control(ControlData *pdata, float v_sample, float i_sample, float i_o_sample)
{
    if(pdata == NULL) return 0.0f;
    
    // --------------------- 第一步：过压保护判断（最高优先级） ---------------------
    if(v_sample >= BUCK_V_PROTECT_MAX)
    {
        pdata->buck_duty = 0.0f;  // 电压达到18.5V，占空比置0，停止充电
        // 保护时重置前馈历史值
        pdata->buck_d_ff_prev = 0.0f;
        pdata->buck_i_o_prev = 0.0f;
        return pdata->buck_duty;
    }
    
    // --------------------- 第二步：电流环反馈控制（原有PI逻辑） ---------------------
    // 1. 电流采样值低通滤波（滤除噪声）
    float i_sample_filtered = BUCK_Current_LowPassFilter(pdata, i_sample);
    
    // 2. 计算电流环误差：参考值(1A) - 滤波后采样电流
    float e_i = BUCK_I_CHARGE_REF - i_sample_filtered;
    
    // 3. 电流环PI控制（低超调系数）
    float u_i = BUCK_Current_PIControl(pdata, e_i);
		
		    // ===== 新增：PI输出限幅 =====
    if(u_i > 0.9f) u_i = 0.9f;  // PI输出上限90%
    if(u_i < 0.1f) u_i = 0.1f;  // PI输出下限10%
    
    // 4. 相位延迟补偿（保证稳定性）
    u_i = BUCK_Current_PhaseLag(pdata, u_i);
    
    // --------------------- 第三步：新增前馈补偿（抵消负载电流扰动） ---------------------
    float D_ff = BUCK_Current_FeedForward(pdata, i_o_sample);
    
    // --------------------- 第四步：总占空比 = PI反馈 + 前馈补偿 ---------------------
    float D_total = u_i + D_ff;
    
    // --------------------- 第五步：占空比最终限幅 ---------------------
    pdata->buck_duty = D_total;
    if(pdata->buck_duty > 0.9f) pdata->buck_duty = 0.9f;
    if(pdata->buck_duty < 0.1f) pdata->buck_duty = 0.1f;
    
    return pdata->buck_duty;
}

/************************ BOOST相关函数（保留原代码，不修改） ************************/
float BOOST_Current_LowPassFilter(ControlData *pdata, float x)
{
    if(pdata == NULL) return 0.0f;
    
    float omega_lf = 2 * BOOST_PI * BOOST_FC_I;
    float k_lf = omega_lf * BOOST_T_SAMPLE;
    float b0 = k_lf / (2 + k_lf);
    float b1 = k_lf / (2 + k_lf);
    float a1 = (k_lf - 2) / (2 + k_lf);
    
    float y = b0 * x + b1 * pdata->boost_x_lf_prev - a1 * pdata->boost_y_lf_prev;
    
    pdata->boost_x_lf_prev = x;
    pdata->boost_y_lf_prev = y;
    
    return y;
}

float BOOST_Current_Control(ControlData *pdata, float e_i)
{
    if(pdata == NULL) return 0.0f;
    
    float omega_lf = 2 * BOOST_PI * BOOST_FC_I;
    float k_lf = omega_lf * BOOST_T_SAMPLE;
    float b0_lf = k_lf / (2 + k_lf);
    float b1_lf = k_lf / (2 + k_lf);
    float a1_lf = (k_lf - 2) / (2 + k_lf);
    
    float b0 = BOOST_KP_I * b0_lf;
    float b1 = BOOST_KP_I * b1_lf;
    float a1 = a1_lf;
    
    float u_i = b0 * e_i + b1 * pdata->boost_e_i_prev - a1 * pdata->boost_u_i_prev;
    
    if(u_i > 1.0f) u_i = 1.0f;
    if(u_i < 0.0f) u_i = 0.0f;
    
    pdata->boost_e_i_prev = e_i;
    pdata->boost_u_i_prev = u_i;
    
    return u_i;
}

float BOOST_Voltage_PIControl(ControlData *pdata, float e_v)
{
    if(pdata == NULL) return 0.0f;
    
    float a0 = BOOST_KP_V + (BOOST_KI_V * BOOST_T_SAMPLE) / 2;
    float a1 = (BOOST_KI_V * BOOST_T_SAMPLE) / 2 - BOOST_KP_V;
    
    float u_v = a0 * e_v + a1 * pdata->boost_e_v_prev + pdata->boost_u_v_prev;
    
    if(u_v > 1.0f) u_v = 1.0f;
    if(u_v < 0.0f) u_v = 0.0f;
    
    pdata->boost_e_v_prev = e_v;
    pdata->boost_u_v_prev = u_v;
    
    return u_v;
}

float BOOST_Voltage_PhaseLead(ControlData *pdata, float x)
{
    if(pdata == NULL) return 0.0f;
    
    float tau = 1/(2 * BOOST_PI * BOOST_FC_LEAD * BOOST_ALPHA_LEAD);
    float alpha_tau = 1/(2 * BOOST_PI * BOOST_FC_LEAD);
    float b0 = (BOOST_T_SAMPLE + 2 * tau) / (BOOST_T_SAMPLE + 2 * alpha_tau);
    float b1 = (BOOST_T_SAMPLE - 2 * tau) / (BOOST_T_SAMPLE + 2 * alpha_tau);
    float a1 = (BOOST_T_SAMPLE - 2 * alpha_tau) / (BOOST_T_SAMPLE + 2 * alpha_tau);
    
    float y = b0 * x + b1 * pdata->boost_x_lead_prev - a1 * pdata->boost_y_lead_prev;
    
    pdata->boost_x_lead_prev = x;
    pdata->boost_y_lead_prev = y;
    
    return y;
}

float BOOST_DoubleLoop_Control(ControlData *pdata, float v_sample, float i_sample)
{
    if(pdata == NULL) return 0.0f;
    
    float e_v = BOOST_UO_REF - v_sample;
    float u_v = BOOST_Voltage_PIControl(pdata, e_v);
    u_v = BOOST_Voltage_PhaseLead(pdata, u_v);
    u_v *= BOOST_FEEDFORWARD_GAIN;
    pdata->boost_i_ref = u_v;
    
    float i_sample_filtered = BOOST_Current_LowPassFilter(pdata, i_sample);
    float e_i = pdata->boost_i_ref - i_sample_filtered;
    float u_i = BOOST_Current_Control(pdata, e_i);
    
    pdata->boost_duty = u_i;
    
    if(pdata->boost_duty > 1.0f) pdata->boost_duty = 1.0f;
    if(pdata->boost_duty < 0.0f) pdata->boost_duty = 0.0f;
    
    return pdata->boost_duty;
}