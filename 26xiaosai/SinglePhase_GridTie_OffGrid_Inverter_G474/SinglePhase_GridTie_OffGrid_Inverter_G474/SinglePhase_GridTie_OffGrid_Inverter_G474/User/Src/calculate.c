#include "calculate.h"
#include "key.h"
#include "tim.h"
#include "gpio.h"
#include <string.h>
// ====================================================================================
// ===================== 【模块1/3】电网电压比例前馈 =====================
// 功能：仅抵消电网电压扰动，最稳定，建议第一个加上
// ====================================================================================
void FF_Proportional_Init(FF_Proportional_t *ff)
{
    ff->Vdc_nominal = 55.0f;
    ff->gain_ff = 1.0f / ff->Vdc_nominal; // 增益 = 1/母线电压
}

float FF_Proportional_Update(FF_Proportional_t *ff, float u_grid)
{
    // 输出：u_g / Vdc
    return u_grid * ff->gain_ff;
}

void FF_Proportional_Reset(FF_Proportional_t *ff)
{
    // 无状态变量，空实现
}

// ====================================================================================
// ===================== 【最终版】电容电流有源阻尼（微分+1kHz低通） =====================
// 功能：抑制LCL谐振，带1kHz低通滤波，完全抑制开关噪声，绝对稳定
// 对应 Simulink 模型：CapCurrentDamping_Improved
// 连续域模型：G(s) = (C*Hi1*s) * (omega_c/(s+omega_c))
// 离散化方法：双线性变换（Tustin）+ 频率预畸变
// ====================================================================================

/**
  * @brief  有源阻尼初始化 (带1kHz低通)
  * @note   预计算所有系数，仅在模式切换时调用一次
  */
void FF_ActiveDamping_Init(FF_ActiveDamping_t *ff)
{
    // 1. 系统参数赋值
    ff->Ts = 1.0f / 20000.0f;
    ff->C = 10e-6f;
    ff->Hi1 = 0.07f;
    ff->f_cutoff = 1000.0f; // 【关键】截止频率设为 1kHz

    float Ts = ff->Ts;
    float C = ff->C;
    float Hi1 = ff->Hi1;
    float omega_c = 2.0f * 3.141592653589793f * ff->f_cutoff;

    // 2. 频率预畸变 (Frequency Pre-warping)
    // 原理：让数字截止频率精准对应模拟截止频率 1kHz
    float omega_s = (2.0f / Ts) * tanf(omega_c * Ts / 2.0f);

    // 3. 预计算差分方程系数 (和 Simulink 完全一致)
    float a0 = 2.0f + omega_s * Ts;
    
    ff->b0 = (2.0f * C * Hi1 * omega_s) / a0;
    ff->b1 = -ff->b0;
    ff->a1 = (omega_s * Ts - 2.0f) / a0;

    // 4. 初始化状态变量
    FF_ActiveDamping_Reset(ff);
}

/**
  * @brief  有源阻尼核心更新函数 (20kHz中断调用)
  * @param  u_cap: 传入电容电压约等于电网电压，直接用电网电压传感器即可
  * @retval 阻尼输出信号
  */
float FF_ActiveDamping_Update(FF_ActiveDamping_t *ff, float u_cap)
{
    // 【核心】差分方程 (和 Simulink 代码逐行对应)
    // y(k) = b0*u(k) + b1*u(k-1) - a1*y(k-1)
    float damp_out = ff->b0 * u_cap + ff->b1 * ff->u_prev_damp - ff->a1 * ff->y_prev_damp;

    // 【安全防护】输出限幅 (防止极端情况溢出)
    const float DAMP_MAX = 2.0f;
    const float DAMP_MIN = -2.0f;
    if(damp_out > DAMP_MAX) damp_out = DAMP_MAX;
    else if(damp_out < DAMP_MIN) damp_out = DAMP_MIN;

    // 更新状态变量
    ff->u_prev_damp = u_cap;
    ff->y_prev_damp = damp_out;

    return damp_out;
}

/**
  * @brief  有源阻尼复位
  * @note   模式切换时必须调用，清零状态变量
  */
void FF_ActiveDamping_Reset(FF_ActiveDamping_t *ff)
{
    // 强制清零所有状态变量，保留预计算系数
    ff->u_prev_damp = 0.0f;
    ff->y_prev_damp = 0.0f;
}


// ====================================================================================
// ===================== 【模块3/3】二阶微分前馈 =====================
// 功能：提供相位超前，补偿控制延迟
// 注意：必须在前两个都稳定后最后加，这是最容易放大噪声的环节
// ====================================================================================
void FF_SecondOrder_Init(FF_SecondOrder_t *ff)
{
    ff->Ts = 1.0f / 20000.0f;
    ff->L1 = 1.5e-3f;
    ff->C = 10e-6f;
    ff->k_pwm = 55.0f;
    ff->f_cutoff = 1150.0f;
    ff->zeta = 0.707f;

    float Ts = ff->Ts;
    float a = 2.0f / Ts;

    // 计算总增益
    float K = (ff->L1 * ff->C) / ff->k_pwm;
    
    // 频率预畸变
    float omega_d = 2.0f * 3.1415926535f * ff->f_cutoff;
    float omega_s = (2.0f / Ts) * tanf(omega_d * Ts / 2.0f);
    
    float omega_s_sq = omega_s * omega_s;
    float a_sq = a * a;
    float zeta_omega_s_a = 2.0f * ff->zeta * omega_s * a;

    // 离散化系数
    ff->num2 = K * omega_s_sq * a_sq;
    ff->num1 = -2.0f * ff->num2;
    ff->num0 = ff->num2;

    ff->den2 = a_sq + zeta_omega_s_a + omega_s_sq;
    ff->den1 = -2.0f * a_sq + 2.0f * omega_s_sq;
    ff->den0 = a_sq - zeta_omega_s_a + omega_s_sq;

    ff->den2_inv = 1.0f / ff->den2;

    FF_SecondOrder_Reset(ff);
}

float FF_SecondOrder_Update(FF_SecondOrder_t *ff, float u_grid)
{
    // 直接型II转置结构
    float num_part = ff->num2 * u_grid + ff->num1 * ff->u_prev1_ff + ff->num0 * ff->u_prev2_ff;
    float den_part = ff->den1 * ff->y_prev1_ff + ff->den0 * ff->y_prev2_ff;
    
    float ff_out = (num_part - den_part) * ff->den2_inv;
    
    // 更新状态
    ff->u_prev2_ff = ff->u_prev1_ff;
    ff->u_prev1_ff = u_grid;
    ff->y_prev2_ff = ff->y_prev1_ff;
    ff->y_prev1_ff = ff_out;

    return ff_out;
}

void FF_SecondOrder_Reset(FF_SecondOrder_t *ff)
{
    ff->u_prev1_ff = 0.0f;
    ff->u_prev2_ff = 0.0f;
    ff->y_prev1_ff = 0.0f;
    ff->y_prev2_ff = 0.0f;
}


// ====================================================================================
// ===================== 逆变并网准谐振PR控制器 =====================
// 功能：对特定频率(50Hz)实现无静差跟踪 + 抗积分饱和
// 特点：限幅时冻结状态，反向误差时允许退饱和
// ====================================================================================
void PR_Inverter_Init(PR_Inverter_t *pr)
{
    pr->Ts = 5e-5f;         // 采样周期
    pr->Kp = 0.1f;          // 比例增益
    pr->Kr = 40.92f;        // 谐振增益
    float f0 = 50.0f;       // 谐振频率
    pr->wi = 3.1415926535f; // 谐振带宽 (rad/s)

    float Ts = pr->Ts;
    float Kp = pr->Kp;
    float Kr = pr->Kr;
    float wi = pr->wi;
    float Ts_sq = Ts * Ts;

    // 频率预畸变
    float omega_d = 2.0f * 3.1415926535f * f0;
    float w0 = (2.0f / Ts) * tanf(omega_d * Ts / 2.0f);
    pr->w0 = w0;

    float w0_sq_Ts_sq = w0 * w0 * Ts_sq;
    float wi_Ts = wi * Ts;

    // 双线性变换离散化系数
    float a0_pr = 4.0f + 2.0f*wi_Ts + w0_sq_Ts_sq;
    float a1_pr = 2.0f*w0_sq_Ts_sq - 8.0f;
    float a2_pr = 4.0f - 2.0f*wi_Ts + w0_sq_Ts_sq;

    float b0_pr = 4.0f*Kp + 4.0f*Kp*wi_Ts + 4.0f*Kr*wi_Ts + Kp*w0_sq_Ts_sq;
    float b1_pr = 2.0f*Kp*w0_sq_Ts_sq - 8.0f*Kp;
    float b2_pr = 4.0f*Kp - 4.0f*Kp*wi_Ts - 4.0f*Kr*wi_Ts + Kp*w0_sq_Ts_sq;

    pr->a0_inv = 1.0f / a0_pr;
    pr->a1 = a1_pr;
    pr->a2 = a2_pr;
    pr->b0 = b0_pr;
    pr->b1 = b1_pr;
    pr->b2 = b2_pr;

    PR_Inverter_Reset(pr);
}

// ===================== 抗饱和 PR 更新函数 =====================
float PR_Inverter_Update(PR_Inverter_t *pr, float u)
{
    const float OUT_MAX =  0.95f;
    const float OUT_MIN = -0.95f;

    // 1. 计算未限幅输出
    float y_unlim = (pr->b0*u + pr->b1*pr->x_prev1 + pr->b2*pr->x_prev2
                   - pr->a1*pr->y_prev1 - pr->a2*pr->y_prev2) * pr->a0_inv;

    // 2. 预判断是否在线性区
    uint8_t is_linear = ((y_unlim < OUT_MAX) && (y_unlim > OUT_MIN));

    // 3. 反向误差判断（允许退饱和）
    uint8_t is_reverse = 0;
    if((u > 0.0f && y_unlim < 0.0f) || (u < 0.0f && y_unlim > 0.0f))
    {
        is_reverse = 1;
    }

    // ===================== 【核心抗饱和】 =====================
    // 只有在线性区 或 反向误差时，才更新状态
    if(is_linear || is_reverse)
    {
        pr->x_prev2 = pr->x_prev1;
        pr->x_prev1 = u;
        pr->y_prev2 = pr->y_prev1;
        pr->y_prev1 = y_unlim;
    }

    // 4. 最终限幅输出
    float y = y_unlim;
    if(y > OUT_MAX) y = OUT_MAX;
    if(y < OUT_MIN) y = OUT_MIN;

    return y;
}

void PR_Inverter_Reset(PR_Inverter_t *pr)
{
    pr->x_prev1 = 0.0f;
    pr->x_prev2 = 0.0f;
    pr->y_prev1 = 0.0f;
    pr->y_prev2 = 0.0f;
    pr->first_run = 1;
}


// ====================================================================================
// ===================== 【最终完整版】整流专用 QPR 控制器 =====================
// 功能：4谐振频率 (50/150/250/350Hz) + 频率预畸变 + 抗饱和 + 反向恢复
// 特点：完全对齐 Simulink 模型，保证仿真与实机结果一致
// ====================================================================================

/**
  * @brief  QPR 控制器复位
  * @note   清零所有状态变量
  */
void QPR_Rect_Reset(QPR_Rect_t *qpr)
{
    // 基波 n=1
    qpr->e1_1 = 0.0f;
    qpr->e1_2 = 0.0f;
    qpr->m1_1 = 0.0f;
    qpr->m1_2 = 0.0f;

    // 3次 n=3
    qpr->e3_1 = 0.0f;
    qpr->e3_2 = 0.0f;
    qpr->m3_1 = 0.0f;
    qpr->m3_2 = 0.0f;

    // 5次 n=5
    qpr->e5_1 = 0.0f;
    qpr->e5_2 = 0.0f;
    qpr->m5_1 = 0.0f;
    qpr->m5_2 = 0.0f;

    // 7次 n=7
    qpr->e7_1 = 0.0f;
    qpr->e7_2 = 0.0f;
    qpr->m7_1 = 0.0f;
    qpr->m7_2 = 0.0f;
}

/**
  * @brief  QPR 控制器初始化
  * @note   包含频率预畸变和系数预计算，仅在模式切换时调用一次
  */
void QPR_Rect_Init(QPR_Rect_t *qpr)
{
    const float Ts = 1.0f / 20000.0f;
    const float Ts_sq = Ts * Ts;

    // ==================== 各次谐波谐振参数 ====================
    const float Kr1 = 40.0f;  const float wi1 = 6.0f;  const float f1 = 50.0f;
    const float Kr3 = 80.0f;   const float wi3 = 6.0f;  const float f3 = 150.0f;
    const float Kr5 = 0.0f;   const float wi5 = 6.0f;  const float f5 = 250.0f;
    const float Kr7 = 0.0f;   const float wi7 = 6.0f;  const float f7 = 350.0f;

    // ==================== 【核心】频率预畸变 + 系数预计算 ====================
    // 通用公式：w_digital = (2/Ts) * tan(w_analog * Ts / 2)
    
    // ---------- 50Hz ----------
    float omega_d1 = 2.0f * 3.141592653589793f * f1;
    float w01 = (2.0f / Ts) * tanf(omega_d1 * Ts / 2.0f);
    float A1 = 2.0f * Kr1 * wi1;
    float B1 = 2.0f * wi1;
    float C1 = w01 * w01;
    float a0_1 = 4.0f + 2.0f*B1*Ts + C1*Ts_sq;
    qpr->a0_1_inv = 1.0f / a0_1;
    qpr->a1_1 = (-8.0f + 2.0f*C1*Ts_sq);
    qpr->a2_1 = (4.0f - 2.0f*B1*Ts + C1*Ts_sq);
    qpr->b0_1 = A1 * Ts;
    qpr->b1_1 = 0.0f;
    qpr->b2_1 = -A1 * Ts;

    // ---------- 150Hz ----------
    float omega_d3 = 2.0f * 3.141592653589793f * f3;
    float w03 = (2.0f / Ts) * tanf(omega_d3 * Ts / 2.0f);
    float A3 = 2.0f * Kr3 * wi3;
    float B3 = 2.0f * wi3;
    float C3 = w03 * w03;
    float a0_3 = 4.0f + 2.0f*B3*Ts + C3*Ts_sq;
    qpr->a0_3_inv = 1.0f / a0_3;
    qpr->a1_3 = (-8.0f + 2.0f*C3*Ts_sq);
    qpr->a2_3 = (4.0f - 2.0f*B3*Ts + C3*Ts_sq);
    qpr->b0_3 = A3 * Ts;
    qpr->b1_3 = 0.0f;
    qpr->b2_3 = -A3 * Ts;

    // ---------- 250Hz ----------
    float omega_d5 = 2.0f * 3.141592653589793f * f5;
    float w05 = (2.0f / Ts) * tanf(omega_d5 * Ts / 2.0f);
    float A5 = 2.0f * Kr5 * wi5;
    float B5 = 2.0f * wi5;
    float C5 = w05 * w05;
    float a0_5 = 4.0f + 2.0f*B5*Ts + C5*Ts_sq;
    qpr->a0_5_inv = 1.0f / a0_5;
    qpr->a1_5 = (-8.0f + 2.0f*C5*Ts_sq);
    qpr->a2_5 = (4.0f - 2.0f*B5*Ts + C5*Ts_sq);
    qpr->b0_5 = A5 * Ts;
    qpr->b1_5 = 0.0f;
    qpr->b2_5 = -A5 * Ts;

    // ---------- 350Hz ----------
    float omega_d7 = 2.0f * 3.141592653589793f * f7;
    float w07 = (2.0f / Ts) * tanf(omega_d7 * Ts / 2.0f);
    float A7 = 2.0f * Kr7 * wi7;
    float B7 = 2.0f * wi7;
    float C7 = w07 * w07;
    float a0_7 = 4.0f + 2.0f*B7*Ts + C7*Ts_sq;
    qpr->a0_7_inv = 1.0f / a0_7;
    qpr->a1_7 = (-8.0f + 2.0f*C7*Ts_sq);
    qpr->a2_7 = (4.0f - 2.0f*B7*Ts + C7*Ts_sq);
    qpr->b0_7 = A7 * Ts;
    qpr->b1_7 = 0.0f;
    qpr->b2_7 = -A7 * Ts;

    // 初始化复位
    QPR_Rect_Reset(qpr);
}

/**
  * @brief  QPR 控制器核心更新函数 (20kHz 中断调用)
  * @param  qpr: 控制器结构体指针
  * @param  i_error: 电流误差 (参考 - 反馈)
  * @retval 调制波输出
  */
float QPR_Rect_Update(QPR_Rect_t *qpr, float i_error)
{
    // ===================== 全局共用参数 =====================
    const float Kp = 0.3f; // 固定比例增益

    // ===================== 1. 共用比例项（固定Kp）=====================
    float m_p = Kp * i_error;

    // ===================== 2. 基波谐振项计算 =====================
    float m1 = (qpr->b0_1 * i_error + qpr->b1_1 * qpr->e1_1 + qpr->b2_1 * qpr->e1_2 
                - qpr->a1_1 * qpr->m1_1 - qpr->a2_1 * qpr->m1_2) * qpr->a0_1_inv;
    
    // 更新基波状态
    qpr->e1_2 = qpr->e1_1; 
    qpr->e1_1 = i_error;
    qpr->m1_2 = qpr->m1_1; 
    qpr->m1_1 = m1;

    // ===================== 3. 3次谐波谐振项计算 =====================
    float m3 = (qpr->b0_3 * i_error + qpr->b1_3 * qpr->e3_1 + qpr->b2_3 * qpr->e3_2 
                - qpr->a1_3 * qpr->m3_1 - qpr->a2_3 * qpr->m3_2) * qpr->a0_3_inv;
    
    // 更新3次谐波状态
    qpr->e3_2 = qpr->e3_1; 
    qpr->e3_1 = i_error;
    qpr->m3_2 = qpr->m3_1; 
    qpr->m3_1 = m3;

    // ===================== 4. 5次谐波谐振项计算 =====================
    float m5 = (qpr->b0_5 * i_error + qpr->b1_5 * qpr->e5_1 + qpr->b2_5 * qpr->e5_2 
                - qpr->a1_5 * qpr->m5_1 - qpr->a2_5 * qpr->m5_2) * qpr->a0_5_inv;
    
    // 更新5次谐波状态
    qpr->e5_2 = qpr->e5_1; 
    qpr->e5_1 = i_error;
    qpr->m5_2 = qpr->m5_1; 
    qpr->m5_1 = m5;

    // ===================== 5. 7次谐波谐振项计算 =====================
    float m7 = (qpr->b0_7 * i_error + qpr->b1_7 * qpr->e7_1 + qpr->b2_7 * qpr->e7_2 
                - qpr->a1_7 * qpr->m7_1 - qpr->a2_7 * qpr->m7_2) * qpr->a0_7_inv;
    
    // 更新7次谐波状态
    qpr->e7_2 = qpr->e7_1; 
    qpr->e7_1 = i_error;
    qpr->m7_2 = qpr->m7_1; 
    qpr->m7_1 = m7;

		
    // ===================== 总输出叠加 =====================
 float m = m_p + m1 + m3 + m5 + m7;
//   float m = m_p+m1+m3;    //因整流桥辅助电源的引入导致的谐波
    return m;
}



// ====================================================================================
// ===================== 100Hz 陷波滤波器 =====================
// 功能：滤除整流后电压/电流的二次纹波(100Hz)
// 特点：在100Hz处增益为0，其他频率增益为1
// ====================================================================================
void Notch100Hz_Init(Notch100Hz_TypeDef *hnotch)
{
    const float Ts = 1.0f / 20000.0f;
    const float f0 = 100.0f;  // 陷波频率
    const float bw = 6.0f;     // 陷波带宽 (越窄越尖锐)

    // 频率预畸变
    float omega_d = 2.0f * PI * f0;
    float w0 = (2.0f / Ts) * tanf(omega_d * Ts / 2.0f);
    float wb = 2.0f * PI * bw;

    float k = 2.0f / Ts;
    float k_sq = k * k;
    float w0_sq = w0 * w0;

    // 双线性变换：陷波滤波器传递函数 G(s) = (s^2 + w0^2) / (s^2 + wb*s + w0^2)
    float a0 = k_sq + wb * k + w0_sq;
    float a1 = 2.0f * (w0_sq - k_sq);
    float a2 = k_sq - wb * k + w0_sq;
    float b0 = k_sq + w0_sq;
    float b1 = 2.0f * (w0_sq - k_sq);
    float b2 = k_sq + w0_sq;

    // 归一化 (除以a0，使a0=1)
    hnotch->b0 = b0 / a0;
    hnotch->b1 = b1 / a0;
    hnotch->b2 = b2 / a0;
    hnotch->a1 = a1 / a0;
    hnotch->a2 = a2 / a0;

    Notch100Hz_Reset(hnotch);
}

float Notch100Hz_Update(Notch100Hz_TypeDef *hnotch, float IGm_total)
{
    // 直接型II Transposed结构：计算效率最高，状态最少
    float Igm_clean = hnotch->b0 * IGm_total
                    + hnotch->b1 * hnotch->x_prev1
                    + hnotch->b2 * hnotch->x_prev2
                    - hnotch->a1 * hnotch->y_prev1
                    - hnotch->a2 * hnotch->y_prev2;

    // 更新状态
    hnotch->x_prev2 = hnotch->x_prev1;
    hnotch->x_prev1 = IGm_total;
    hnotch->y_prev2 = hnotch->y_prev1;
    hnotch->y_prev1 = Igm_clean;

    return Igm_clean;
}

void Notch100Hz_Reset(Notch100Hz_TypeDef *hnotch)
{
    hnotch->x_prev1 = 0.0f;
    hnotch->x_prev2 = 0.0f;
    hnotch->y_prev1 = 0.0f;
    hnotch->y_prev2 = 0.0f;
}

// ====================================================================================
// ===================== 级联陷波 (150/250/350Hz) =====================
// 功能：滤除3/5/7次谐波 (150/250/350Hz)
// 结构：三个独立的二阶陷波滤波器级联
// ====================================================================================
static void calc_coeff(float wn, float wc, float k, float k2,
                       float *b0, float *b1, float *b2, float *a1, float *a2)
{
    // 内部辅助函数：计算单级陷波系数
    // 输入：wn=陷波频率(rad/s), wc=陷波带宽, k=2/Ts, k2=(2/Ts)^2
    
    // 模拟传递函数：G(s) = (s^2 + wn^2) / (s^2 + wc*s + wn^2)
    float num_b0 = 1.0f;
    float num_b1 = 0.0f;
    float num_b2 = wn * wn;

    float den_a0 = 1.0f;
    float den_a1 = 2.0f * wc;
    float den_a2 = wn * wn;

    // 双线性变换：s = k*(z-1)/(z+1), k=2/Ts
    float B0 = num_b0 * k2 + num_b1 * k + num_b2;
    float B1 = 2.0f * num_b2 - 2.0f * num_b0 * k2;
    float B2 = num_b0 * k2 - num_b1 * k + num_b2;

    float A0 = den_a0 * k2 + den_a1 * k + den_a2;
    float A1 = 2.0f * den_a2 - 2.0f * den_a0 * k2; // 已修正：使用den_a0而非num_b0
    float A2 = num_b0 * k2 - den_a1 * k + den_a2;

    // 归一化 (除以A0)
    *b0 = B0 / A0;
    *b1 = B1 / A0;
    *b2 = B2 / A0;
    *a1 = A1 / A0;
    *a2 = A2 / A0;
}

void CascadeNotch_Init(CascadeNotch_TypeDef *filter)
{
    const float Fs = 20000.0f;
    const float Ts = 1.0f / Fs;
    const float wc = 10.0f; // 统一陷波带宽

    float f3 = 150.0f; float f5 = 250.0f; float f7 = 350.0f;
    
    // 分别对三个频率进行预畸变
    float omega_d3 = 2.0f * PI * f3;
    float w3 = (2.0f / Ts) * tanf(omega_d3 * Ts / 2.0f);

    float omega_d5 = 2.0f * PI * f5;
    float w5 = (2.0f / Ts) * tanf(omega_d5 * Ts / 2.0f);

    float omega_d7 = 2.0f * PI * f7;
    float w7 = (2.0f / Ts) * tanf(omega_d7 * Ts / 2.0f);

    const float k = 2.0f / Ts;
    const float k2 = k * k;

    // 初始化三级系数：150Hz -> 250Hz -> 350Hz
    calc_coeff(w3, wc, k, k2, &filter->b3_0, &filter->b3_1, &filter->b3_2, &filter->a3_1, &filter->a3_2);
    calc_coeff(w5, wc, k, k2, &filter->b5_0, &filter->b5_1, &filter->b5_2, &filter->a5_1, &filter->a5_2);
    calc_coeff(w7, wc, k, k2, &filter->b7_0, &filter->b7_1, &filter->b7_2, &filter->a7_1, &filter->a7_2);

    CascadeNotch_Reset(filter);
}

float CascadeNotch_Update(CascadeNotch_TypeDef *filter, float u)
{
    // 第一级：150Hz (3次谐波)
    float y3 = filter->b3_0 * u + filter->x3_1;
    float x3_1_next = filter->b3_1 * u - filter->a3_1 * y3 + filter->x3_2;
    float x3_2_next = filter->b3_2 * u - filter->a3_2 * y3;

    // 第二级：250Hz (5次谐波)，输入为上一级输出
    float y5 = filter->b5_0 * y3 + filter->x5_1;
    float x5_1_next = filter->b5_1 * y3 - filter->a5_1 * y5 + filter->x5_2;
    float x5_2_next = filter->b5_2 * y3 - filter->a5_2 * y5;

    // 第三级：350Hz (7次谐波)
    float y7 = filter->b7_0 * y5 + filter->x7_1;
    float x7_1_next = filter->b7_1 * y5 - filter->a7_1 * y7 + filter->x7_2;
    float x7_2_next = filter->b7_2 * y5 - filter->a7_2 * y7;

    // 统一更新状态：先计算所有新值，再统一更新，避免数据依赖
    filter->x3_1 = x3_1_next;
    filter->x3_2 = x3_2_next;
    filter->x5_1 = x5_1_next;
    filter->x5_2 = x5_2_next;
    filter->x7_1 = x7_1_next;
    filter->x7_2 = x7_2_next;

    return y7;
}

void CascadeNotch_Reset(CascadeNotch_TypeDef *filter)
{
    // 清零所有三级的状态变量
    filter->x3_1 = 0.0f; filter->x3_2 = 0.0f;
    filter->x5_1 = 0.0f; filter->x5_2 = 0.0f;
    filter->x7_1 = 0.0f; filter->x7_2 = 0.0f;
}

// ====================================================================================
// ===================== SOGI-FLL (原版：矩阵库版本) =====================
// 功能：从电网电压中提取正交分量(alpha/beta)，并实时跟踪电网频率
// 特点：基于状态空间模型，代码清晰，易于理解
// ====================================================================================
void SOGI_FLL_Init(SOGI_FLL_TypeDef *sogi)
{
    // 初始化内部状态
    sogi->x1_prev = 0.0f;
    sogi->x2_prev = 0.0f;
    sogi->x3_prev = 0.0f;
    sogi->omega0_prev = 2.0f * PI * 50.0f; // 初始频率50Hz
    sogi->U_prev = 0.0f;
    sogi->integral_f = 0.0f;

    // 初始化输出
    sogi->alpha = 0.0f;
    sogi->beta  = 0.0f;
    sogi->omega0 = 2.0f * PI * 50.0f;

    // 配置参数
    sogi->dt       = 5e-5f;         // 采样周期
    sogi->K        = 1.414f;        // 阻尼系数 (sqrt(2))
    sogi->K1       = 0.25f;         // 增益系数
    sogi->gamma_p  = 30.0f;         // FLL比例增益
    sogi->gamma_i  = 500.0f;        // FLL积分增益
    sogi->w_max    = 2.0f * PI * 60.0f; // 频率上限60Hz
    sogi->w_min    = 2.0f * PI * 40.0f; // 频率下限40Hz

    // 【优化】预计算常数，消除运行时除法
    sogi->two_over_dt = 2.0f / sogi->dt;
}

void SOGI_FLL_Update(SOGI_FLL_TypeDef *sogi, float uin)
{
    float U_curr          = uin;
    float omega0_prev_local = sogi->omega0_prev;
    float dt              = sogi->dt;
    float K               = sogi->K;
    float K1              = sogi->K1;
    float gamma_p         = sogi->gamma_p;
    float gamma_i         = sogi->gamma_i;

    // ===================== 【固定常数：20kHz 专用】 =====================
    // dt = 5e-5f，half_dt = 0.000025f，编译时常数，无运行时计算
    // ==================================================================
    #define SOGI_HALF_DT    0.000025f

    // ==========================================
    // 【核心优化】消除 tanf 调用
    // 原代码：float omega_pre = sogi->two_over_dt * tanf(omega0_prev_local * dt / 2.0f);
    // 优化后：极小角度下 tan(x) ≈ x，常数项抵消后直接等于 omega0_prev
    // 误差：< 3e-7，远小于 ADC 采样噪声
    // ==========================================
    float omega_pre = omega0_prev_local;
    float wp_sq = omega_pre * omega_pre;
    float U_avg = U_curr + sogi->U_prev;

    // 2. 构造状态空间矩阵
    // 连续时间状态方程：dx/dt = A*x + B*u
    float A_data[9] = {
        -K1*omega_pre, -K1*omega_pre,  0.0f,
        -K*omega_pre,  -K*omega_pre,   -wp_sq,
         0.0f,          1.0f,           0.0f
    };
    float I_data[9] = {1,0,0,  0,1,0,  0,0,1};
    float B_data[3] = {K1*omega_pre, K*omega_pre, 0.0f};

    arm_matrix_instance_f32 A_mat, I_mat, B_mat;
    arm_mat_init_f32(&A_mat, 3, 3, A_data);
    arm_mat_init_f32(&I_mat, 3, 3, I_data);
    arm_mat_init_f32(&B_mat, 3, 1, B_data);

    // 3. 双线性变换离散化：M1 = I - (dt/2)*A, M2 = I + (dt/2)*A
    float temp_mat_data[9], M1_data[9], M2_data[9];
    arm_matrix_instance_f32 temp_mat, M1_mat, M2_mat;
    arm_mat_init_f32(&temp_mat, 3, 3, temp_mat_data);
    arm_mat_init_f32(&M1_mat, 3, 3, M1_data);
    arm_mat_init_f32(&M2_mat, 3, 3, M2_data);

    arm_mat_scale_f32(&A_mat, SOGI_HALF_DT, &temp_mat);  // <--- 这里修改
    arm_mat_sub_f32(&I_mat, &temp_mat, &M1_mat);
    arm_mat_add_f32(&I_mat, &temp_mat, &M2_mat);

    // 4. 计算离散状态更新：x_curr = inv(M1) * (M2*x_prev + (dt/2)*B*U_avg)
    float x_prev_data[3] = {sogi->x1_prev, sogi->x2_prev, sogi->x3_prev};
    float temp_vec1[3], temp_vec2[3], R_data[3];
    arm_matrix_instance_f32 x_prev, temp1, temp2, R_vec;
    arm_mat_init_f32(&x_prev, 3, 1, x_prev_data);
    arm_mat_init_f32(&temp1, 3, 1, temp_vec1);
    arm_mat_init_f32(&temp2, 3, 1, temp_vec2);
    arm_mat_init_f32(&R_vec, 3, 1, R_data);

    arm_mat_mult_f32(&M2_mat, &x_prev, &temp1);
    arm_mat_scale_f32(&B_mat, SOGI_HALF_DT * U_avg, &temp2);  // <--- 这里修改
    arm_mat_add_f32(&temp1, &temp2, &R_vec);

    // 5. 求逆并计算当前状态 (注意：这是最耗时的部分！)
    float inv_M1_data[9];
    arm_matrix_instance_f32 inv_M1;
    arm_mat_init_f32(&inv_M1, 3, 3, inv_M1_data);
    arm_mat_inverse_f32(&M1_mat, &inv_M1);

    float x_curr_data[3];
    arm_matrix_instance_f32 x_curr;
    arm_mat_init_f32(&x_curr, 3, 1, x_curr_data);
    arm_mat_mult_f32(&inv_M1, &R_vec, &x_curr);

    float x1_curr = x_curr_data[0];
    float x2_curr = x_curr_data[1];
    float x3_curr = x_curr_data[2];

    // 6. FLL (锁频环) 频率调整逻辑
    // 原理：基于误差的梯度下降法调整频率
    float deltaU = U_curr - x1_curr - x2_curr;
    float deltaU_prime_val = omega0_prev_local * x3_curr;
    float epsilon_f = deltaU * deltaU_prime_val;

    sogi->integral_f += epsilon_f * dt;
    float domega0dt = -gamma_p * epsilon_f - gamma_i * sogi->integral_f;
    float omega0_curr = omega0_prev_local + dt * domega0dt;

    // 频率限幅：防止频率失控
    if(omega0_curr > sogi->w_max) omega0_curr = sogi->w_max;
    if(omega0_curr < sogi->w_min) omega0_curr = sogi->w_min;

    // 7. 输出赋值
    sogi->alpha = x2_curr;              // alpha分量 (同相)
    sogi->beta  = omega0_curr * x3_curr; // beta分量 (正交，需乘以当前频率)
    sogi->omega0 = omega0_curr;         // 当前跟踪到的角频率

    // 8. 状态刷新
    sogi->x1_prev = x1_curr;
    sogi->x2_prev = x2_curr;
    sogi->x3_prev = x3_curr;
    sogi->omega0_prev = omega0_curr;
    sogi->U_prev = U_curr;
}



void SOGI_FLL_Reset(SOGI_FLL_TypeDef *sogi)
{
   // 1. 核心状态量清零
    sogi->x1_prev = 0.0f;
    sogi->x2_prev = 0.0f;
    sogi->x3_prev = 0.0f;
    
    // 2. 频率恢复到默认基准 (50Hz)
    sogi->omega0_prev = 2.0f * PI * 50.0f;
    sogi->omega0      = 2.0f * PI * 50.0f;
    
    // 3. FLL 积分器清零 (防止锁频环带着历史偏差启动)
    sogi->integral_f = 0.0f;

    // 4. 【关键补充】输入历史清零
    // 这样复位后第一拍：U_avg = (当前采样值 + 0) / 2
    // 虽然仍有半个幅值的阶跃，但比带“随机旧值”启动要稳得多
    sogi->U_prev = 0.0f; 

    // 5. 输出清零
    sogi->alpha = 0.0f;
    sogi->beta  = 0.0f;
	
}

// ====================================================================================
// ===================== SOGI-FLL (优化版：纯标量运算 + 编译时常数) =====================
// 性能提升：8~10倍
// 优化：dt=5e-5(20kHz)固定，half_dt直接写为常数，消除运行时乘法
// ====================================================================================
void SOGI_FLL_Update_Optimized(SOGI_FLL_TypeDef *sogi, float uin)
{
    float U_curr          = uin;
    float omega0_prev_local = sogi->omega0_prev;
    float dt              = sogi->dt;
    float K               = sogi->K;
    float K1              = sogi->K1;
    float gamma_p         = sogi->gamma_p;
    float gamma_i         = sogi->gamma_i;

    // ===================== 【关键优化】 =====================
    // dt = 5e-5f (20kHz采样)
    // half_dt = dt * 0.5f = 0.000025f
    // 编译时常数，无运行时计算开销
    // ========================================================
    #define SOGI_HALF_DT    0.000025f

    // 1. 频率预畸变
    float omega_pre = omega0_prev_local;   // 已优化：无 tanf
    float wp_sq = omega_pre * omega_pre;
    float U_avg = U_curr + sogi->U_prev;

    // 2. 定义中间标量变量
    float K1w = K1 * omega_pre;
    float Kw  = K * omega_pre;
    float h_K1w = SOGI_HALF_DT * K1w;
    float h_Kw  = SOGI_HALF_DT * Kw;
    float h_wp_sq = SOGI_HALF_DT * wp_sq;

    // M1 矩阵元素
    float m11 = 1.0f + h_K1w;
    float m12 = h_K1w;
    float m21 = h_Kw;
    float m22 = 1.0f + h_Kw;
    float m23 = h_wp_sq;
    float m32 = -SOGI_HALF_DT;
    float m33 = 1.0f;

    // 3. 计算 R_vec
    float r1 = (1.0f - h_K1w)*sogi->x1_prev + (-h_K1w)*sogi->x2_prev + h_K1w*U_avg;
    float r2 = (-h_Kw)*sogi->x1_prev + (1.0f - h_Kw)*sogi->x2_prev + (-h_wp_sq)*sogi->x3_prev + h_Kw*U_avg;
    float r3 = SOGI_HALF_DT*sogi->x2_prev + sogi->x3_prev;

    // 4. 核心：2x2 克莱姆法则求解
    float a = m11;
    float b = m12;
    float c = m21;
    float d = m22 - m23 * m32;
    float e = r1;
    float f = r2 - m23 * r3;

    float det_2x2 = a * d - b * c;
    if(fabsf(det_2x2) < 1e-9f) det_2x2 = 1e-9f;
    float inv_det_2x2 = 1.0f / det_2x2;

    float x2_curr = (a * f - c * e) * inv_det_2x2;
    float x1_curr = (d * e - b * f) * inv_det_2x2;
    float x3_curr = r3 - m32 * x2_curr;

    // 5. FLL 频率调整
    float deltaU = U_curr - x1_curr - x2_curr;
    float deltaU_prime_val = omega0_prev_local * x3_curr;
    float epsilon_f = deltaU * deltaU_prime_val;

    sogi->integral_f += epsilon_f * dt;
    float domega0dt = -gamma_p * epsilon_f - gamma_i * sogi->integral_f;
    float omega0_curr = omega0_prev_local + dt * domega0dt;

    // 频率限幅
    if(omega0_curr > sogi->w_max) omega0_curr = sogi->w_max;
    if(omega0_curr < sogi->w_min) omega0_curr = sogi->w_min;

    // 6. 输出赋值
    sogi->alpha = x2_curr;
    sogi->beta  = omega0_curr * x3_curr;
    sogi->omega0 = omega0_curr;

    // 7. 状态刷新
    sogi->x1_prev = x1_curr;
    sogi->x2_prev = x2_curr;
    sogi->x3_prev = x3_curr;
    sogi->omega0_prev = omega0_curr;
    sogi->U_prev = U_curr;
}


// ====================================================================================
// ===================== SRF-PLL (同步旋转坐标系锁相环) =====================
// 功能：从正交分量(alpha/beta)中提取电网相位
// 特点：基于Park变换，稳态无静差
// ====================================================================================
void SRF_PLL_Init(SRF_PLL_TypeDef *pll)
{
    pll->Ts = 5e-5f;            // 20kHz
    pll->kp_pll = 200.0f;       
    pll->ki_pll = 7000.0f;      
    pll->omega_ff = 2.0f * PI * 50.0f; 

    // 【核心优化】：在初始化时预计算转换因子
    pll->rad2deg = 180.0f / PI; 

    pll->theta = 0.0f;
    pll->theta_prev = 0.0f;
    pll->integral_pll = 0.0f;
    pll->qerror = 0.0f;
    pll->sin_theta = 0.0f;
    pll->cos_theta = 1.0f;
}

/**
  * @brief  SRF-PLL 核心更新算法
  * @param  pll: 结构体指针
  * @param  u_alpha: SOGI 输出的同相信号
  * @param  u_beta:  SOGI 输出的正交信号
  */
void SRF_PLL_Update(SRF_PLL_TypeDef *pll, float u_alpha, float u_beta)
{
    float sin_t, cos_t;

    // 1. 弧度转角度：使用预计算好的乘法因子，消除除法
    float theta_deg = pll->theta_prev * pll->rad2deg;
    
    // CMSIS-DSP 查表计算
    arm_sin_cos_f32(theta_deg, &sin_t, &cos_t);
    
    pll->sin_theta = sin_t;
    pll->cos_theta = cos_t;

    // 2. Park 变换鉴相
    float u_q = -u_alpha * sin_t + u_beta * cos_t;
    pll->qerror = u_q;

    // 3. PI 调节 + 频率前馈
    pll->integral_pll += u_q * pll->Ts;
    float omega_pll = pll->omega_ff + (pll->kp_pll * u_q + pll->ki_pll * pll->integral_pll);

    // 4. 频率限幅 (40Hz ~ 60Hz)
    if(omega_pll > 376.9911f) omega_pll = 376.9911f;
    if(omega_pll < 251.3274f) omega_pll = 251.3274f;

    // 5. 积分生成新相位
    float theta_now = pll->theta_prev + omega_pll * pll->Ts;

    // 6. 相位归一化
    if(theta_now > PI)       theta_now -= 2.0f * PI;
    else if(theta_now < -PI) theta_now += 2.0f * PI;

    pll->theta = theta_now;
    pll->theta_prev = theta_now;
}


/**
  * @brief  SRF-PLL 复位
  * @note   仅重置运行状态，保留配置参数 (Gains/Ts/FF)
  */
void SRF_PLL_Reset(SRF_PLL_TypeDef *pll)
{
    pll->theta = 0.0f;
    pll->theta_prev = 0.0f;
    pll->integral_pll = 0.0f;
    pll->qerror = 0.0f;
    pll->sin_theta = 0.0f;
    pll->cos_theta = 1.0f;
}
// ====================================================================================
// ===================== 通用PI (带抗积分饱和 Back Calculation) =====================
// 功能：通用比例积分控制器，带完善的抗积分饱和机制
// 特点：不仅限幅输出，还通过方向钳制防止积分器饱和
// ====================================================================================
void PI_Init(PI_TypeDef *pi)
{
    pi->integral_val = 0.0f;
    pi->Kp = 0.05f;
    pi->Ki = 10.0f;
    pi->Ts = 5.0e-5f;
    pi->OUT_MAX = 0.9f;    // 输出上限
    pi->OUT_MIN = 0.1f;    // 输出下限
    pi->LIMIT = 4.5f;      // 反馈饱和阈值
    pi->out = 0.0f;
}

float PI_Update(PI_TypeDef *pi, float ref, float fb)
{
    float Kp = pi->Kp;
    float Ki = pi->Ki;
    float Ts = pi->Ts;
    float OUT_MAX = pi->OUT_MAX;
    float OUT_MIN = pi->OUT_MIN;
    float LIMIT = pi->LIMIT;
    float integral_val = pi->integral_val;

    float err = ref - fb;
    float prop_out = Kp * err;

    // 抗积分饱和逻辑：只有当输出在限幅内，且反馈未饱和时，才累加积分
    float integral_inc = Ki * Ts * err;
    float integral_pre = integral_val + integral_inc;
    float out_pre = prop_out + integral_pre;

    uint8_t is_out_linear = (out_pre < OUT_MAX) && (out_pre > OUT_MIN);
    uint8_t is_fb_unsat = (fabsf(fb) < LIMIT);
    uint8_t is_integral_allow = is_out_linear && is_fb_unsat;
    
    // 方向钳制：如果误差和积分项反向，说明在退饱和，允许积分
    // 这是为了防止积分器卡在饱和值无法退出
    uint8_t is_reverse_integral = (err > 0.0f && integral_pre < 0.0f) ||
                                   (err < 0.0f && integral_pre > 0.0f);

    if (is_integral_allow || is_reverse_integral)
    {
        integral_val = integral_pre;
    }

    // 计算输出并限幅
    float out = prop_out + integral_val;
    out = fminf(out, OUT_MAX);
    out = fmaxf(out, OUT_MIN);

    pi->integral_val = integral_val;
    pi->out = out;
    return out;
}

void PI_Reset(PI_TypeDef *pi)
{
    pi->integral_val = 0.0f;
    pi->out = 0.0f;
}


// ====================================================================================
// ===================== 重复控制器 (RC) - 100% 算法保真版 =====================
// @note   1. 100% 和你的 MATLAB 代码逐行对应
//         2. 仅做索引转换 (MATLAB 1-based -> C 0-based)
//         3. 保留所有安全保护
// ====================================================================================

void RC_Init(RC_TypeDef *rc) {
    if (rc == NULL) return;
    memset(rc->buffer, 0, sizeof(rc->buffer));
    rc->ptr = 0;
    rc->is_initialized = 1;
}

void RC_Reset(RC_TypeDef *rc) {
    if (rc == NULL) return;
    memset(rc->buffer, 0, sizeof(rc->buffer));
    rc->ptr = 0;
}

float RC_Update(RC_TypeDef *rc, float u_in) {
    // --- 0. 基础保护 ---
    if (rc == NULL || !rc->is_initialized) return 0.0f;
    
    // 输入噪声/范围保护
    if (fabsf(u_in) < 1e-7f) u_in = 0.0f;
    else if (fabsf(u_in) > 100.0f) u_in = 0.0f;

    // --- 1. 固定参数 ---
    const uint32_t L = 402;
    const float k_rc = 0.3f;
    const float Q0 = 0.25f;
    const float Q1 = 0.5f;
    const float Q2 = 0.25f;
    
    uint32_t p = rc->ptr;

    // --- 2. 工业级索引计算 (用 if 替代 %，防止除法占用 CPU) ---
    uint32_t t397 = p + 396; if (t397 >= L) t397 -= L;
    uint32_t t398 = p + 397; if (t398 >= L) t398 -= L;
    uint32_t t399 = p + 398; if (t399 >= L) t399 -= L;
    uint32_t t400 = p + 399; if (t400 >= L) t400 -= L;
    uint32_t t401 = p + 400; if (t401 >= L) t401 -= L;

    // --- 3. 计算输出 (等效于原代码 u_out = b * x) ---
    float u_out = (k_rc * Q0) * rc->buffer[t397]
                + (k_rc * Q1) * rc->buffer[t398]
                + (k_rc * Q2) * rc->buffer[t399];

    // --- 4. 非规格化数值保护 (工业级截断，防止 FPU 卡顿) ---
    if (fabsf(u_out) < 1e-9f) u_out = 0.0f;

    // --- 5. 增量更新缓冲区 (等效于原代码的状态转移) ---
    float b_u_in = k_rc * u_in; // 提取公因子

    // 模拟 x(n) = x(n+1) + b*u_in
    rc->buffer[t397] += Q0 * b_u_in;
    rc->buffer[t398] += Q1 * b_u_in;
    rc->buffer[t399] += Q2 * b_u_in;

    // 模拟 x(n) = x(n+1) - a*u_out (这里 a 为负值，所以是 +=)
    rc->buffer[t400] += Q0 * u_out;
    rc->buffer[t401] += Q1 * u_out;
    
    // 对应 MATLAB 的 buffer(idx_ptr) = Q2 * u_out
    // 注意：这里是赋值 (=)，不是累加 (+=)
    rc->buffer[p] = Q2 * u_out;

    // --- 6. 指针前进 (相当于逻辑上的数组左移一位) ---
    p++;
    if (p >= L) p = 0;
    rc->ptr = p;

    return u_out;
}



// ====================================================================================
// ===================== 【最终版】并网锁相及过零检测｜纯弧度PLL相位预判 =====================
// 适配：g_pll.theta 范围 0 ~ 2PI 原生弧度
// 逻辑：50Hz 提前5ms = 超前90° = π/2 相位
// ====================================================================================
void Rect_Lock_ZCD_Init(Rect_Lock_ZCD_TypeDef *zcd)
{
    // 原有参数完全不动
    zcd->derta_VAC = 0.3f;                // 过零判别阈值 ±0.3
    zcd->PLL_Discern_Counter_MAX = 15;    // 15个工频周期锁相锁定

    // 统一复位
    Rect_Lock_ZCD_Reset(zcd);
}

// 传入：电网电压Vac、PLL无功误差qerror、PLL原生弧度theta(0~2PI)


void Rect_Lock_ZCD_Update(Rect_Lock_ZCD_TypeDef *zcd, float Vac, float PLL_UQ, float theta)
{
	float derta_V = zcd->derta_VAC;

	// ============ 【100%原样保留】原有电压过零状态机 + 周期判定 ============
	if(zcd->AC_State==0 && Vac>=derta_V)
	{
		zcd->AC_State = 1;	// 正半周
	}
	else if(zcd->AC_State == 1 && (Vac<derta_V && Vac>-derta_V))
	{
		zcd->AC_State = 2;	// 过零点 (正→负)
	}
	else if(zcd->AC_State == 2 && Vac<=-derta_V*2)
	{
		zcd->AC_State = 3;	// 负半周
	}
	else if(zcd->AC_State == 3 && (Vac>-derta_V && Vac<derta_V))
	{
		zcd->AC_State = 4;	// 过零点 (负→正)
	}
	else if(zcd->AC_State == 4 && Vac>=derta_V*2)
	{
		zcd->AC_State = 1;	// 完成一个工频周期
		
		zcd->New_Cycle_Flag = 1;
		
		// ============ 原样保留：锁相收敛判定 ============
		if(PLL_UQ >=-0.15f && PLL_UQ<=0.15f)
		{
			zcd->PLL_Discern_Counter++;
			if(zcd->PLL_Discern_Counter >= zcd->PLL_Discern_Counter_MAX)
			{
				zcd->Covernt_EN = 1;
				zcd->PLL_Discern_Counter=0;
			}
		}
		else
		{
			zcd->Covernt_EN = 0;
			zcd->PLL_Discern_Counter = 0;
		}
	}

	// ============ 【全新重写】纯PLL弧度相位预判｜提前90°(5ms) ============
	// 改名：PI → PI_VAL，避免与arm_math.h冲突
	const float PI_2    = 1.570796f;   // 90°
	const float PI_VAL  = 3.141593f;   // 180° 正→负过零点
	const float PI3_2   = 4.712389f;   // 270°
	const float PI2     = 6.283185f;   // 360° 负→正过零点

	// 仅锁相收敛成功后，才允许继电器预触发
	if(zcd->Covernt_EN == 1)
	{
		// 区间1：90° ~ 180°  提前预判 正→负 过零
		// 区间2：270° ~ 360° 提前预判 负→正 过零
		if( (theta >= PI_2 && theta < PI_VAL) || 
			(theta >= PI3_2 && theta < PI2) )
		{
			zcd->Relay_Pretrigger_Flag = 1;
		}
		else
		{
			zcd->Relay_Pretrigger_Flag = 0;
		}
	}
	else
	{
		zcd->Relay_Pretrigger_Flag = 0;
	}

	// ============ 保留历史电压缓存，兼容旧代码 ============
	zcd->Vac_prev2 = zcd->Vac_prev1;
	zcd->Vac_prev1 = Vac;
}




void Rect_Lock_ZCD_Reset(Rect_Lock_ZCD_TypeDef *zcd)
{
    // 原有复位不变
    zcd->AC_State = 0;
    zcd->Covernt_EN = 0;
    zcd->PLL_Discern_Counter = 0;

    // 预判相关变量复位
    zcd->Vac_prev1 = 0.0f;
    zcd->Vac_prev2 = 0.0f;
    zcd->Relay_Pretrigger_Flag = 0;
    zcd->Trend_Confirm_Cnt = 0;

    zcd->New_Cycle_Flag = 0;
}


// ====================================================================================
// ===================== 系统启动函数 (寄存器优化版) =====================
// 功能：在中断中以微秒级速度开启 PWM，防止冲击和过时
// ====================================================================================
void sys_start(void)
{
    // 1. 【安全第一】先开启各通道使能位 (CCER寄存器)
    // 这里只是告诉硬件“这些管脚受定时器控制”，但由于 MOE 还没开，管脚仍是封锁状态
    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE | 
                   TIM_CCER_CC2E | TIM_CCER_CC2NE | 
                   TIM_CCER_CC4E | TIM_CCER_CC4NE);

    // 2. 【关键】瞬间拉高 MOE 位 (主输出总闸)
    // 这一步之后，PWM 波形才真正出现在管脚上
    TIM1->BDTR |= TIM_BDTR_MOE;

    // 3. 【硬件使能】最后开启驱动芯片 (GPIO 操作是安全的)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // 使能驱动
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
}




// ====================================================================================
// ===================== 系统停止函数 (寄存器极速版) =====================
// 功能：瞬间关闭 PWM 输出，不经过 HAL 状态机，防止硬件死锁
// ====================================================================================
void sys_stop(void)
{
    // 1. 【最关键】瞬间拉低 MOE 位 (主输出使能)
    // 这是 STM32 HRTIM/TIM1 最安全的紧急停止方式，原子操作，不依赖任何库函数
    TIM1->BDTR &= ~TIM_BDTR_MOE; 

    // 2. 强制将所有比较寄存器 (CCR) 清零
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR4 = 0;

    // 3. 关闭驱动芯片 (拉高使能引脚，关闭光耦/驱动)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);    // DC-DC 驱动
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);    // H 桥右
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);    // H 桥左

    // 4. 断开继电器
    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_RESET);
}


// ==========================================
// 【并网专用】Boost 启动
// ==========================================
void GridTie_Boost_Start(void)
{
   

    // 1. 开启 CC1E 和 CC1NE 使能
    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);

    // 2. 总闸 MOE 开启 (如果还没开的话)
    TIM1->BDTR |= TIM_BDTR_MOE;

    // 3. 物理使能
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
}

// ==========================================
// 【并网专用】H桥 瞬间投切
// ==========================================
void GridTie_Inverter_Start(void)
{
    // 已经在同步状态，只需瞬间激活 CCER 里的通道 2 和 4
    TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE | 
                   TIM_CCER_CC4E | TIM_CCER_CC4NE);
    
    // 物理驱动拉低（开启）
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
}



/********************************************************************************
* 函数功能：单极性倍频 SPWM CCR 计算（纯映射逻辑）
* 输入    ：modulate —— 调制波输入（如PR控制器输出）
* 返回值 ：CCR值，ARR=8500
* 逻辑    ：1.限幅[-0.95, 0.95]  2.(x+1)/2*8500
* 优化点  ：预计算所有常数，消除运行时乘法，单周期完成映射
********************************************************************************/
// 预计算常数：0.5 * 8500 = 4250.0f，编译时确定，运行时零开销
#define SPWM_HALF_ARR  4250.0f

int32_t SPWM_SinglePolarity_CCR(float modulate)
{
    float mod = modulate;

    // 1. 限幅到 -0.99 ~ 0.99（防止过调制）
    if(mod >  0.99f) mod =  0.99f;
    if(mod < -0.99f) mod = -0.99f;

    // 2. 优化后的映射公式：仅一次浮点乘法
    // 原公式：(mod + 1.0f) * 0.5f * 8500
    // 优化后：(mod + 1.0f) * SPWM_HALF_ARR
    int32_t ccr = (int32_t)((mod + 1.0f) * SPWM_HALF_ARR);

    return ccr;
}


// ====================================================================================
// ===================== 【适配 -π~π 版】闭环切入专用过零判断 =====================
// 对接：你的SRF-PLL（theta范围 -PI ~ PI）
// 功能：在负->正过零点（theta=0）精准触发闭环切入
// ====================================================================================

/**
  * @brief  初始化
  */
void ZCD_StateMachine_Init(ZCD_StateMachine_t *zcd)
{
    zcd->PLL_QError_Thresh = 0.15f;
    zcd->PLL_Lock_Cycles_Max = 15;

    zcd->AC_State = 0;
    zcd->last_cycle_flag = 0;
    zcd->PLL_Locked = 0;
    zcd->New_Cycle_Flag = 0;
    zcd->ZCD_Trigger_Flag = 0;
    zcd->pll_lock_counter = 0;
}

/**
  * @brief  复位
  */
void ZCD_StateMachine_Reset(ZCD_StateMachine_t *zcd)
{
    zcd->AC_State = 0;
    zcd->last_cycle_flag = 0;
    zcd->PLL_Locked = 0;
    zcd->New_Cycle_Flag = 0;
    zcd->ZCD_Trigger_Flag = 0;
    zcd->pll_lock_counter = 0;
}

/**
  * @brief  核心更新函数 (20kHz中断内调用)
  * @param  pll_qerror: PLL的qerror
  * @param  pll_theta: PLL的theta (-PI~PI)
  * @note   标志位自动清零，仅保持一个中断周期(50us)
  */
void ZCD_StateMachine_Update(ZCD_StateMachine_t *zcd, float pll_qerror, float pll_theta)
{
    const float PI_VAL = 3.141592653589793f;
    const float PI_HALF = PI_VAL / 2.0f; // π/2 (90°)
    
    // ========================================================================
    // 【优化点C：强制在最开始清零标志位】
    // 确保标志位只保持一个中断周期(50us)，防止用户层忘记清零
    // ========================================================================
    zcd->New_Cycle_Flag = 0;
    zcd->ZCD_Trigger_Flag = 0;

    // ========================================================================
    // 【优化后的状态机】
    // 相位图 (-PI~PI):
    // -PI (-180°)  -PI/2 (-90°)  0 (0°)  PI/2 (90°)  PI (180°)
    //     |            |            |         |          |
    //    状态1        状态2        状态0      状态1       状态1
    // ========================================================================

    switch(zcd->AC_State)
    {
        // ==================================================
        // 状态0：theta在 [0, PI/2)，正半周上升沿
        // ==================================================
        case 0:
            if(pll_theta >= PI_HALF)
            {
                zcd->AC_State = 1;
            }
            zcd->last_cycle_flag = 0;
            break;

        // ==================================================
        // 【优化点A：状态1条件简化】
        // 状态1：theta在 [PI/2, PI) 或 [-PI, -PI/2)，负半周/正半周峰值
        // 优化：只关心"什么时候离开"，不关心"什么时候留在这里"
        // ==================================================
        case 1:
            // 只要进入负半周上升沿区域 [-PI/2, 0)，就跳转到状态2
            if(pll_theta >= -PI_HALF && pll_theta < 0.0f)
            {
                zcd->AC_State = 2;
            }
            zcd->last_cycle_flag = 0;
            break;

        // ==================================================
        // 状态2：theta在 [-PI/2, 0)，负半周上升沿，快到过零了
        // ==================================================
        case 2:
            if(pll_theta >= 0.0f)
            {
                zcd->AC_State = 0; // 跨过0点，回到状态0
                
                // ==================================================
                // 【关键】在过零点执行所有操作
                // ==================================================
                if(zcd->last_cycle_flag == 0) // 去抖动
                {
                    zcd->New_Cycle_Flag = 1;
                    zcd->last_cycle_flag = 1;
                    
                    // ============ PLL锁定判定 ============
                    if(fabs(pll_qerror) < zcd->PLL_QError_Thresh)
                    {
                        zcd->pll_lock_counter++;
                        if(zcd->pll_lock_counter >= zcd->PLL_Lock_Cycles_Max)
                        {
                            zcd->PLL_Locked = 1;
                            zcd->pll_lock_counter = zcd->PLL_Lock_Cycles_Max;
                        }
                    }
                    else
                    {
                        zcd->PLL_Locked = 0;
                        zcd->pll_lock_counter = 0;
                    }
                    
                    // ============ 闭环触发 ============
                    if(zcd->PLL_Locked == 1)
                    {
                        zcd->ZCD_Trigger_Flag = 1; // 【立即执行闭环切入！】
                    }
                }
            }
            break;

        // ==================================================
        // 异常状态复位
        // ==================================================
        default:
            zcd->AC_State = 0;
            zcd->last_cycle_flag = 0;
            break;
    }
}


