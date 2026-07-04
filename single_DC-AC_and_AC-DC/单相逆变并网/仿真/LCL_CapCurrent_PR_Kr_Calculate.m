% LCL_Grid_Connected_Inverter_PR_Controller_Kr_Parameter_Bounds_Calculation.m
% 《含有电容电流反馈的LCL 并网逆变器_PR 控制器_Kr 参数上下限计算.m》  
% 公式匹配：(8.31) Kr下限（Tfo约束）、(8.33) Kr上限（PM约束）
% LCL并网逆变器 PR控制器 Kr 完整计算
% 适配参数：fc=1373, Hi1=0.07, Kp=0.253, Hi2=1
clc; clear; close all;

%% ===================== 1. 你指定的所有参数 =====================
% 控制参数
fc = 1373;              % 电流环截止频率，单位：Hz
Hi1 = 0.07;             % 电容电流反馈系数
Kp_input = 0.253;       % 你指定的比例系数
Hi2 = 1;                 % 网侧电流反馈系数

% LCL滤波器参数
L1 = 1.2e-3;            % 逆变侧电感，单位：H (1.2mH)
L2 = 120e-6;            % 网侧电感，单位：H (120uH)
C = 10e-6;              % 滤波电容，单位：F (10uF)

% 系统与稳定性指标
fs = 20000;              % 采样/开关频率，单位：Hz (20kHz)
Kpwm = 45;               % PWM桥臂增益
Tfo = 73;                % 基波环路增益，单位：dB
PM = 45;                 % 目标相位裕度，单位：°
wi = pi;                 % PR谐振带宽，单位：rad/s
wo = 100*pi;             % 基波角频率，单位：rad/s

%% ===================== 2. 中间参数自动计算 =====================
Ts = 1/fs;                                   % 采样周期，单位：s
L_total = L1 + L2;                           % 总电感，单位：H
fo = wo/(2*pi);                              % 基波频率，单位：Hz (50Hz)
fi = wi/(2*pi);                              % PR谐振带宽对应频率，单位：Hz (0.5Hz)
% LCL谐振频率
L_parallel = (L1 * L2) / (L1 + L2);         % 两侧电感并联值
fr = 1/(2*pi * sqrt(L_parallel * C));        % 谐振频率，单位：Hz

% 增益换算（dB转线性值）
Tfo_gain = 10^(Tfo/20);
PM_rad = PM * pi / 180;                      % 相位裕度转弧度

% 打印中间参数
fprintf('=== 中间参数计算结果 ===\n');
fprintf('总电感 L1+L2 = %.6f H\n', L_total);
fprintf('LCL谐振频率 fr = %.2f Hz\n', fr);
fprintf('采样周期 Ts = %.6f s\n', Ts);

%% ===================== 3. Kp 验证（公式8.25）=====================
Kp_theory = (2*pi*fc*L_total) / (Hi2 * Kpwm);
fprintf('\n=== Kp 验证结果 ===\n');
fprintf('理论计算 Kp = %.3f\n', Kp_theory);
fprintf('你指定的 Kp = %.3f\n', Kp_input);
if abs(Kp_theory - Kp_input) < 0.01
    fprintf('✅ Kp 验证通过，参数匹配正确\n');
else
    fprintf('⚠️  Kp 存在偏差，建议检查参数\n');
end

%% ===================== 4. Kr 下限计算（Tfo约束，公式8.31）=====================
Kr_Tfo = (Tfo_gain * fo - fc) * (2*pi*L_total) / (Hi2 * Kpwm);
fprintf('\n=== Kr 约束计算结果 ===\n');
fprintf('Kr 下限（Tfo=%ddB约束）：Kr >= %.2f\n', Tfo, Kr_Tfo);

%% ===================== 5. Kr 上限计算（PM约束，公式8.33 1:1匹配）=====================
% 公共项预计算
arg1 = 3*pi*fc*Ts;
sin_arg1 = sin(arg1);
cos_arg1 = cos(arg1);
arg2 = arg1 + PM_rad;
tan_arg2 = tan(arg2);

% 核心中间项 term_A
term_A = (2*pi*(fr^2 - fc^2)*L1) / (fc * Kpwm * Hi1) + sin_arg1;

% 分子与分母
numerator_part = term_A - tan_arg2 * cos_arg1;
denominator_part = term_A * tan_arg2 + cos_arg1;

% 前置系数
coeff_front = (pi * fc^2 * L_total) / (Kpwm * Hi2 * fi);

% 最终Kr上限
Kr_PM = coeff_front * (numerator_part / denominator_part);
fprintf('Kr 上限（PM=%d°约束）：Kr <= %.2f\n', PM, Kr_PM);

%% ===================== 6. 最终结论与推荐值 =====================
fprintf('\n=== 最终结论 ===\n');
if Kr_PM > Kr_Tfo
    fprintf('✅ 存在可行的 Kr 取值区间：%.2f <= Kr <= %.2f\n', Kr_Tfo, Kr_PM);
    Kr_recommend = (Kr_Tfo + Kr_PM) / 2;
    fprintf('推荐取值：Kr = %.2f（取中间值，兼顾稳态误差与稳定性）\n', Kr_recommend);
    
    fprintf('\n=== 完整推荐参数汇总 ===\n');
    fprintf('电流环截止频率 fc = %d Hz\n', fc);
    fprintf('电容电流反馈系数 Hi1 = %.2f\n', Hi1);
    fprintf('比例系数 Kp = %.3f\n', Kp_input);
    fprintf('PR谐振系数 Kr = %.2f\n', Kr_recommend);
else
    fprintf('⚠️  警告：当前参数下无可行的 Kr 取值区间！\n');
    fprintf('原因：Kr 上限(%.2f) < Kr 下限(%.2f)\n', Kr_PM, Kr_Tfo);
end