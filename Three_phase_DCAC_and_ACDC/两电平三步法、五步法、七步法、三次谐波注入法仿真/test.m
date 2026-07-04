%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 文件名: svpwm_ref_params.m
% 功能: 两电平七段式SVPWM开环参考电压参数精确计算
% 输入: 直流母线电压Udc, 目标线电压有效值V_line_rms
% 输出: αβ轴参考电压参数表格 + Simulink可用全局变量
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc; close all;

%% ===================== 1. 核心参数定义 =====================
Udc = 60.0;                  % 直流母线电压(V)
V_line_rms_target = 32.0;    % 目标线电压有效值(V)
f_base = 50.0;               % 基波频率(Hz)
f_int = 20000.0;             % 中断频率(Hz)
T_s = 1/f_int;               % 采样时间(s)

%% ===================== 2. 数学公式推导 =====================
% 两电平SVPWM电压关系:
% 线电压有效值 V_line_rms = (√3/√2) * V_ref_peak
% 相电压峰值 V_ref_peak = (V_line_rms * √2)/√3
% 调制比 M = V_ref_peak / (Udc/2)
% 最大线性调制比 M_max = 2/√3 ≈ 1.1547
% 最大线性线电压有效值 V_line_max = (Udc/√3) ≈ 34.64V

%% ===================== 3. 目标参数计算 =====================
V_ref_peak = (V_line_rms_target * sqrt(2)) / sqrt(3);
M = V_ref_peak / (Udc/2);
V_line_max = Udc / sqrt(3);
M_max = 2/sqrt(3);

%% ===================== 4. 生成参数对比表格 =====================
% 常用输出电压对应的参数表
V_line_list = [24, 28, 30, 32, 34, V_line_max];
V_ref_list = (V_line_list * sqrt(2)) / sqrt(3);
M_list = V_ref_list / (Udc/2);

% 构建表格
param_table = table(...
    V_line_list', ...
    round(V_ref_list, 4)', ...
    round(M_list, 4)', ...
    'VariableNames', {'线电压有效值(V)', '相电压峰值(V)', '调制比M'});

%% ===================== 5. 输出结果 =====================
fprintf('=========================================\n');
fprintf('两电平SVPWM参考电压参数计算结果\n');
fprintf('=========================================\n');
fprintf('直流母线电压 Udc = %.1f V\n', Udc);
fprintf('最大线性线电压有效值 = %.2f V\n', V_line_max);
fprintf('最大线性调制比 M_max = %.4f\n\n', M_max);

fprintf('目标输出参数:\n');
fprintf('  线电压有效值: %.1f V\n', V_line_rms_target);
fprintf('  相电压峰值: %.4f V\n', V_ref_peak);
fprintf('  调制比: %.4f\n\n', M);

fprintf('常用输出电压参数对照表:\n');
disp(param_table);

%% ===================== 6. 生成Simulink可用变量 =====================
% 将参数导出到工作区，Simulink模块可直接引用
assignin('base', 'Udc', Udc);
assignin('base', 'V_line_rms_target', V_line_rms_target);
assignin('base', 'V_ref_peak', V_ref_peak);
assignin('base', 'M', M);
assignin('base', 'f_base', f_base);
assignin('base', 'T_s', T_s);

fprintf('\n=========================================\n');
fprintf('参数已成功导入Simulink工作区!\n');
fprintf('Simulink Sine Wave模块参数设置:\n');
fprintf('  V_alpha: 幅值=V_ref_peak, 频率=f_base, 相位=0, 采样时间=T_s\n');
fprintf('  V_beta:  幅值=V_ref_peak, 频率=f_base, 相位=90, 采样时间=T_s\n');
fprintf('=========================================\n');

%% ===================== 7. 绘制参考电压波形(可选) =====================
t = 0:T_s:0.04; % 绘制2个周期
V_alpha = V_ref_peak * sin(2*pi*f_base*t);
V_beta = V_ref_peak * cos(2*pi*f_base*t);

figure('Name', 'αβ轴参考电压波形');
plot(t, V_alpha, 'b-', 'LineWidth', 1.5);
hold on;
plot(t, V_beta, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('时间 (s)');
ylabel('电压 (V)');
title('SVPWM αβ轴参考电压波形');
legend('V_\alpha', 'V_\beta');
ylim([-30 30]);