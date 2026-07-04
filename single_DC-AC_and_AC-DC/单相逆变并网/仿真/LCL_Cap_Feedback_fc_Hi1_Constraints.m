
% LCL_Cap_Feedback_fc_Hi1_Constraints.m
% 含有电容电流反馈有源阻尼LCL并网逆变器的pr控制器fc和Hi1参数约束曲线绘制
% 匹配你手绘的可行域版本：修正上下限约束逻辑
clc; clear; close all;

%% ===================== 1. 固定参数定义 =====================
% 稳定性指标
GM1 = -3;             % 单位：dB（上限约束）
GM2 = 3;              % 单位：dB（下限约束）
PM = 45;              % 相位裕度，单位：°（上限约束）
Tfo = 73;             % 低频增益，单位：dB（上限约束）

% LCL滤波器参数
L1 = 1.2e-3;          % 逆变侧电感，单位：H (1.2mH)
L2 = 120e-6;          % 网侧电感，单位：H (120uH)
C = 10e-6;            % 滤波电容，单位：F (10uF)

% 系统与控制参数
fs = 20000;           % 采样/开关频率，单位：Hz (20kHz)
Kpwm = 45;            % PWM桥臂增益
wi = pi;              % PR控制器谐振带宽，单位：rad/s
wo = 100*pi;          % 基波角频率，单位：rad/s

%% ===================== 2. 中间参数计算 =====================
Ts = 1/fs;                                % 采样周期，单位：s
fi = wi/(2*pi);                           % wi对应的频率，单位：Hz (0.5Hz)
fo = wo/(2*pi);                           % 基波频率，单位：Hz (50Hz)
% LCL滤波器谐振频率
L_parallel = (L1 * L2) / (L1 + L2);      % 两侧电感并联值
fr = 1/(2*pi * sqrt(L_parallel * C));     % 谐振频率，单位：Hz

% 增益换算（dB转线性值）
Tfo_gain = 10^(Tfo/20);
GM1_gain = 10^(GM1/20);
GM2_gain = 10^(GM2/20);
PM_rad = PM * pi / 180;                   % 相位裕度转弧度
fs_6 = fs / 6;                             % 公式中的fs/6

%% ===================== 3. 横坐标fc范围设置 =====================
fc = 500:1:2000;  % 电流环截止频率，单位：Hz
% 预分配数组
H_GM1 = zeros(size(fc));
H_GM2 = zeros(size(fc));
H_PM = zeros(size(fc));

%% ===================== 4. 循环计算每个fc对应的约束值 =====================
for i = 1:length(fc)
    fc_now = fc(i);
    
    % ---------------- 4.1 GM1上限约束：H_i1 ≤ H_GM1 ----------------
    H_GM1(i) = GM1_gain * 2*pi*fc_now*L1 / Kpwm;
    
    % ---------------- 4.2 GM2下限约束：H_i1 ≥ H_GM2 ----------------
    term1 = GM2_gain * (fr/fs_6)^2 * fc_now;
    term2 = (fs_6^2 - fr^2) / fs_6;
    H_GM2_calc = (2*pi*L1 / Kpwm) * (term1 + term2);
    H_GM2(i) = max(H_GM2_calc, 0);  % 下限不能小于0
    
    % ---------------- 4.3 PM/Tfo上限约束：H_i1 ≤ H_PM ----------------
    % 公共项预计算
    arg1 = 3*pi*fc_now*Ts;
    tan_arg1 = tan(arg1);
    arg2 = arg1 + PM_rad;
    tan_arg2 = tan(arg2);
    cos_arg1 = cos(arg1);
    fo_term = Tfo_gain * fo - fc_now;
    
    % 分子部分
    numerator_A = (2*pi*L1 * (fr^2 - fc_now^2)) / (fc_now * Kpwm * cos_arg1);
    numerator_B = pi*fc_now^2 - 2*pi*fi * fo_term * tan_arg2;
    numerator = numerator_A * numerator_B;
    
    % 分母部分
    denominator_A = 2*pi*fi * fo_term * (tan_arg2 * tan_arg1 + 1);
    denominator_B = pi*fc_now^2 * (tan_arg2 - tan_arg1);
    denominator = denominator_A + denominator_B;
    
    % 上限约束，负数值代表该频率无可行解，取0
    H_PM_calc = numerator / denominator;
    H_PM(i) = max(H_PM_calc, 0);
end

%% ===================== 5. 绘图（匹配你手绘的可行域）=====================
figure('Color','white','Position',[100,100,800,600]);
hold on; grid on; box on;

% 绘制三条约束曲线，线型和论文完全匹配
plot(fc, H_GM1, 'k--', 'LineWidth',1.5, 'DisplayName',['GM1=',num2str(GM1),'dB']);
plot(fc, H_GM2, 'k:', 'LineWidth',1.5, 'DisplayName',['GM2=',num2str(GM2),'dB']);
plot(fc, H_PM, 'k-', 'LineWidth',1.5, 'DisplayName',['Tfo=',num2str(Tfo),'dB, PM=',num2str(PM),'°']);

% ---------------- 核心修改：匹配你手绘的可行域 ----------------
% 约束逻辑：H_i1 必须同时满足
% 1. 大于等于下限 H_GM2
% 2. 小于等于上限 min(H_GM1, H_PM)
H_upper = min([H_GM1; H_PM], [], 1);  % 两个上限取最小值
H_lower = H_GM2;                        % 下限

% 筛选有效可行域：上界 > 下界的部分
valid_idx = H_upper > H_lower;
fc_valid = fc(valid_idx);
H_upper_valid = H_upper(valid_idx);
H_lower_valid = H_lower(valid_idx);

% 填充可行域（你画的红色区域）：上下边界之间的区域
fill([fc_valid, fliplr(fc_valid)], [H_lower_valid, fliplr(H_upper_valid)], ...
     [0.8 0.2 0.2], 'FaceAlpha', 0.4, 'EdgeColor', 'none', 'DisplayName','可行域');

% 坐标轴设置，和你的图完全匹配
ylim([0, 0.12]);  % 纵坐标范围适配你的图
xlim([500, 2000]);

% 坐标轴标签
xlabel('f_c / Hz', 'FontSize',12, 'FontName','宋体');
ylabel('H_{i1}', 'FontSize',12, 'FontName','宋体');
set(gca, 'XTick',500:500:2000);
set(gca, 'YTick',0:0.02:0.12);

% 图例与网格
legend('Location','best', 'FontSize',10);
set(gca, 'GridLineStyle',':', 'GridAlpha',0.7);