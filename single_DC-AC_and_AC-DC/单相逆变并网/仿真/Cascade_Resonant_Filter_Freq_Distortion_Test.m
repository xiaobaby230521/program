% 文件名：Cascade_Resonant_Filter_Freq_Distortion_Test.m
%《级联谐振滤波器频率畸变测试.m》
% 【模块核心功能】
% 1. 测试场景构建：生成含频率畸变的电网电压信号（0.2s基波从50Hz阶跃至45Hz），叠加5V直流偏置+150/250Hz谐波（3/5次）；
% 2. 滤波器设计：基于二阶谐振滤波器设计固定频率（150/250Hz）陷波滤波器，目标是滤除电网中的3/5次谐波；
% 3. 离散化实现：采用Tustin（双线性变换）法将连续域传递函数离散化，适配10kHz采样率；
% 4. 级联滤波：将3次（150Hz）和5次（250Hz）谐振滤波器级联，对含畸变的输入信号进行谐波滤除；
% 5. 结果验证：对比滤波前后的波形，验证滤波器在基波频率畸变时仍能稳定滤除固定频率谐波的能力，并且保留基波的直流分量
% 【关键参数说明】
% - 采样频率Fs=10kHz：兼顾滤波精度与计算效率，适配单片机工程落地；
% - 截止角频率wc=25rad/s：由响应时间ts=0.12s推导，控制滤波器动态响应速度；
% - 陷波频率150/250Hz：固定针对电网3/5次谐波，不受基波频率（50→45Hz）变化影响；
% - Tustin离散化：保证离散域滤波器频率特性与连续域一致，无频率混叠。
% 【适用场景】
% 适用于电网谐波治理、新能源并网等场景中，对固定频率谐波（3/5次）的滤除验证，尤其适配基波频率波动的工况。

clear; clc; close all;

%% ===================== 1. 仿真参数 =====================
Fs = 1e5; % 采样频率 100kHz
Ts = 1/Fs; % 采样周期
T_sim = 0.4; % 仿真时长 0.4s
t = 0:Ts:T_sim-Ts; % 时间轴、
len = length(t); % 总采样点数
mutate_idx = round(len/2); % 0.2s频率畸变时刻

% 电网参数
U_rms = 24; % 基波有效值 24V
A_fund = U_rms*sqrt(2); % 基波幅值 ≈33.94V
DC_offset = 5; % 5V直流偏置
A_3h = 0.1*A_fund; % 3次谐波幅值（基波的10%）
A_5h = 0.1*A_fund; % 5次谐波幅值（基波的10%）

% 频率参数
f0_nominal = 50; % 标称基波频率（滤波器锁定）
w0_nominal = 2*pi*f0_nominal;
w3 = 3*w0_nominal; % 150Hz（3次谐波，固定）
w5 = 5*w0_nominal; % 250Hz（5次谐波，固定）

f0_distort = 45; % 畸变后的基波频率
w0_distort = 2*pi*f0_distort;

% 滤波器核心参数（论文标准）
ts = 0.12; % 响应时间
wc = 3 / ts; % 截止角频率 = 25 rad/s

%% ===================== 2. 生成带频率畸变的测试信号 =====================
Uin_raw = zeros(1, len);
for i = 1:len
if i <= mutate_idx
% 前0.2s：50Hz基波 + 5V直流 + 150/250Hz谐波
Uin_raw(i) = DC_offset + A_fund*sin(w0_nominal*t(i)) ...
+ A_3h*sin(w3*t(i)) + A_5h*sin(w5*t(i));
else
% 后0.2s：45Hz基波 + 5V直流 + 原150/250Hz谐波（相位连续）
phase_offset = w0_nominal*t(mutate_idx) - w0_distort*t(mutate_idx);
Uin_raw(i) = DC_offset + A_fund*sin(w0_distort*t(i)+phase_offset) ...
+ A_3h*sin(w3*t(i)) + A_5h*sin(w5*t(i));
end
end

%% ===================== 3. 使用c2d离散化传递函数 =====================
s = tf('s');

% 3次谐波滤波器 G3(s) = (s^2 + w3^2) / (s^2 + 2*wc*s + w3^2)
G3 = (s^2 + w3^2) / (s^2 + 2*wc*s + w3^2);
% 5次谐波滤波器 G5(s) = (s^2 + w5^2) / (s^2 + 2*wc*s + w5^2)
G5 = (s^2 + w5^2) / (s^2 + 2*wc*s + w5^2);

% 使用Tustin法离散化
G3_d = c2d(G3, Ts, 'tustin');
G5_d = c2d(G5, Ts, 'tustin');

% 提取离散滤波器系数
[b3, a3] = tfdata(G3_d, 'v');
[b5, a5] = tfdata(G5_d, 'v');

%% ===================== 4. 滤波计算 =====================
% 初始化滤波器状态
x3 = zeros(length(b3)-1, 1); % 3次滤波器状态
x5 = zeros(length(b5)-1, 1); % 5次滤波器状态

Uin_filtered = zeros(1, len);

for i = 1:len
% 3次谐波滤波
[y3, x3] = filter(b3, a3, Uin_raw(i), x3);
% 5次谐波滤波（级联）
[y5, x5] = filter(b5, a5, y3, x5);
% 保存输出
Uin_filtered(i) = y5;
end

%% ===================== 5. 结果绘图 =====================
figure('Color','white','Position',[100,100,1000,600]);

% 子图1：原始输入（带频率畸变）
subplot(2,1,1);
plot(t, Uin_raw, 'b', 'LineWidth',1.2); hold on;
plot([t(mutate_idx) t(mutate_idx)], ylim, 'k--', 'LineWidth',1);
plot(t, ones(1,len)*DC_offset, 'k:', 'LineWidth',1);
title('子图1：滤波器输入（50→45Hz基波+5V直流+150/250Hz谐波）','FontSize',12);
xlabel('时间 t (s)'); ylabel('幅值 (V)');
legend('原始输入','频率畸变时刻 (0.2s)','5V直流偏置','Location','best');
grid on; axis tight;

% 子图2：滤波输出（滤除150/250Hz谐波）
subplot(2,1,2);
plot(t, Uin_filtered, 'r', 'LineWidth',1.2); hold on;
plot([t(mutate_idx) t(mutate_idx)], ylim, 'k--', 'LineWidth',1);
plot(t, ones(1,len)*DC_offset, 'k:', 'LineWidth',1);
title('子图2：滤波器输出（滤除150/250Hz谐波，基波50→45Hz）','FontSize',12);
xlabel('时间 t (s)'); ylabel('幅值 (V)');
legend('滤波后输出','频率畸变时刻 (0.2s)','5V直流偏置','Location','best');
grid on; axis tight;