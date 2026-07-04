%% 单相LCL整流器 电流环 QPR控制 开/闭环伯德图（横坐标：Hz）
%% 参数：Lg=120uH, Cf=10uF, L=1.5mH, 内阻0.1Ω, 电网24V, Udc=40V
%% QPR：wc=π, w0=100π(50Hz)
clear; clc; close all;

%% ===================== 1. 系统参数 =====================
Lg      = 120e-6;      % 网侧电感
L       = 1.5e-3;      % 变换器侧电感
Cf      = 10e-6;       % 滤波电容
rg      = 0.1;         % 寄生电阻
r       = 0.1;
Udc     = 40;          % 直流侧目标值 40V

% QPR控制器参数
wc      = 15;
w0      = 100*pi;
Kp      = 0.6;
Kr      = 55;

%% ===================== 2. 被控对象 G_i(s) = i/m =====================
num_Gi = [0,  -Udc*Lg*Cf,  0,  -Udc ];
den_Gi = [ Lg*Cf*L, L*Cf*rg+Lg*Cf*r+L, Lg+L+r*Lg+rg*L, r+rg ];
G_i = tf(num_Gi, den_Gi);

%% ===================== 3. QPR控制器 =====================
num_PR = [ Kp, 2*Kp*wc + 2*Kr*wc, Kp*w0^2 ];
den_PR = [ 1, 2*wc, w0^2 ];
G_PR = tf(num_PR, den_PR);

%% ===================== 4. 开环传递函数 =====================
G_open = G_PR * G_i;

%% ===================== 5. 伯德图配置（横坐标改为 Hz）=====================
opts = bodeoptions;
opts.FreqUnits = 'Hz';   % ✅ 核心：将频率单位从 rad/s 改为 Hz
opts.Grid = 'on';        % 开启网格

%% ===================== 6. 开环伯德图 =====================
figure(1);
bode(G_open, opts);
title('电流环开环伯德图（横坐标：Hz）');

% 稳定性分析（频率单位转换为 Hz）
[Gm, Pm, Wcg, Wcp] = margin(G_open);
fprintf('========== 稳定性分析 ==========\n');
fprintf('相位裕度 = %.2f °\n', Pm);
fprintf('截止频率 = %.2f Hz\n', Wcp/(2*pi));  % Wcp 是 rad/s，除以 2π 转 Hz

%% ===================== 7. 闭环传递函数 + 伯德图 =====================
G_closed = feedback(G_open, 1);

figure(2);
bode(G_closed, opts);
title('电流环闭环伯德图（横坐标：Hz）');

%% ===================== 8. 输出传递函数 =====================
fprintf('\n被控对象 G_i(s):\n'); disp(G_i);
fprintf('QPR控制器 G_PR(s):\n'); disp(G_PR);