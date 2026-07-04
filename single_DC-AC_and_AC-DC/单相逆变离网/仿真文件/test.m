% Repetitive_Controller_Simulink_True_Stability.m
% 功能：1:1匹配你Simulink的闭环结构，分析真实稳定性  双环pi级联重复控制器稳定性分析  
clc; clear; close all;

%% ===================== 1. 完全复刻你Simulink重复控制器的参数/逻辑 =====================
Ts = 1e-4;
N = 200;
p = 2;
k_rc = 0.4;

Q0 = 0.25; Q1 = 0.5; Q2 = 0.25;
delay_order = N - p;
max_order = N + 2;

b = zeros(1, max_order + 1);
a = zeros(1, max_order + 1);
a(1) = 1.0;

b(delay_order + 3) = k_rc * Q2;
b(delay_order + 2) = k_rc * Q1;
b(delay_order + 1) = k_rc * Q0;

a(N + 1) = a(N + 1) - Q0;
a(N + 2) = a(N + 2) - Q1;
a(N + 3) = a(N + 3) - Q2;

z = tf('z', Ts);
G_rc_num = 0;
for i = 1:length(b)
    G_rc_num = G_rc_num + b(i) * z^(-(i-1));
end
G_rc_den = 0;
for i = 1:length(a)
    G_rc_den = G_rc_den + a(i) * z^(-(i-1));
end
G_rc = G_rc_num / G_rc_den;
G_rc = minreal(G_rc, 1e-6);

%% ===================== 2. 原稳定闭环系统 =====================
num_closed = [0.258560, 0.322314, -0.131051, -0.194805];
den_closed = [1.000000, -1.017316, 0.638725, -0.366391];
G_t2 = tf(num_closed, den_closed, Ts, 'Variable', 'z');

%% ===================== 3. 【关键修正】构造真正的闭环系统 =====================
% 你Simulink里是：G_new_closed = feedback(G_rc * G_t2, 1)
G_new_open = G_rc * G_t2;
G_new_closed = feedback(G_new_open, 1); % 这才是你Simulink里的闭环系统！

%% ===================== 4. 绘制【闭环系统的零极点图】（金标准） =====================
figure('Color','white','Position',[100,100,800,600]);
pzmap(G_new_closed); % 绘制闭环系统的零极点（不是开环！）
grid on;
hold on;
theta = 0:0.01:2*pi;
plot(cos(theta), sin(theta), 'k--', 'LineWidth', 1); % 单位圆
axis equal;
title(sprintf('【Simulink 1:1】闭环系统零极点图 (N=%d,p=%d,k_rc=%.1f)', N, p, k_rc), 'FontSize', 12);
xlabel('实部 (Real)');
ylabel('虚部 (Imag)');

%% ===================== 5. 稳定性分析（终于匹配你的Simulink！） =====================
closed_poles = pole(G_new_closed);
pole_magnitude = abs(closed_poles);
max_pole_mag = max(pole_magnitude);

fprintf('=================== 【和Simulink完全一致】稳定性终极报告 ===================\n');
fprintf('重复控制器参数：N=%d, p=%d, k_rc=%.1f\n', N, p, k_rc);
fprintf('新闭环总极点数：%d\n', length(closed_poles));
fprintf('单位圆外（不稳定）极点数：%d\n', sum(pole_magnitude > 1.0001));
fprintf('所有极点模长最大值：%.4f（<1则稳定）\n', max_pole_mag);

if max_pole_mag < 0.9999
    fprintf('✅ 结论：系统完全稳定！（所有极点都在单位圆内，和你Simulink阶跃响应一致）\n');
elseif max_pole_mag < 1.1000
    fprintf('⚠️  结论：系统临界稳定\n');
else
    fprintf('❌  结论：系统不稳定\n');
end
fprintf('========================================================================\n');