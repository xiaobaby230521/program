%% ========================================================================
% 文件名：SinglePhase_Inverter_DualLoop_Bode_Compare.m
% 功能：单相离网逆变器 电压电流双环控制 开环/闭环传递函数伯德图对比分析
% 核心对比：无系统延时  VS  1.5Ts数字控制延时（ZOH+计算延时等效）
% 绘图设置：横坐标单位Hz、相位强制修正为-90°起始、双窗口独立显示
% 适用场景：LC滤波逆变器双环控制稳定性分析、延时对系统特性影响研究
%注意：开环传递函数的谐振峰是二阶pade逼近的结果无需理会，实际硬件不会产生谐振峰。
% 作者：工程仿真 | 日期：2025
% ========================================================================
clc; clear; close all;
s = tf('s'); 

%% 1. 逆变器主电路参数（LC滤波+逆变桥增益）
L = 1.5e-3;               % 滤波电感 (H)
C = 10e-6;                % 滤波电容 (F)
R_L = 0.1;                % 电感寄生电阻 (Ω)
Kpwm = 45;                % 逆变桥等效增益

%% 2. 数字控制参数
fs = 10e3;                % 开关/采样频率 (Hz)
Ts = 1/fs;                % 采样周期 (s)
tau = 1.5*Ts;             % 总控制延时：1.5Ts (ZOH 0.5Ts + 刷新1Ts)

%% 3. 双环控制器参数（无延时 / 带延时两组参数）
% 工况1：无系统延时 控制器参数
Kp_i1=0.4; Kp_v1=0.16; Ki_v1=450; k1=0.0;
% 工况2：1.5Ts系统延时 控制器参数
Kp_i2=0.05; Kp_v2=0.05; Ki_v2=2200; k2=0;

Z_L = s*L + R_L;          % 电感阻抗

%% 4. 延时环节：二阶Pade近似（1.5Ts延时等效传递函数）
G_delay = (tau^2*s^2 - 6*tau*s + 12) / (tau^2*s^2 + 6*tau*s + 12);

%% ===================== 开环传递函数计算 =====================
% 工况1：无延时 系统开环传递函数
Gi1=Kp_i1; Gv1=Kp_v1+Ki_v1/s; Yc1=s*C+k1;
Gopen1 = minreal( (Gi1*Gv1*Kpwm) / (1 + Z_L*Yc1 + Gi1*Kpwm*Yc1) );

% 工况2：1.5Ts延时 系统开环传递函数
Gi2=Kp_i2; Gv2=Kp_v2+Ki_v2/s; Yc2=s*C+k2;
Gopen2 = minreal( (Gi2*Gv2*Kpwm*G_delay) / (1 + Z_L*Yc2 + Gi2*Kpwm*G_delay*Yc2) );

%% ===================== 闭环传递函数计算（单位负反馈） =====================
Gcl1 = minreal(Gopen1/(1+Gopen1));  % 无延时 闭环传递函数
Gcl2 = minreal(Gopen2/(1+Gopen2));  % 1.5Ts延时 闭环传递函数

%% ===================== 稳定裕度分析（幅值穿越频率+相位裕度+幅值裕度） =====================
[GM1,PM1,~,Wc1] = margin(Gopen1); fc1=Wc1/(2*pi); GM1_dB = 20*log10(GM1);
[GM2,PM2,~,Wc2] = margin(Gopen2); fc2=Wc2/(2*pi); GM2_dB = 20*log10(GM2);

% 命令窗输出分析结果
fprintf('==================== 开环稳定裕度分析结果 ====================\n');
fprintf('无延时系统：\n');
fprintf('   截止频率 fc = %.2f Hz\n',fc1);
fprintf('   相位裕度 PM = %.2f °\n',PM1);
fprintf('   幅值裕度 GM = %.2f dB\n',GM1_dB);
fprintf('\n');
fprintf('1.5Ts延时系统：\n');
fprintf('   截止频率 fc = %.2f Hz\n',fc2);
fprintf('   相位裕度 PM = %.2f °\n',PM2);
fprintf('   幅值裕度 GM = %.2f dB\n',GM2_dB);
fprintf('=================================================================\n');

%% ===================== 频率轴设置（横坐标：Hz） =====================
f = logspace(0,5,1000);    % 频率范围：1Hz~100kHz
w = 2*pi*f;               % 转换为角频率(rad/s)，用于bode计算

%% ===================== 窗口1：开环传递函数伯德图（核心对比） =====================
figure('Name','开环伯德图对比_有无延时','Position',[100,100,1200,700]);

% 获取幅值/相位原始数据
[mag1, phase1, ~] = bode(Gopen1, w);
[mag2, phase2, ~] = bode(Gopen2, w);
mag1 = squeeze(mag1); phase1 = squeeze(phase1);
mag2 = squeeze(mag2); phase2 = squeeze(phase2);

% 相位解包裹 + 强制修正起始相位为-90°
phase1_unwrap = unwrap(phase1*pi/180)*180/pi;
phase2_unwrap = unwrap(phase2*pi/180)*180/pi;
phase2_offset = phase2_unwrap - (phase2_unwrap(1) + 90); 

% 幅频特性
subplot(2,1,1);
semilogx(f, 20*log10(mag1), 'b-', f, 20*log10(mag2), 'r-', 'LineWidth',1.2);
grid on; legend('无延时','1.5Ts延时','Location','best');
title('开环传递函数伯德图（单相离网逆变器双环控制）'); ylabel('幅值 (dB)');

% 相频特性（锁定-180°~0°，消除相位包裹）
subplot(2,1,2);
semilogx(f, phase1_unwrap, 'b-', f, phase2_offset, 'r-', 'LineWidth',1.2);
grid on; legend('无延时','1.5Ts延时','Location','best');
ylabel('相位 (deg)'); xlabel('频率 (Hz)');
ylim([-180, 0]); 

%% ===================== 窗口2：闭环传递函数伯德图 =====================
figure('Name','闭环伯德图对比_有无延时','Position',[100,200,1200,700]);
bode(Gcl1, Gcl2, w); grid on;
legend('无延时-闭环','1.5Ts延时-闭环','Location','best');
title('闭环传递函数伯德图（单相离网逆变器双环控制）');
xlabel('频率 (Hz)');

% 闭环相位范围约束
ax_phase = findall(gcf,'Type','axes','Tag','Phase');
axes(ax_phase); ylim([-180, 0]);