% SinglePhase_OffGrid_ClosedLoop_Discrete.m
% 功能：闭环传递函数双线性(Tustin)离散化，开环/闭环伯德图分窗口绘制，输出完整稳定指标
% 离散化方法：双线性变换(Tustin)
% 适配硬件：L=1.2mH, C=10uF, kpwm=45

clc; clear; close all;

%% ===================== 1. 系统核心参数定义 =====================
% --- 主电路参数（严格匹配论文符号）---
L = 1.5e-3;               % 滤波电感 1.2mH
C = 10e-6;                % 滤波电容 10uF
R = 0.1;                  % 电感寄生电阻 0.1Ω
Kpwm = 45;                % PWM桥臂增益 k_pwm
fs = 10e3;                % 采样/开关频率 10kHz
Ts = 1/fs;                % 采样周期

% --- 控制参数 ---
G_i = tf(0.4);            % 电流环控制器 G_i(s)：纯比例
Kp_v = 0.16;              % 电压环PI比例系数
Ki_v = 450;               % 电压环PI积分系数

%% ===================== 2. 连续域模型与指标计算 =====================
s = tf('s');

% --- 论文公式(3-7)：闭环传递函数 G_t2(s) = v_o(s)/v_ref(s) ---
Z_L = s*L + R;             % 电感复阻抗 Z_L(s)
Y_C = s*C;                 % 电容复导纳 Y_C(s)
G_v = Kp_v + Ki_v / s;     % 电压环PI控制器 G_v(s)

Numerator_cont = G_i * G_v * Kpwm;
Denominator_cont = 1 + Z_L*Y_C + G_i * Kpwm * (G_v + Y_C);
G_t2_cont = minreal(Numerator_cont / Denominator_cont); % 连续域闭环传递函数

% --- 连续域开环传递函数（用于稳定性分析，核心！）---
G_open_cont = minreal(G_t2_cont / (1 - G_t2_cont));

% --- 连续域稳定性指标计算 ---
[GM_cont, PM_cont, Wcg_cont, Wcp_cont] = margin(G_open_cont);
GM_cont_dB = 20*log10(GM_cont);
Wcp_cont_Hz = Wcp_cont / (2*pi);
Wcg_cont_Hz = Wcg_cont / (2*pi);

% --- 命令行输出连续域信息 ---
fprintf('==================== 连续域模型信息 ====================\n');
fprintf('=== 连续域闭环传递函数 G_t2(s) ===\n');
disp(G_t2_cont);
fprintf('\n=== 连续域开环传递函数 G_open(s)（稳定性分析用）===\n');
disp(G_open_cont);
fprintf('\n=== 连续域开环稳定性指标 ===\n');
fprintf('截止频率 fc = %.2f Hz\n', Wcp_cont_Hz);
fprintf('相位裕度 PM = %.2f °\n', PM_cont);
fprintf('相位穿越频率 fcg = %.2f Hz\n', Wcg_cont_Hz);
fprintf('幅值裕度 GM = %.2f dB\n', GM_cont_dB);
fprintf('=========================================================\n\n');

%% ===================== 3. 双线性(Tustin)离散化 =====================
% 闭环传递函数双线性离散化
G_t2_disc = minreal(c2d(G_t2_cont, Ts, 'tustin'));
% 开环传递函数同步双线性离散化（与闭环离散化方法统一）
G_open_disc = minreal(c2d(G_open_cont, Ts, 'tustin'));

% --- 提取离散化后传递函数的分子分母系数 ---
[num_d_closed, den_d_closed] = tfdata(G_t2_disc, 'v'); % 闭环系数
[num_d_open, den_d_open] = tfdata(G_open_disc, 'v');   % 开环系数

% --- 离散域稳定性指标计算 ---
[GM_disc, PM_disc, Wcg_disc, Wcp_disc] = margin(G_open_disc);
GM_disc_dB = 20*log10(GM_disc);
Wcp_disc_Hz = Wcp_disc / (2*pi);
Wcg_disc_Hz = Wcg_disc / (2*pi);

% --- 命令行输出离散域信息 ---
fprintf('==================== 离散域模型信息 ====================\n');
fprintf('=== 离散域闭环传递函数 G_t2(z)（双线性Tustin离散化） ===\n');
disp(G_t2_disc);
fprintf('\n=== 离散域开环传递函数 G_open(z)（稳定性分析用）===\n');
disp(G_open_disc);
fprintf('\n=== 离散化后传递函数系数 ===\n');
fprintf('闭环分子（z降幂）：['); fprintf('%.6f ', num_d_closed); fprintf(']\n');
fprintf('闭环分母（z降幂）：['); fprintf('%.6f ', den_d_closed); fprintf(']\n');
fprintf('开环分子（z降幂）：['); fprintf('%.6f ', num_d_open); fprintf(']\n');
fprintf('开环分母（z降幂）：['); fprintf('%.6f ', den_d_open); fprintf(']\n');
fprintf('\n=== 离散域开环稳定性指标 ===\n');
fprintf('截止频率 fc = %.2f Hz\n', Wcp_disc_Hz);
fprintf('相位裕度 PM = %.2f °\n', PM_disc);
fprintf('相位穿越频率 fcg = %.2f Hz\n', Wcg_disc_Hz);
fprintf('幅值裕度 GM = %.2f dB\n', GM_disc_dB);
fprintf('=========================================================\n');

%% ===================== 4. 伯德图绘制（分两个独立窗口） =====================
% === 窗口1：开环伯德图（核心！用于稳定性分析）===
figure(1); % 独立窗口1
set(figure(1), 'Color','white','Position',[100,100,1200,700]);
opts_open = bodeoptions;
opts_open.FreqUnits = 'Hz';
opts_open.MagUnits = 'dB';
opts_open.PhaseUnits = 'deg';
opts_open.Grid = 'on';
opts_open.XLim = [10, fs/2]; % 绘制到奈奎斯特频率的一半
opts_open.PhaseWrapping = 'on';
opts_open.PhaseWrappingBranch = -180;

h_open = bodeplot(G_open_cont, G_open_disc, opts_open);
title('连续域 vs 离散域 开环传递函数伯德图（稳定性分析核心）','FontSize',12);

% 修正开环伯德图纵坐标范围
ax_open = getaxes(h_open);
ax_open(1).YLim = [-60, 40];  % 开环幅频：覆盖增益裕度范围
ax_open(2).YLim = [-270, 90]; % 开环相频：显示完整相位裕度

% 开环图例
legend(ax_open(1), '连续域开环 G_open(s)','离散域开环 G_open(z)','Location','best','FontSize',10);

% === 窗口2：闭环伯德图（用于动态特性分析）===
figure(2); % 独立窗口2
set(figure(2), 'Color','white','Position',[200,200,1200,700]);
opts_closed = bodeoptions;
opts_closed.FreqUnits = 'Hz';
opts_closed.MagUnits = 'dB';
opts_closed.PhaseUnits = 'deg';
opts_closed.Grid = 'on';
opts_closed.XLim = [10, fs/2];
opts_closed.PhaseWrapping = 'on';
opts_closed.PhaseWrappingBranch = -180;

h_closed = bodeplot(G_t2_cont, G_t2_disc, opts_closed);
title('连续域 vs 离散域 闭环传递函数伯德图（动态特性分析）','FontSize',12);

% 修正闭环伯德图纵坐标范围
ax_closed = getaxes(h_closed);
ax_closed(1).YLim = [-60, 10];  % 闭环幅频：-60dB ~ 10dB（覆盖闭环增益特性）
ax_closed(2).YLim = [-180, 0];  % 闭环相频：-180° ~ 0°（符合闭环系统相位特性）

% 闭环图例
legend(ax_closed(1), '连续域闭环 G_t2(s)','离散域闭环 G_t2(z)','Location','best','FontSize',10);