% SOGI-FLL 离散化仿真（修正版）：5V直流偏置 + 50Hz→45Hz频率畸变
% 【模块功能介绍】
% 本模块实现二阶广义积分器-频率锁定环（SOGI-FLL）的离散化仿真，核心功能如下：
% 1. 输入处理：生成含5V直流偏置的电网电压信号，0.2s时刻频率从50Hz阶跃至45Hz；
% 2. 核心算法：基于前向欧拉法离散化SOGI-FLL连续域微分方程，实现直流分量滤除+频率自适应跟踪；
% 3. SOGI功能：通过二阶广义积分器分离输入电压的基波同相/正交分量，滤除直流偏置；
% 4. FLL功能：通过频率锁定环实时跟踪电网频率变化，0.2s频率阶跃后快速收敛至45Hz；
% 5. 性能评估：计算频率跟踪静态误差、波动RMS、输出幅值精度等关键指标，量化算法性能；
% 6. 可视化：绘制输入/输出电压波形、频率跟踪曲线，直观展示算法效果。
% 【核心参数说明】
% - SOGI增益K=1.414：保证SOGI谐振特性，实现基波分量提取；
% - 直流消除器增益K1=0.25：增强直流分量抑制能力；
% - FLL增益gamma=20：调节频率跟踪动态响应速度；
% - 采样率100kHz（dt=1e-5s）：兼顾仿真精度与计算效率。
clear; clc; close all;

%% ===================== 1. 离散化仿真参数 =====================
dt = 1e-5; % 离散化步长（100kHz采样率）
t_end = 0.4; % 总仿真时长
t = 0:dt:t_end; % 离散时间序列
N = length(t); % 采样点数
mutate_t = 0.2; % 频率畸变时刻

% 电网参数：5V直流偏置 + 24V有效值正弦波（50Hz→45Hz畸变）
U_dc = 5; % 直流偏置 5V
U_rms = 24; 
A = U_rms * sqrt(2);% 正弦波峰值≈33.94V
f1 = 50; f2 = 45; 
omega1 = 2*pi*f1; omega2 = 2*pi*f2;

% 生成离散化输入电压序列U(k)
U = zeros(N, 1);
for k = 1:N
t_now = t(k);
if t_now <= mutate_t
U(k) = U_dc + A * sin(omega1 * t_now);
else
U(k) = U_dc + A * sin(omega2 * t_now + (omega1-omega2)*mutate_t);
end
end

% 算法固定参数
K = 1.414; % SOGI增益
K1 = 0.25; % 直流消除器增益
gamma = 20; % FLL增益

%% ===================== 2. 离散化状态初始化 =====================
x1 = zeros(N, 1);
x2 = zeros(N, 1);
x3 = zeros(N, 1);
omega0 = zeros(N, 1);
omega0(1) = 2*pi*50; % 初始角频率（50Hz）

%% ===================== 3. 主循环（前向欧拉法离散化） =====================
for k = 2:N
% 1. 获取上一时刻状态和当前输入
x1_prev = x1(k-1);
x2_prev = x2(k-1);
x3_prev = x3(k-1);
omega0_prev = omega0(k-1);

U_curr = U(k);
% 2. 计算上一时刻的状态导数（严格匹配连续域ODE）
%    论文式(12)：ẋ = A·x + B·U
%    其中：
%         [ -K₁ω₀  -K₁ω₀   0     ]
%    A =  [ -Kω₀   -Kω₀   -ω₀²   ]
%         [  0      1      0     ]
%
%         [ K₁ω₀ ]
%    B =  [ Kω₀  ]
%         [  0   ]
A_mat = [
    -K1*omega0_prev, -K1*omega0_prev, 0;
    -K*omega0_prev,  -K*omega0_prev,  -omega0_prev^2;
    0,                1,                0
];
B_mat = [
    K1*omega0_prev;
    K*omega0_prev;
    0
];

% 按行计算状态导数：ẋ₁, ẋ₂, ẋ₃
% dx₁/dt = A(1,:)·[x₁;x₂;x₃] + B(1)·U
dx1dt_prev = A_mat(1,:)*[x1_prev; x2_prev; x3_prev] + B_mat(1)*U_curr;
% dx₂/dt = A(2,:)·[x₁;x₂;x₃] + B(2)·U
dx2dt_prev = A_mat(2,:)*[x1_prev; x2_prev; x3_prev] + B_mat(2)*U_curr;
% dx₃/dt = A(3,:)·[x₁;x₂;x₃] + B(3)·U
dx3dt_prev = A_mat(3,:)*[x1_prev; x2_prev; x3_prev] + B_mat(3)*U_curr;

% 3. 前向欧拉法更新SOGI状态
x1(k) = x1_prev + dt * dx1dt_prev;
x2(k) = x2_prev + dt * dx2dt_prev;
x3(k) = x3_prev + dt * dx3dt_prev;

% 4. 计算频率误差和更新（前向欧拉法）
deltaU = U_curr - x1(k) - x2(k);   %输出误差Eu=Uin-X1-X2
deltaU_prime = omega0_prev * x3(k);   %输出Uβ=w0*x3
epsilon_f = deltaU * deltaU_prime;     %频率误差Ef=uβ*Eu

domega0dt_prev = -gamma * epsilon_f;    %频率状态导数更新
omega0(k) = omega0_prev + dt * domega0dt_prev;    %w0=w0（k-1）+dw0/dt

% 频率限幅
w_max = 2*pi*60; w_min = 2*pi*40;
omega0(k) = max(min(omega0(k), w_max), w_min);
end

%% ===================== 4. 结果数据处理 =====================
U_prime = x2; % 同相输出 U' = x2（无直流）
deltaU_prime = omega0 .* x3; % 正交输出 ΔU' = ω₀·x3（无直流）
f_est = omega0 ./ (2*pi); % 估计频率（Hz）
f_grid = (t <= mutate_t).*50 + (t > mutate_t).*45; % 电网实际频率

%% ===================== 5. 品质指标计算 =====================
% ---------------------- 5.1 频率跟踪品质 ----------------------
% 划分稳态区间（避开暂态：0.1~0.2s为50Hz稳态，0.3~0.4s为45Hz稳态）
idx_50Hz = find(t >= 0.1 & t <= 0.2); % 50Hz稳态区间
idx_45Hz = find(t >= 0.3 & t <= 0.4); % 45Hz稳态区间

% 50Hz区间频率指标
f_est_50 = f_est(idx_50Hz);
f_mean_50 = mean(f_est_50); % 50Hz区间平均估计频率
f_error_50 = abs(f_mean_50 - 50); % 50Hz区间频率静态误差
f_rms_50 = rms(f_est_50 - 50); % 50Hz区间频率波动RMS
f_max_50 = max(f_est_50); % 50Hz区间最大频率
f_min_50 = min(f_est_50); % 50Hz区间最小频率

% 45Hz区间频率指标
f_est_45 = f_est(idx_45Hz);
f_mean_45 = mean(f_est_45); % 45Hz区间平均估计频率
f_error_45 = abs(f_mean_45 - 45); % 45Hz区间频率静态误差
f_rms_45 = rms(f_est_45 - 45); % 45Hz区间频率波动RMS
f_max_45 = max(f_est_45); % 45Hz区间最大频率
f_min_45 = min(f_est_45); % 45Hz区间最小频率

% ---------------------- 5.2 输出波形幅值品质 ----------------------
% 50Hz区间幅值指标
U_prime_50 = U_prime(idx_50Hz);
U_rms_50 = rms(U_prime_50); % 同相输出有效值
U_peak_50 = max(abs(U_prime_50)); % 同相输出峰值
U_error_rms_50 = abs(U_rms_50 - 24); % 有效值偏差（理论24V）
U_thd_50 = 0; % 简化：若需计算THD需FFT，此处先设为0（可扩展）

% 45Hz区间幅值指标
U_prime_45 = U_prime(idx_45Hz);
U_rms_45 = rms(U_prime_45); % 同相输出有效值
U_peak_45 = max(abs(U_prime_45)); % 同相输出峰值
U_error_rms_45 = abs(U_rms_45 - 24); % 有效值偏差（理论24V）

%% ===================== 6. 命令行窗口输出品质指标 =====================
fprintf('=======================================\n');
fprintf('SOGI-FLL 仿真品质指标（步长dt=1e-5）\n');
fprintf('=======================================\n');

% 频率跟踪指标输出
fprintf('\n【频率跟踪品质】\n');
fprintf('50Hz稳态区间（0.1~0.2s）：\n');
fprintf(' 平均估计频率：%.4f Hz\n', f_mean_50);
fprintf(' 静态误差：%.4f Hz\n', f_error_50);
fprintf(' 波动RMS：%.6f Hz\n', f_rms_50);
fprintf(' 频率范围：[%.4f, %.4f] Hz\n', f_min_50, f_max_50);

fprintf('45Hz稳态区间（0.3~0.4s）：\n');
fprintf(' 平均估计频率：%.4f Hz\n', f_mean_45);
fprintf(' 静态误差：%.4f Hz\n', f_error_45);
fprintf(' 波动RMS：%.6f Hz\n', f_rms_45);
fprintf(' 频率范围：[%.4f, %.4f] Hz\n', f_min_45, f_max_45);

% 幅值品质指标输出
fprintf('\n【输出波形幅值品质】\n');
fprintf('50Hz稳态区间（0.1~0.2s）：\n');
fprintf(' 同相输出有效值：%.4f V（理论24V）\n', U_rms_50);
fprintf(' 有效值偏差：%.4f V\n', U_error_rms_50);
fprintf(' 同相输出峰值：%.4f V（理论≈33.94V）\n', U_peak_50);

fprintf('45Hz稳态区间（0.3~0.4s）：\n');
fprintf(' 同相输出有效值：%.4f V（理论24V）\n', U_rms_45);
fprintf(' 有效值偏差：%.4f V\n', U_error_rms_45);
fprintf(' 同相输出峰值：%.4f V（理论≈33.94V）\n', U_peak_45);

fprintf('=======================================\n');

%% ===================== 7. 结果绘图 =====================
figure('Color','white','Position',[100,100,1000,600]);

% 子图1：输入电压 + SOGI输出电压
subplot(2,1,1);
plot(t, U, 'b', 'LineWidth',1.2, 'DisplayName','输入电压（含5V直流）'); hold on;
plot(t, U_prime, 'r', 'LineWidth',1.2, 'DisplayName','SOGI同相输出 U''（无直流）');
plot(t, deltaU_prime, 'g', 'LineWidth',1.2, 'DisplayName','SOGI正交输出 ΔU''（无直流）');
plot(t, ones(1,N)*U_dc, 'k:', 'LineWidth',1, 'DisplayName','5V直流偏置');
plot([mutate_t mutate_t], ylim, 'm--', 'LineWidth',1, 'DisplayName','频率畸变时刻（0.2s）');
title('输入电压与SOGI正交输出对比（含5V直流偏置）','FontSize',12);
xlabel('时间 t (s)'); ylabel('幅值 (V)');
legend('Location','best'); grid on; axis tight;

% 子图2：频率自适应跟踪效果
subplot(2,1,2);
plot(t, f_est, 'm', 'LineWidth',1.2, 'DisplayName','ω₀（自适应估计频率）'); hold on;
plot(t, f_grid, 'y--', 'LineWidth',1.2, 'DisplayName','电网实际频率（50→45Hz）');
plot([mutate_t mutate_t], ylim, 'r--', 'LineWidth',1, 'DisplayName','频率畸变时刻（0.2s）');
title('SOGI-FLL频率跟踪效果（含5V直流偏置）','FontSize',12);
xlabel('时间 t (s)'); ylabel('频率 (Hz)');
legend('Location','best'); grid on; axis tight; ylim([40, 55]);