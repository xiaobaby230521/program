%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 【代码名称】 SOGI_FLL_SRF_PLL_20kHz_Precise_Phase_Locking_Standard_50Hz.m
% 【核心功能】 20kHz采样率下，SOGI-FLL + SRF-PLL 双级联实现精准电网锁相
% 【关键特点】
%   1. 采样率：20kHz（离散化步长 dt=5e-5s），适配电力电子数字控制
%   2. 离散化方法：双线性变换（含频率预畸变），二阶精度，零数值误差
%   3. 算法架构：
%      - 第一级：SOGI-FLL（二阶广义积分器-频率锁定环）
%        * 功能：生成无直流的αβ两相正交信号，自适应跟踪电网频率
%      - 第二级：SRF-PLL（同步旋转坐标系锁相环）
%        * 功能：基于αβ信号实现精准相位锁定，强制q轴分量Vq→0
%   4. 测试场景：标准50Hz、24V有效值纯正弦波（无直流偏置、无频率畸变）
%   5. 验证指标：PLL的q轴分量Vq_hat稳态趋近于0，证明锁相精度
% 【作者/备注】 电力电子数字控制锁相环仿真
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% SOGI-FLL + SRF-PLL 标准50Hz正弦波（无直流、无频率畸变）
% 观测PLL的q分量是否收敛到0
clear; clc; close all;

%% ===================== 1. 仿真参数 =====================
dt = 5e-5;        % 20kHz采样，离散化步长50μs
t_end = 0.4;      % 总仿真时长0.4s
t = 0:dt:t_end;   % 离散时间序列
N = length(t);     % 总采样点数

% 电网：纯50Hz正弦波，24V有效值，无直流、无频率畸变
U_rms = 24;       % 电网电压有效值24V
A = U_rms * sqrt(2); % 电网电压峰值≈33.94V
f_grid = 50;      % 电网额定频率50Hz
omega = 2*pi*f_grid; % 电网额定角频率

% 输入电压：纯正弦波（列向量形式）
U = A * sin(omega * t);
U = U(:);

% SOGI-FLL参数（20kHz下优化配置）
K = 1.414;        % SOGI增益（√2，优化正交性）
K1 = 0.25;        % 直流消除器增益
gamma_p = 15;     % FLL比例增益
gamma_i = 15;     % FLL积分增益

%% ===================== 2. SOGI 状态初始化 =====================
x1 = zeros(N, 1); % SOGI状态变量1（直流消除器）
x2 = zeros(N, 1); % SOGI状态变量2（同相输出uα）
x3 = zeros(N, 1); % SOGI状态变量3（积分器状态）
omega0 = zeros(N, 1); % SOGI-FLL估计角频率
omega0(1) = 2*pi*50; % 初始角频率设为50Hz
U_prev = U(1);    % 双线性变换需要：保存上一时刻输入
integral_f = 0;   % FLL积分器状态

%% ===================== 3. SRF-PLL 参数与状态 =====================
k_pll_p = 120;    % PLL比例增益（加快响应）
k_pll_i = 6000;   % PLL积分增益（消除稳态误差）

theta_hat   = zeros(N, 1); % PLL估计相位θ̂
omega_g_hat = zeros(N, 1); % PLL估计角频率ω̂_g
Vd_hat      = zeros(N, 1); % d轴电压分量V̂_d（对应幅值）
Vq_hat      = zeros(N, 1); % q轴电压分量V̂_q（核心观测目标，锁相成功→0）
integral_pll = 0;           % PLL积分器状态

theta_hat(1)   = 0;         % 初始相位清零
omega_g_hat(1) = 2*pi*50;   % 初始角频率设为50Hz

%% ===================== 4. 主循环（20kHz离散化迭代） =====================
for k = 2:N
    % -------------------------- 第一级：SOGI-FLL --------------------------
    % 1. 获取上一时刻状态和当前输入
    x_prev      = [x1(k-1); x2(k-1); x3(k-1)];
    omega0_prev = omega0(k-1);
    U_curr      = U(k);

    % 2. 双线性变换核心1：频率预畸变（消除离散化频率偏移）
    omega_pre = (2/dt) * tan(omega0_prev * dt / 2);

    % 3. 构造连续域状态矩阵A和B（三状态SOGI结构）
    A_mat = [
        -K1*omega_pre, -K1*omega_pre, 0;
        -K*omega_pre,  -K*omega_pre,  -omega_pre^2;
        0,              1,              0
    ];
    B_mat = [K1*omega_pre; K*omega_pre; 0];

    % 4. 双线性变换核心2：显式更新状态（二阶精度）
    % 公式：x(k) = inv(I - (dt/2)A) · [(I + (dt/2)A)x(k-1) + (dt/2)B(U(k)+U(k-1))]
    I_mat = eye(3);
    M1 = I_mat - (dt/2)*A_mat;
    M2 = I_mat + (dt/2)*A_mat;
    U_avg = (U_curr + U_prev);
    x_curr = M1 \ (M2 * x_prev + (dt/2) * B_mat * U_avg);

    % 5. 更新SOGI状态变量
    x1(k) = x_curr(1);
    x2(k) = x_curr(2);
    x3(k) = x_curr(3);
    U_prev = U_curr;

    % 6. FLL频率自适应更新（PI控制器）
    deltaU = U_curr - x1(k) - x2(k);          % 直流误差
    deltaU_prime = omega0_prev * x3(k);        % 正交输出
    epsilon_f = deltaU * deltaU_prime;          % 频率误差信号

    integral_f = integral_f + epsilon_f * dt;   % 积分项更新
    domega0dt  = -gamma_p * epsilon_f - gamma_i * integral_f; % 频率变化率
    omega0(k)  = omega0_prev + dt * domega0dt;  % 频率更新

    % 频率限幅（40Hz-60Hz，防止发散）
    w_max = 2*pi*60; w_min = 2*pi*40;
    omega0(k) = max(min(omega0(k), w_max), w_min);

    % -------------------------- 第二级：SRF-PLL（精准锁相核心） --------------------------
    % 1. 获取SOGI输出的αβ正交信号（无直流）
    Valpha = x2(k);          % α轴信号：同相输出 uα = x2
    Vbeta  = omega0(k) * x3(k); % β轴信号：正交输出 uβ = ω₀·x3

    % 2. Park变换（αβ→dq）：核心鉴相环节
    theta_prev = theta_hat(k-1);
    c = cos(theta_prev);
    s = sin(theta_prev);

    Vd_hat(k) =  Valpha * c + Vbeta * s;  % d轴分量（对应电压幅值）
    Vq_hat(k) = -Valpha * s + Vbeta * c;  % q轴分量（相位误差，锁相成功→0）

    % 3. PI环路控制器（强制Vq→0）
    integral_pll = integral_pll + Vq_hat(k) * dt;          % 积分项更新
    omega_g_hat(k) = k_pll_p * Vq_hat(k) + k_pll_i * integral_pll; % 直接输出角频率

    % 4. 积分环节生成相位（对应框图1/s）
    theta_hat(k) = theta_prev + omega_g_hat(k) * dt;

    % 5. 相位归一化（±π，防止数值溢出）
    if theta_hat(k) > pi
        theta_hat(k) = theta_hat(k) - 2*pi;
    elseif theta_hat(k) < -pi
        theta_hat(k) = theta_hat(k) + 2*pi;
    end
end

%% ===================== 5. 结果数据处理 =====================
U_prime      = x2;                % SOGI同相输出 uα
deltaU_prime = omega0 .* x3;      % SOGI正交输出 uβ
f_est        = omega0 / (2*pi);   % SOGI-FLL估计频率（Hz）
f_pll        = omega_g_hat / (2*pi); % SRF-PLL估计频率（Hz）

%% ===================== 6. q分量稳态指标计算（验证锁相精度） =====================
idx_steady = find(t >= 0.1 & t <= 0.4); % 选取0.1s-0.4s作为稳态区间
Vq_mean = mean(Vq_hat(idx_steady));      % q分量均值（理想→0）
Vq_rms  = rms(Vq_hat(idx_steady));       % q分量RMS（理想→0）

% 命令行输出锁相精度指标
fprintf('===================================\n');
fprintf('【20kHz SOGI-FLL+SRF-PLL 锁相精度验证】\n');
fprintf('===================================\n');
fprintf('q分量均值 (Vq_mean) = %.6f V\n', Vq_mean);
fprintf('q分量RMS  (Vq_rms)  = %.6f V\n', Vq_rms);
fprintf('===================================\n');

%% ===================== 7. 结果绘图 =====================
figure('Color','white','Position',[100,100,1000,700]);

% 子图1：输入电网电压 & SOGI正交输出
subplot(3,1,1);
plot(t, U, 'b','LineWidth',1.1,'DisplayName','电网电压（24V/50Hz）'); hold on;
plot(t, U_prime, 'r','LineWidth',1.1,'DisplayName','SOGI同相输出 uα');
plot(t, deltaU_prime, 'g','LineWidth',1.1,'DisplayName','SOGI正交输出 uβ');
title('电网电压与SOGI正交输出（20kHz采样）','FontSize',12);
xlabel('时间 t (s)'); ylabel('幅值 (V)');
legend('Location','best'); grid on; axis tight;

% 子图2：频率跟踪效果
subplot(3,1,2);
plot(t, f_est, 'm','LineWidth',1.2,'DisplayName','SOGI-FLL估计频率'); hold on;
plot(t, f_pll, 'c','LineWidth',1.2,'DisplayName','SRF-PLL输出频率');
yline(50,'k--','50Hz（电网额定频率）');
title('频率自适应跟踪效果','FontSize',12);
xlabel('时间 t (s)'); ylabel('频率 (Hz)');
legend('Location','best'); grid on; ylim([49.5 50.5]);

% 子图3：PLL q分量（核心验证图，锁相成功应趋近0）
subplot(3,1,3);
plot(t, Vq_hat, 'r','LineWidth',1.2,'DisplayName','q轴分量 V_q');
yline(0,'k--','V_q = 0（理想锁相）');
title('SRF-PLL q轴分量（锁相精度验证）','FontSize',12);
xlabel('时间 t (s)'); ylabel('幅值 (V)');
legend('Location','best'); grid on; axis tight;