% SOGI_FLL_PLL_Modelled.m
% 严格仿照PLL_Controller函数逻辑，步长1e-5，完美锁定
%验证sogi_FLL后级是否加入PLL带来的相位影响
clear; clc; close all;

%% ===================== 1. 仿真参数配置 =====================
dt = 1e-5;                % 仿真步长（与PLL_Controller适配）
t_end = 0.5;              % 仿真时间（确保收敛）
t = 0:dt:t_end;
N = length(t);

% 输入信号：24V/50Hz纯正弦（无直流偏置）
U_rms = 24;
A = U_rms * sqrt(2);      % 峰值≈33.94V（正幅值目标）
f_grid = 50;
omega_grid = 2*pi*f_grid;
U = A * cos(omega_grid * t)';   % 输入信号

% 全局定义真实相位（无归一化，避免误导）
theta_true = omega_grid * t';    % 电网真实相位（连续值）

% SOGI-FLL参数（不变，保证频率跟踪）
K = 1.414;
K1 = 0.25;
gamma = 20;

% PLL参数（严格仿照PLL_Controller，适配dt=1e-5）
Kp_pll = 180;               % 比例增益（与函数一致）
Ki_pll = 3200;              % 积分增益（与函数一致）
omega0 = 2*pi*f_grid;       % 额定角频率

%% ===================== 2. 状态初始化 =====================
% SOGI-FLL状态
x1 = zeros(N, 1);
x2 = zeros(N, 1);  % SOGI输出α（同相分量，sinωt）
x3 = zeros(N, 1);
omega0_sogi = zeros(N, 1);
omega0_sogi(1) = omega_grid;
theta_sogi = zeros(N, 1); % SOGI开环相位（连续值）

% PLL状态（仿照PLL_Controller）
theta_prev = 0;             % 初始相位（rad）
omega_prev = omega0;        % 初始角频率
integral_sum = 0;           % 积分项累加器
theta_pll = zeros(N, 1);    % PLL闭环相位（连续值）
omega_pll = zeros(N, 1);    % PLL闭环角频率

%% ===================== 3. 主循环（严格仿照PLL_Controller逻辑） =====================
for k = 2:N
    % ---------------------- 3.1 SOGI-FLL 计算 ----------------------
    x1_prev = x1(k-1);
    x2_prev = x2(k-1);
    x3_prev = x3(k-1);
    omega0_prev_sogi = omega0_sogi(k-1);
    U_curr = U(k);

    A_mat = [
        -K1*omega0_prev_sogi, -K1*omega0_prev_sogi, 0;
        -K*omega0_prev_sogi,  -K*omega0_prev_sogi,  -omega0_prev_sogi^2;
        0,                1,                0
    ];
    B_mat = [K1*omega0_prev_sogi; K*omega0_prev_sogi; 0];

    dx1dt_prev = A_mat(1,:)*[x1_prev; x2_prev; x3_prev] + B_mat(1)*U_curr;
    dx2dt_prev = A_mat(2,:)*[x1_prev; x2_prev; x3_prev] + B_mat(2)*U_curr;
    dx3dt_prev = A_mat(3,:)*[x1_prev; x2_prev; x3_prev] + B_mat(3)*U_curr;

    x1(k) = x1_prev + dt * dx1dt_prev;
    x2(k) = x2_prev + dt * dx2dt_prev;
    x3(k) = x3_prev + dt * dx3dt_prev;

    deltaU = U_curr - x1(k) - x2(k);
    deltaU_prime = omega0_prev_sogi * x3(k);  % SOGI输出β（正交分量，cosωt）
    epsilon_f = deltaU * deltaU_prime;
    domega0dt_prev = -gamma * epsilon_f;
    omega0_sogi(k) = omega0_prev_sogi + dt * domega0dt_prev;
    omega0_sogi(k) = max(min(omega0_sogi(k), 2*pi*60), 2*pi*40);

    % SOGI开环相位积分
    theta_sogi(k) = theta_sogi(k-1) + (omega0_sogi(k) + omega0_prev_sogi) * dt / 2;

    % ---------------------- 3.2 PLL闭环锁相（严格仿照PLL_Controller） ----------------------
    % SOGI输出：alpha = x2(k)（sin_signal = sinωt），beta = deltaU_prime（cos_signal = cosωt）
    alpha = x2(k);         % 同相分量
    beta = deltaU_prime;  % 正交分量 滞后α90°

    % 1. 虚拟Park变换（计算q轴误差，强制q=0）
    cos_theta = cos(theta_prev);
    sin_theta = sin(theta_prev);
    % 核心：与PLL_Controller完全一致的q_error公式
    q_error = -alpha * sin_theta + beta * cos_theta;

    % 2. 标准离散PI控制器
    proportional = Kp_pll * q_error;
    integral_sum = integral_sum + Ki_pll * q_error * dt;
    delta_omega = proportional + integral_sum;

    % 3. 锁定的瞬时角频率
    omega_curr = omega0 + delta_omega;

    % 4. 频率积分得到相位（梯形积分，高精度）
    theta_curr = theta_prev + (omega_curr + omega_prev) * dt / 2;
    % 相位归一化（限制在[-π, π]，避免数值溢出）
    theta_curr = mod(theta_curr + pi, 2*pi) - pi;

    % 5. 更新状态（供下一时刻使用）
    theta_prev = theta_curr;
    omega_prev = omega_curr;

    % 保存当前相位和角频率
    theta_pll(k) = theta_curr;
    omega_pll(k) = omega_curr;
end

%% ===================== 4. 标准Park/反Park变换 =====================
% SOGI输出的α/β（严格匹配定义）
alpha_sogi = x2;
beta_sogi = omega0_sogi .* x3;

% 1. 用SOGI开环θ做标准Park变换（仍有偏差）
d_sogi = alpha_sogi .* cos(theta_sogi) + beta_sogi .* sin(theta_sogi);  % d = α·cosθ + β·sinθ
q_sogi = -alpha_sogi .* sin(theta_sogi) + beta_sogi .* cos(theta_sogi); % q = -α·sinθ + β·cosθ

% 2. 用PLL闭环θ做标准Park变换（完美锁定）
d_pll = alpha_sogi .* cos(theta_pll) + beta_sogi .* sin(theta_pll);    % d = α·cosθ + β·sinθ
q_pll = -alpha_sogi .* sin(theta_pll) + beta_sogi .* cos(theta_pll);   % q = -α·sinθ + β·cosθ

% 标准反Park变换
alpha_from_sogi = d_sogi .* cos(theta_sogi) - q_sogi .* sin(theta_sogi);
alpha_from_pll = d_pll .* cos(theta_pll) - q_pll .* sin(theta_pll);

%% ===================== 5. 绘图（修正相位差，无视觉误导） =====================
figure('Color','white','Position',[100,100,1200,1000]);

% 场景1：相位偏差对比（核心，修正相位差）
subplot(4,1,1);
delta_theta_sogi = mod(theta_sogi - theta_true + pi, 2*pi) - pi;  % 修正SOGI相位差
delta_theta_pll = mod(theta_pll - theta_true + pi, 2*pi) - pi;    % 修正PLL相位差
plot(t, delta_theta_sogi, 'b', 'LineWidth',1.2, 'DisplayName','SOGI相位偏差'); hold on;
plot(t, delta_theta_pll, 'r', 'LineWidth',1.2, 'DisplayName','PLL相位偏差');
plot(t, zeros(1,N), 'k:', 'LineWidth',1, 'DisplayName','理想偏差=0');
title('场景1：相位偏差对比（已修正相位差，无归一化误导）');
xlabel('t (s)'); ylabel('相位偏差 (rad)');
legend('Location','best'); grid on; axis tight;
ylim([-2, 2]); % 聚焦偏差范围

% 场景2-1：d轴对比（标准Park变换）
subplot(4,1,2);
plot(t, d_sogi, 'b', 'LineWidth',1.2, 'DisplayName','d（SOGIθ）'); hold on;
plot(t, d_pll, 'r', 'LineWidth',1.2, 'DisplayName','d（PLLθ）');
plot(t, ones(1,N)*A, 'k:', 'LineWidth',1, 'DisplayName','理想幅值（+33.94V）');
title('场景2-1：d轴对比（d=α·cosθ+β·sinθ）');
xlabel('t (s)'); ylabel('d (V)');
legend('Location','best'); grid on; axis tight;

% 场景2-2：q轴对比（标准Park变换）
subplot(4,1,3);
plot(t, q_sogi, 'b', 'LineWidth',1.2, 'DisplayName','q（SOGIθ）'); hold on;
plot(t, q_pll, 'r', 'LineWidth',1.2, 'DisplayName','q（PLLθ）');
plot(t, zeros(1,N), 'k:', 'LineWidth',1, 'DisplayName','理想q=0');
title('场景2-2：q轴对比（q=-α·sinθ+β·cosθ）');
xlabel('t (s)'); ylabel('q (V)');
legend('Location','best'); grid on; axis tight;
ylim([-5, 5]); % 聚焦q轴收敛

% 场景3：反Park输出α与输入信号对比
subplot(4,1,4);
plot(t, U, 'k', 'LineWidth',1.5, 'DisplayName','输入信号'); hold on;
plot(t, alpha_from_sogi, 'b--', 'LineWidth',1.2, 'DisplayName','α（SOGIθ反Park）');
plot(t, alpha_from_pll, 'r--', 'LineWidth',1.2, 'DisplayName','α（PLLθ反Park）');
title('场景3：反Park输出α与输入信号对比');
xlabel('t (s)'); ylabel('幅值 (V)');
legend('Location','best'); grid on; axis tight;

%% ===================== 6. 定量输出锁定结果（稳态0.3~0.5s） =====================
idx_steady = find(t >= 0.3 & t <= 0.5);

% 关键指标计算
d_pll_steady = mean(d_pll(idx_steady));       % PLL d轴稳态值
q_pll_steady_rms = rms(q_pll(idx_steady));    % PLL q轴稳态波动
delta_theta_pll_rms = rms(mod(theta_pll(idx_steady) - theta_true(idx_steady) + pi, 2*pi) - pi); % 修正相位偏差RMS

% 打印结果
fprintf('=======================================\n');
fprintf('PLL完美锁定结果（稳态区间0.3~0.5s）\n');
fprintf('=======================================\n');
fprintf('d轴稳态值：%.2f V（理论+33.94V）\n', d_pll_steady);
fprintf('q轴稳态波动RMS：%.4f V（理想=0）\n', q_pll_steady_rms);
fprintf('PLL相位偏差RMS：%.4f rad（理想=0）\n', delta_theta_pll_rms);
fprintf('=======================================\n');