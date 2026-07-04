% SinglePhase_Inverter_Repetitive_Control_Simulation.m
% 功能：仅保留空载+纯电阻负载，仿真0.4s，无THD计算
% 负载时序：0~0.2s空载 → 0.2~0.4s纯12Ω电阻
% 电流环结构：电压环输出(+)、负载电流(+)、电感电流(-)

clc; clear; close all;

%% ===================== 1. 系统核心参数定义 =====================
% --- 主电路硬件参数 ---
L = 1.5e-3;               % 滤波电感 1.5mH
C = 10e-6;                % 滤波电容 10uF
R_L = 0.1;                 % 电感寄生电阻 0.1Ω
Kpwm = 45;                 % PWM桥臂增益（直流母线45V）

% --- 控制参数（保留你调好的最优参数）---
Kcp = 0.5;                 % 电流环纯比例增益
Kp_v = 0.16;               % 电压环PI比例系数
Ki_v = 450;                % 电压环PI积分系数

% --- 保护限幅参数 ---
U_inv_max = Kpwm;           % PWM输出上限（匹配母线电压）
U_inv_min = -Kpwm;          % PWM输出下限

% --- 仿真参数 ---
fs = 10e3;                 % 采样频率 10kHz
Ts = 1/fs;                 % 采样周期
t_total = 0.4;             % 总仿真时间 0.4s
N_total = floor(t_total / Ts); % 总采样点数
t = (0:N_total-1)*Ts;      % 时间轴

% --- 负载设置：仅空载+纯电阻 ---
R_load_noload = 1e9;       % 空载电阻
R_load_load = 12;           % 纯电阻负载阻值

% 初始化负载参数数组
R_load = R_load_noload * ones(1, N_total);
R_load(floor(0.2/Ts)+1:end) = R_load_load; % 0.2s切换为12Ω

% --- 参考电压信号 ---
V_rms = 24;                 % 输出有效值 24V
V_pk = V_rms * sqrt(2);    % 峰值≈33.94V
f0 = 50;                    % 基波频率 50Hz
v_ref = V_pk * sin(2*pi*f0*t); % 参考正弦信号

%% ===================== 2. 仿真初始化 =====================
% --- 信号存储数组 ---
v_o = zeros(1, N_total);   % 输出电压
i_L = zeros(1, N_total);   % 滤波电感电流
i_o = zeros(1, N_total);   % 负载电流
e_v = zeros(1, N_total);   % 电压环误差
u_v_out = zeros(1, N_total); % 电压环输出

% --- 状态变量初始化 ---
pi_integral = 0;           % 电压环PI积分器状态
% 积分限幅
integral_max = 8;
integral_min = -8;

% --- 初始值 ---
v_o(1) = 0;
i_L(1) = 0;
i_o(1) = 0;

%% ===================== 3. 离散仿真主循环 =====================
for k = 1:N_total-1
    % -------------------------- 1. 电压环误差计算 --------------------------
    e_v(k) = v_ref(k) - v_o(k);
    
    % -------------------------- 2. 电压环PI控制器（带积分限幅）--------------------------
    pi_proportional = Kp_v * e_v(k);
    pi_integral = pi_integral + Ki_v * Ts * e_v(k);
    pi_integral = max(min(pi_integral, integral_max), integral_min);
    u_v_out(k) = pi_proportional + pi_integral;
    
    % -------------------------- 3. 电流环（完全匹配你的结构）--------------------------
    i_err = u_v_out(k) + i_o(k) - i_L(k);
    u_i_out = Kcp * i_err;
    
    % -------------------------- 4. PWM输出（带母线限幅）--------------------------
    u_inv = Kpwm * u_i_out;
    u_inv = max(min(u_inv, U_inv_max), U_inv_min);
    
    % -------------------------- 5. LC滤波器离散更新（2阶状态方程，纯电阻负载）--------------------------
    R_current = R_load(k);
    
    % LC滤波器连续状态方程
    A = [ -R_L/L,        -1/L;
           1/C,           -1/(C*R_current) ];
    B = [ 1/L; 0 ];
    
    % ZOH离散化
    Phi = expm(A*Ts);
    Gamma = (Phi - eye(2)) * inv(A) * B;
    
    % 状态更新
    x_state = [i_L(k); v_o(k)];
    x_next = Phi * x_state + Gamma * u_inv;
    
    % 更新信号
    i_L(k+1) = x_next(1);
    v_o(k+1) = x_next(2);
    i_o(k+1) = v_o(k+1) / R_current; % 纯电阻负载电流
end

% 补全最后一个点的信号
e_v(end) = v_ref(end) - v_o(end);
u_v_out(end) = Kp_v * e_v(end) + pi_integral;
i_o(end) = v_o(end) / R_load(end);

%% ===================== 4. 性能指标计算（仅有效值和误差，无THD）=====================
% 空载稳态段：0.1~0.2s
idx_noload = find(t >= 0.1 & t < 0.2);
v_o_noload_rms = rms(v_o(idx_noload));
err_noload = abs(v_o_noload_rms - V_rms)/V_rms * 100;

% 纯电阻稳态段：0.3~0.4s
idx_res = find(t >= 0.3);
v_o_res_rms = rms(v_o(idx_res));
err_res = abs(v_o_res_rms - V_rms)/V_rms * 100;

% 命令行输出结果
fprintf('==================== 仿真结果 ====================\n');
fprintf('1. 空载段（0.1~0.2s）\n');
fprintf('   输出电压有效值：%.2f V | 稳态误差：%.2f %%\n', v_o_noload_rms, err_noload);
fprintf('\n2. 纯电阻负载段（0.3~0.4s，12Ω）\n');
fprintf('   输出电压有效值：%.2f V | 稳态误差：%.2f %%\n', v_o_res_rms, err_res);
fprintf('==================================================\n');

%% ===================== 5. 结果绘图 =====================
figure('Color','white','Position',[100,100,1200,700]);

% 子图1：参考电压 vs 输出电压
subplot(3,1,1);
plot(t, v_ref, 'k--', 'LineWidth',1.2, 'DisplayName','参考电压 v_{ref}');
hold on; grid on;
plot(t, v_o, 'b-', 'LineWidth',1.2, 'DisplayName','输出电压 v_o');
xline(0.2, 'r--', 'LineWidth',1.5, 'DisplayName','纯电阻负载接入');
xlabel('时间 (s)');
ylabel('电压 (V)');
title('仅双闭环控制 - 逆变器输出电压波形','FontSize',12);
legend('Location','best');
ylim([-40, 40]);

% 子图2：滤波电感电流
subplot(3,1,2);
plot(t, i_L, 'g-', 'LineWidth',1.2, 'DisplayName','滤波电感电流 i_L');
hold on; grid on;
xline(0.2, 'r--', 'LineWidth',1.5);
xlabel('时间 (s)');
ylabel('电流 (A)');
title('滤波电感电流波形','FontSize',12);
legend('Location','best');

% 子图3：负载电流
subplot(3,1,3);
plot(t, i_o, 'm-', 'LineWidth',1.2, 'DisplayName','负载电流 i_o');
hold on; grid on;
xline(0.2, 'r--', 'LineWidth',1.5);
xlabel('时间 (s)');
ylabel('电流 (A)');
title('负载电流波形','FontSize',12);
legend('Location','best');

% 负载切换瞬态放大图
figure('Color','white','Position',[100,100,1200,400]);
plot(t, v_ref, 'k--', 'LineWidth',1.2, 'DisplayName','参考电压 v_{ref}');
hold on; grid on;
plot(t, v_o, 'b-', 'LineWidth',1.2, 'DisplayName','输出电压 v_o');
xline(0.2, 'r--', 'LineWidth',1.5, 'DisplayName','负载切换时刻');
xlabel('时间 (s)');
ylabel('电压 (V)');
title('空载→纯电阻负载 瞬态波形放大','FontSize',12);
legend('Location','best');
xlim([0.18, 0.25]);
ylim([-40, 40]);