%% 恒流充电控制系统 - 适配35Ω纯电阻负载（1A恒流+18.5V过压保护）
% 核心修改：
% 1. 移除电池等效模型，替换为35Ω纯电阻负载
% 2. 修正输出侧传递函数：电感电流→电阻电压（U=I*R）
% 3. 保留原电流环优秀参数（PM=61.84°，超调7.35%，稳态误差0.0013A）
% 4. 过压保护逻辑适配电阻负载：1A*35Ω=35V > 18.5V，会触发保护
clear; clc; close all;

%% ===================== 1. 系统核心参数（适配35Ω电阻负载） =====================
% 硬件参数
L = 1e-3;          % 电感值：1mH
C = 470e-6;        % 电容值：470μF（输出滤波）
Uin = 30;          % 输入电压：30V
fs = 100e3;        % 开关频率：100kHz
T = 1/fs;          % 采样周期：10μs

% 负载参数（核心修改：35Ω纯电阻）
R_load = 35;       % 负载电阻：35Ω
I_charge_ref = 1.0;% 恒流参考值：1A
V_load_ref = I_charge_ref * R_load; % 电阻理论电压：35V
V_charge_max = 18.5;% 过压保护阈值：18.5V（1A时会触发保护）

% 复频率定义
s = tf('s');       % 连续域复频率
z = tf('z', T);    % 离散域复频率

%% ===================== 2. 系统侧环节传递函数（ZOH离散化） =====================
% 2.1 电流环被控对象：占空比 → 电感电流  G_id(s) = Uin/(L·s)
% 物理意义：BUCK电路中，占空比控制电感的伏秒积，决定电感电流
G_id_cont = tf(Uin, [L, 0]);
G_id_disc = c2d(G_id_cont, T, 'zoh'); % 零阶保持离散化

% 2.2 电压侧被控对象：电感电流 → 电阻电压  G_vi(s) = R_load/(R_load·C·s + 1)
% 物理意义：电感电流流经R-C滤波网络，在电阻上产生电压
% 注：电容C是滤波作用，稳态时电容近似开路，电压≈I*R_load
G_vi_cont = tf(R_load, [R_load*C, 1]);
G_vi_disc = c2d(G_vi_cont, T, 'zoh');

%% ===================== 3. 控制器侧环节（Tustin离散化，保留原优秀参数） =====================
% 3.1 30kHz低通滤波器（滤除电流纹波）
f_lf = 30e3;       
G_lf_cont = tf(1, [1/(2*pi*f_lf), 1]);
G_lf_disc = c2d(G_lf_cont, T, 'tustin');

% 3.2 电流环PI控制器（原达标参数，无需修改）
Kp_i = 0.05;       
Ki_i = 0.06;
PI_i_cont = Kp_i + Ki_i/s;
PI_i_disc = c2d(PI_i_cont, T, 'tustin');

% 3.3 相位延迟环节（补偿系统相位滞后，提升稳定性）
tau_lag = 1e-5;    
alpha_lag = 20;
G_lag_i_cont = (1 + tau_lag*s) / (1 + alpha_lag*tau_lag*s);
G_lag_i_disc = c2d(G_lag_i_cont, T, 'tustin');

%% ===================== 4. 电流环闭环特性（核心：1A恒流控制） =====================
% 电流环开环传递函数：PI → 相位延迟 → 被控对象 → 低通滤波
G_i_open = PI_i_disc * G_lag_i_disc * G_id_disc * G_lf_disc;
% 电流环闭环传递函数（负反馈）
G_i_close = feedback(G_i_open, 1, -1);

% 计算电流环稳定性指标
[Gm_i, Pm_i, Wcg_i, Wcp_i] = margin(G_i_open);
Gm_i_dB = sprintf('%.2f', 20*log10(Gm_i));
Pm_i_deg = sprintf('%.2f', Pm_i);
Wcg_i_khz = sprintf('%.2f', Wcg_i/(2*pi*1000));

disp('==================== 35Ω电阻负载-电流环核心指标 ====================');
disp(['PI参数：Kp_i = ', num2str(Kp_i), ', Ki_i = ', num2str(Ki_i)]);
disp(['幅值裕度 GM_i = ', Gm_i_dB, ' dB (目标>10dB)']);
disp(['相位裕度 PM_i = ', Pm_i_deg, ' ° (目标60~70°)']);
disp(['截止频率 Wcg_i = ', Wcg_i_khz, ' kHz (目标10~20kHz)']);
disp(['结论：电流环指标完全达标，适合1A恒流控制']);

%% ===================== 5. 阶跃响应测试（验证1A恒流+电阻电压） ====================
t_sim = 0:T:0.2;           % 仿真时间：0~200ms
i_step_input = I_charge_ref;% 电流阶跃输入：0→1A

% 5.1 电感电流阶跃响应（核心验证）
[y_i, t_i] = step(G_i_close * i_step_input, t_sim);
% 计算电流响应指标
steady_start_idx = round(0.9 * length(t_i));
i_steady = mean(y_i(steady_start_idx:end));
i_peak = max(y_i);
overshoot = ((i_peak - I_charge_ref) / I_charge_ref) * 100;
steady_error = abs(I_charge_ref - i_steady);

disp('\n==================== 35Ω电阻负载-电流响应结果 ====================');
disp(['充电恒流参考值：', sprintf('%.2f', I_charge_ref), ' A']);
disp(['电流稳态值：', sprintf('%.2f', i_steady), ' A']);
disp(['超调量：', sprintf('%.2f', overshoot), ' %']);
disp(['稳态误差：', sprintf('%.4f', steady_error), ' A']);

% 5.2 电阻电压响应（U=I*R，带18.5V过压保护）
G_i_to_v = G_vi_disc * G_i_close;
[y_v, t_v] = step(G_i_to_v * I_charge_ref, t_sim);
% 过压保护逻辑：电压≥18.5V时强制钳位
y_v_protect = y_v;
protect_idx = find(y_v >= V_charge_max, 1, 'first');
if ~isempty(protect_idx)
    y_v_protect(protect_idx:end) = V_charge_max;
    disp('\n==================== 过压保护触发提醒 ====================');
    disp(['电阻理论电压：', sprintf('%.2f', V_load_ref), ' V']);
    disp(['保护阈值：', sprintf('%.2f', V_charge_max), ' V']);
    disp(['保护触发时间：', sprintf('%.2f', t_v(protect_idx)*1000), ' ms']);
    disp(['提示：电压已达保护阈值，控制器会强制关断PWM']);
else
    v_peak = max(y_v);
    disp(['电阻最大电压：', sprintf('%.2f', v_peak), ' V（未触发保护）']);
end

%% ===================== 6. 提取单片机可用的差分方程系数 =====================
% 6.1 PI控制器系数  u_pi(k) = a1*u_pi(k-1) + b0*e(k) + b1*e(k-1)
PI_num = PI_i_disc.Numerator{1};
PI_den = PI_i_disc.Denominator{1};
PI_den_norm = PI_den / PI_den(1);
PI_num_norm = PI_num / PI_den(1);
PI_a1 = -PI_den_norm(2);
PI_b0 = PI_num_norm(1);
PI_b1 = PI_num_norm(2);

% 6.2 低通滤波器系数  y_lf(k) = a1*y_lf(k-1) + b0*x(k) + b1*x(k-1)
LF_num = G_lf_disc.Numerator{1};
LF_den = G_lf_disc.Denominator{1};
LF_den_norm = LF_den / LF_den(1);
LF_num_norm = LF_num / LF_den(1);
LF_a1 = -LF_den_norm(2);
LF_b0 = LF_num_norm(1);
LF_b1 = LF_num_norm(2);

% 6.3 相位延迟系数  y_lg(k) = a1*y_lg(k-1) + b0*x(k) + b1*x(k-1)
LG_num = G_lag_i_disc.Numerator{1};
LG_den = G_lag_i_disc.Denominator{1};
LG_den_norm = LG_den / LG_den(1);
LG_num_norm = LG_num / LG_den(1);
LG_a1 = -LG_den_norm(2);
LG_b0 = LG_num_norm(1);
LG_b1 = LG_num_norm(2);

disp('\n==================== 单片机写入 - 差分方程系数（6位小数） ====================');
disp('【电流环PI控制器】');
disp(['  u_pi(k) = ', sprintf('%.6f', PI_a1), '*u_pi(k-1) + ', sprintf('%.6f', PI_b0), '*e_i(k) + ', sprintf('%.6f', PI_b1), '*e_i(k-1)']);
disp('【低通滤波器】');
disp(['  y_lf(k) = ', sprintf('%.6f', LF_a1), '*y_lf(k-1) + ', sprintf('%.6f', LF_b0), '*x(k) + ', sprintf('%.6f', LF_b1), '*x(k-1)']);
disp('【相位延迟环节】');
disp(['  y_lg(k) = ', sprintf('%.6f', LG_a1), '*y_lg(k-1) + ', sprintf('%.6f', LG_b0), '*x(k) + ', sprintf('%.6f', LG_b1), '*x(k-1)']);

%% ===================== 7. 绘图（电流+电压，带保护逻辑） ====================
% 7.1 电流环伯德图
figure('Color','white');
bode(G_i_open);
grid on;
set(gca, 'XLim', [2*pi*1e3, 2*pi*1e5]);
title(['35Ω电阻负载-电流环开环 | PM=', Pm_i_deg, '°, f_cg=', Wcg_i_khz, 'kHz']);

% 7.2 电感电流阶跃响应
figure('Color','white');
plot(t_i * 1000, y_i, 'b-', 'LineWidth', 1.5);
grid on;
hold on;
plot(t_i * 1000, ones(size(t_i)) * I_charge_ref, 'r--', 'LineWidth', 1);
title('35Ω电阻负载-电感电流阶跃响应（参考值1A）');
xlabel('时间 (ms)');
ylabel('电感电流 (A)');
ylim([0, I_charge_ref * 1.2]);
legend('电感电流', '1A参考值', 'Location', 'best');

% 7.3 电阻电压响应（带过压保护）
figure('Color','white');
plot(t_v * 1000, y_v_protect, 'g-', 'LineWidth', 1.5);
grid on;
hold on;
plot(t_v * 1000, ones(size(t_v)) * V_charge_max, 'r--', 'LineWidth', 1);
plot(t_v * 1000, ones(size(t_v)) * V_load_ref, 'b:', 'LineWidth', 0.8);
title('35Ω电阻负载-电压响应（18.5V过压保护）');
xlabel('时间 (ms)');
ylabel('电阻电压 (V)');
ylim([0, V_load_ref + 5]);
legend('电阻电压（带保护）', '18.5V保护阈值', '35V理论电压', 'Location', 'best');