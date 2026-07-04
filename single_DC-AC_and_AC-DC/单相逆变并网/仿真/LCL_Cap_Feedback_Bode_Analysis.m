% LCL_Cap_Feedback_Bode_Analysis.m
% 含有电容电流反馈LCL并网逆变器开环传递函数伯德图分析
% 相位裕度计算优化：增加相位解绕处理
% 适配参数：Hi1=0.07, Kp=0.253, Hi2=1, Kr=40.92
clc; clear; close all;

%% ===================== 1. 核心参数定义 =====================
% 控制参数
Hi1 = 0.07;             % 电容电流反馈系数
Kp = 0.253;             % PR控制器比例系数
Hi2 = 1;                 % 网侧电流反馈系数
Kr = 40.92;              % PR控制器谐振系数

% LCL滤波器参数
L1 = 1.5e-3;            % 逆变侧电感，单位：H (1.2mH)
L2 = 120e-6;            % 网侧电感，单位：H (120uH)
C = 10e-6;              % 滤波电容，单位：F (10uF)

% 系统与控制参数
fs = 20000;              % 采样/开关频率，单位：Hz (20kHz)
Kpwm = 45;               % PWM桥臂增益
wo = 100*pi;             % 基波角频率，单位：rad/s (50Hz)
wi = pi;                 % PR谐振带宽，单位：rad/s

% 目标设计指标
target_fc = 1373;        % 目标截止频率，单位：Hz
target_PM = 45;          % 目标相位裕度，单位：°
target_GM1 = -3;         % 目标GM1（fr处），单位：dB
target_GM2 = 3;          % 目标GM2（fs/6处），单位：dB
target_Tfo = 73;         % 目标Tfo（50Hz处），单位：dB

%% ===================== 2. 中间参数计算 =====================
Ts = 1/fs;                                   % 采样周期，单位：s
tau = 1.5 * Ts;                              % 1.5Ts控制延时
L_total = L1 + L2;                           % 总电感
% LCL谐振频率
L_parallel = (L1 * L2) / (L1 + L2);
fr = 1/(2*pi * sqrt(L_parallel * C));
fs_6 = fs / 6;                               % fs/6频率点
fprintf('=== 系统核心参数 ===\n');
fprintf('LCL谐振频率 fr = %.2f Hz\n', fr);
fprintf('fs/6 = %.2f Hz\n', fs_6);
fprintf('控制延时 tau = %.6f s\n', tau);

%% ===================== 3. 构建传递函数 =====================
s = tf('s');

% PR调节器（公式8.30）
G_PR = Kp + (2*Kr*wi*s) / (s^2 + 2*wi*s + wo^2);

% 延时环节Pade近似（2阶）
[num_pade, den_pade] = pade(tau, 2);
G_delay = tf(num_pade, den_pade);

% 校正后开环传递函数（公式8.4）
num_TD = Hi2 * Kpwm * G_delay * G_PR;
den_after = s^3 * L1*L2*C + s^2 * L2*C*Hi1*Kpwm*G_delay + s * L_total;
T_D_after = num_TD / den_after;

%% ===================== 4. 伯德图数据计算与关键指标提取 =====================
% 生成频率点：10Hz~10kHz，对数分布
f_log = logspace(1, 4, 10000);
w_log = 2*pi*f_log;

% 计算伯德图数据
[mag, phase] = bode(T_D_after, w_log);
mag_db = 20*log10(squeeze(mag));
phase_deg = squeeze(phase);

% 相位解绕处理
phase_deg_unwrap = unwrap(phase_deg * pi/180) * 180/pi;

% --- 4.1 找到第一次0dB穿越频率 fc ---
cross_idx = find((mag_db(1:end-1) > 0) & (mag_db(2:end) <= 0), 1, 'first');
if isempty(cross_idx)
    fc_actual = NaN;
    PM_actual = NaN;
    fprintf('⚠️  警告：未找到0dB穿越频率！\n');
else
    % 线性插值精确计算fc
    f1 = f_log(cross_idx);
    f2 = f_log(cross_idx+1);
    m1 = mag_db(cross_idx);
    m2 = mag_db(cross_idx+1);
    fc_actual = f1 + (0 - m1) * (f2 - f1) / (m2 - m1);
    
    % 相位裕度计算
    p1 = phase_deg_unwrap(cross_idx);
    p2 = phase_deg_unwrap(cross_idx+1);
    phase_fc_unwrap = p1 + (fc_actual - f1) * (p2 - p1) / (f2 - f1);
    phase_fc_normalized = mod(phase_fc_unwrap, -360);
    PM_actual = 180 + phase_fc_normalized;
    
    fprintf('调试信息：fc处解绕相位 = %.2f°, 归一化相位 = %.2f°\n', phase_fc_unwrap, phase_fc_normalized);
end

% --- 4.2 计算 fr 处的增益 GM1 ---
[~, fr_idx] = min(abs(f_log - fr));
GM1_actual = mag_db(fr_idx);

% --- 4.3 计算 fs/6 处的增益 GM2 ---
[~, fs6_idx] = min(abs(f_log - fs_6));
GM2_actual = mag_db(fs6_idx);

% --- 4.4 计算 50Hz 处的增益 Tfo ---
[~, f50_idx] = min(abs(f_log - 50));
Tfo_actual = mag_db(f50_idx);

%% ===================== 5. 命令行输出验证结果 =====================
fprintf('\n=== 关键指标计算结果 ===\n');
fprintf('1. 截止频率 fc（第一次0dB穿越）：\n');
fprintf('   目标值：%.2f Hz\n', target_fc);
fprintf('   实际值：%.2f Hz\n', fc_actual);
if abs(fc_actual - target_fc) < 200
    fprintf('   ✅ 截止频率匹配良好\n');
else
    fprintf('   ⚠️  截止频率偏差较大，建议调整Kp\n');
end

fprintf('\n2. 相位裕度 PM（基于fc计算）：\n');
fprintf('   目标值：≥ %.2f °\n', target_PM);
fprintf('   实际值：%.2f °\n', PM_actual);
if PM_actual >= target_PM
    fprintf('   ✅ 相位裕度满足要求\n');
else
    fprintf('   ⚠️  相位裕度不足，建议调整Hi1或Kr\n');
end

fprintf('\n3. GM1（谐振频率fr=%.2fHz处增益）：\n', fr);
fprintf('   目标值：≤ %.2f dB\n', target_GM1);
fprintf('   实际值：%.2f dB\n', -GM1_actual);
% ---------------- 核心修正：GM1判断逻辑加负号 ----------------
if -GM1_actual <= target_GM1
    fprintf('   ✅ GM1满足要求（谐振峰抑制充分）\n');
else
    fprintf('   ⚠️  GM1不足，谐振峰未充分抑制，建议增大Hi1\n');
end

fprintf('\n4. GM2（fs/6=%.2fHz处增益）：\n', fs_6);
fprintf('   目标值：≥ %.2f dB\n', target_GM2);
fprintf('   实际值：%.2f dB\n', -GM2_actual);
% ---------------- 核心修正：GM2判断逻辑加负号 ----------------
if -GM2_actual <= target_GM2
    fprintf('   ✅ GM2满足要求\n');
else
    fprintf('   ⚠️  GM2不足，建议调整Hi1\n');
end

fprintf('\n5. Tfo（50Hz处增益）：\n');
fprintf('   目标值：≥ %.2f dB\n', target_Tfo);
fprintf('   实际值：%.2f dB\n', Tfo_actual);
if Tfo_actual >= target_Tfo
    fprintf('   ✅ Tfo满足要求（稳态误差小）\n');
else
    fprintf('   ⚠️  Tfo不足，建议增大Kr\n');
end

%% ===================== 6. 绘制校正后的伯德图 =====================
figure('Color','white','Position',[100,100,900,700]);

% 幅频特性
subplot(2,1,1);
semilogx(f_log, mag_db, 'k-', 'LineWidth',1.5);
hold on; grid on;
% 标注关键频率点
plot([fc_actual, fc_actual], ylim, 'r--', 'LineWidth',1.2, 'DisplayName',['fc=',num2str(fc_actual,'%.1f'),'Hz']);
plot([fr, fr], ylim, 'b--', 'LineWidth',1.2, 'DisplayName',['fr=',num2str(fr,'%.1f'),'Hz']);
plot([fs_6, fs_6], ylim, 'g--', 'LineWidth',1.2, 'DisplayName',['fs/6=',num2str(fs_6,'%.1f'),'Hz']);
plot(xlim, [0, 0], 'k:', 'LineWidth',1.0); % 0dB线
ylabel('幅值 (dB)', 'FontSize',11);
title('LCL并网逆变器 校正后开环传递函数伯德图', 'FontSize',12);
legend('Location','best', 'FontSize',9);
ylim([-60, 100]);

% 相频特性
subplot(2,1,2);
semilogx(f_log, phase_deg_unwrap, 'k-', 'LineWidth',1.5);
hold on; grid on;
% 标注关键频率点
plot([fc_actual, fc_actual], ylim, 'r--', 'LineWidth',1.2);
plot([fr, fr], ylim, 'b--', 'LineWidth',1.2);
plot([fs_6, fs_6], ylim, 'g--', 'LineWidth',1.2);
% 标注-180°线和-360°线
plot(xlim, [-180, -180], 'k:', 'LineWidth',1.0);
plot(xlim, [-360, -360], 'k:', 'LineWidth',1.0);
xlabel('频率 (Hz)', 'FontSize',11);
ylabel('相位 (°)', 'FontSize',11);