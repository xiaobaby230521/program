%LCL_Undamped_fc_Kr_Constraint_Region.m
% 无阻尼LCL并网PR控制器 fc-Kr 约束区域求解（目标：获取满足 GM₂/T₍fo₎/PM 约束的最大截止频率 fc）
% 功能：根据 GM₂≥3dB、T₍fo₎≥73dB、PM≥45° 约束条件，绘制可行参数区域并计算关键交点
% 核心目标：在满足全部稳定/性能约束下，得到尽可能大的系统截止频率 fc
clear; clc; close all;

%% ===================== 1. 核心参数（fs=20kHz 工况 + 你的滤波器参数） =====================
% ---------- LCL 滤波器参数 ----------
L1 = 1.2e-3;     % 逆变侧电感 (H)
L2 = 120e-6;     % 网侧电感 (H)
C  = 10e-6;      % 滤波电容 (F)

% ---------- 采样与控制参数 ----------
fs = 20e3;       % 采样频率 (Hz)
Ts = 1/fs;       % 采样周期 (s)，Ts = 1/fs
Kpwm = 45;       % PWM 增益
Hi2  = 1;         % 网侧电流反馈系数

% ---------- 基础控制目标 ----------
fo   = 50;       % 电网基波频率 (Hz)
omega_i = pi;    % PR 调节器谐振角频率 (rad/s)
fi   = omega_i/(2*pi); % PR 调节器谐振频率 (Hz)

% ---------- 约束目标（论文要求） ----------
PM_target = 45;  % 相位裕度 PM ≥ 45°
GM2_target = 3;  % 幅值裕度 GM₂ ≥ 3dB
Tfo_target = 73; % 基波处环路增益 T₍fo₎ ≥ 73dB

%% ===================== 2. 谐振频率锁定（保证 GM₂ 约束位置） =====================
fr = 4600; % 论文滤波器 III 谐振频率 (Hz)，确保 GM₂ 竖线在 1000Hz 右侧
fprintf('=== 参数核对 ===\n');
fprintf('采样频率 fs：%.0f Hz → 采样周期 Ts=%.e s (%.f 微秒)\n', fs, Ts, Ts*1e6);
fprintf('谐振频率 fr：%.0f Hz\n', fr);

%% ===================== 3. 截止频率 fc 扫描范围 =====================
fc_min = 500;
fc_max = 2500;
fc = linspace(fc_min, fc_max, 500);

%% ===================== 4. 约束边界计算（严格对应论文公式 8.31/8.37/8.38） =====================
% ---------- 约束 1：T₍fo₎ ≥ 73dB → Kr_Tfo（公式 8.31） ----------
Kr_Tfo = (10^(Tfo_target/20)*fo - fc) * 2*pi*(L1+L2) / (Hi2*Kpwm);

% ---------- 约束 2：GM₂ ≥ 3dB → fc_GM_bound（公式 8.38） ----------
fc_GM_bound = 10^(-GM2_target/20) * (fs/6) * (fr^2 - (fs/6)^2) / fr^2;
fprintf('GM₂=3dB 约束边界 fc = %.2f Hz\n', fc_GM_bound);

% ---------- 约束 3：PM ≥ 45° → Kr_PM（公式 8.37） ----------
PM_rad = PM_target * pi/180;
angle_term = 3*pi*fc*Ts + PM_rad;
angle_term(angle_term > pi/2 - 0.05) = pi/2 - 0.05; % 防止 tan(π/2) 溢出
term = tan(angle_term);
Kr_PM = pi * fc.^2 .* (L1+L2) ./ (Kpwm * Hi2 * fi * term);

%% ===================== 5. 关键交点输出（GM₂ 竖线与两条约束曲线的交点） =====================
fprintf('\n=== GM₂ 约束边界关键交点 ===\n');
% 1. T₍fo₎ 曲线与 GM₂ 竖线交点
Kr_Tfo_GM2 = interp1(fc, Kr_Tfo, fc_GM_bound);
fprintf('1. T₍fo₎ 曲线 & GM₂ 竖线 交点：fc=%.2f Hz，Kr=%.2f\n', fc_GM_bound, Kr_Tfo_GM2);

% 2. PM 曲线与 GM₂ 竖线交点
Kr_PM_GM2 = interp1(fc, Kr_PM, fc_GM_bound);
fprintf('2. PM 曲线 & GM₂ 竖线 交点：fc=%.2f Hz，Kr=%.2f\n', fc_GM_bound, Kr_PM_GM2);

%% ===================== 6. 绘制可行约束区域 =====================
figure('Color','w','Position',[100,100,800,600]);
hold on; grid on;

% 绘制三条约束边界
plot(fc, Kr_Tfo, 'k-', 'LineWidth',1.5, 'DisplayName','T_{fo}=73dB 约束边界');
xline(fc_GM_bound, 'k--', 'LineWidth',1.5, 'DisplayName','GM_2=3dB 约束边界');
plot(fc, Kr_PM, 'k-.', 'LineWidth',1.5, 'DisplayName','PM=45° 约束边界');

% 绘制可行阴影区域（满足全部约束的 fc-Kr 组合）
idx = fc < fc_GM_bound;
fc_fill = [fc(idx), fliplr(fc(idx))];
Kr_fill = [Kr_Tfo(idx), fliplr(Kr_PM(idx))];
valid_idx = ~isinf(Kr_fill) & ~isnan(Kr_fill) & (Kr_fill > 0);
fc_fill = fc_fill(valid_idx);
Kr_fill = Kr_fill(valid_idx);
fill(fc_fill, Kr_fill, [0.8,0.8,0.8], 'FaceAlpha',0.3, 'EdgeColor','none');

% 标注典型工况点
your_fc = 1100;
your_Kr = interp1(fc, Kr_Tfo, your_fc);
plot(your_fc, your_Kr, 'ro', 'MarkerSize',6, 'DisplayName','典型工况点');
text(your_fc+50, your_Kr+5, sprintf('(%.0f, %.0f)', your_fc, round(your_Kr)));

% 图表标注
xlabel('f_c / Hz','FontSize',12);
ylabel('K_r','FontSize',12);
title('满足 GM₂/T₍fo₎/PM 约束的 fc-Kr 可行区域（目标：最大截止频率 fc）','FontSize',14);
xlim([500, 2500]);
ylim([0, 100]);
legend('Location','northeast','FontSize',10);
set(gca,'FontSize',10);