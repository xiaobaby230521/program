%% 1:1复刻论文 无电容电流阻尼LCL并网逆变器开环伯德图（参数已全部确定）
clear; clc; close all;

%% ===================== 1. 可修改参数区（论文目标值+系统参数） =====================
% --- 目标值 ---
fc_target = 1100;    % 目标截止频率 fc，单位：Hz
PM_target =45;       % 目标相位裕度 PM，单位：°
T_fo_target = 73;   % 目标50Hz基波增益 T_fo，单位：dB
GM2_target = 3;     % 目标fs/6处幅值裕度 GM2，单位：dB

% --- 系统硬件参数 ---
L1 = 1.2e-3;     % 逆变侧电感 (H)
L2 = 120e-6;     % 网侧电感 (H)
C  = 10e-6;      % 滤波电容 (F)
fs = 20e3;        % 采样频率 Hz
Ts = 1/fs;        % 采样周期 s
Hi2 = 1;       % 电流反馈系数
Kpwm = 45;     % PWM增益
Kp = 0.2;        % 论文PR比例增益
Kr = 50;          % 论文PR谐振增益
wo = 2*pi*50;     % 基波角频率
wi = pi;          % 谐振带宽

% 计算关键频率
fr = 1/(2*pi*sqrt((L1*L2)/(L1+L2)*C)); % LCL谐振频率
f_s6 = fs/6;                              % fs/6 论文关键分界频率

%% ===================== 2. 开环传递函数（二阶Padé） =====================
s = tf('s');
% PR调节器（论文标准形式）
Gi = Kp + (2*Kr*wi*s)/(s^2 + 2*wi*s + wo^2);
% 二阶Padé近似1.5Ts延迟
[num_delay, den_delay] = pade(1.5*Ts, 2);
G_delay = tf(num_delay, den_delay);
% LCL滤波器传递函数
den_LCL = [L1*L2*C, 0, (L1+L2), 0];
G_LCL = tf(1, den_LCL);
% 开环传递函数
T_D_nodamp = tf(Hi2 * Kpwm) * G_delay * Gi * G_LCL;

%% ===================== 3. 核心修正：用连续相位精准定位穿越点 =====================
% 生成全频段高分辨率频率点
f = logspace(1, 4, 10000); % 提升分辨率，避免穿越点遗漏
w = 2*pi*f;
mag = zeros(size(f));
phase_raw = zeros(size(f));

for i = 1:length(f)
    [m, p] = bode(T_D_nodamp, w(i));
    mag(i) = 20*log10(squeeze(m));
    phase_raw(i) = squeeze(p);
end

% --- 第一步：先做相位负向解包裹，得到连续无翻转的相位曲线 ---
phase_wrapped = mod(phase_raw + 180, 360) - 180;
phase_unwrapped = zeros(size(phase_wrapped));
phase_unwrapped(1) = phase_wrapped(1);
for i = 2:length(phase_wrapped)
    delta = phase_wrapped(i) - phase_wrapped(i-1);
    if delta > 300
        phase_unwrapped(i) = phase_unwrapped(i-1) + delta - 360;
    else
        phase_unwrapped(i) = phase_unwrapped(i-1) + delta;
    end
end

% --- 1. 找第一次0dB穿越点（fc） ---
zero_cross_idx = find(diff(sign(mag)) < 0, 1);
fc = f(zero_cross_idx);
mag_fc = mag(zero_cross_idx);
% 修正PM计算
phase_fc = mod(phase_raw(zero_cross_idx) + 180, 360) - 180;
PM = 180 + phase_fc;

% --- 2. 核心修正：用连续相位找fc之后的真实-180°穿越点 ---
% 只关注fc之后的频段
fc_after_idx = find(f > fc, 1);
phase_continuous_after = phase_unwrapped(fc_after_idx:end);
f_after = f(fc_after_idx:end);

% 找相位从大于-180° → 小于-180°的负向穿越点（连续相位无翻转，100%准确）
phase_cross_idx_after = find(diff(sign(phase_continuous_after + 180)) < 0, 1);
% 空值保护
if isempty(phase_cross_idx_after)
    [~, phase_cross_idx_after] = min(abs(phase_continuous_after + 180));
end
% 还原到全频段的索引
phase_180_cross_idx = fc_after_idx - 1 + phase_cross_idx_after;

% 提取穿越点参数
fg = f(phase_180_cross_idx);
mag_fg = mag(phase_180_cross_idx);
GM = -mag_fg;

% --- 3. fs/6处的幅值裕度GM2和相位 ---
[mag_s6, ~] = bode(T_D_nodamp, 2*pi*f_s6);
mag_s6_dB = 20*log10(squeeze(mag_s6));
GM2 = -mag_s6_dB;
% 找到fs/6对应的连续相位
[~, f_s6_idx] = min(abs(f - f_s6));
phase_f_s6 = phase_unwrapped(f_s6_idx);

% --- 4. 50Hz基波增益T_fo ---
[mag_fo, ~] = bode(T_D_nodamp, 2*pi*50);
T_fo_dB = 20*log10(squeeze(mag_fo));

% --- 5. 找到LCL谐振频率fr对应的真实相位 ---
[~, fr_idx] = min(abs(f - fr));
phase_fr = phase_unwrapped(fr_idx);

%% ===================== 4. 1:1复刻论文绘图 =====================
figure('Color','w','Position',[100,100,900,700]);

% --- 上半部分：幅值图 ---
ax1 = subplot(2,1,1);
semilogx(f, mag, 'k', 'LineWidth',1.5);
grid on;
ylabel('|T_D|/dB');
title('图 8.12 采用滤波器 III 的无阻尼并网电流环开环波特图');
set(ax1, 'FontSize',12, 'XLim',[10, 1e4], 'YLim',[-50, 100]);
xticklabels({});

% 标注fc
hold(ax1, 'on');
plot(fc, mag_fc, 'ro', 'MarkerSize',8);
text(fc*1.1, mag_fc+10, ...
    sprintf('fc=%.2fHz\n幅值=%.2fdB\nPM=%.2f°', fc, mag_fc, PM), ...
    'Color','red','FontSize',10);
hold(ax1, 'off');

% --- 下半部分：相位图 ---
ax2 = subplot(2,1,2);
semilogx(f, phase_unwrapped, 'k', 'LineWidth',1.5);
grid on;
xlabel('频率 (Hz)');
ylabel('∠T_D/(°)');
set(ax2, 'FontSize',12, 'XLim',[10, 1e4], 'YLim',[-540, 0]);

% 标注真实-180°穿越点
hold(ax2, 'on');
plot(fg, -180, 'bo', 'MarkerSize',8);
text(fg*1.1, -180-30, ...
    sprintf('fg=%.2fHz\n幅值=%.2fdB\nGM2=%.2fdB', fg, mag_fg, GM), ...
    'Color','blue','FontSize',10);
hold(ax2, 'off');

% --- 论文关键频率点垂直虚线 ---
fo = 50;
fc_paper = 1100;
line([fo fo], get(ax2,'YLim'),'Color','k','LineStyle',':');
line([fc_paper fc_paper], get(ax2,'YLim'),'Color','k','LineStyle',':');
line([f_s6 f_s6], get(ax2,'YLim'),'Color','r','LineStyle','--');
line([fr fr], get(ax2,'YLim'),'Color','k','LineStyle',':');
text(fo, -50, 'f_o', 'HorizontalAlignment','center','FontSize',10);
text(fc_paper, -50, 'f_c', 'HorizontalAlignment','center','FontSize',10);
text(f_s6, -50, 'f_s/6', 'HorizontalAlignment','center','Color','r','FontSize',10);
text(fr, -50, 'f_r', 'HorizontalAlignment','center','FontSize',10);

%% ===================== 5. 按你的要求输出参数（仿真结果GM2改为真实穿越点的GM） =====================
fprintf('====================================================\n');
fprintf('          最终精准版 系统参数汇总          \n');
fprintf('====================================================\n');
fprintf('论文目标：fc=%.1fHz | PM>%.1f° | T_fo>%.1fdB | GM2>%.1fdB\n', fc_target, PM_target, T_fo_target, GM2_target);
fprintf('仿真结果：fc=%.2fHz | PM=%.2f° | T_fo=%.2fdB | GM2=%.2fdB\n', fc, PM, T_fo_dB, GM); % 核心修改：最后一个参数从GM2改为GM
fprintf('LCL谐振频率fr=%.2fHz | 真实相位=%.2f°\n', fr, phase_fr);
fprintf('fs/6处：频率=%.2fHz | 相位=%.2f° | 幅值裕度GM=%.2fdB\n', f_s6, phase_f_s6, GM2);
fprintf('真实穿越-180°处：频率=%.2fHz | 幅值裕度GM2=%.2fdB\n', fg, GM);
fprintf('====================================================\n');


