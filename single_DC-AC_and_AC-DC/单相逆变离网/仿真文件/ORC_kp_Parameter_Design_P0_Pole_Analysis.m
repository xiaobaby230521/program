% ORC_kp_Parameter_Design_P0_Pole_Analysis.m
% 功能：奇次谐波重复控制器KP参数设计（0.5~20全范围测试）
% 定义：P0(z) = p(z)/(1 + kp·p(z))，p(z)为离散被控对象
% 采样时间：Ts=1e-4s（10kHz）

%% ===================== 1. 初始化设置 =====================
clear; clc; close all;
Ts = 1e-4; % 采样周期

% 预定义中文字体（优先用黑体，兼容性最强，所有MATLAB都支持）
CN_FONT = 'SimHei';
EN_FONT = 'Times New Roman';

%% ===================== 2. 定义被控对象p(z) =====================
num_p = [0.258560 0.322314 -0.131051 -0.194805]; % 分子（z降幂）
den_p = [1.000000 -1.017316 0.638725 -0.366391]; % 分母（z降幂）
p_z = tf(num_p, den_p, Ts);
disp('==================== 被控对象p(z)离散传递函数 ====================');
disp(p_z);

%% ===================== 3. 定义kp取值序列（0.5~20，步进0.5） =====================
kp_list = 0.5:0.5:20; % 扩展范围：0.5~20，步进0.5
num_kp = length(kp_list);

% 绘图颜色配置：使用颜色渐变（从蓝到红，对应kp从小到大）
color_map = colormap(jet(num_kp));

%% ===================== 4. 循环计算：极点+相位裕度 =====================
real_pole_list = zeros(num_kp, 1); % 存储实极点
max_mag_list = zeros(num_kp, 1); % 存储极点最大模长
pm_list = zeros(num_kp, 1); % 存储相位裕度
gm_list = zeros(num_kp, 1); % 存储增益裕度
stable_list = false(num_kp, 1); % 存储稳定性标志

disp(' ');
disp('==================== 关键kp测试结果总结 ====================');
for i = 1:num_kp
    kp = kp_list(i);
    
    % --- 4.1 计算P0(z)极点 ---
    num_P0 = num_p;
    den_P0 = den_p + kp * num_p;
    poles = roots(den_P0);
    
    % 提取实极点（虚部<1e-6）
    real_pole = poles(abs(imag(poles)) < 1e-6);
    real_pole_list(i) = real_pole;
    max_mag = max(abs(poles));
    max_mag_list(i) = max_mag;
    
    % --- 4.2 计算开环相位裕度 ---
    G_open = kp * p_z; % 开环传递函数：G_open = kp·p(z)
    [Gm, Pm, ~, ~] = margin(G_open);
    pm_list(i) = Pm;
    gm_list(i) = 20*log10(Gm); % 转换为dB
    
    % --- 4.3 稳定性判断 ---
    if max_mag < 1
        stable_list(i) = true;
    else
        stable_list(i) = false;
    end
    
    % --- 4.4 仅输出关键kp的结果（避免刷屏） ---
    if ismember(kp, [0.5, 5, 8, 11, 14, 17, 20])
        disp(['kp = ', num2str(kp), ' 时：']);
        disp(['  实极点位置：', num2str(real_pole)]);
        disp(['  极点最大模长：', num2str(max_mag)]);
        disp(['  相位裕度 PM = ', num2str(Pm), ' °']);
        gm_dB = 20*log10(Gm);
        disp(['  增益裕度 GM = ', num2str(gm_dB), ' dB']);
        if stable_list(i)
            disp('  稳定性：所有极点在单位圆内，系统稳定');
        else
            disp('  稳定性：存在极点在单位圆外/上，系统不稳定');
        end
        disp('--------------------------------------------------');
    end
end

% --- 4.5 输出稳定性边界 ---
stable_idx = find(stable_list, 1, 'first');
stable_kp_min = kp_list(stable_idx);
stable_idx = find(stable_list, 1, 'last');
stable_kp_max = kp_list(stable_idx);

pm30_idx = find(pm_list>=30, 1, 'first');
pm30_kp_min = kp_list(pm30_idx);
pm30_idx = find(pm_list>=30, 1, 'last');
pm30_kp_max = kp_list(pm30_idx);

pm15_idx = find(pm_list>=15, 1, 'first');
pm15_kp_min = kp_list(pm15_idx);
pm15_idx = find(pm_list>=15, 1, 'last');
pm15_kp_max = kp_list(pm15_idx);

disp(' ');
disp('==================== 稳定性边界总结 ====================');
disp(['测试范围内，系统稳定的kp区间：[', num2str(stable_kp_min), ', ', num2str(stable_kp_max), ']']);
disp(['相位裕度≥30°的kp区间：[', num2str(pm30_kp_min), ', ', num2str(pm30_kp_max), ']']);
disp(['相位裕度≥15°的kp区间：[', num2str(pm15_kp_min), ', ', num2str(pm15_kp_max), ']']);

%% ===================== 5. 绘图1：全范围极点分布图 =====================
figure('Color','w','Position',[100,100,600,600]);
ax = gca;
hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');

% 绘制单位圆
theta = linspace(0, 2*pi, 1000);
plot(ax, cos(theta), sin(theta), 'k--', 'LineWidth', 1.5, 'DisplayName','单位圆');

% 绘制实轴、虚轴
plot(ax, [-1.2, 1.2], [0, 0], 'k-', 'LineWidth', 1);
plot(ax, [0, 0], [-1.2, 1.2], 'k-', 'LineWidth', 1);

% 绘制每个kp的极点（使用颜色渐变）
for i = 1:num_kp
    poles = roots(den_p + kp_list(i) * num_p);
    plot(ax, real(poles), imag(poles), 'o', ...
        'MarkerSize', 4, 'MarkerFaceColor', color_map(i,:), ...
        'MarkerEdgeColor', color_map(i,:), 'HandleVisibility','off');
end

% 格式美化（中文单独设置字体，避免覆盖）
xlabel('实轴', 'FontSize', 12, 'FontName', CN_FONT);
ylabel('虚轴', 'FontSize', 12, 'FontName', CN_FONT);
title('不同k_p取值时P_0(z)的极点分布图', 'FontSize', 14, 'FontName', CN_FONT);
xlim(ax, [-1.2, 1.2]);
ylim(ax, [-1.2, 1.2]);
legend(ax, 'Location','best', 'FontSize', 10, 'FontName', CN_FONT);
% 仅设置刻度数字为英文字体，不覆盖中文标签
ax.XAxis.FontName = EN_FONT;
ax.YAxis.FontName = EN_FONT;
ax.FontSize = 10;

% 添加颜色条（表示kp大小）
c = colorbar(ax);
c.Label.String = 'k_p取值（从小到大：蓝→红）';
c.Label.FontSize = 10;
c.Label.FontName = CN_FONT;
caxis([min(kp_list), max(kp_list)]);

%% ===================== 6. 绘图2：实极点区域放大图 =====================
figure('Color','w','Position',[800,100,600,400]);
ax = gca;
hold(ax, 'on'); grid(ax, 'on');

% 绘制每个kp的实极点（颜色渐变）
scatter(ax, real_pole_list, zeros(num_kp, 1), 30, kp_list, 'filled');

% 格式美化
xlabel('实轴（实极点位置）', 'FontSize', 12, 'FontName', CN_FONT);
ylabel('虚轴', 'FontSize', 12, 'FontName', CN_FONT);
title('实极点位置放大图', 'FontSize', 14, 'FontName', CN_FONT);
xlim(ax, [min(real_pole_list)-0.02, max(real_pole_list)+0.02]);
ylim(ax, [-0.1, 0.1]);
% 刻度数字用英文字体
ax.XAxis.FontName = EN_FONT;
ax.YAxis.FontName = EN_FONT;
ax.FontSize = 10;

% 添加颜色条
c = colorbar(ax);
c.Label.String = 'k_p取值';
c.Label.FontSize = 10;
c.Label.FontName = CN_FONT;
caxis([min(kp_list), max(kp_list)]);

%% ===================== 7. 绘图3：实极点随kp变化曲线 =====================
figure('Color','w','Position',[100,550,800,400]);
ax = gca;
hold(ax, 'on'); grid(ax, 'on');

plot(ax, kp_list, real_pole_list, 'b-', 'LineWidth', 1.5);

% 格式美化
xlabel('k_p取值', 'FontSize', 12, 'FontName', CN_FONT);
ylabel('实极点位置', 'FontSize', 12, 'FontName', CN_FONT);
title('实极点位置随k_p的变化曲线', 'FontSize', 14, 'FontName', CN_FONT);
xticks(ax, 0:2:20);
% 刻度数字用英文字体
ax.XAxis.FontName = EN_FONT;
ax.YAxis.FontName = EN_FONT;
ax.FontSize = 10;

%% ===================== 8. 绘图4：相位裕度随kp变化曲线 =====================
figure('Color','w','Position',[800,550,800,400]);
ax = gca;
hold(ax, 'on'); grid(ax, 'on');

% 绘制相位裕度曲线
plot(ax, kp_list, pm_list, 'r-', 'LineWidth', 1.5, 'DisplayName','相位裕度');

% 绘制工程参考线
yline(ax, 30, 'k--', 'LineWidth', 1.2, 'DisplayName','工程最优下限（30°）');
yline(ax, 15, 'k:', 'LineWidth', 1.2, 'DisplayName','鲁棒性最低要求（15°）');

% 格式美化
xlabel('k_p取值', 'FontSize', 12, 'FontName', CN_FONT);
ylabel('相位裕度（°）', 'FontSize', 12, 'FontName', CN_FONT);
title('相位裕度随k_p的变化曲线', 'FontSize', 14, 'FontName', CN_FONT);
xticks(ax, 0:2:20);
legend(ax, 'Location','best', 'FontSize', 10, 'FontName', CN_FONT);
% 刻度数字用英文字体
ax.XAxis.FontName = EN_FONT;
ax.YAxis.FontName = EN_FONT;
ax.FontSize = 10;