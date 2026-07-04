% 文件名：ORC_m_kr_Design_Paper.m
% 功能：根据确定好的kp、Q(z)、S(z)等参数对奇数次重复控制器Zm和kr的参数进行设计
% 严格按论文思路，遍历m=1~10选最优m，计算kr范围，遍历kr绘制奈奎斯特曲线
% 论文逻辑：m选最优 → 按式(16)算kr上限 → 遍历kr绘制H(e^jωTs)奈奎斯特曲线验证

%% ===================== 1. 系统基础参数 =====================
clear; clc; close all;

% 核心控制参数
kp = 1;
Q = 0.95;
Ts = 1e-4;
fs = 1/Ts;
f1 = 50;
N = fs/f1;  % 基波周期采样点数N=200

% 被控对象p(z)
num_p = [0.258560 0.322314 -0.131051 -0.194805];
den_p = [1.000000 -1.017316 0.638725 -0.366391];

% 论文S(z)
num_S = [0.004824, 0.0193, 0.02895, 0.0193, 0.004824];
den_S = [1, -2.3695, 2.314, -1.0547, 0.1874];

%% ===================== 2. 前置验证：P0(z)稳定性 =====================
num_P0 = num_p;
den_P0 = den_p + kp * num_p;
P0_z = tf(num_P0, den_P0, Ts);
poles_P0 = pole(P0_z);

disp('==================== 【前置验证：P0(z)稳定性】 ====================');
disp(['P0(z)极点最大模长：', num2str(max(abs(poles_P0)))]);
if max(abs(poles_P0)) < 1
    disp('✅ P0(z)稳定');
else
    disp('❌ P0(z)失稳');
    return;
end

%% ===================== 3. 遍历m=1~10，计算总相移 =====================
m_list = 1:10;
num_m = length(m_list);
f_range = 0:1:1000;
omega_range = 2*pi*f_range;

[H_P0, ~] = freqz(num_P0, den_P0, f_range, fs);
[H_S, ~] = freqz(num_S, den_S, f_range, fs);
theta_P0 = angle(H_P0);
theta_S = angle(H_S);
Np = abs(H_P0);
Ns = abs(H_S);

phase_total = zeros(num_m, length(f_range));
gamma_abs_mean = zeros(num_m, 1);

for i = 1:num_m
    m = m_list(i);
    theta_m = m * omega_range * Ts;
    gamma_omega = unwrap(theta_P0 + theta_S + theta_m);
    phase_total(i,:) = gamma_omega;
    gamma_abs_mean(i) = mean(abs(gamma_omega));
end

%% ===================== 4. 选最优m值 =====================
[min_mean, opt_idx] = min(gamma_abs_mean);
m_opt = m_list(opt_idx);
gamma_opt = phase_total(opt_idx,:);

disp(' ');
disp('==================== m=1~10 相移计算结果 ====================');
for i = 1:num_m
    disp(['m=', num2str(m_list(i)), '  平均相移绝对值：', num2str(rad2deg(gamma_abs_mean(i))), ' °']);
end
disp(' ');
disp('==================== 最优m值结果 ====================');
disp(['【最优m值】m=', num2str(m_opt)]);

%% ===================== 5. 按论文式(16)计算kr范围 =====================
term1 = 1/(1/Q + 1);
cos_gamma = cos(gamma_opt);
min_cos_gamma = min(cos_gamma);
term2 = 2 * min_cos_gamma;
Np_Ns = Np .* Ns;
max_Np_Ns = max(Np_Ns);
term3 = max_Np_Ns;

kr_max_theory = term1 * term2 / term3;
kr_max_floor = floor(kr_max_theory);

disp(' ');
disp('==================== kr取值范围计算 ====================');
disp(['kr理论最大值：', num2str(kr_max_theory)]);
disp(['kr稳定取值范围：0 < kr < ', num2str(kr_max_floor)]);
disp(['工程推荐kr：', num2str(floor(kr_max_floor*0.7))]);

%% ===================== 6. 绘制相频特性图 =====================
figure('Color','w','Position',[100,100,800,500]);
ax = gca;
hold(ax, 'on'); grid(ax, 'on');

color_map = colormap(jet(num_m));
for i = 1:num_m
    plot(ax, f_range, rad2deg(phase_total(i,:)), ...
        'LineWidth', 1.5, 'Color', color_map(i,:), ...
        'DisplayName', ['m=', num2str(m_list(i))]);
end
yline(ax, 0, 'k--', 'LineWidth', 1.2, 'DisplayName','0°参考线');

hx = xlabel(ax, '频率 f/Hz');
hy = ylabel(ax, '相位 /°');
ht = title(ax, 'S(z)·P0(z)·z^m 相频特性');
hl = legend(ax, 'Location','best');

try
    set(hx, 'FontSize', 12, 'FontName', 'SimHei');
    set(hy, 'FontSize', 12, 'FontName', 'SimHei');
    set(ht, 'FontSize', 14, 'FontName', 'SimHei');
    set(hl, 'FontSize', 10, 'FontName', 'SimHei');
catch
    set(hx, 'FontSize', 12);
    set(hy, 'FontSize', 12);
    set(ht, 'FontSize', 14);
    set(hl, 'FontSize', 10);
end

ax.XAxis.FontName = 'Times New Roman';
ax.YAxis.FontName = 'Times New Roman';
ax.FontSize = 10;
xlim(ax, [0, 1000]);
ylim(ax, [-150, 270]);

%% ===================== 7. 绘制幅频特性图 =====================
figure('Color','w','Position',[100,600,800,500]);
ax2 = gca;
hold(ax2, 'on'); grid(ax2, 'on');

plot(ax2, f_range, 20*log10(Np_Ns), 'LineWidth', 1.5);

hx2 = xlabel(ax2, '频率 f/Hz');
hy2 = ylabel(ax2, '幅值 /dB');
ht2 = title(ax2, 'S(z)·P0(z) 幅频特性');

try
    set(hx2, 'FontSize', 12, 'FontName', 'SimHei');
    set(hy2, 'FontSize', 12, 'FontName', 'SimHei');
    set(ht2, 'FontSize', 14, 'FontName', 'SimHei');
catch
    set(hx2, 'FontSize', 12);
    set(hy2, 'FontSize', 12);
    set(ht2, 'FontSize', 14);
end

ax2.XAxis.FontName = 'Times New Roman';
ax2.YAxis.FontName = 'Times New Roman';
ax2.FontSize = 10;
xlim(ax2, [0, 1000]);
ylim(ax2, [-80, -10]);

%% ===================== 8. 新增：遍历kr=0.2~2.4，绘制奈奎斯特曲线（和论文图8一致） =====================
% 论文定义：H(e^jωTs) = Q·z^(-N/2)·[kr·z^m·S·P0 - 1]
% 遍历范围：kr=0.2:0.2:2.4
kr_list = 0.2:0.2:2.4;
num_kr = length(kr_list);

% 全频段频率范围：0~fs/2（奈奎斯特频率），完整绘制曲线
f_nyq = 0:1:fs/2;
omega_nyq = 2*pi*f_nyq;

% 预计算全频段的P0(z)和S(z)频率特性
[H_P0_nyq, ~] = freqz(num_P0, den_P0, f_nyq, fs);
[H_S_nyq, ~] = freqz(num_S, den_S, f_nyq, fs);

% 预计算z^(-N/2)和z^m的频率响应
z_N2 = exp(-1j * omega_nyq * Ts * N/2);  % z^(-N/2)
z_m_opt = exp(1j * omega_nyq * Ts * m_opt); % z^m_opt

% -------------------------- 图8(a)：多kr奈奎斯特曲线 --------------------------
figure('Color','w','Position',[100,100,600,600]);
ax3 = gca;
hold(ax3, 'on'); grid(ax3, 'on'); axis(ax3, 'equal');

% 绘制单位圆（稳定性边界）
theta_circle = linspace(0, 2*pi, 1000);
plot(ax3, cos(theta_circle), sin(theta_circle), 'k--', 'LineWidth', 1.5, 'DisplayName','单位圆');
plot(ax3, [-1.2, 1.2], [0, 0], 'k-', 'LineWidth', 1);
plot(ax3, [0, 0], [-1.2, 1.2], 'k-', 'LineWidth', 1);

% 遍历每个kr，绘制H的奈奎斯特曲线
color_map_kr = colormap(jet(num_kr));
for i = 1:num_kr
    kr = kr_list(i);
    % 论文定义的H(e^jωTs)
    H_nyq = Q * z_N2 .* (kr * z_m_opt .* H_S_nyq .* H_P0_nyq - 1);
    plot(ax3, real(H_nyq), imag(H_nyq), ...
        'LineWidth', 1.5, 'Color', color_map_kr(i,:), ...
        'DisplayName', ['k_r=', num2str(kr)]);
end

% 标签设置
hx3 = xlabel(ax3, '实轴');
hy3 = ylabel(ax3, '虚轴');
ht3 = title(ax3, 'H(e^{jωT_s})的奈奎斯特曲线（多k_r）');
hl3 = legend(ax3, 'Location','best');

try
    set(hx3, 'FontSize', 12, 'FontName', 'SimHei');
    set(hy3, 'FontSize', 12, 'FontName', 'SimHei');
    set(ht3, 'FontSize', 14, 'FontName', 'SimHei');
    set(hl3, 'FontSize', 9, 'FontName', 'SimHei');
catch
    set(hx3, 'FontSize', 12);
    set(hy3, 'FontSize', 12);
    set(ht3, 'FontSize', 14);
    set(hl3, 'FontSize', 9);
end

ax3.XAxis.FontName = 'Times New Roman';
ax3.YAxis.FontName = 'Times New Roman';
ax3.FontSize = 10;
xlim(ax3, [-1.2, 1.2]);
ylim(ax3, [-1.2, 1.2]);

% -------------------------- 图8(b)：推荐kr的单独奈奎斯特曲线 --------------------------
% 工程推荐kr取理论最大值的70%，如果小于0.2则取0.2
kr_recommend = max(floor(kr_max_floor*0.7), 0.2);
% 确保推荐kr在遍历列表里
if ~ismember(kr_recommend, kr_list)
    kr_recommend = kr_list(find(kr_list >= kr_recommend, 1));
end

figure('Color','w','Position',[800,100,600,600]);
ax4 = gca;
hold(ax4, 'on'); grid(ax4, 'on'); axis(ax4, 'equal');

% 绘制单位圆
plot(ax4, cos(theta_circle), sin(theta_circle), 'k--', 'LineWidth', 1.5, 'DisplayName','单位圆');
plot(ax4, [-1.2, 1.2], [0, 0], 'k-', 'LineWidth', 1);
plot(ax4, [0, 0], [-1.2, 1.2], 'k-', 'LineWidth', 1);

% 绘制推荐kr的H曲线
H_nyq_recommend = Q * z_N2 .* (kr_recommend * z_m_opt .* H_S_nyq .* H_P0_nyq - 1);
plot(ax4, real(H_nyq_recommend), imag(H_nyq_recommend), ...
    'b-', 'LineWidth', 1.5, 'DisplayName', ['k_r=', num2str(kr_recommend)]);

% 标注50Hz、750Hz频点（和论文一致）
f_mark = [50, 750];
[H_P0_mark, ~] = freqz(num_P0, den_P0, f_mark, fs);
[H_S_mark, ~] = freqz(num_S, den_S, f_mark, fs);
z_N2_mark = exp(-1j * 2*pi*f_mark*Ts*N/2);
z_m_mark = exp(1j * 2*pi*f_mark*Ts*m_opt);
H_mark_total = Q * z_N2_mark .* (kr_recommend * z_m_mark .* H_S_mark .* H_P0_mark - 1);

for i = 1:length(f_mark)
    plot(ax4, real(H_mark_total(i)), imag(H_mark_total(i)), 'ko', 'MarkerSize', 6, 'MarkerFaceColor','k');
    text(ax4, real(H_mark_total(i))+0.05, imag(H_mark_total(i))+0.05, [num2str(f_mark(i)), ' Hz'], 'FontSize', 10);
end

% 标签设置
hx4 = xlabel(ax4, '实轴');
hy4 = ylabel(ax4, '虚轴');
ht4 = title(ax4, ['k_r=', num2str(kr_recommend), '时H(e^{jωT_s})的奈奎斯特曲线']);
hl4 = legend(ax4, 'Location','best');

try
    set(hx4, 'FontSize', 12, 'FontName', 'SimHei');
    set(hy4, 'FontSize', 12, 'FontName', 'SimHei');
    set(ht4, 'FontSize', 14, 'FontName', 'SimHei');
    set(hl4, 'FontSize', 10, 'FontName', 'SimHei');
catch
    set(hx4, 'FontSize', 12);
    set(hy4, 'FontSize', 12);
    set(ht4, 'FontSize', 14);
    set(hl4, 'FontSize', 10);
end

ax4.XAxis.FontName = 'Times New Roman';
ax4.YAxis.FontName = 'Times New Roman';
ax4.FontSize = 10;
xlim(ax4, [-1.2, 1.2]);
ylim(ax4, [-1.2, 1.2]);

% 输出奈奎斯特验证结果
disp(' ');
disp('==================== 奈奎斯特稳定性验证 ====================');
disp(['遍历kr范围：0.2~2.4，步进0.2']);
disp(['推荐kr值：', num2str(kr_recommend)]);
disp('✅ 若H(e^{jωT_s})的轨迹完全在单位圆内，则系统稳定');