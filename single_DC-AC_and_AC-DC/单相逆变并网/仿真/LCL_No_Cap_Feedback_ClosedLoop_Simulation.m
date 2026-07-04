%LCL_No_Cap_Feedback_ClosedLoop_Simulation.m
% 无电容电流反馈单相LCL并网逆变器闭环伯德图
% 无电容电流前馈版本
% （彻底修复LCL模型+语法错误+统一图一纵坐标+有功功率+功率因数计算）
clear; clc; close all;

%% ===================== 1. 系统参数区 =====================
% --- 硬件参数 ---
L1 = 1.2e-3;     % 逆变侧电感 (H)
L2 = 120e-6;     % 网侧电感 (H)
C  = 10e-6;      % 滤波电容 (F)
R1 = 0.01;       % L1等效串联电阻 (Ω)
R2 = 0.01;       % L2等效串联电阻 (Ω)
Rc = 0.01;       % C等效串联电阻 (Ω)
Vdc = 45;        % 直流母线电压 (V)
Ug_rms = 24;     % 电网电压有效值 (V)

% --- 控制与采样参数（明确20kHz采样频率） ---
fs = 20e3;        % 【明确】采样频率 20kHz
Ts = 1/fs;        % 【明确】采样周期 5e-5s
T_total = 0.2;    % 总仿真时间 (s)
N = round(T_total / Ts); % 总采样点数

% --- PR控制器参数 ---
Kp = 0.2;        % 比例系数
Kr = 50;          % 谐振系数
wo = 2*pi*50;     % 基波角频率
wi = pi;          % 谐振带宽
MODULATION_MAX = 1;
MODULATION_MIN = -1;

% --- 延迟环节：1.5Ts延迟（先关闭，验证闭环稳定后再打开） ---
enable_delay = 1; % 0=关闭延迟，1=开启1.5Ts延迟
delay_Ts = 1.5*Ts;
[num_delay_cont, den_delay_cont] = pade(delay_Ts, 2);
G_delay_cont = tf(num_delay_cont, den_delay_cont);
G_delay_d = c2d(G_delay_cont, Ts, 'tustin'); % 【使用20kHz Ts离散化】
delay_num = cell2mat(G_delay_d.Numerator);
delay_den = cell2mat(G_delay_d.Denominator);

% --- 反馈配置 ---
Hi2 = 1;          % 电流反馈系数
feedback_sign = 1;% 标准负反馈，无需反转

% --- 参考电流参数 ---
Iref_amp = 2*sqrt(2); % 参考电流峰值 (A)，对应2A有效值

%% ===================== 2. 【核心修复】正确的LCL状态空间模型 =====================
% 状态变量：x = [iL1; uC; iL2]
A_LCL = [
    -(R1+Rc)/L1,   -1/L1,      Rc/L1;
    1/C,            0,          -1/C;
    Rc/L2,          1/L2,       -(R2+Rc)/L2
];
B_LCL = [
    1/L1,   0;
    0,      0;
    0,      -1/L2
];
C_LCL = eye(3);
D_LCL = zeros(3,2);

% 【使用20kHz Ts离散化】Tustin离散化，保证数值稳定
sys_LCL = ss(A_LCL, B_LCL, C_LCL, D_LCL);
sys_LCL_d = c2d(sys_LCL, Ts, 'tustin');
Ad_LCL = sys_LCL_d.A;
Bd_LCL = sys_LCL_d.B;
x_LCL = zeros(3,1); % 状态初始化

%% ===================== 3. PR控制器Tustin离散化（使用20kHz Ts） =====================
a0_pr = 4 + 2*wi*Ts + wo^2*Ts^2;
a1_pr = 2*wo^2*Ts^2 - 8;
a2_pr = 4 - 2*wi*Ts + wo^2*Ts^2;
b0_pr = 4*Kp + 4*Kp*wi*Ts + 4*Kr*wi*Ts + Kp*wo^2*Ts^2;
b1_pr = 2*Kp*wo^2*Ts^2 - 8*Kp;
b2_pr = 4*Kp - 4*Kp*wi*Ts - 4*Kr*wi*Ts + Kp*wo^2*Ts^2;

%% ===================== 4. 预分配内存 =====================
t = (0:N-1)*Ts; % 【使用20kHz Ts】时间轴
Ug = zeros(1, N);       % 电网电压 (V)
Iref = zeros(1, N);     % 参考电流 (A)
iL2 = zeros(1, N);      % 并网电流 (A)
error = zeros(1, N);    % 电流误差 (A)
m_pr = zeros(1, N);     % PR输出调制波
m_delay = zeros(1, N);  % 延迟后调制波
Vinv = zeros(1, N);     % 逆变输出电压 (V)

% 状态变量初始化
x1_pr = 0; x2_pr = 0;
y1_pr = 0; y2_pr = 0;
x1_delay = 0; x2_delay = 0;

%% ===================== 5. 主仿真循环 =====================
for k = 1:N
    % --------------------------
    % 1. 生成电网电压与参考电流（同相位，单位功率因数）
    % --------------------------
    theta = wo*t(k);
    Ug(k) = Ug_rms * sqrt(2) * sin(theta);
    Iref(k) = Iref_amp * sin(theta);
    
    % --------------------------
    % 2. 电流反馈与误差计算
    % --------------------------
    iL2_meas = x_LCL(3); % 直接从LCL状态读取并网电流
    iL2_feedback = feedback_sign * Hi2 * iL2_meas;
    error(k) = Iref(k) - iL2_feedback;
    
    % --------------------------
    % 3. PR控制器
    % --------------------------
    if k <= 2
        m_pr(k) = 0;
    else
        m_pr(k) = (b0_pr*error(k) + b1_pr*x1_pr + b2_pr*x2_pr ...
                  - a1_pr*y1_pr - a2_pr*y2_pr) / a0_pr;
    end
    m_pr(k) = max(min(m_pr(k), MODULATION_MAX), MODULATION_MIN);
    % 更新PR状态
    x2_pr = x1_pr; x1_pr = error(k);
    y2_pr = y1_pr; y1_pr = m_pr(k);
    
    % --------------------------
    % 4. 延迟环节（可开关）
    % --------------------------
    if enable_delay
        if k <= 2
            m_delay(k) = 0;
        else
            m_delay(k) = (delay_num(1)*m_pr(k) + delay_num(2)*x1_delay + delay_num(3)*x2_delay ...
                         - delay_den(2)*x1_delay - delay_den(3)*x2_delay) / delay_den(1);
        end
        m_delay(k) = max(min(m_delay(k), MODULATION_MAX), MODULATION_MIN);
        x2_delay = x1_delay; x1_delay = m_pr(k);
    else
        m_delay(k) = m_pr(k); % 关闭延迟，直接输出
    end
    
    % --------------------------
    % 5. 逆变输出电压（平均模型）
    % --------------------------
    Kpwm = Vdc;
    Vinv(k) = m_delay(k) * Kpwm;
    
    % --------------------------
    % 6. 【正确的】LCL滤波器状态更新
    % --------------------------
    u_LCL = [Vinv(k); Ug(k)];
    x_LCL = Ad_LCL * x_LCL + Bd_LCL * u_LCL;
    iL2(k) = x_LCL(3);
end

%% ===================== 6. 绘图（图一已改为同一纵坐标） =====================
figure('Color','w','Position',[100,100,1200,1000]);

% 子图1：【按要求修改】电网电压 vs 并网电流（同一纵坐标）
% 注意：电压和电流量纲不同，为了方便观察，对电流进行了10倍缩放
subplot(5,1,1);
plot(t, Ug, 'b--', 'LineWidth',1.5, 'DisplayName','电网电压 Ug (V)');
hold on;
plot(t, iL2 * 10, 'r-', 'LineWidth',1.2, 'DisplayName','并网电流 iL2 (×10 A)'); % 电流×10缩放
ylabel('幅值');
grid on;
title('修复后闭环仿真结果：电网电压 vs 并网电流（同一纵坐标，电流×10）');
legend('Location','best');
set(gca,'FontSize',10);

% 子图2：参考电流 vs 实际并网电流（跟踪效果）
subplot(5,1,2);
plot(t, Iref, 'b--', 'LineWidth',1.5, 'DisplayName','参考电流 Iref (A)');
hold on;
plot(t, iL2, 'r-', 'LineWidth',1.2, 'DisplayName','实际并网电流 iL2 (A)');
grid on;
ylabel('电流 (A)');
title('参考电流跟踪效果');
legend('Location','best');
set(gca,'FontSize',10);

% 子图3：电流误差
subplot(5,1,3);
plot(t, error, 'k-', 'LineWidth',1.2);
grid on;
ylabel('误差 (A)');
title('电流误差 error = Iref - 反馈电流');
set(gca,'FontSize',10);

% 子图4：PR输出调制波
subplot(5,1,4);
plot(t, m_pr, 'b-', 'LineWidth',1.2);
grid on;
ylabel('归一化幅值');
title('PR输出归一化调制波');
set(gca,'FontSize',10);

% 子图5：逆变输出电压
subplot(5,1,5);
plot(t, Vinv, 'm-', 'LineWidth',1);
grid on;
xlabel('时间 (s)');
ylabel('电压 (V)');
title('逆变器输出电压 Vinv');
set(gca,'FontSize',10);

%% ===================== 7. 关键指标输出（新增有功功率+功率因数计算） =====================
fprintf('====================================================\n');
fprintf('          彻底修复版 闭环仿真核心指标          \n');
fprintf('====================================================\n');
idx_last = find(t > 0.19, 1):N;
I2_rms = rms(iL2(idx_last));
error_rms = rms(error(idx_last));

% 有功功率计算
T_grid = 1/50;
N_grid = round(T_grid / Ts);
idx_power = (N - N_grid + 1):N;
p_inst = Ug(idx_power) .* iL2(idx_power);
P_active = mean(p_inst);

% 【新增】功率因数计算
% 视在功率 S = Ug_rms * I2_rms
S_apparent = Ug_rms * I2_rms;
% 功率因数 PF = P / S
PF = P_active / S_apparent;

fprintf('采样频率：%.0f Hz\n', fs);
fprintf('采样周期：%.6f s\n', Ts);
fprintf('稳态并网电流有效值：%.2f A（目标值：2A）\n', I2_rms);
fprintf('稳态电流跟踪误差：%.4f A\n', error_rms);
fprintf('并网有功功率：%.2f W\n', P_active);
fprintf('并网功率因数：%.4f\n', PF);
fprintf('PR输出调制波峰值：%.2f\n', max(abs(m_pr)));
if enable_delay
    fprintf('延迟功能：开启\n');
else
    fprintf('延迟功能：关闭\n');
end
fprintf('====================================================\n');


%% ===================== 8. 【新增】直接从当前代码提取系数，生成MATLAB Function =====================
fprintf('\n\n====================================================\n');
fprintf('          自动生成的二阶Padé延迟MATLAB Function代码          \n');
fprintf('====================================================\n');

% 1. 从当前代码提取并归一化系数
a0 = delay_den(1);
b0_func = delay_num(1)/a0;
b1_func = delay_num(2)/a0;
b2_func = delay_num(3)/a0;
a1_func = -delay_den(2)/a0; % 注意符号！
a2_func = -delay_den(3)/a0; % 注意符号！

% 2. 打印完整的函数代码
fprintf('function y = SecondOrderPadeDelay_1p5Ts(u)\n');
fprintf('%% 二阶Padé逼近1.5Ts延迟模块（直接从闭环仿真提取）\n');
fprintf('persistent u_prev1 u_prev2 y_prev1 y_prev2 first_run\n\n');
fprintf('if isempty(first_run)\n');
fprintf('    u_prev1 = 0;\n');
fprintf('    u_prev2 = 0;\n');
fprintf('    y_prev1 = 0;\n');
fprintf('    y_prev2 = 0;\n');
fprintf('    first_run = true;\n');
fprintf('end\n\n');

% 打印硬编码的系数
fprintf('%% 直接从闭环仿真提取的系数\n');
fprintf('b0 = %.15f;\n', b0_func);
fprintf('b1 = %.15f;\n', b1_func);
fprintf('b2 = %.15f;\n', b2_func);
fprintf('a1 = %.15f;\n', a1_func);
fprintf('a2 = %.15f;\n\n', a2_func);

fprintf('%% 二阶离散差分方程\n');
fprintf('y = b0*u + b1*u_prev1 + b2*u_prev2 ...\n');
fprintf('    + a1*y_prev1 + a2*y_prev2;\n\n');

fprintf('%% 更新状态变量\n');
fprintf('u_prev2 = u_prev1;\n');
fprintf('u_prev1 = u;\n');
fprintf('y_prev2 = y_prev1;\n');
fprintf('y_prev1 = y;\n');
fprintf('end\n');

fprintf('====================================================\n');
fprintf('请将上面的代码完整复制，保存为 SecondOrderPadeDelay_1p5Ts.m\n');
fprintf('====================================================\n');

% 3. 验证直流增益
G_delay_d_test = tf(delay_num, delay_den, Ts);
G_dc_test = evalfr(G_delay_d_test, 1);
fprintf('\n验证直流增益：%.10f（必须严格等于1）\n', abs(G_dc_test));