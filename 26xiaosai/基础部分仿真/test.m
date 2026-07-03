clear; clc;

%% 核心参数
Amp = 0.85;          
ARR = 8500;       
N   = 400;        

% 精准生成 400点正弦 0~2pi
theta = (0:N-1) * 2*pi / N;   
sin_standard = Amp * sin(theta);  

% ========== 核心：不缩0~1，直接 0~2 映射 ==========
temp      = sin_standard + 1;        % -1~1  →  0~2
CCR_Table = round( temp * ARR / 2 ); % 0~2   →  0~4249

% 输出原始正弦数组（保留你原来功能）
fprintf('===========================\n');
fprintf('标准正弦波数组：\n');
fprintf('===========================\n');
fprintf('[');
for i = 1:N
    fprintf('%.3f,', sin_standard(i));
end
fprintf('\b];\n\n');

% 输出 C 语言 CCR 数组
fprintf('const uint16_t CCR_SinTable[400] = {\n');
for i = 1:N
    fprintf('%d,', CCR_Table(i));
    if mod(i, 20) == 0
        fprintf('\n');
    end
end
fprintf('\b};\n');

% 双窗口绘图，完全对齐
figure;
stem(sin_standard, 'filled', 'b');
grid on;
title('原始正弦波 (-0.9 ~ 0.9)');
xlim([0 N]);

figure;
stem(CCR_Table, 'filled', 'r');
grid on;
title('CCR映射波形 (0 ~ 8500)');
xlim([0 N]);