clear
P=bodeoptions;
P.Grid='on';
P.XLim={[10 100000]};%设置横轴范围
P.FreqUnits='Hz';%将横坐标单位换位HZ  
s=tf('s');
hold on;

load('date.mat')                   %扫频数据导入
freq = v10Abodedate(:,1);           %数据转换matlab格式
mag = v10Abodedate(:,2);
phase = v10Abodedate(:,3);

frequency = freq*2*pi;           %in rad/s
gain = 10.^(mag/20);             %dB转换增益
response = gain.*exp(1i*phase*pi/180);  %转换复数形式   
data = frd(response,frequency);     %构建频率响应模型
bode(data,P)    


%pi传递函数  零极点
% fz=1000;
% T=1/(2*pi*fz);
% T=0.000159155;
% k=10;   
% Gpi=k*(1+s*T)/(s*T);
% bode(Gpi,P);   hold on;  
% Gfp=1/(1+s*1/(2*pi*1000));
% Gfz=(1+s*1/(2*pi*1000));
% bode(Gfp,P);sys = tfest(data,3)                %拟合预估3个极点
% bode(sys)
% bode(Gfz,P);
% bode(Gfz*Gfp,P);
%改进PI   TPYE2
fz=700;
wz=1/(2*pi*fz);
k=0.12;%0.12;   
fp=30000;
wp=1/(2*pi*fp);
GTPYEII=k*(1+s*wz)/(s*wz)*1/(1+s*wp);
bode(GTPYEII,P);   hold on; 
bode(GTPYEII*data,P);
%PR
% Kp2 = 20; wc2 = 0.9*2*pi; Kr2 = 2700; w0 = 100*pi;
% G_QPR = Kp2 + Kr2*wc2*s / (s^2 + 2*wc2*s + w0^2)
% bode(G_QPR, P);



ts=20e-6;                         %%离散时间 
sysd=c2d(GTPYEII,ts,'tustin')       %%离散对象   离散时间   离散方式

[num_d,den_d]=tfdata(sysd,'v');   %%AB系数
vpa(num_d,8)                      %设置离散化得到的AB系数的有效位数   分子
vpa(den_d,8)                      %分母


 


