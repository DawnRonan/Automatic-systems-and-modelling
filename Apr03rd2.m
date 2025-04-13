clc; clear; close all;

% input signal
R3 = 1;
d = 0;  % 扰动 D(s)
N1 = 0;  % 传感器噪声 N(s)
N2 = 0;
N3 = 0;
% 定义拉普拉斯变量
s = tf('s');

% 定义传递函数 G(s) 和控制器 C(s)
G1 = 5 / (0.5*s + 1);  % 被控对象 G(s)
H1 = 0.7;  % 传感器（单位反馈）

% 计算闭环传递函数（输入 R(s) 到输出 Y(s)）
T1 = feedback(G1, H1);

% 计算扰动传递函数（D(s) 到 Y(s)）

Td1 = d * T1;

% 计算传感器噪声传递函数（N(s) 到 Y(s)）

Tn1 = -N1 * T1;

T1 = T1 + Td1 + Tn1;  % 更新闭环传递函数

T1 = 10 / s * T1;

G2 = T1;
H2 = 0.5;
T2 = feedback(G2, H2);


Tn2 = -N2 * T2;

T2 = T2 + Tn2;
T2 = (5 + (1 / s)) * (1/s) * T2;

G3 = T2;
H3 = 1;
T3 = feedback(G3, H3);

Tn3 = -N3 * T3;
E3 = R3 - T3 - Tn3;

disp('Error Signal E3(s):');
disp(E3);


figure;
subplot(3,1,1);
step(R3);
title('闭环系统对 R3 的阶跃响应');

subplot(3,1,2);
step(T3);
title('外部扰动 D(s) 对 Y(s) 的阶跃响应');

subplot(3,1,3);
step(E3);
title('传感器噪声 N3 对 Y(s) 的阶跃响应');