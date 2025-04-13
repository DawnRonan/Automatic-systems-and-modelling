close all;
clear all;
% 被控对象传递函数
num = 3.75;                % 分子系数
den = [0.15 1.0225 0.15];          % 分母系数
G = tf(num, den)       % 被控对象 G(s)
% Ziegler-Nichols 方法的临界增益和周期
%Ku = 13;  % 假设的临界增益
%Pu = 1.2;   % 假设的临界周期
Ku = 13;  % 假设的临界增益
Pu = 1.2;   % 假设的临界周期

% 根据 Ziegler-Nichols 方法计算 PID 参数
Kp = 0.6 * Ku
Ki = 2 * Kp / Pu
Kd = Kp * Pu / 8

% 输出 PID 控制器增益
disp(['Kp: ', num2str(Kp)]);
disp(['Ki: ', num2str(Ki)]);
disp(['Kd: ', num2str(Kd)]);

% 创建 PID 控制器传递函数
%C = pid(Ku, 0, 0);
C = pid(Kp, Ki, Kd);
num2 = 1;
den2 = 1;
H = tf(num2,den2)
% 串联控制器与被控对象
sys_cl = feedback(C * G * H, 1)
t=[0:0.01:2];
% 设置单位阶跃输入信号
u = ones(size(t));  % 单位阶跃输入

% 仿真系统响应
figure;
lsim(sys_cl, u, t);
title('闭环系统的单位阶跃响应');
xlabel('时间 (秒)');
ylabel('输出');
