Kp = 1;
Ki = 1;
Kd = 1;
K = 1;
% 定义传递函数
num = [3.75*Kd*K 3.75*Kp*K 3.75*Ki*K];  % 分子系数
den = [0.15 1.0225 0.15 0];  % 分母系数
sys = tf(num, den)

% 转换为状态空间模型
[A, B, C, D] = tf2ss(num, den);

% 显示状态空间矩阵
disp('状态空间矩阵 A:');
disp(A);
disp('状态空间矩阵 B:');
disp(B);
disp('状态空间矩阵 C:');
disp(C);
disp('状态空间矩阵 D:');
disp(D);

desired_poles = [-2.7+3.12i, -2.7-3.12i, -20];  % 期望的闭环极点位置
% 使用 place 函数计算反馈增益矩阵 K
K = place(A, B, desired_poles)

% 闭环状态空间矩阵
A_cl = A - B * K;  % 闭环系统的状态矩阵
B_cl = B;          % 输入矩阵不变
C_cl = C;          % 输出矩阵不变
D_cl = D;          % 传递矩阵不变

% 创建闭环系统
sys_cl = ss(A_cl, B_cl, C_cl, D_cl)
% 从状态空间模型转换为传递函数
[num_cl, den_cl] = ss2tf(A_cl, B_cl, C_cl, D_cl)
sys = tf(num_cl, den_cl)
% 设置仿真时间
t = 0:0.01:5;  % 5秒的仿真时间

% 设置单位阶跃输入信号
u = ones(size(t));  % 单位阶跃输入

% 仿真系统响应
figure;
lsim(sys_cl, u, t);
title('闭环系统的单位阶跃响应 (lsim)');
xlabel('时间 (秒)');
ylabel('输出');