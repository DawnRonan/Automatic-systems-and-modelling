% 被控对象传递函数
num = [3.75];
den = [0.15, 1.0225, 0.15];
G = tf(num, den);

% 设置初始比例增益（我们将逐渐增加它）
Kp = 1;  % 初始比例增益
C = pid(Kp, 0, 0);  % 只使用比例控制器

% 使用负反馈闭环系统
sys_cl = feedback(C * G, 1);


% 尝试不同的 Kp 值，观察响应
Kp_values = 0:0.01:10;  % 增益的一个范围，可以根据需要调整
for Kp = Kp_values
    C = pid(Kp, 0, 0);  % 只使用比例控制器
    sys_cl = feedback(C * G, 1);
    
    % 绘制单位阶跃响应
    figure(1);
    step(sys_cl);
    title(['单位阶跃响应 Kp = ', num2str(Kp)]);
    pause(0.1);  % 暂停一下，便于观察响应的变化
end