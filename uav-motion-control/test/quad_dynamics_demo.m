clear
clc

%加载参数
params=load_params();  %加载参数
m = params.m;
g = params.g;
k = params.k;
kd = params.kd;
I = params.I;
L = params.L;
b = params.b;

% 模拟时长
start_time = 0;
end_time = 10;
dt = 0.005;
times = start_time:dt:end_time;

% 初始状态
r = [0; 0; 0];
rdot = zeros(3, 1);
euler = [pi/3;pi/3;pi/3];
eulerdot = 0.5 * ones(3,1);
omega=eulerdot2omega(eulerdot,euler);
rotorspeeds=2*[1 1 1 1];

trajectories(:,1)=r;
attitudes(:,1)=euler;

for i=1:length(times)
    
    % 计算线加速度和角加速度
    R = zyxEuler2rotMat(euler);
    a = rotor2acc(rotorspeeds, R, rdot, m, g, k, kd);
    omegadot = angular_acceleration(rotorspeeds, omega, I, L, b, k);
    
    % 更新参数
    eulerdot = omega2eulerdot(omega, euler);
    euler = euler + dt * eulerdot;
    omega = omega + dt * omegadot;
    
    r = r + dt * rdot;
    rdot = rdot + dt * a;
    % 记录当前时刻的位置和姿态
    trajectories(:,i)=r;
    attitudes(:,i)=euler;
    
    % todo 根据控制器更新rotorspeeds
    
    
    % todo 可视化
    % [~, fig]=static_quadrotor_plot(r,euler);
    % hold on
    % plot3(trajectories(1,:),trajectories(2,:),trajectories(3,:))
    % hold off
    
    % pause(0.002);
end

plot3(trajectories(1,:),trajectories(2,:),trajectories(3,:))
view(45,45)