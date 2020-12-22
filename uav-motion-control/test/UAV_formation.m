clear all; close all;
num_uav = 10;
dt = 0.01;
params = load_params();
quads = cell(1,num_uav);
for i = 1:num_uav
    quads{i} = Quadrotor(params);
    quads{i}.dt = dt;
end

duration = 20;
tspan = 0:dt:duration;

% waypts = [0,0,0;
%            1,2,2.5;
%            2,3,3;
%            3,2,2.5;
%            4,0,2]';
% waypts = [
%     0,0,0;
%     rand(1,3);
%     2*rand(1,3);
%     3*rand(1,3);
%     4*rand(1,3);
%     5*rand(1,3);
% ].';

% load waypts;
% P = waypts;
% P(3,:) = P(3,:)+1;
% waypts = [[0;0;0],P];

load circle.mat
waypts = P(:,1:15);

v0 = [0,0,0];
a0 = [0,0,0];
v1 = [0,0,0];
a1 = [0,0,0];

[pt,vt,at,Jt] = min_snap_simple_fcn(waypts,v0,a0,v1,a1,duration,tspan);
disp(['max vt' num2str(max(vt(1,:)))])
disp(['max at' num2str(max(at(1,:)))])
disp(['max jt' num2str(max(Jt(1,:)))])
[Rt,Rdt] = jtraj(0,0,tspan);

for k = 1:length(tspan)
    t = tspan(k);
    p = pt(:,k); v = vt(:,k); a = at(:,k); J = Jt(:,k); yaw = Rt(k); yawd = Rdt(k);
    
    noise = 0;
    p_c = quads{1}.position + randn(1)*noise;
    v_c = quads{1}.velocity + randn(1)*noise;
    omg_c = quads{1}.Omega + randn(1)*noise;

    [u1,u2] = uav_controller(p,v,a,J,yaw,yawd,p_c, v_c, quads{1}.attitude, omg_c, quads{1}.m, quads{1}.g,0.5,0.8,diag([1,1,1.2]),diag([0.1,0.1,1.2]));
    
    rotorSpeeds = get_rotorspeed(u1,u2,quads{1}.k,quads{1}.L,quads{1}.b);
    
    quads{1}.updateState(rotorSpeeds);
    
    yaw_d(:,k) = yaw;
    yawd_d(:,k) = yawd;
end

traj = quads{1}.position_H;
traj(:,1) = [];

figure()
subplot(3,2,1)
plot3(traj(1,:), traj(2,:), traj(3,:), 'b')
hold on; grid on; view(45,45)
plot3(pt(1,:), pt(2,:), pt(3,:), 'r')
title('Simulation result'); legend('actual trajectory', 'desired trajectory')
axis equal
subplot(3,2,2)
plot(tspan, traj(1,:), tspan, pt(1,:))
xlabel('t'); ylabel('x'); legend('actual', 'desired')
title('x coordinates');
subplot(3,2,3)
plot(tspan, traj(2,:), tspan, pt(2,:))
xlabel('t'); ylabel('y'); legend('actual', 'desired')
title('y coordinates');
subplot(3,2,4)
plot(tspan, traj(3,:), tspan, pt(3,:))
xlabel('t'); ylabel('z'); legend('actual', 'desired')
title('z coordinates');
subplot(3,2,5)
plot(tspan,yaw_d,tspan,quads{1}.Omega_H(2,2:end))
legend('des','act')
title('yaw')
subplot(3,2,6)
plot(tspan,yawd_d,tspan,yaw_d)
grid on
legend('yawd','yaw')
title('yaw desired')

err = norm(traj - pt);
x_err = norm(traj(1,:) - pt(1,:));
y_err = norm(traj(2,:) - pt(2,:));
z_err = norm(traj(3,:) - pt(3,:));
fprintf('Error: %.8f\n', err);
fprintf('X Error: %.8f\n', x_err);
fprintf('Y Error: %.8f\n', y_err);
fprintf('Z Error: %.8f\n', z_err);
