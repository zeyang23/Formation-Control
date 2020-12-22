clear all; close all;

params = load_params();
quad_a = Quadrotor(params);

dt = 0.01;
quad_a.dt = dt;

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
    p_c = quad_a.position + randn(1)*noise;
    v_c = quad_a.velocity + randn(1)*noise;
    omg_c = quad_a.Omega + randn(1)*noise;

    [u1,u2] = quad_a.uav_controller(p,v,a,J,yaw,yawd,0.5,0.8,diag([1,1,1.2]),diag([0.1,0.1,1.2]));
    
    rotorSpeeds = get_rotorspeed(u1,u2,quad_a.k,quad_a.L,quad_a.b);
    
    quad_a.updateState(rotorSpeeds);
    
    yaw_d(:,k) = yaw;
    yawd_d(:,k) = yawd;
    
    % static_quadrotor_plot(quad_a.position, quad_a.attitude);
    % hold on
    % plot3(pt(1,:), pt(2,:), pt(3,:), 'b');
    % plot3(quad_a.position_H(1,:), quad_a.position_H(2,:), quad_a.position_H(3,:),'r');
    % title('real time result');
    % hold off
    % pause(dt)
end

traj = quad_a.position_H;
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
plot(tspan,yaw_d,tspan,quad_a.Omega_H(2,2:end))
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
