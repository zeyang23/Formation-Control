clear all; close all; clc

params = load_params();
quad_a = Quadrotor(params);

dt = 0.01;
quad_a.dt = dt;

duration = 12;
tspan = 0:dt:duration;

%input
k = 4;
n = 6;
P(:,1) = zeros(3,1);
for i = 1:n
    P(1,i) = cos((i-1)/n*2*pi)-1;
    P(2,i) = sin((i-1)/n*2*pi);
    P(3,i) = 1;
end
load waypts;
P = waypts;
P(3,:) = P(3,:)+1;
P = [[0;0;0],P];
% load scan.mat;
% P = P(:,1:15);

[pt,vt,at,Jt] = BSplineC(P,k,tspan,[3,4,3],0,1,2,2);

%%
for k = 1:length(tspan)
    t = tspan(k);
    p = pt(:,k); v = vt(:,k); a = at(:,k); J = Jt(:,k); yaw = 0; yd = 0;
    
    noise = 0;
    p_c = quad_a.position + randn(1)*noise;
    v_c = quad_a.velocity + randn(1)*noise;
    omg_c = quad_a.Omega + randn(1)*noise;

    [u1,u2] = controller(p,v,a,J,yaw,yd,p_c, v_c, quad_a.attitude, omg_c, quad_a.m, quad_a.g,1.5,0.8,diag([1,1,1.2]),diag([0.1,0.1,1.2]));
    
    spd = get_rotorspeed(u1,u2,quad_a.k,quad_a.L,quad_a.b);
    
    quad_a.updateState(spd)

    traj_d(:,k) = p;
    yaw_d(:,k) = yaw;
    yawd_d(:,k) = yd;
    % static_quadrotor_plot(quad_a.position, quad_a.attitude);
    % hold on
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