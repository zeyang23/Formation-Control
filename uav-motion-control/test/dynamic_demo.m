clear all; close all;
addpath(genpath('..'));
params = load_params();
quad_a = Quadrotor(params); save_gif = false; use_bspline = false;

% dt = 0.01; quad_a.dt = dt; end_t = 15; tspan = 0:dt:end_t;
% 
% load circle.mat;
% % P(3,:) = P(3,:)+1;
% % P = [[0;0;0;0],P];
% waypts = P(1:3,1:16);
% 
% psi_pts=P(4,1:16);
% psi_dot_0 = 0;
% psi_dot2_0 = 0;
% psi_dot_1 = 0;
% psi_dot2_1 = 0;
% 
% v0 = [0,0,0]; a0 = [0,0,0]; v1 = [0,0,0]; a1 = [0,0,0];
% 
% if use_bspline
%     [pt,vt,at,Jt] = BSplineC(waypts,4,tspan,[3,4,2.5],1,0);
    Kp1 = 0; Kd1 = 0; Kp2 = diag([0,0,0]); Kd2 = diag([0,0,0]);
% else
%     [pt,vt,at,Jt] = min_snap_simple_fcn(waypts,v0,a0,v1,a1,end_t,tspan);
%     Kp1 = 0.8; Kd1 = 0.5; Kp2 = diag([6,6,1.8]); Kd2 = diag([1.5,1.5,1]);
% %     Kp1 = 1.8; Kd1 = 5; Kp2 = diag([1.5,1.5,0.5]); Kd2 = diag([0.4,0.4,0.3]);
% end
% [yaw_t,yaw_dot_t]= min_psi_dot2(psi_pts,psi_dot_0,psi_dot2_0,psi_dot_1,psi_dot2_1,tspan(end),tspan(2)-tspan(1));

% if save_gif
%     figure()
%     plot3(pt(1,:), pt(2,:), pt(3,:), 'b');
%     hold on; grid on; 
%     plot3(waypts(1,:), waypts(2,:), waypts(3,:), 'ro') ;
%     view(45,45); title('real time result');
%     ax = [-1.5,4.5,-0.5,4.5,-2,6];
%     axis(ax);
% end
load('out.mat')
t = out1.STATES(:,1)';
pt = zeros(3,length(t));
vt = zeros(3,length(t));
at = zeros(3,length(t));
Jt = zeros(3,length(t));
pti(1,:) = out1.STATES(:,2)';
pti(2,:) = out1.STATES(:,4)';
pti(3,:) = out1.STATES(:,6)';
vti(1,:) = out1.STATES(:,3)';
vti(2,:) = out1.STATES(:,5)';
vti(3,:) = out1.STATES(:,7)';
yaw_ti = out1.STATES(:,12)';
yaw_dot_ti = out1.STATES(:,13)';
dt = 0.01; quad_a.dt = dt; end_t = 15; tspan = 0:dt:end_t;
pt = zeros(3,length(tspan));
vt = zeros(3,length(tspan));
pt(1,:) = spline(t,pti(1,:),tspan);
pt(2,:) = spline(t,pti(2,:),tspan);
pt(3,:) = spline(t,pti(3,:),tspan);
yaw_t = spline(t,yaw_ti,tspan);
yaw_dot_t = FDMinter3(tspan,yaw_t);
vt = FDMinter3(tspan,pt);
at = FDMinter3(tspan,vt);
Jt = FDMinter3(tspan,at);
omega_h = 912.109; %rad/s
rotorSpeeds = omega_h*ones(4,1);
quad_a.updateState(rotorSpeeds);

for k = 1:length(tspan)
    t = tspan(k);
    p = pt(:,k); v = vt(:,k); a = at(:,k); J = Jt(:,k);
    yaw = yaw_t(k); yaw_dot = yaw_dot_t(k);
    
    noise = 0.1;
    p_c = quad_a.position * (randn(1)*noise+1);
    v_c = quad_a.velocity * (randn(1)*noise+1);
    omg_c = quad_a.Omega * (randn(1)*noise+1);

    % 0,0: yaw, yaw_dot
    [u1,u2] = controller(p,v,a,J,yaw,yaw_dot,p_c, v_c, quad_a.attitude, omg_c, quad_a.m, quad_a.g, Kp1,Kd1,Kp2,Kd2);
    
    rotorSpeeds = get_rotorspeed(u1,u2,quad_a.k,quad_a.L,quad_a.b);
    
    quad_a.updateState(rotorSpeeds);
    
    if save_gif
        static_quadrotor_plot(quad_a.position, quad_a.attitude, quad_a.L, 1);
        hold on
        plot3(pt(1,:), pt(2,:), pt(3,:), 'b');
        plot3(quad_a.position_H(1,:), quad_a.position_H(2,:), quad_a.position_H(3,:),'r');
        title('real time result');
        hold off
%         ax = [-1,6,-1,6,-1,3]; % for scan
        ax = [-2.5,3.5,-2,4,-1,1.5]; % for circle
%         ax = [-10,1,-6,1,-1,3];
%         view(135,45)

        axis(ax);

        img = frame2im(getframe(gcf));
        [img, map] = rgb2ind(img, 256);
        if mod(k,10) == 2
            if k == 2
                imwrite(img, map, 'test.gif', 'gif', 'Loopcount', inf, 'DelayTime', dt);
            else
                imwrite(img, map, 'test.gif', 'gif', 'WriteMode', 'append', 'DelayTime', dt);
            end
        end
    end
end

traj = quad_a.position_H;
rotorSpeeds_H = quad_a.rotorspeeds;
rotorSpeeds_H_dot = FDMinter3(tspan,rotorSpeeds_H);
K_V = 920; %rpm/V
nB = 2;
rho = 1.225; %kg/m^3
mB = 0.0055; %kg
m = 1.3; %kg
Tf = 4e-2; %N*m
r = 0.12; %m
l = 0.175; %m
Df = 2e-4; %N*m*s/rad
epsilon = 0.004; %m
Ix = 0.081; %kg*m^2
Iy = 0.081; %kg*m^2
Iz = 0.142; %kg*m^2
R = 0.2; % Ohm
C_T = 0.0048;
Jm = 4.9e-6; %kg*m^2
C_Q = 2.3515e-4;
omega_max = 1047.197; %rad/s
r_rot = 0.014; %m
m_rot = 0.025; %kg

%Calculated constants
K_E = 9.5493/K_V; %V*s/rad
K_T = K_E; %per page 3.
J_L = 1/4*nB*mB*(r-epsilon)^2;
A = pi*r^2;
kb = C_T*rho*A*r^2;
kt = C_Q*rho*A*r^3;

J = Jm + J_L;  % total moment of inertia
g = 9.8066;

c1 = R*Tf^2/K_T^2;
c2 = Tf/K_T*(2*R*Df/K_T+K_E);
c3 = Df/K_T*(R*Df/K_T+K_E)+2*R*Tf*kt/K_T^2;
c4 = kt/K_T*(2*R*Df/K_T+K_E);
c5 = R*kt^2/K_T^2;
c6 = 2*R*J*Tf/K_T^2;
c7 = R*J^2/K_T^2;
c8 = J/K_T*(2*R*Df/K_T+K_E);
c9 = 2*R*J*kt/K_T^2;
integr = 0;
for i = 1:length(tspan)
    for j = 1:4
        ff = c1+c2*rotorSpeeds_H(j,i)+c3*rotorSpeeds_H(j,i)^2+c4*rotorSpeeds_H(j,i)^3+c5*rotorSpeeds_H(j,i)^4+ ...
            c6*rotorSpeeds_H_dot(j,i)+c7*rotorSpeeds_H_dot(j,i)^2+c8*rotorSpeeds_H_dot(j,i)*rotorSpeeds_H(j,i)+c9*rotorSpeeds_H_dot(j,i)*...
            rotorSpeeds_H(j,i)^2;
        integr = integr+ff*dt;
    end
end
fff = c1+c2*omega_h+c3*omega_h^2+c4*omega_h^3+c5*omega_h^4;
disp(['staying spent energy' num2str(fff*4*tspan(end))])
disp(['spent energy' num2str(integr)])
figure()
plot3(traj(1,:), traj(2,:), traj(3,:), 'b')
hold on; grid on; view(45,45)
plot3(pt(1,:), pt(2,:), pt(3,:), 'r')
% plot3(waypts(1,:), waypts(2,:), waypts(3,:), 'ro') ;
title('Simulation result'); legend('actual trajectory', 'desired trajectory')


figure()
plot3(pt(1,:), pt(2,:), pt(3,:))
view(45,45); grid on; hold on;
% plot3(waypts(1,:), waypts(2,:), waypts(3,:), 'ro') ;
title('Desired trajectory')
legend('desired trajectory')

figure()
subplot(2,2,1)
plot(tspan, traj(1,3:end), tspan, pt(1,:))
xlabel('t'); ylabel('x'); legend('actual', 'desired')
title('x coordinates');
subplot(2,2,2)
plot(tspan, traj(2,3:end), tspan, pt(2,:))
xlabel('t'); ylabel('y'); legend('actual', 'desired')
title('y coordinates');
subplot(2,2,3)
plot(tspan, traj(3,3:end), tspan, pt(3,:))
xlabel('t'); ylabel('z'); legend('actual', 'desired')
title('z coordinates');
subplot(2,2,4)
plot(tspan, (quad_a.euler_H(3,3:end)), tspan, yaw_t)
xlabel('t'); ylabel('yaw'); legend('actual', 'desired')
title('yaw angle');


err = norm(traj(:,3:end) - pt);
x_err = norm(traj(1,3:end) - pt(1,:));
y_err = norm(traj(2,3:end) - pt(2,:));
z_err = norm(traj(3,3:end) - pt(3,:));
yaw_err = norm(quad_a.euler_H(3,2:end) - yaw_t.');
fprintf('Error: %.8f\n', err);
fprintf('X Error: %.8f\n', x_err);
fprintf('Y Error: %.8f\n', y_err);
fprintf('Z Error: %.8f\n', z_err);
fprintf('Yaw Error: %.8f\n', yaw_err);

rmpath(genpath('..'));
