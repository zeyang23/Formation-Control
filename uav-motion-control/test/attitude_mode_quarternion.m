clear all; close all;
addpath(genpath('..'));

load('qomegai.mat')
load('qRi.mat')
params = load_params();
quad_a = Quadrotor(params); save_gif = false; use_bspline = false;

dt = 0.01; quad_a.dt = dt; end_t = 15; tspan = 0:dt:end_t;

K_R = diag([0.1,0.1,0.1]);
K_omega = diag([0.1,0.1,0.1]);
K_z = 100;
K_zdot = 20;
% if save_gif
%     figure()
%     plot3(pt(1,:), pt(2,:), pt(3,:), 'b');
%     hold on; grid on; 
%     plot3(waypts(1,:), waypts(2,:), waypts(3,:), 'ro') ;
%     view(45,45); title('real time result');
%     ax = [-1.5,4.5,-0.5,4.5,-2,6];
%     axis(ax);
% end
error = zeros(3,length(tspan));
RR = zeros(3,3,length(tspan));
Rd = zeros(3,3,length(tspan));
close all;
for k = 1:length(tspan)
    t = tspan(k);
    R_desired = Ri(:,:,k);
    omega_desired = omegai(:,k);
    
    noise = 0.0;
    p_c = quad_a.position * (randn(1)*noise+1);
    v_c = quad_a.velocity * (randn(1)*noise+1);
    a_c = quad_a.acceleration * (randn(1)*noise+1);
    omg_c = quad_a.Omega * (randn(1)*noise+1);

    % 0,0: yaw, yaw_dot
    [u1,u2] = control3(R_desired,omega_desired,0,0,quad_a.attitude,omg_c,p_c(3),v_c(3),a_c(3),quad_a.I,quad_a.m,quad_a.g,K_R,K_omega,K_z,K_zdot);
    
    rotorSpeeds = get_rotorspeed(u1,u2,quad_a.k,quad_a.L,quad_a.b);
    
    quad_a.updateState(rotorSpeeds);
    
    if save_gif
        static_quadrotor_plot(quad_a.position, quad_a.attitude, quad_a.L, 1);
        hold on
        plot3(pt(1,:), pt(2,:), pt(3,:), 'b');
        plot3(quad_a.position_H(1,:), quad_a.position_H(2,:), quad_a.position_H(3,:),'r');
        title('real time result');
        hold off
        ax = [-1,6,-1,6,-1,3];
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
    
    Rd(:,:,k)  = R_desired;
    RR(:,:,k) = quad_a.attitude;
    error(:,k) = vex(logm(Ri(:,:,k)))'-vex(logm(RR(:,:,k)))';
    ZZ(k) = p_c(3);
end

figure()
plot(tspan,error(1,:))
hold on
plot(tspan,error(2,:))
plot(tspan,error(3,:))
title('exponential coordinates error')
xlabel('time,s')
ylabel('error')
legend('r1','r2','r3')
grid on
figure()
plot(tspan,ZZ)
title('z error')
xlabel('time,s')
ylabel('error')
grid on
rmpath(genpath('..'));

figure()
hold on
z = linspace(0,10,length(tspan));
T = zeros(4,4,length(tspan));
Td = zeros(4,4,length(tspan));
for i = 1:100:length(tspan)
    T(:,:,i) = transl([z(i),0,0]);
    T(1:3,1:3,i) = RR(:,:,i);
    Td(:,:,i) = transl([z(i),0,0]);
    Td(1:3,1:3,i) = Rd(:,:,i);
    trplot(T(:,:,i),'color','r')
    trplot(Td(:,:,i),'color','b')
end
title('attitude interpolation')
view(45,45)
axis equal
grid on

