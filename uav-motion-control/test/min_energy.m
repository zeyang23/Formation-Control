
load energy.mat
rs=[0;0;0];
re1=[4;5;6];
re2=[4;5;12];

ts=0;
te=20;

psi_s=0;
psi_e=pi/4;
% figure()
% title('trajectory')
% xlabel('x')
% ylabel('y')
% zlabel('z')
% hold on
% plot3(rs(1),rs(2),rs(3),'*')
% plot3(re1(1),re1(2),re1(3),'*')
% plot3(out1.STATES(:,2),out1.STATES(:,4),out1.STATES(:,6))
% legend('start point','end point')
% grid on
% axis equal
% view(45,45)
% figure
% plot3(out2.STATES(:,2),out2.STATES(:,4),out2.STATES(:,6))
% title('trajectory')
% xlabel('x')
% ylabel('y')
% zlabel('z')

figure()
subplot(2,2,1)
plot(out1.STATES(:,1),out1.STATES(:,2))
hold on
plot(out1.STATES(:,1),out1.STATES(:,4))
plot(out1.STATES(:,1),out1.STATES(:,6))
legend('x','y','z')
xlabel('t / s')
ylabel('traj / m')
title('trajectory')

subplot(2,2,2)
plot(out1.STATES(:,1),out1.STATES(:,3))
hold on
plot(out1.STATES(:,1),out1.STATES(:,5))
plot(out1.STATES(:,1),out1.STATES(:,7))
legend('xdot','ydot','zdot')
xlabel('t / s')
ylabel('vel / m/s')
title('velocity')

subplot(2,2,3)
plot(out1.STATES(:,1),out1.STATES(:,8))
hold on
plot(out1.STATES(:,1),out1.STATES(:,10))
plot(out1.STATES(:,1),out1.STATES(:,12))
legend('phi','theta','psi')
xlabel('t / s')
ylabel('euler / rad')
title('attitude')

subplot(2,2,4)
plot(out1.STATES(:,1),out1.STATES(:,14))
hold on
plot(out1.STATES(:,1),out1.STATES(:,15))
plot(out1.STATES(:,1),out1.STATES(:,16))
plot(out1.STATES(:,1),out1.STATES(:,17))
legend('omega1','omega2','omega3','omega4')
xlabel('t / s')
ylabel('omega / rad/s')
title('rotor speed')