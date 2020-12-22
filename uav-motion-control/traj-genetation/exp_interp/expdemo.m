clear all; close all; clc

addpath(genpath('../..'));

% r1=[0;0;0];
% R1=expm(hat(r1));
% r2=[0.5;-0.1;-0.3];
% R2=expm(hat(r2));
% r3=[1;0.2;-1];
% R3=expm(hat(r3));
% r4=[0;-0.2;-2.2];
% R4=expm(hat(r4));

R1=eye(3);
R2=rotx(30);
R3=roty(30)*R2;
R4=rotz(30)*R3;

R(:,:,1)=R1;
R(:,:,2)=R2;
R(:,:,3)=R3;
R(:,:,4)=R4;

T=[0;1;2;3];
omega0=[0.6,0.1,-0.3];

ti=linspace(0,2.3,10001);
% c1s = spline(T,expCs(1,:),ti);
% c2s = spline(T,expCs(2,:),ti);
% c3s = spline(T,expCs(3,:),ti);
% 计算指数坐标r 3*1向量
% ri = spline(T,expCs,ti);
ri = CubicExpInterp(T,R,ti,omega0);

% 三点中心差分计算r的导数
ri_dot=zeros(3,length(ti));
ri_dot(:,1)=(ri(:,2)-ri(:,1))/(ti(2)-ti(1));
ri_dot(:,end)=(ri(:,end)-ri(:,end-1))/(ti(end)-ti(end-1));
for j=2:length(ti)-1
    ri_dot(:,j)=(ri(:,j+1)-ri(:,j-1))/(ti(j+1)-ti(j-1));
end

% 根据rdot计算角速度
omegai=zeros(3,length(ti));
for j=1:length(ti)
    % omegai(:,j)=rdot2omega(ri(:,j),ri_dot(:,j));
    omegai(:,j)=expCdot2omega(ri(:,j),ri_dot(:,j));
end

hold on
plot(ti,ri(1,:))
plot(ti,ri(2,:))
plot(ti,ri(3,:))
title('r')

figure
hold on
plot(ti,ri_dot(1,:))
plot(ti,ri_dot(2,:))
plot(ti,ri_dot(3,:))
title('rdot')

figure
hold on
plot(ti,omegai(1,:))
plot(ti,omegai(2,:))
plot(ti,omegai(3,:))
title('omega')

% figure
% E=zeros(3,length(ti));
% for j=1:size(Ri,3)
%     E(:,j)=rotm2eul(Ri(:,:,j),'XYZ');
% end
% 
% hold on
% plot(ti,E(1,:))
% plot(ti,E(2,:))
% plot(ti,E(3,:))
% 
% rotm2eul(R(:,:,2),"XYZ")
% rotm2eul(R(:,:,3),"XYZ")
% rotm2eul(R(:,:,4),"XYZ")

% 检验连续性
% R1*my_exp(a(:,1)+b(:,1)+c(:,1))-R2
% R2*my_exp(a(:,2)+b(:,2)+c(:,2))-R3
% R3*my_exp(a(:,3)+b(:,3)+c(:,3))-R4

rmpath(genpath('../..'));