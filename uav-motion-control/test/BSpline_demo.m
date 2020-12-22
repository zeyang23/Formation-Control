clear all; close all; clc
%input
k = 4;
n = 6;
P(:,1) = zeros(3,1);
for i = 1:n
    P(1,i) = cos((i-1)/n*2*pi)-1;
    P(2,i) = sin((i-1)/n*2*pi);
    P(3,i) = 1;
    P(4,i) = (i-1)/n*2*pi;
end
P = [[0;0;0;0],P];

% n = 5;
% m = 5;
% count = 1;
% signal = 1;
% for i = 1:n
%     for j = 1:m
%         P(1,count) = signal*(j-1)+(n-1)*(1-signal)/2;
%         P(2,count) = i;
%         P(3,count) = 1;
%         P(4,count) = (1-signal)/2*pi;
%         count = count + 1;
%     end
%     signal = -signal;
% end
% P = [[0;0;0;0],P];

% load waypts
% P = waypts;
% P(3,:) = P(3,:)+1;
% P = [[0;0;0],P];

% figure()
% p_u = BSpline(P,4,linspace(0,1,200));
% plot3(P(1,:),P(2,:),P(3,:),...
%                         'o','LineWidth',1,...
%                         'MarkerEdgeColor','k',...
%                         'MarkerFaceColor','g',...
%                         'MarkerSize',6);
% hold on
% plot3(p_u(1,:),p_u(2,:),p_u(3,:));
% p_sample = BSpline(P,4,0.5);
% plot3(p_sample(1,:),p_sample(2,:),p_sample(3,:),...
%                         'o','LineWidth',1,...
%                         'MarkerEdgeColor','k',...
%                         'MarkerFaceColor','r',...
%                         'MarkerSize',6);


% function demo
t = linspace(0,10,1000);
max = [3,5,5];
%Vmax,Amax,Jmax
isOnPts = 1;
isGraph = 1;
sample_density = 2;
type = 2;%1 for time-scaling,2 for Linear Programming, 3 for LP without jerk constraints
tic
[p_u,v_u,a_u,j_u] = BSplineC(P(1:3,:),4,t,max,isOnPts,isGraph,sample_density,type);