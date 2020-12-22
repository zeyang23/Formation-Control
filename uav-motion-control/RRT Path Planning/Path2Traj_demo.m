clear
clc
figure()
warning('off')

map = load('maze.mat');

start = [206, 198];
goal = [700, 612];

map = map.map;

k = 10000;
step = 50;
prob = 0.3;

[nodes, branches, path] = rrt(map,start, goal, k, step, prob);

[~, pathCount] = size(path);
    
for ii = 1 : pathCount
        
        waypts(ii,:)=[nodes(path(ii), 1), nodes(path(ii), 2),0];
       
end

waypts=1/100*double(waypts);
waypts=waypts';
waypts(1,:)=waypts(1,:)-waypts(1,1);
waypts(2,:)=waypts(2,:)-waypts(2,1);
waypts(3,:)=waypts(3,:)-waypts(3,1);
P = waypts;
P(3,:) = P(3,:)+1;
P = [[0;0;0],P];
for i = 1:size(P,2)-1
    dx = P(1,i+1)-P(1,i);
    dy = P(2,i+1)-P(2,i);
    P(4,i) = atan2(dy,dx);
end
P(4,end) = P(4,end-1);
% waypts = [0,0,0;
%            1,2,2.5;
%            2,3,3;
%            3,2,2.5;
%            4,0,2]';
v0 = [0,0,0];
a0 = [0,0,0];
v1 = [0,0,0];
a1 = [0,0,0];
T = 50;
h=0.05;

min_snap_simple(waypts,v0,a0,v1,a1,T,h)

% psi_pts=linspace(0,pi/4,length(waypts));
% psi_dot_0 = 0;
% psi_dot2_0 = 0;
% psi_dot_1 = 0;
% psi_dot2_1 = 0;
% min_psi_dot2(psi_pts,psi_dot_0,psi_dot2_0,psi_dot_1,psi_dot2_1,T,h)
% title('psi')
% legend('keyframes','psi')