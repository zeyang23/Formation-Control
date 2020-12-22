warning('off')
close all;
start = [206, 198];
goal = [700, 612];
load('RRT_data.mat')
load('realPathsnap.mat')

imshow(int32(1 - map), []);
title('RRT Path Planning');

hold on;

% 画出所有树枝
[Count, ~] = size(branches);

for ii = 1 : Count
    plot(nodes(ii, 1), nodes(ii, 2), 'b*','MarkerSize',4);
    plot([nodes(branches(ii, 1), 1), nodes(branches(ii, 2), 1)], ...
        [nodes(branches(ii, 1), 2), nodes(branches(ii, 2), 2)], ...
        'b', 'LineWidth', 1);
end

plot(start(1), start(2), 'g*','MarkerSize',8 );
plot(goal(1), goal(2), 'r*', 'MarkerSize',8);

% 画出路径
[~, pathCount] = size(path);

for ii = 1 : pathCount - 1
    
    plot([nodes(path(ii), 1), nodes(path(ii + 1), 1)], ...
        [nodes(path(ii), 2), nodes(path(ii + 1), 2)], ...
        'r', 'LineWidth', 1);
end

[~, pathCount] = size(path);

waypts = [];
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
hold on
traj(1,:)=traj(1,:)*100+700;
traj(2,:)=traj(2,:)*100+612;
plot(traj(1,:), traj(2,:), 'p');