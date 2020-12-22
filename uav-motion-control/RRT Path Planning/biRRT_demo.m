clc;
warning('off')

map = load('maze.mat');

start = [206, 198];
goal = [700, 612];

map = map.map;

iters = 10000;
step = 50;
prob = 0.3;

[nodes1,branches1,path1,nodes2,branches2, path2] = bi_rrt(map, start, goal, iters, step, prob);

imshow(int32(1 - map), []);
title('biRRT Path Planning');
   
hold on;
    
% 画出所有树枝
[Count1, ~] = size(branches1);
    
    for ii = 1 : Count1
        plot(nodes1(ii, 1), nodes1(ii, 2), 'b*','MarkerSize',4);
        plot([nodes1(branches1(ii, 1), 1), nodes1(branches1(ii, 2), 1)], ...
        [nodes1(branches1(ii, 1), 2), nodes1(branches1(ii, 2), 2)], ...
         'b', 'LineWidth', 1);
    end
    
    plot(start(1), start(2), 'g*','MarkerSize',8 );
    plot(goal(1), goal(2), 'r*', 'MarkerSize',8);
    
    % 画出路径
    [~, pathCount1] = size(path1);
    
    for ii = 1 : pathCount1 - 1
        
        plot([nodes1(path1(ii), 1), nodes1(path1(ii + 1), 1)], ...
        [nodes1(path1(ii), 2), nodes1(path1(ii + 1), 2)], ...
         'black', 'LineWidth', 1);
    end
    
    
%    % 画出所有树枝
[Count2, ~] = size(branches2);
    
    for ii = 1 : Count2
        plot(nodes2(ii, 1), nodes2(ii, 2), 'b*','MarkerSize',4);
        plot([nodes2(branches2(ii, 1), 1), nodes2(branches2(ii, 2), 1)], ...
        [nodes2(branches2(ii, 1), 2), nodes2(branches2(ii, 2), 2)], ...
         'r', 'LineWidth', 1);
    end
    

    
    % 画出路径
    [~, pathCount2] = size(path2);
    
    for ii = 1 : pathCount2 - 1
        
        plot([nodes2(path2(ii), 1), nodes2(path2(ii + 1), 1)], ...
        [nodes2(path2(ii), 2), nodes2(path2(ii + 1), 2)], ...
         'black', 'LineWidth', 1);
    end