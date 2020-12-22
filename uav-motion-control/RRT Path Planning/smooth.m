function [path_smooth] = smooth(map, path, vertices, delta)
%SMOOTH Smoothing algorithm for obtaining a shorter and less noisy path.
%   We will use the greedy approach: Connect q_goal from q_start,
%   if it fails try from a closer position until it connects.
%   Once q_goal is connected, start again with its directly connected position.
%
% map: matrix that you can obtain loading the ?.mat? files.
%
% path: list of vertex indices from the start vertex (q_start) to the goal vertex (q_goal).
% The list MUST be represented as a row vector.
%
% vertices: list of x and y coordinates of the vertices.
% The first vertex will correspond to the start position and the last one will correspond t
% the goal position. The variable MUST have 2 columns for x and y coordinates and
% n rows (being n the number of vertices found in the tree).
%
% delta: incremental distance that will be used to check if direct connection between
% the vertices of the path is inside the free space. The edges will be divided obtaining
% several points, each of them separated this delta distance.
%
% path_smooth: reduced list of vertex indices from the start vertex (q_start) to the goal vertex
% (q_goal) after applying the smoothing algorithm. The list MUST be represented as a row vector.

path_smooth = path(1); % initing with goal
currentIndex = 1; % path array iterator
currentSmoothIndex = numel(path); % path reverse array iterator

while currentIndex < numel(path)
    
    while currentIndex < currentSmoothIndex
        
        if isEdgeBelongsFreeSpace(map, vertices(path(currentSmoothIndex), :), vertices(path(currentIndex), :), delta)
            path_smooth = [path_smooth, path(currentSmoothIndex)];
            currentIndex = currentSmoothIndex;
            break;
        else
            currentSmoothIndex = currentSmoothIndex - 1;
        end
        
    end
    
    currentSmoothIndex = numel(path);
    
end

% rrtSmoothDraw(map, path_smooth, vertices);

end

function [isBelongsFreeSpace] = isEdgeBelongsFreeSpace(map, startPoint, endPoint, delta)
    
    v = double(endPoint - startPoint);
    
    distance = norm(v);
    
    u = v / distance;
    
    intermediatePointCount = distance / delta;
    
    currentCoordinate = double(startPoint);
    
    for ii = 1 : intermediatePointCount
        
        currentCoordinate = currentCoordinate + (delta * u);
        
        if map(int32(currentCoordinate(2)), int32(currentCoordinate(1))) == 1
            isBelongsFreeSpace = 0;
            return;
        end
        
    end
    
    isBelongsFreeSpace = 1;

end

function rrtSmoothDraw(map, path_smooth, vertices)

    imshow(int32(1 - map), []);
    title('RRT (Rapidly-Exploring Random Trees) - Smooth Path');
    % imagesc(1 - map);
    % colormap(gray);
    
    hold on;

    [~, pathCount] = size(path_smooth);
    
    for ii = 1 : pathCount - 1
        %plot(vertices(ii, 1), vertices(ii, 2), 'cyan*', 'linewidth', 1);
        plot([vertices(path_smooth(ii), 1), vertices(path_smooth(ii + 1), 1)], ...
        [vertices(path_smooth(ii), 2), vertices(path_smooth(ii + 1), 2)], ...
         'r', 'LineWidth', 2);
    end
    
end