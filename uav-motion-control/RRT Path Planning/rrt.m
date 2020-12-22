function [nodes, branches, path] = rrt(map, start, goal, iters, step, prob)

if nargin < 3
    error('not enough input');
elseif nargin < 4
    iters = 10000;
    step = 50;
    prob = 0.3;
elseif nargin < 5
    step = 50;
    prob = 0.3;
elseif nargin < 6
    prob = 0.3;
end

[Height, Width] = size(map); % columns as the x axis and the rows as the y axis.

nodes = start; % Initialize the vertices variable with q_start

branches = [];

for i = 1 : iters
    
    if rand() < prob
        rand_point = goal;
    else
        rand_point = [randi(Width) randi(Height)];
    end
    
    nearest_index=knnsearch(nodes,rand_point);
    nearest_nodes=nodes(nearest_index,:);

    new_nodes = Grow(nearest_nodes, rand_point, step); 
    
    if new_nodes(1) < 1 || new_nodes(2) < 1 || new_nodes(1) > Width || new_nodes(2) > Height
        continue;
    end
    
    if map(new_nodes(2), new_nodes(1)) == 0 
        
        if Check_nonCollision(map, nearest_nodes, new_nodes)
            
            % Add q_new in vertices 
            nodes = [nodes; new_nodes];
            % Add [index(q_new) index(q_near)] in edges
            [new_nodes_index, ~] = size(nodes);
            branches = [branches; [int32(new_nodes_index), int32(nearest_index)]];
            
            % ºÏ≤‚÷’÷πÃıº˛
            Idx=rangesearch(nodes,goal,step);
            Idx=cell2mat(Idx);
            TF = isempty(Idx);
            
            if ~TF
                
                if ~isequal(new_nodes, goal) 
                    nodes = [nodes; goal];
                    [new_index, ~] = size(nodes);
                    branches = [branches; [int32(new_index), int32(Idx(1))]];
                end
                
                path = BackTracking(branches);
                
                return;
            end
        end
    end
end

    path = int32.empty(0, 2);
    
    error('solution not found ');
end


function [new_nodes] = Grow(nearest_nodes, rand_point, step)

    v = double(rand_point) - double(nearest_nodes);
    
    u = v / norm(v);
    
    new_nodes = int32(double(nearest_nodes) + step * u);
    
end


function [flag] = Check_nonCollision(map, nearest_nodes, new_nodes)
    
    subdivision = 10;
    
    v = double(new_nodes) - double(nearest_nodes);
    
    distance = norm(v);
    
    ve = v / distance;
    
    delta = distance / subdivision;
    
    checkpoint = double(nearest_nodes);
    
    for j = 1 : subdivision
        
        checkpoint = checkpoint + (delta * ve);
        
        if map(int32(checkpoint(2)), int32(checkpoint(1))) == 1 % map(q_new(1), q_new(2))
            flag = 0;
            return;
        end
        
    end
    
    flag = 1;

end

function [path] = BackTracking(branches)

    path = branches(end, 1);
    
    backtracking = branches(end, 2);
    
    while backtracking ~= 1
        Index = find(branches(:, 1) == backtracking);
        backtracking = branches(Index(1), 2);
        path = [path, backtracking];
    end
end