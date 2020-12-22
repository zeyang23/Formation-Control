function [nodes1, branches1,path1,nodes2,branches2, path2] = bi_rrt(map, start, goal, iters, step, prob)

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

nodes1 = start; % Initialize the vertices variable with q_start
nodes2 = goal;

branches1=[];
branches2 = [];

for i = 1 : iters
    
    if rand() < prob
        rand_point1 = goal;
    else
        rand_point1 = [randi(Width) randi(Height)];
    end
    
    nearest_index1=knnsearch(nodes1,rand_point1);
    nearest_nodes1=nodes1(nearest_index1,:);

    new_nodes1 = Grow(nearest_nodes1, rand_point1, step); 
    
    
    
    
    if rand() < prob
        rand_point2 = start;
    else
        rand_point2 = [randi(Width) randi(Height)];
    end
    
    nearest_index2=knnsearch(nodes2,rand_point2);
    nearest_nodes2=nodes2(nearest_index2,:);

    new_nodes2 = Grow(nearest_nodes2, rand_point2, step); 
    
    
    if (new_nodes1(1) < 1 || new_nodes1(2) < 1 || new_nodes1(1) > Width || new_nodes1(2) > Height)...
            || (new_nodes2(1) < 1 || new_nodes2(2) < 1 || new_nodes2(1) > Width || new_nodes2(2) > Height)
        continue;
    end
    
    if (map(new_nodes1(2), new_nodes1(1)) == 0) && map(new_nodes2(2), new_nodes2(1)) == 0
        
        if Check_nonCollision(map, nearest_nodes1, new_nodes1) && Check_nonCollision(map, nearest_nodes2, new_nodes2)
            
            nodes1 = [nodes1; new_nodes1];
            [new_nodes_index1, ~] = size(nodes1);
            branches1 = [branches1; [int32(new_nodes_index1), int32(nearest_index1)]];
            
            nodes2 = [nodes2; new_nodes2];
            [new_nodes_index2, ~] = size(nodes2);
            branches2 = [branches2; [int32(new_nodes_index2), int32(nearest_index2)]];
                   
            % ºÏ≤‚÷’÷πÃıº˛
            Idx=rangesearch(nodes2,new_nodes1,step);
            Idx=cell2mat(Idx);
            TF = isempty(Idx);
            
            
            if (~TF) 
                if ~Check_nonCollision(map, new_nodes1, nodes2(Idx(1)))
                    break;
                end
                    
                nodes2 = [nodes2; new_nodes1];
                [new_index, ~] = size(nodes2);
                branches2 = [branches2; [int32(new_index), int32(Idx(1))]];
%                 Index = find(branches2(:, 1) == Idx(1));
%                 branches2(Index(1)+1:end,:)=[];
%                 nodes2(Index(1)+1:end,:)=[];
%                 if ~isequal(new_nodes1, nodes2(Idx(1))) 
%                     
%                     nodes2 = [nodes2; new_nodes1];
%                     [new_index, ~] = size(nodes2);
%                     branches2 = [branches2; [int32(new_index), int32(Idx(1))]];
%                 else
%                     Index = find(branches2(:, 1) == Idx(1));
%                     branches2(Index(1)+1:end,:)=[];
%                     nodes2(Index(1)+1:end,:)=[];
%                 end
                
                path1 = BackTracking(branches1);
                path2 = BackTracking(branches2);
                
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