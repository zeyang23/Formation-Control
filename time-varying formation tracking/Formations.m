% Class: Formations. for dynamic formation selection
classdef Formations < handle
    properties
        robot_number;
        Targets;% desired points(3-D array)
        Incidences;% cell format 
        Laplacians;% 3-d
        Connects;
        current_index;% Current formation tracked by robots
        Formation_numbers;
    end
    
    methods
        function item = Formations(targets,connects)
            item.robot_number=size(targets,1);
            item.Targets=targets;
            item.Connects=connects;
            item.current_index=1;
            item.Formation_numbers = size(targets,3);
        end
               
        function cal_matrices(obj)
              n = obj.robot_number;% n is the robot number
              obj.Laplacians =zeros(n,n,obj.Formation_numbers);
              obj.Incidences = cell(1,obj.Formation_numbers);
            for k = 1:obj.Formation_numbers
                connects_k = obj.Connects{k};
                 m = size(connects_k,1);% m is the edge number
                 D = zeros(n,m); 
                 for i = 1:m             
                    D(connects_k(i,1) ,i) = -1;
                    D(connects_k(i,2) ,i) = 1;
                  end
                  obj.Incidences{k} = D;
                  obj.Laplacians(:,:,k) = D * D';
            end
                
        end
            
      
        
        function global_error=cal_global_error(obj,ksi)
            global_error=zeros(obj.Formation_numbers,1);
            pos = get_pos(ksi);
            for i =1 : obj.Formation_numbers
                for j=1:obj.robot_number
                    for k =1:obj.robot_number
                        temp = norm(pos(j,:)-pos(k,:)-obj.Targets(j,:,i)+obj.Targets(k,:,i));
                        global_error(i)=global_error(i)+temp^2;
                    end
                end
            end
        end
        
        
        function local_error=cal_local_error(obj,ksi)
            local_error=zeros(obj.robot_number,obj.Formation_numbers);
             pos = get_pos(ksi);
            for k =1:obj.Formation_numbers
                for i=1:obj.robot_number        
                    neighbor = find(obj.Laplacians(i,:,k) ==-1);
                    n_neighbor = length(neighbor);
                    for j =1:n_neighbor
                         temp = norm(pos(i,:)-pos(j,:)-obj.Targets(i,:,k)+obj.Targets(neighbor(j),:,k));
                         local_error(i,k) = local_error(i,k)+temp^2;
                    end
                end
            end
            
        end
             
    end
end