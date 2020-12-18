% Class: Formations. for dynamic formation selection
classdef Formations < handle
    properties
        robot_number;
        Targets;% desired points(3-D array)
        Incidences;% cell format 
        Laplacians;% 3-D array
        Connects;
        current_index;% Current formation tracked by robots
        formation_number;
        
        global_error;
        local_error;
        
        % parameters for dynamic formation selection
        decision_counter; % used to config when to make decisions
        global_error_estimate; % 
        estimator_state;
        info_rate;
    end
    
    methods
        function item = Formations(targets,connects)
            item.robot_number=size(targets,1);
            item.Targets=targets;
            item.Connects=connects;
            item.current_index=1;
            item.formation_number = size(targets,3);
            item.decision_counter=1;
        end
               
        function cal_matrices(obj)
              n = obj.robot_number;% n is the robot number
              obj.Laplacians =zeros(n,n,obj.formation_number);
              obj.Incidences = cell(1,obj.formation_number);
            for k = 1:obj.formation_number
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
        
        
        function cal_global_error(obj,ksi)
            obj.global_error=zeros(obj.formation_number,1);
            pos = get_pos(ksi);
            for i =1 : obj.formation_number
                for j=1:obj.robot_number
                    for k =1:obj.robot_number
                        temp = norm(pos(j,:)-pos(k,:)-...
                            obj.Targets(j,:,i)+obj.Targets(k,:,i));
                        obj.global_error(i)=obj.global_error(i)+temp^2;
                    end
                end
            end
        end
        
        
        function cal_local_error(obj,ksi)
            obj.local_error=zeros(obj.robot_number,obj.formation_number);
             pos = get_pos(ksi);
            for k =1:obj.formation_number
                for i=1:obj.robot_number        
                    neighbor = find(obj.Laplacians(i,:,k) ==-1);
                    n_neighbor = length(neighbor);
                    for j =1:n_neighbor
                         temp = norm(pos(i,:)-pos(neighbor(j),:)...
                             -obj.Targets(i,:,k)+obj.Targets(neighbor(j),:,k));
                         obj.local_error(i,k) = obj.local_error(i,k)+temp^2;
                    end
                end
            end
            
        end
        
        function make_decision(obj)
            robot_decisions=zeros(obj.robot_number,1);
            for i=1:obj.robot_number
                [~,index]=min(obj.global_error_estimate(i,:));
                robot_decisions(i)=index;
            end
            
            % check if all the agents make the same decision
            table = tabulate(robot_decisions);
            if size(table,1)==1
                obj.current_index=robot_decisions(1);
            else
                warning("decision diverge");
                obj.current_index=round(sum(robot_decisions)/obj.robot_number);
            end
        end
        
        function update_estimate(obj,ksi,gamma,h)
            if (obj.decision_counter == 1)
                obj.init_estimate(ksi,gamma);
            end  
            
            for k=1:obj.formation_number
                estimator_state_dot=-obj.Laplacians(:,:,k)*obj.global_error_estimate(:,k);
                global_error_estimate_dot=obj.info_rate*(obj.local_error(:,k)-obj.global_error_estimate(:,k))...
                                         -obj.Laplacians(:,:,k)*(obj.global_error_estimate(:,k)+obj.estimator_state(:,k));
                
                obj.estimator_state(:,k)=obj.estimator_state(:,k)+estimator_state_dot*h;
                    
                obj.global_error_estimate(:,k)=obj.global_error_estimate(:,k)+global_error_estimate_dot*h;
            end
                       
            if (mod(obj.decision_counter,6) == 0)
                obj.make_decision();
                obj.init_estimate(ksi,gamma);
            end        
        end
        
        function init_estimate(obj,ksi,gamma)
            obj.info_rate=gamma;
            obj.cal_global_error(ksi);
            obj.cal_local_error(ksi);
            
            obj.global_error_estimate=obj.local_error;
            obj.estimator_state=zeros(obj.robot_number,obj.formation_number);
        end
        
        function update_counter(obj)
            obj.decision_counter=obj.decision_counter+1;
        end
        
    end
end