% Class for dynamic formation selection
classdef Formations < handle
    properties
        robot_number; % the number of robots
        Targets;      % desired target position for all robots (3-D array)
        Incidences;   % incidence matrices for all graphs (cell) 
        Laplacians;   % Laplacians for all graphs (3_D array) 
        Connects;     % define all the edges of all graphs (cell)
        current_index;% the index of current target formation
        formation_number; % the number of formations
        
        global_error; % current global error for all formations (1-d array)
        local_error;  % current local error for all formations (2-d array)
        
        % formation error log
        % every row is the global error trajectory of one formation
        
        global_error_log % record the history of global error (2-d array)
        
        % define as cell
        % each matrix in the cell is the local error trajectory of one formation
        
        local_error_log % record the history of local error (cell)         
        global_error_estimate_log % record the history of global error estimate (cell)
        estimator_state_log % record the history of estimator state (cell)
        
        % parameters for dynamic formation selection
        decision_counter; % increase by 1 at every time step
        global_error_estimate; % current global error estimate for all formations (2-d array)
        estimator_state;       % current estimator state for all formations (2-d array)
        info_rate;             % the weight of instantaneous information in proportional-integral estimator
    end
    
    methods
        function item = Formations(targets,connects)
        % calculate necessary properties at initialization
            
            item.robot_number=size(targets,1);
            item.Targets=targets;
            item.Connects=connects;
            item.current_index=1;
            item.formation_number = size(targets,3);
            item.decision_counter=1;
     
        end
        
        
        function cal_matrices(obj)
        % calculate all the matrices of all formations
        
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
        % calculate current global error for all formations
        
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
        % calculate current local error of for all formations
        
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
        % robots make their decisions to choose their target formation
        
            robot_decisions=zeros(obj.robot_number,1);
            for i=1:obj.robot_number
                [~,index]=min(obj.global_error_estimate(i,:));
                robot_decisions(i)=index;
            end
            
            % check if all the agents make the same decision
            table = tabulate(robot_decisions);
            if table(end) == 100
                obj.current_index=robot_decisions(1);
            else
                warning("%d decision diverge",obj.decision_counter);
                
                obj.current_index=round(sum(robot_decisions)/obj.robot_number);
            end
        end
        
        
        function update_estimate(obj,ksi,gamma,h,Tconv)
        % update the important functions in dynamic formation selection algotithm
            
            if (obj.decision_counter == 1)
                obj.init_estimate(ksi,gamma);
                obj.initialize_log();   
            end  
            
            obj.cal_global_error(ksi);
            obj.cal_local_error(ksi);
            
            % write in log
            if (obj.decision_counter == 1)
                
            else
                obj.update_log();
            end
            
            for k=1:obj.formation_number
                estimator_state_dot=-obj.Laplacians(:,:,k)*obj.global_error_estimate(:,k);
%                 global_error_estimate_dot=obj.info_rate*(obj.local_error(:,k)-obj.global_error_estimate(:,k))...
%                                          -obj.Laplacians(:,:,k)*(obj.global_error_estimate(:,k)+obj.estimator_state(:,k));
                
                obj.estimator_state(:,k)=obj.estimator_state(:,k)+estimator_state_dot*h;
                
                global_error_estimate_dot=-obj.Laplacians(:,:,k)*(obj.global_error_estimate(:,k));
                    
                obj.global_error_estimate(:,k)=obj.global_error_estimate(:,k)+global_error_estimate_dot*h;
                
            end
            
            obj.update_counter();  
            if (mod(obj.decision_counter,Tconv) == 0)
                obj.make_decision();
                obj.init_estimate(ksi,gamma);
            end        
%             if (mod(obj.decision_counter,Tconv) == 0)
%                 obj.make_decision();
%                 
%             end    
%             
%             if (mod(obj.decision_counter,Tconv) == 10)
%                 obj.init_estimate(ksi,gamma);
%             end

        end
        
        
        function init_estimate(obj,ksi,gamma)
        % initialize the values of functions in dynamic formation selection algotithm
        
            obj.info_rate=gamma;
            obj.cal_global_error(ksi);
            obj.cal_local_error(ksi);
            
            obj.global_error_estimate=obj.local_error;
            obj.estimator_state=zeros(obj.robot_number,obj.formation_number);
            
                     
        end
        
        
        function update_counter(obj)
        % update decision_counter
        
            obj.decision_counter=obj.decision_counter+1;
        end
        
        
        function centralized_decision(obj,ksi)
        % centralized dynamic formation selection
        
            obj.cal_global_error(ksi);
            [~,index]=min(obj.global_error);
            obj.current_index=index;
        end
        
        
        function initialize_log(obj)
        % initialize logs of functions in dynamic formation selection
        
            % initialize log
            obj.global_error_log=obj.global_error;
            obj.local_error_log = cell(1,obj.formation_number);
            obj.global_error_estimate_log = cell(1,obj.formation_number);
            obj.estimator_state_log = cell(1,obj.formation_number);
            
            for m =1:obj.formation_number
                obj.local_error_log{m}=obj.local_error(:,m);
                obj.global_error_estimate_log{m}=obj.global_error_estimate(:,m);
                obj.estimator_state_log{m}=obj.estimator_state(:,m);
            end
        end
        
        
        function update_log(obj)
        % update logs of functions in dynamic formation selection
        
            obj.global_error_log=[obj.global_error_log,obj.global_error];
            for m =1:obj.formation_number
                obj.local_error_log{m}=[obj.local_error_log{m},obj.local_error(:,m)];
                obj.global_error_estimate_log{m}=[obj.global_error_estimate_log{m},obj.global_error_estimate(:,m)];
                obj.estimator_state_log{m}=[obj.estimator_state_log{m},obj.estimator_state(:,m)];
            end
        end
        
    end
end