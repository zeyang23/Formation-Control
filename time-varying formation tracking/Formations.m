% Class: Formations. for dynamic formation selection
classdef Formations
    properties
        robot_number;
        Targets;
        Connects;
        Incidences;
        Laplacians;
        current_index;
    end
    
    methods
        function item = Formations(targets,connects)
            item.robot_number=size(targets,1);
            item.Targets=targets;
            item.Connects=connects;
            item.current_index=1;
        end
        
        function cal_Incidences(obj)
            
        end
        
        function cal_Laplacians(obj)
            
        end
        
        function global_error=cal_global_error(obj,ksi)
            global_error=0;
        end
        
        function local_error=cal_local_error(obj,ksi)
            local_error=zeros(obj.robot_number,1);
        end
             
    end
end