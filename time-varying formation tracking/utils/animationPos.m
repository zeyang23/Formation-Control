%animation process
%other to be changed: 所有controller的函数都要增加输出H
classdef animationPos < handle
    properties
        robotNumber;
        fig;
        % aviObject;
        xlim;
        ylim;
        desiredPos;
        actualPos;
        panPos;
        fps;
        UAVRadius;
    end
    
    methods
        %{
            inputs:
                init_states: double column vector with a size of (4*num,1)
                demo_name: string vector
                fps_init: fps int
        %}
        function item = animationPos(init_states,demo_name,fps_init)
            if(size(init_states,2) ~= 1), print("error: matrix init_states has more columns than needed");end
            item.robotNumber=length(init_states)/4;
            item.fig = figure('name',demo_name);
            xlabel("x/m"); ylabel("y/m"); title(demo_name); hold on;
%             video_title = strcat(demo_name, ".avi");
%             item.aviObject = VideoWriter(video_title);
%             item.aviObject.FrameRate = 30;
            % open(item.aviObject);
            item.fps = fps_init;
            item.xlim = zeros(1,2);
            item.ylim = zeros(1,2);
            cal_limits(item,init_states,[]);
            item.desiredPos = [];
            item.actualPos = [];
            item.panPos = [];
            item.Obsplot();
            item.UAVRadius = 0.25;
        end
        
        %{
            inputs:
                states: all the states, containing the previous ones and the present
                desiredStates: the No.i+1 desired states
        %}
        function animation_update(item,states,desiredStates,i)
            clf
            duration = 1;
            if size(states,2) <= duration
                cal_limits(item,states(:,i+1),desiredStates);
                set(gca,'XLim', [item.xlim(1),item.xlim(2)+2], 'YLim', [item.ylim(1)-2,item.ylim(2)+2]);grid on;hold on;
                num = item.robotNumber;
                for k = 1:num
                    item.desiredPos(k) = plot(desiredStates(4*k-3),desiredStates(4*k-1),'*','Color',[k/num,k/num,1-k/num]);
                    item.actualPos(k) = plot(states(4*k-3,i+1),states(4*k-1,i+1),'o','MarkerFaceColor',[k/num,k/num,1-k/num],'MarkerEdgeColor','none');
                    % item.panPos(k) = item.CirclePlot(states(4*k-3,i+1),states(4*k-1,i+1),item.UAVRadius,k);
                    plot(states(4*k-3,1:i+1),states(4*k-1,1:i+1),'Color',[k/num,k/num,1-k/num]);
                end
            else
                cal_limits(item,states(:,i+1-duration:i+1),desiredStates);
                set(gca,'XLim', [item.xlim(1),item.xlim(2)+2], 'YLim', [item.ylim(1)-2,item.ylim(2)+2]);grid on;hold on;
                num = item.robotNumber;
                for k = 1:num
                    item.desiredPos(k) = plot(desiredStates(4*k-3),desiredStates(4*k-1),'*','Color',[k/num,k/num,1-k/num]);
                    item.actualPos(k) = plot(states(4*k-3,i+1-duration:i+1),states(4*k-1,i+1-duration:i+1),'o','MarkerFaceColor',[k/num,k/num,1-k/num],'MarkerEdgeColor','none');
                    % item.panPos(k) = item.CirclePlot(states(4*k-3,i+1),states(4*k-1,i+1),item.UAVRadius,k);
                    plot(states(4*k-3,i+1-duration:i+1),states(4*k-1,i+1-duration:i+1),'Color',[k/num,k/num,1-k/num]);
                end
            end
            axis equal; 
%             M = getframe;
%             % hold off;
%             writeVideo(item.aviObject, M);
            pause(1/item.fps);
        end
        
        function cal_limits(item,states,desiredStates)
            states = [states,desiredStates];
            item.xlim(1) = floor(min(min(states(1:4:end)),item.xlim(1)));
            item.xlim(2) = ceil(max(max(states(1:4:end)),item.xlim(2)));
            item.ylim(1) = floor(min(min(states(3:4:end)),item.ylim(1)));
            item.ylim(2) = ceil(max(max(states(3:4:end)),item.ylim(2)));
        end
        
        function cir = CirclePlot(item,x,y,Radius,j)
            figure(item.fig);
            cmap = hsv(5); %// define colors. You could change `hsv` to `jet`, `cool`, ...
            alpha = 0.2; %// define level of transparency
            t = 0 : .1 : 2 * pi;
            Rx = Radius * cos(t) + x;
            Ry = Radius * sin(t) + y;
            cir=patch(Rx, Ry, cmap(j,:), 'facealpha', alpha, 'edgecolor', 'none');
        end
        
        function Obsplot(item)
            figure(item.fig);
            c = [0; 0; 6;6];
            v = [5 0.5; 15 0.5;15 3;5 3];
            f = [1 2 3 4];
            patch('Faces',f,'Vertices',v,'FaceColor',[0.8500 0.3250 0.0980],'EdgeColor','none','FaceAlpha',0.4)
            
            c = [0; 0; 6;6];
            v(:,2)=-v(:,2);
            f = [1 2 3 4];
            patch('Faces',f,'Vertices',v,'FaceColor',[0.9290 0.6940 0.1250],'EdgeColor','none','FaceAlpha',0.4)
            
        end
        
%         function animation_stop(item)
%             close(item.aviObject);
%         end
    end
end