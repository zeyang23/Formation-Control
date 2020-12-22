function [p_t,v_t,a_t,j_t] = BSplineC(P,k,t,cap,OnPts,Graph,sample_density,type)
n = size(P,2)-1;
NodeVector = U_quasi_uniform(n,k); % 准均匀B样条的节点矢量
% 经过way points，控制点修正
if OnPts
    [P,P2] = on_way_pts(P,k,NodeVector);
    p_tmp = BSpline(P,k,linspace(0,1,n+1));
    disp(['off way points by: ' num2str(norm(p_tmp-P2))]);
else
    P2 = P;
end

%轨迹规划
if type == 1
    sample_size = n*sample_density+2;
    u = linspace(0,1,sample_size);
    udot = zeros(1,length(u));
    [~,v_u,a_u,~] = BSplineDrv(P,n,k,u);
    udot = forwardScan(u,udot,v_u,a_u,cap);
    figure()
    plot(u,udot)
    udot = backwardScan(u,udot,v_u,a_u,cap);
    hold on
    plot(u,udot)
    legend("forward","backward")
    xlabel("u");ylabel("du/dt")
    grid on
else
    if type == 2
        u = linspace(0,1,length(t));
        [~,v_u,a_u,j_u] = BSplineDrv(P,n,k,u);
        [Vsq_u,kapsq] = VelPlan(t,v_u,a_u,cap(1),cap(2));
        udot = JerkMaxed(Vsq_u,v_u,a_u,j_u,t,kapsq,cap(3));
    end
end

%轨迹插值
[ut,udott,uddott] = calcUt(t,u,udot);
[p_t,v_u,a_u,j_u] = BSplineDrv(P,n,k,ut);
disp("spent time calculating")
disp(toc)

%分析
path = 0;
v_t = zeros(3,length(t));
a_t = zeros(3,length(t));
Vt = zeros(1,length(t));
for i = 1:length(t)
    v_t(:,i) = v_u(:,i)*udott(i);
    a_t(:,i) = a_u(:,i)*(udott(i))^2 + v_u(:,i)*uddott(i);
    Vt(i) = norm(v_u(:,i)*udott(i));
    path = path + Vt(i)*(t(2)-t(1));
end
j_t = FDMinter3(t,a_t);
disp(['total length ' num2str(path)])
if (ut(end)>1-1e-5)
    disp(['total time ' num2str(min(t(ut>1-1e-5)))])
else
    disp('cannot finish path under given constraint')
end

if Graph
    % graphing
    figure()
    hold on
    plot3(P2(1, :), P2(2, :),P2(3, :),...
        'o','LineWidth',1,...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor','g',...
        'MarkerSize',6);
%     plot3(p_sample(1, :), p_sample(2, :),p_sample(3, :),...
%         'o','LineWidth',1,...
%         'MarkerEdgeColor','k',...
%         'MarkerFaceColor','b',...
%         'MarkerSize',8);
    plot3(p_t(1,:), p_t(2,:), p_t(3,:), 'Marker','.','LineStyle','-', 'Color',[.3 .6 .9]);
    quiver3(p_t(1,:), p_t(2,:), p_t(3,:),v_t(1,:), v_t(2,:), v_t(3,:), 3)
    quiver3(p_t(1,:), p_t(2,:), p_t(3,:),a_t(1,:), a_t(2,:), a_t(3,:), 3)
    grid on;axis equal
    legend('control point','p','v','a')
    title('B-Spline Demo')
    xlabel('x');ylabel('y');zlabel('z')
    view(45,45)
    
    figure()
    subplot(3,1,1)
    plot(t,p_t(1,:))
    hold on
    plot(t,p_t(2,:))
    plot(t,p_t(3,:))
    legend('x','y','z')
    xlabel('time,s');ylabel('m');
    title('x y z position')
    grid on
    
    subplot(3,1,2)
    plot(t,v_t(1,:))
    hold on
    plot(t,v_t(2,:))
    plot(t,v_t(3,:))
    legend('xdot','ydot','zdot')
    xlabel('time,s');ylabel('m/s');
    title('x y z velocity')
    grid on
    
    subplot(3,1,3)
    plot(t,a_t(1,:))
    hold on
    plot(t,a_t(2,:))
    plot(t,a_t(3,:))
    legend('xddot','yddot','zddot')
    xlabel('time,s');ylabel('m/s^2');
    title('x y z accelaration')
    grid on
    
    figure()
    plot(t,j_t(1,:))
    hold on
    plot(t,j_t(2,:))
    plot(t,j_t(3,:))
    legend('xdddot','ydddot','zdddot')
    xlabel('time,s');ylabel('m/s^3');
    title('x y z jerk')
    grid on
end
end