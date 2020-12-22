function [p_u,v_u,a_u,j_u] = BSplineDrv(P,n,k,u)
[p_u,NodeVector4v] = BSpline(P,k,u);
% ÇúÏßµ¼Ê¸¼ÆËã
for i = 0:n-1
    Q(:,i+1) = k/(NodeVector4v(i+k+1)-NodeVector4v(i+1))*(P(:,i+2)-P(:,i+1));
end
[v_u,NodeVector4a] = BSpline(Q,k-1,u);

for i = 0:n-2
    Q2(:,i+1) = (k-1)/(NodeVector4a(i+k)-NodeVector4a(i+1))*(Q(:,i+2)-Q(:,i+1));
end
[a_u,NodeVector4j] = BSpline(Q2,k-2,u);

for i = 0:n-3
    Q3(:,i+1) = (k-2)/(NodeVector4j(i+k-1)-NodeVector4j(i+1))*(Q2(:,i+2)-Q2(:,i+1));
end
j_u = BSpline(Q3,k-3,u);

% v_ud = FDMinter3(u,p_u);
% a_ud = FDMinter3(u,v_u);
% j_ud = FDMinter3(u,a_u);
% disp(['v off: ' num2str(norm(v_ud-v_u))])
% disp(['a off: ' num2str(norm(a_ud-a_u))])
% disp(['j off: ' num2str(norm(j_ud-j_u))])
% plot(u,v_ud(1,:))
% hold on
% plot(u,a_ud(1,:))
% plot(u,j_ud(1,:))
% legend('v','a','j')

    % graphing
%     figure()
%     plot3(p_u(1,:), p_u(2,:), p_u(3,:), 'Marker','.','LineStyle','-', 'Color',[.3 .6 .9]);
%     hold on
%     plot3(P(1,:),P(2,:),P(3,:),...
%                         'o','LineWidth',1,...
%                         'MarkerEdgeColor','k',...
%                         'MarkerFaceColor','g',...
%                         'MarkerSize',6);
%     quiver3(p_u(1,:), p_u(2,:), p_u(3,:),v_u(1,:), v_u(2,:), v_u(3,:), 3)
%     quiver3(p_u(1,:), p_u(2,:), p_u(3,:),a_u(1,:), a_u(2,:), a_u(3,:), 3)
%     grid on;axis equal
%     legend('B-Spline','control points','velocity','accel')
%     title('B-Spline Demo')
%     xlabel('x');ylabel('y');zlabel('z')
end