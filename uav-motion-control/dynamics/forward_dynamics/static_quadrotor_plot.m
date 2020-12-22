function [points, fig]=static_quadrotor_plot(r,rot_mat,quadLength, figureNo)
    % qs: 
    %       x, y, z: ×ø±ê
    %       p,q,r: Z-X-Y Å·À­½Ç
    % quadLength: quadrotor arm length, default is 1
    %
    % figureNo: figure number, default is 1

    if nargin <= 3
        figureNo = 1;
        if nargin == 2
            quadLength = 1;
        end
    end

    x = r(1); y = r(2); z = r(3);

    Q_points = [0,0,0;1,0,0;-1,0,0;0,0,0;0,1,0;0,-1,0]*quadLength;
    axisPoints = [1,0,0;0,1,0;0,0,1]*0.6*quadLength;
    
    Q_points = (rot_mat*Q_points.').';
    axisPoints = (rot_mat*axisPoints.').';
    
    X = Q_points(:,1)+x; Y = Q_points(:,2)+y; Z = Q_points(:,3)+z;
    points = [X,Y,Z];
    
    fig = figure(figureNo);
    plot3(X,Y,Z); hold on; plot3(X,Y,Z, 'ro'); grid on;
    quiver3(x*ones(3,1),y*ones(3,1),z*ones(3,1),axisPoints(:,1),axisPoints(:,2),axisPoints(:,3));
    text(axisPoints(:,1)+x,axisPoints(:,2)+y,axisPoints(:,3)+z,{'x','y','z'})
    hold off

    xlabel('x'); ylabel('y'); zlabel('z');
end