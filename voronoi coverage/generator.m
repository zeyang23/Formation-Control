% This DEMO calculates a Voronoi diagram with arbitrary points in arbitrary
% polytope/polyheron in 2D/3D

clear all;close all;clc
%% generate random samples
n = 40;        % number of points
m = 40;         % number of boundary point-candidates
d = 2;          % dimension of the space
tol = 1e-07;            % tolerance value used in "inhull.m" (larger value high precision, possible numerical error)
pos0 = rand(n,d);       % generate random points
bnd0 = rand(m,d);       % generate boundary point-candidates
K = convhull(bnd0);
bnd_pnts = bnd0(K,:);   % take boundary points from vertices of convex polytope formed with the boundary point-candidates
%% take points that are in the boundary convex polytope
in = inhull(pos0,bnd0,[],tol); 
% inhull.m is written by John D'Errico that efficiently check if points are
% inside a convex hull in n dimensions
% We use the function to choose points that are inside the defined boundary
u1 = 0;
for i = 1:size(pos0,1)
    if in(i) ==1
        u1 = u1 + 1;
        pos(u1,:) = pos0(i,:);
    end
end
%% 
% =========================================================================
% INPUTS:
% pos       points that are in the boundary      n x d matrix (n: number of points d: dimension) 
% bnd_pnts  points that defines the boundary     m x d matrix (m: number of vertices for the convex polytope
% boundary d: dimension)
% -------------------------------------------------------------------------
% OUTPUTS:
% vornb     Voronoi neighbors for each generator point:     n x 1 cells
% vorvx     Voronoi vertices for each generator point:      n x 1 cells
% =========================================================================

[vornb,vorvx] = polybnd_voronoi(pos,bnd_pnts);

%% PLOT

for i = 1:size(vorvx,2)
    col(i,:)= rand(1,3);
end

switch d
    case 2
        figure('position',[0 0 600 600],'Color',[1 1 1]);
        for i = 1:size(pos,1)
        plot(vorvx{i}(:,1),vorvx{i}(:,2),'-r')
        hold on;
        end
        plot(bnd_pnts(:,1),bnd_pnts(:,2),'-');
        hold on;
        plot(pos(:,1),pos(:,2),'Marker','o','MarkerFaceColor',[0 .75 .75],'MarkerEdgeColor','k','LineStyle','none');
        axis('equal')
        axis([0 1 0 1]);
        set(gca,'xtick',[0 1]);
        set(gca,'ytick',[0 1]);        
    case 3
        figure('position',[0 0 600 600],'Color',[1 1 1]);
        for i = 1:size(pos,1)
        K = convhulln(vorvx{i});
        trisurf(K,vorvx{i}(:,1),vorvx{i}(:,2),vorvx{i}(:,3),'FaceColor',col(i,:),'FaceAlpha',0.5,'EdgeAlpha',1)
        hold on;
        end
        scatter3(pos(:,1),pos(:,2),pos(:,3),'Marker','o','MarkerFaceColor',[0 .75 .75], 'MarkerEdgeColor','k');
        axis('equal')
        axis([0 1 0 1 0 1]);
        set(gca,'xtick',[0 1]);
        set(gca,'ytick',[0 1]);
        set(gca,'ztick',[0 1]);
        set(gca,'FontSize',16);
        xlabel('X');ylabel('Y');zlabel('Z');
        
end
