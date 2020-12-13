function [vornb,vorvx,Aaug,baug] = polybnd_voronoi(pos,bnd_pnts)
% -------------------------------------------------------------------------
% -------------------------------------------------------------------------
% [Voronoi neighbor,Voronoi vertices] = voronoi_3d(points, boundary)
% Given n points a bounded space in R^2/R^3, this function calculates
% Voronoi neighbor/polygons associated with each point (as a generator).
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
% This functions works for d = 2, 3
% -------------------------------------------------------------------------
% This function requires:
%       vert2lcon.m (Matt Jacobson / Michael Keder)
%       pbisec.m (by me)
%       con2vert.m (Michael Keder)
% -------------------------------------------------------------------------
% Written by Hyongju Park, hyongju@gmail.com / park334@illinois.edu

K = convhull(bnd_pnts);
bnd_pnts = bnd_pnts(K,:);

[Abnd,bbnd] = vert2lcon(bnd_pnts); 
% obtain inequality constraints for convex polytope boundary
% vert2lcon.m by Matt Jacobson that is an extension of the 'vert2con' by
% Michael Keder

% find Voronoi neighbors using Delaunay triangulation
switch size(pos,2)
    case 2
        TRI = delaunay(pos(:,1),pos(:,2));        
    case 3
        TRI = delaunay(pos(:,1),pos(:,2),pos(:,3));        
end

for i = 1:size(pos,1)
    k = 0;
    for j = 1:size(TRI,1)
        if ~isempty(MY_intersect(i,TRI(j,:)))
            k = k + 1;
            neib2{i}(k,:) = MY_setdiff(TRI(j,:),i);
        end
    end
    neib3{i} = unique(neib2{i});
    if size(neib3{i},1) == 1
        vornb{i} = neib3{i};
    else
        vornb{i} = neib3{i}';
    end
end
% obtain perpendicular bisectors
for i = 1:size(pos,1)
    k = 0;
    for j = 1:size(vornb{i},2)
        k = k + 1;
        [A{i}(k,:),b{i}(k,:)] = pbisec(pos(i,:), pos(vornb{i}(j),:));
    end
end
% obtain MY_intersection between bisectors + boundary

for i = 1:size(pos,1)
    Aaug{i} = [A{i};Abnd];
    baug{i} = [b{i};bbnd];
end

% convert set of inequality constraints to the set of vertices at the
% intersection of those inequalities used 'con2vert.m' by Michael Kleder 
for i =1:size(pos,1)
   V{i}= MY_con2vert(Aaug{i},baug{i});
   ID{i} = convhull(V{i});
   vorvx{i} = V{i}(ID{i},:);
end
