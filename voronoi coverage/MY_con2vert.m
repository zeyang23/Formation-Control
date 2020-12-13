function [V,nr] = MY_con2vert(A,b)
% Edited by Hyongju Park (Aug 11, 2015)
% *** Changed the function to skip the error messeage 
% ----------------------------------------------------
% CON2VERT - convert a convex set of constraint inequalities into the set
%            of vertices at the intersections of those inequalities;i.e.,
%            solve the "vertex enumeration" problem. Additionally,
%            identify redundant entries in the list of inequalities.
% 
% V = con2vert(A,b)
% [V,nr] = con2vert(A,b)
% 
% Converts the polytope (convex polygon, polyhedron, etc.) defined by the
% system of inequalities A*x <= b into a list of vertices V. Each ROW
% of V is a vertex. For n variables:
% A = m x n matrix, where m >= n (m constraints, n variables)
% b = m x 1 vector (m constraints)
% V = p x n matrix (p vertices, n variables)
% nr = list of the rows in A which are NOT redundant constraints
% 
% NOTES: (1) This program employs a primal-dual polytope method.
%        (2) In dimensions higher than 2, redundant vertices can
%            appear using this method. This program detects redundancies
%            at up to 6 digits of precision, then returns the
%            unique vertices.
%        (3) Non-bounding constraints give erroneous results; therefore,
%            the program detects non-bounding constraints and returns
%            an error. You may wish to implement large "box" constraints
%            on your variables if you need to induce bounding. For example,
%            if x is a person's height in feet, the box constraint
%            -1 <= x <= 1000 would be a reasonable choice to induce
%            boundedness, since no possible solution for x would be
%            prohibited by the bounding box.
%        (4) This program requires that the feasible region have some
%            finite extent in all dimensions. For example, the feasible
%            region cannot be a line segment in 2-D space, or a plane
%            in 3-D space.
%        (5) At least two dimensions are required.
%        (6) See companion function VERT2CON.
%        (7) ver 1.0: initial version, June 2005
%        (8) ver 1.1: enhanced redundancy checks, July 2005
%        (9) Written by Michael Kleder
%
% EXAMPLES:
%
% % FIXED CONSTRAINTS:
% A=[ 0 2; 2 0; 0.5 -0.5; -0.5 -0.5; -1 0];
% b=[4 4 0.5 -0.5 0]';
% figure('renderer','zbuffer')
% hold on
% [x,y]=ndgrid(-3:.01:5);
% p=[x(:) y(:)]';
% p=(A*p <= repmat(b,[1 length(p)]));
% p = double(all(p));
% p=reshape(p,size(x));
% h=pcolor(x,y,p);
% set(h,'edgecolor','none')
% set(h,'zdata',get(h,'zdata')-1) % keep in back
% axis equal
% V=con2vert(A,b);
% plot(V(:,1),V(:,2),'y.')
% 
% % RANDOM CONSTRAINTS:
% A=rand(30,2)*2-1;
% b=ones(30,1);
% figure('renderer','zbuffer')
% hold on
% [x,y]=ndgrid(-3:.01:3);
% p=[x(:) y(:)]';
% p=(A*p <= repmat(b,[1 length(p)]));
% p = double(all(p));
% p=reshape(p,size(x));
% h=pcolor(x,y,p);
% set(h,'edgecolor','none')
% set(h,'zdata',get(h,'zdata')-1) % keep in back
% axis equal
% set(gca,'color','none')
% V=con2vert(A,b);
% plot(V(:,1),V(:,2),'y.')
% 
% % HIGHER DIMENSIONS:
% A=rand(15,5)*1000-500;
% b=rand(15,1)*100;
% V=con2vert(A,b)
% 
% % NON-BOUNDING CONSTRAINTS (ERROR):
% A=[0 1;1 0;1 1];
% b=[1 1 1]';
% figure('renderer','zbuffer')
% hold on
% [x,y]=ndgrid(-3:.01:3);
% p=[x(:) y(:)]';
% p=(A*p <= repmat(b,[1 length(p)]));
% p = double(all(p));
% p=reshape(p,size(x));
% h=pcolor(x,y,p);
% set(h,'edgecolor','none')
% set(h,'zdata',get(h,'zdata')-1) % keep in back
% axis equal
% set(gca,'color','none')
% V=con2vert(A,b); % should return error
% 
% % NON-BOUNDING CONSTRAINTS WITH BOUNDING BOX ADDED:
% A=[0 1;1 0;1 1];
% b=[1 1 1]';
% A=[A;0 -1;0 1;-1 0;1 0];
% b=[b;4;1000;4;1000]; % bound variables within (-1,1000)
% figure('renderer','zbuffer')
% hold on
% [x,y]=ndgrid(-3:.01:3);
% p=[x(:) y(:)]';
% p=(A*p <= repmat(b,[1 length(p)]));
% p = double(all(p));
% p=reshape(p,size(x));
% h=pcolor(x,y,p);
% set(h,'edgecolor','none')
% set(h,'zdata',get(h,'zdata')-1) % keep in back
% axis equal
% set(gca,'color','none')
% V=con2vert(A,b);
% plot(V(:,1),V(:,2),'y.','markersize',20)
%
% % JUST FOR FUN:
% A=rand(80,3)*2-1;
% n=sqrt(sum(A.^2,2));
% A=A./repmat(n,[1 size(A,2)]);
% b=ones(80,1);
% V=con2vert(A,b);
% k=convhulln(V);
% figure
% hold on
% for i=1:length(k)
%     patch(V(k(i,:),1),V(k(i,:),2),V(k(i,:),3),'w','edgecolor','none')
% end
% axis equal
% axis vis3d
% axis off
% h=camlight(0,90);
% h(2)=camlight(0,-17);
% h(3)=camlight(107,-17);
% h(4)=camlight(214,-17);
% set(h(1),'color',[1 0 0]);
% set(h(2),'color',[0 1 0]);
% set(h(3),'color',[0 0 1]);
% set(h(4),'color',[1 1 0]);
% material metal
% for x=0:5:720
%     view(x,0)
%     drawnow
% end
flg = 0;
c = A\b;
tol = 1e-07;
if ~all(abs(A*c - b) < tol)
    %obj1 = @(c) obj(c, {A,b});
    [c,~,ef] = fminsearch( @(x)obj(x, {A,b}),c);
%     [c,~,ef] = fminsearch(@obj,c,A,b);
    if ef ~= 1
        flg = 1;
    end
end
if flg ==1
    V = [];
%     error('Unable to locate a point within the interior of a feasible region.')
else
b = b - A*c;
D = A ./ repmat(b,[1 size(A,2)]);
[~,v2] = convhulln([D;zeros(1,size(D,2))]);
[k,v1] = convhulln(D);
%if v2 > v1
%    error('Non-bounding constraints detected. (Consider box constraints on variables.)')
%end
nr = unique(k(:));
G  = zeros(size(k,1),size(D,2));
for ix = 1:size(k,1)
    F = D(k(ix,:),:);
    G(ix,:)=F\ones(size(F,1),1);
end
V = G + repmat(c',[size(G,1),1]);
[~,I]=unique(num2str(V,6),'rows');
V=V(I,:);
end
return
function d = obj(c,param)
A = param{1};
b = param{2};
% ,A,b
% A=params{1};
% b=params{2};
d = A*c-b;
k=(d>=-1e-15);
d(k)=d(k)+1;
d = max([0;d]);
return



