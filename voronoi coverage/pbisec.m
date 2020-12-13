% The function finds perpendicular bisector between two points in 2D/3D
% Hyongju Park / hyongju@gmail.com
% input: two points in 2D/3D
% output: inequality Ax <= b

function [A,b] = pbisec(x1, x2)

middle_pnt = mean([x1;x2],1);
n_vec = (x2 - x1) / norm(x2 - x1);
Ad = n_vec;
bd = dot(n_vec,middle_pnt);

if Ad * x1' <= bd
    A = Ad;
    b = bd;
else
    A = -Ad;
    b = -bd;
end
