function [r,p,y] = R2rpy(R)
pointNum = size(R,3);
r = zeros(1,pointNum);
p = zeros(1,pointNum);
y = zeros(1,pointNum);

for i = 1:pointNum
    y(1,i) = atan2(R(2,1,i),R(1,1,i));
    p(1,i) = atan2(-R(3,1,i),hypot(R(3,2,i),R(3,3,i)));
    r(1,i) = atan2(R(3,2,i),R(3,3,i));
end
end