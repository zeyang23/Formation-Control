function omega = eulerdot2omega(thetadot, euler)
%-----------------------------------------------------
%  Parameters :
%  thetadot     欧拉角变化率
%  attitude     欧拉角
%  omega        本体坐标系的角速度
%----------------------------------------------------
phi = euler(1);
theta = euler(2);
psi = euler(3);
JB = [1  0           -sin(theta);
         0  cos(phi)    cos(theta)*sin(phi);
         0  -sin(phi)   cos(theta)*cos(phi)];
omega=JB * thetadot; 
end