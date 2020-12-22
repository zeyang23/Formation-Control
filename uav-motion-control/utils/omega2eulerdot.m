function thetadot = omega2eulerdot(omega, euler)

%  计算欧拉角的变化率      
%-----------------------------------------------------
%  Parameters :
%       *.  thetadot ----- 欧拉角变化率
%       *.  attitude ----- 欧拉角
%       *.  omega    ----- 角速度
%----------------------------------------------------
    phi = euler(1);
    theta = euler(2);
    psi = euler(3);
    if abs(theta-pi/2) > 1e-4
        thetadot = [1,0,          -sin(theta);
                    0,cos(phi)    cos(theta)*sin(phi);
                    0,-sin(phi)   cos(theta)*cos(phi)] \ omega ;
    else
        thetadot = omega;
    end
end

