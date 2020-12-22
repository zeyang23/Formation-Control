function R=zxyEuler2rotMat(euler_zxy)
    % euler: Z-X-Y euler angle

    % convert euler angle to rotation matrix
    % use this to calculate x y z in world coordinate

    p= euler_zxy(1); q = euler_zxy(2); r = euler_zxy(3);
    R = [
        cos(p)*cos(r)-sin(q)*sin(p)*sin(r), -cos(q)*sin(p), cos(p)*sin(r)+cos(r)*sin(q)*sin(p);
        cos(r)*sin(p)+cos(p)*sin(q)*sin(r), cos(q)*cos(p), sin(p)*sin(r)-cos(p)*cos(r)*sin(q);
        -cos(q)*sin(r), sin(q), cos(q)*cos(r);
    ];
end