function euler_zxy=rotMat2zxyEuler(R)
    p = atan2(-R(1,2),R(2,2));
    r = atan2(-R(3,1), R(3,3));
    q = atan2(R(3,2), -R(3,1)/sin(r));
    if isnan(q)
        q = atan2(R(3,2),R(3,3)/cos(r));
    end
    euler_zxy = [p,q,r];
end