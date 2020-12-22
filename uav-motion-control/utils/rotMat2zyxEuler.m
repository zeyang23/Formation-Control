function euler=rotMat2zyxEuler(R)
    euler(3) = atan(R(2,1)/R(1,1));
    euler(1) = atan(R(3,2)/R(3,3));
    euler(2) = atan(R(3,1)/(-R(2,1)/sin(euler(3))));
    if isnan(euler(2))
        euler(2) = atan(R(3,1)/(-R(1,1)/cos(euler(3))));
    end
end