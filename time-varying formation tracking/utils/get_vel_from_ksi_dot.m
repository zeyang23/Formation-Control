function vel = get_vel_from_ksi_dot(ksi_dot)
    n = length(ksi_dot)/4;
    vel = zeros(n,2);
    for i =1:n
        vel(i,1) = ksi_dot(4*i-3);
        vel(i,2) = ksi_dot(4*i-1);
    end    
end