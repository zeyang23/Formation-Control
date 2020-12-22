function vel = get_vel(ksi)
    n = length(ksi)/4;
    vel = zeros(n,2);
    for i =1:n
        vel(i,1) = ksi(4*i-2);
        vel(i,2) = ksi(4*i);
    end    
end