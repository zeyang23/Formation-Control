function pos = get_acl(ksi_dot)
    n = length(ksi_dot)/4;
    pos = zeros(n,2);
    for i =1:n
        pos(i,1) = ksi_dot(4*i-2);
        pos(i,2) = ksi_dot(4*i);
    end    
end