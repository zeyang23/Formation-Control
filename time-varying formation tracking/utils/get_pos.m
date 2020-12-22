function pos = get_pos(ksi)
    n = length(ksi)/4;
    pos = zeros(n,2);
    for i =1:n
        pos(i,1) = ksi(4*i-3);
        pos(i,2) = ksi(4*i-1);
    end    
end