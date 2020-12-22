function T = thrust( rotorspeeds, k )
    T = [0; 0; k * sum(rotorspeeds)]; 
end