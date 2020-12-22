function rotorspeeds=get_rotorspeed(T,tau,k,L,b)
    gma=[k*ones(1,4);
           L*k*[1 0 -1 0];
           L*k*[0,1,0,-1];
           b*[1 -1 1 -1]];
    rotorspeeds=gma\[T;tau];
end
