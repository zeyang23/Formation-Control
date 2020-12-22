function expC=R2exp(R)
    if norm(R-eye(3)) < 1e-6
        theta = 0; omega = [0;0;0];
    elseif abs(trace(R) + 1) < 1e-6
        theta = pi;
        omega = 1/sqrt(2*(1+R(3,3)))*[R(1,3);R(2,3);(1+R(3,3))];
        if any(isnan(omega)) || any(isinf(omega))
            omega = 1/sqrt(2*(1+R(2,2)))*[R(1,2);(1+R(2,2));R(3,2)];
        end
        if any(isnan(omega)) || any(isinf(omega))
            omega = 1/sqrt(2*(1+R(1,1)))*[(1+R(1,1));R(2,1);R(3,1)];
        end
    else
        theta = acos(1/2*(trace(R)-1));
        omega = 1/2/sin(theta)*(R-R');
        omega = vex(omega);
    end
    expC = omega(:)*theta;
end