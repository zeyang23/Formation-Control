function R=exp2R(expC)
    theta = norm(expC);
    if abs(theta) < 1e-10
        R = eye(3);
    else
        omega = expC/theta;
        omega = xev(omega);
        R = eye(3) + sin(theta) * omega + (1-cos(theta)) * omega^2;
    end
end
