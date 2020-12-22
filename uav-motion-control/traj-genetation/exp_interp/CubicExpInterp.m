function expC2=CubicExpInterp(ts, Rs, newt, omega0, omega_n)
    if nargin <=4
        omega_n=[0,0,0]';
        if nargin == 3
            omega0 = [0,0,0]';
        end
    end
    n = length(ts);
    expCs = zeros(3,n);
    for k = 1:n
        expCs(:,k) = R2exp(Rs(:,:,k));
    end

    expC_dot_0 = omega2expCdot(expCs(:,1),omega0);
    expC_dot_n = omega2expCdot(expCs(:,n),omega_n);

    expC2 = zeros(3,length(newt));
    for k = 1:3
        expC2(k,:) = clamped_cubic_spline(ts,expCs(k,:),expC_dot_0(k),expC_dot_n(k),newt);
    end
    
end