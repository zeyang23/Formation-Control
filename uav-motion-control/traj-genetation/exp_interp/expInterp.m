function R=expInterp(ts, Rs, newt, omega0, alpha0)
    if nargin <=4
        alpha0 = zeros(3,1);
        if nargin == 3
            omega0 = zeros(3,1);
        end
    end

    r = R2exp(Rs(:,:,1)'*Rs(:,:,2));
    c(:,1)=omega0;
    b(:,1)=1/2*alpha0;
    a(:,1)=r(:,1)-b(:,1)-c(:,1);

    for k = 2:(length(ts)-1)
        s = R2exp(Rs(:,:,k)'*Rs(:,:,k+1)); ns = norm(s);
        t = 3*a(:,k-1) + 2*b(:,k-1) + c(:,k-1);
        u = 6*a(:,k-1) + 2*b(:,k-1);
        c(:,k) = dexpC(-s)*t;
        sst = cross(s,cross(s,t));
        tst = cross(t,cross(s,t));
        ssu = cross(s,cross(s,u));
        b(:,k) = 0.5*(...
            u-s.'*t/(ns^4)*(2*cos(ns)+ns*sin(ns)-2)*(cross(s,t)) - (1-cos(ns))/(ns^2)*(cross(s,u))  ...
            + s.'*t/(ns^5)*(3*sin(ns)-ns*cos(ns)-2*ns)*sst ...
            + (ns-sin(ns))/(ns^3)*(tst+ssu)...
        );
        a(:,k) = s-b(:,k)-c(:,k);
    end
    R = zeros(3,3,length(newt));
    for k = 1:length(newt)
        t = newt(k);
        if t < ts(1)
            R(:,:,k) = nan(3,3);
        elseif t > ts(end)
            R(:,:,k) = nan(3,3);
        else
            id = find((ts-t)>=0,1)-1;
            if t == ts(1)
                id = 1;
            end
            tau = (t-ts(id))/(ts(id+1)-ts(id));
            expC = a(:,id)*tau^3 + b(:,id)*tau^2 + c(:,id)*tau;
            R(:,:,k) = Rs(:,:,id)*exp2R(expC);
        end
    end

end