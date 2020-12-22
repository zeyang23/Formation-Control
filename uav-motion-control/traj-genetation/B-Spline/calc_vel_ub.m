function [ub,rho] = calc_vel_ub(Vsq_max,a_max,v_u,a_u,N)
    ub = zeros(1,N);
    rho = zeros(1,N);
    for i = 1:N
        rho(i) = norm(v_u(:,i+1))^3/norm(cross(v_u(:,i+1),a_u(:,i+1)));
        ub(i) = min(Vsq_max,a_max*rho(i));
    end
end