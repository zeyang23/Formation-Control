% ksi_desired
% ksi_desired_dot
% ksi_desired_dot2
% ksi_desied_dot3 有用吗？

% psi_desired
% psi_desired_dot
% psi_desired_dot2 有用吗？

% ksi
% ksi_dot

% R
% omega
% omega_dot 有用吗？

% m
% g

% Kp_ksi
% Kd_ksi

% 如果需要精细调节，可以设置为对角矩阵
% Kp_omega
% Kd_omega

function [u1,u2]=uav_controller(ksi_desired, ksi_desired_dot, ksi_desired_dot2, ksi_desired_dot3, psi_desired, psi_desired_dot, ksi, ksi_dot, R, omega, m, g, Kp_ksi, Kd_ksi, Kp_omega, Kd_omega)

    e_ksi=ksi-ksi_desired;
    e_ksi_dot=ksi_dot-ksi_desired_dot;
    F_desired = -Kp_ksi*e_ksi-Kd_ksi*e_ksi_dot+m*g*[0;0;1]+m*ksi_desired_dot2;

    u1=dot(F_desired,R(:,3));

    zB_desired=F_desired/norm(F_desired);
    xC_desired=[cos(psi_desired);sin(psi_desired);0];
    yB_desired=cross(zB_desired,xC_desired)/norm(cross(zB_desired,xC_desired));
    xB_desired=cross(yB_desired,zB_desired);


    R_desired=[xB_desired,yB_desired,zB_desired];

    e_R=1/2*vex((R_desired.')*R-(R.')*R_desired);
    e_R = e_R.';

    % omega_desired 到底该怎么算？ 还没想完全清楚。
    h=m/u1*(ksi_desired_dot3-dot(zB_desired,ksi_desired_dot3)*zB_desired);
    p_desired=-dot(h,yB_desired);
    q_desired=dot(h,xB_desired);
    r_desired=psi_desired_dot*dot([0;0;1],zB_desired);
    omega_desired=[p_desired;q_desired;r_desired];

    e_omega=omega-omega_desired;
    u2=-Kd_omega*e_omega-Kp_omega*e_R;
end
