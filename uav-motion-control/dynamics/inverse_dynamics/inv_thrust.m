function T = inv_thrust( rdot,rdot2,m,g,kd )
    % rdot 线速度
    % rdot2 线加速度
    % kd 粘性阻尼系数
    gravity = [0; 0; -g];
    Fd = -kd * rdot;
    T=m*(rdot2-gravity-Fd);
end