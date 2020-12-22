function omegadot = angular_acceleration(rotorspeeds, omega, I, L, b, k)
%   本体坐标系中的角加速度
    tau = torques(rotorspeeds, L, b, k);
    omegadot = I \ (tau - cross(omega, I * omega));
end