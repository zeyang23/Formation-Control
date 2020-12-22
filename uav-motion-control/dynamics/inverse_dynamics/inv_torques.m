function tau = inv_torques( omega, omegadot, I)
    tau=I * omegadot + cross(omega, I * omega);
end

