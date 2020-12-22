function omega=expCdot2omega(expC, expC_dot)
    omega = dexpC(-expC)*expC_dot(:);
end