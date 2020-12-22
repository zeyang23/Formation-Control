function expC_dot=omega2expCdot(expC, omega)
    expC_dot = inv_dexpC(-expC)*omega(:);
end
