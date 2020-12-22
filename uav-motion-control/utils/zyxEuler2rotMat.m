function R = zyxEuler2rotMat(euler)
    phi=euler(1);
    theta=euler(2);
    psy=euler(3);
    R=[cos(psy)*cos(theta),cos(psy)*sin(phi)*sin(theta)-cos(phi)*sin(psy),sin(psy)*sin(phi)+cos(psy)*cos(phi)*sin(theta);
       cos(theta)*sin(psy),cos(psy)*cos(phi)+sin(phi)*sin(psy)*sin(theta),cos(phi)*sin(psy)*sin(theta)-cos(psy)*sin(phi);
       -sin(theta),cos(theta)*sin(phi),cos(phi)*cos(theta)];
end