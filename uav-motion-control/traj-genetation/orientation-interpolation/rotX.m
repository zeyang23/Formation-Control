function R = rotX(roll)
    R = [1 0 0;
        0 cos(roll) -sin(roll);
        0 sin(roll) cos(roll)];
end