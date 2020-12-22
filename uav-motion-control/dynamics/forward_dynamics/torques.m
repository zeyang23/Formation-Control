function tau = torques( rotorspeeds, L, b, k )
    tau = [
        L * k * (rotorspeeds(1) - rotorspeeds(3));
        L * k * (rotorspeeds(2) - rotorspeeds(4));
        b * (rotorspeeds(1) - rotorspeeds(2) + rotorspeeds(3) - rotorspeeds(4))];
end

