function d=dexpC(expC)
    if norm(expC) < 1e-6
        d = eye(3);
    else
        S = 2*sin(norm(expC)/2)/norm(expC);
        d = eye(3) + S^2/2*xev(expC) + (1-S*cos(norm(expC)/2)/(norm(expC)^2))*(xev(expC))^2;
    end
end