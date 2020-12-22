function udotu = udotInterU(x,u,udot)
    if x<0 || x>1
        udotu = 0;
    else
        udotu = spline(u,udot,x);
    end
end

