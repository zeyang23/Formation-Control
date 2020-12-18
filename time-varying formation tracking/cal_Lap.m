function L = cal_Lap(connect,n)
    m = size(connect,1);
    D = zeros(n,m);
    for i = 1:m
        D(connect(i,1) ,i) = -1;
        D(connect(i,2) ,i) = 1;
    end
    L = D*D';
end