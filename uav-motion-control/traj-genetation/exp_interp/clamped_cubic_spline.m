function v=clamped_cubic_spline(x, y, dx1, dx2, u)
    x = x(:).'; y = y(:).';
    a = y;
    delta = x(2:end) - x(1:end-1);
    Delta = y(2:end) - y(1:end-1);
    % 依对角线计算矩阵
    diag2 = delta(1:end-1);
    diag3 = delta(2:end);

    diag1 = zeros(1,length(diag2)); diag1(end + 2) = 1;
    diag1(2:end-1) = 2*(diag2 + diag3); diag1(1) = 1;

    diag2(end + 1) = 0;

    diag3(end + 1) = 0;
    diag3(2:end) = diag3(1:end-1);
    diag3(1) = 0;
    
    n = length(x);
    A = diag(diag1) + diag(diag3, 1) + diag(diag2, -1);
    bb = zeros(1,length(x));
    bb(2:end-1) = 3*(Delta(2:end)./delta(2:end) - Delta(1:end-1)./delta(1:end-1));
    % 添加导数条件
    A(1,1:2) = [2*delta(1), delta(1)];
    A(n, n-1:n) = [delta(end-1), 2*delta(end)];
    bb(1) = 3*(Delta(1)/delta(1) - dx1);
    bb(n) = -3*(Delta(end)/delta(end) - dx2);

    % 解出系数
    c = (A\bb')';
    d = (c(2:end) - c(1:end-1))./(3*delta);
    b = Delta./delta - delta./3.*(2*c(1:end-1) + c(2:end));

    v = zeros(1,length(u));
    for k = 1:length(u)
        kk = find((u(k)-x(1:end-1) > 0)|(u(k)-x(1:end-1) >= 0),1, 'last');
        v(k) = a(kk) + b(kk)*(u(k) - x(kk)) + c(kk)*(u(k) - x(kk))^2 + d(kk)*(u(k) - x(kk))^3;
    end

end