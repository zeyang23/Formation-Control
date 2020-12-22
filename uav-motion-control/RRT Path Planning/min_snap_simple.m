function min_snap_simple(waypts,v0,a0,v1,a1,T,h)

    n_order = 5;
    
    ts = arrangeT(waypts,T);
    tt = 0:h:T;
    
    polys_x = minimum_snap_single_axis_simple(waypts(1,:),ts,n_order,v0(1),a0(1),v1(1),a1(1));
    polys_y = minimum_snap_single_axis_simple(waypts(2,:),ts,n_order,v0(2),a0(2),v1(2),a1(2));
    polys_z = minimum_snap_single_axis_simple(waypts(3,:),ts,n_order,v0(3),a0(3),v1(3),a1(3));
    
     xx = polys_vals(polys_x,ts,tt,0);
    vxx = polys_vals(polys_x,ts,tt,1);
    axx = polys_vals(polys_x,ts,tt,2);
    jxx = polys_vals(polys_x,ts,tt,3);
     yy = polys_vals(polys_y,ts,tt,0);
    vyy = polys_vals(polys_y,ts,tt,1);
    ayy = polys_vals(polys_y,ts,tt,2);
    jyy = polys_vals(polys_y,ts,tt,3);
     zz = polys_vals(polys_z,ts,tt,0);
    vzz = polys_vals(polys_z,ts,tt,1);
    azz = polys_vals(polys_z,ts,tt,2);
    jzz = polys_vals(polys_z,ts,tt,3);
    
    figure
    plot3(waypts(1,:),waypts(2,:),waypts(3,:),'*')
    hold on
    plot3(xx,yy,zz)
end

function polys = minimum_snap_single_axis_simple(waypts,ts,n_order,v0,a0,ve,ae)
p0 = waypts(1);
pe = waypts(end);

n_poly = length(waypts)-1;
n_coef = n_order+1;

% compute Q
Q_all = [];
for i=1:n_poly
    Q_all = blkdiag(Q_all,computeQ(n_order,4,ts(i),ts(i+1)));
end
b_all = zeros(size(Q_all,1),1);

Aeq = zeros(4*n_poly+2,n_coef*n_poly);
beq = zeros(4*n_poly+2,1);

% start/terminal pva constraints  (6 equations)
Aeq(1:3,1:n_coef) = [calc_tvec(ts(1),n_order,0);
                     calc_tvec(ts(1),n_order,1);
                     calc_tvec(ts(1),n_order,2)];
Aeq(4:6,n_coef*(n_poly-1)+1:n_coef*n_poly) = ...
                    [calc_tvec(ts(end),n_order,0);
                     calc_tvec(ts(end),n_order,1);
                     calc_tvec(ts(end),n_order,2)];
beq(1:6,1) = [p0,v0,a0,pe,ve,ae]';

% mid p constraints    (n_ploy-1 equations)
neq = 6;
for i=1:n_poly-1
    neq=neq+1;
    Aeq(neq,n_coef*i+1:n_coef*(i+1)) = calc_tvec(ts(i+1),n_order,0);
    beq(neq) = waypts(i+1);
end

% continuous constraints  ((n_poly-1)*3 equations)
for i=1:n_poly-1
    tvec_p = calc_tvec(ts(i+1),n_order,0);
    tvec_v = calc_tvec(ts(i+1),n_order,1);
    tvec_a = calc_tvec(ts(i+1),n_order,2);
    neq=neq+1;
    Aeq(neq,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_p,-tvec_p];
    neq=neq+1;
    Aeq(neq,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_v,-tvec_v];
    neq=neq+1;
    Aeq(neq,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_a,-tvec_a];
end

Aieq = [];
bieq = [];

p = quadprog(Q_all,b_all,Aieq,bieq,Aeq,beq);

polys = reshape(p,n_coef,n_poly);

end