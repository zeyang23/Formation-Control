function [A,b,qstar] = calc_jerk_cstrts(Vsq,v_u,a_u,j_u,u,Jmax,kap_sq)
N = length(u);
du = u(2) - u(1);
qstar = zeros(1,N);
for i = 1:N
    qstar(i) = Vsq(i)/kap_sq(i);
end

tmp4j = zeros(2*N,N-2,3);
for j = 1:3
    alpha = zeros(1,N);
    beta = zeros(1,N);
    gama = zeros(1,N);
    for i = 1:N
        alpha(i) = v_u(j,i)/2/du/du-3*a_u(j,i)/4/du;
        beta(i) = j_u(j,i)-v_u(j,i)/du/du;
        gama(i) = v_u(j,i)/2/du/du+3*a_u(j,i)/4/du;
    end
    tmp4pos = zeros(N-2,N);
    for i = 2:N-1
        tmp4pos(i-1,i-1:i+1) = sqrt(qstar(i))*[alpha(i),beta(i),gama(i)]+[0,Jmax/2/qstar(i),0];
    end
    tmp4neg = zeros(N-2,N);
    for i = 2:N-1
        tmp4neg(i-1,i-1:i+1) = -sqrt(qstar(i))*[alpha(i),beta(i),gama(i)]+[0,Jmax/2/qstar(i),0];
    end
    tmp4bnd = zeros(4,N-2);
    if v_u(j,2) == 0
        den = 1;
    else
        den = v_u(j,2);
    end
    tmp4bnd(1:2,1:2) = [Jmax/2/qstar(2),N^2*(v_u(j,3))^2*sqrt(qstar(2))/8/den;
                        Jmax/2/qstar(2),-N^2*(v_u(j,3))^2*sqrt(qstar(2))/8/den];
    if v_u(j,N-1) == 0
        den2 = 1;
    else
        den2 = v_u(j,2);
    end
    tmp4bnd(3:4,end-1:end) = [Jmax/2/qstar(N-1),N^2*(v_u(j,N-2))^2*sqrt(qstar(N-1))/8/den2;
                              Jmax/2/qstar(N-1),-N^2*(v_u(j,N-2))^2*sqrt(qstar(N-1))/8/den2];
    tmp4j(:,:,j) = [tmp4pos(:,2:end-1);tmp4neg(:,2:end-1);tmp4bnd];
end
A = [tmp4j(:,:,1);tmp4j(:,:,2);tmp4j(:,:,3)];
b = zeros(6*N,1)+Jmax*3/2;
% q = linprog(-1*ones(1,N-2),A,b);
% plot(q)
% hold on
% plot(qstar)
% legend('jerked','origin')
end

