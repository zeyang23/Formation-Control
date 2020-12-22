function [ut,V] = trajInter(t,P,n,k,Vsq_u)
N = length(t);
V = zeros(1,N);
u = linspace(0,1,N);
Ts = t(2)-t(1);
ut = zeros(1,N);
ut(1) = (u(2)-u(1))/10;
for i = 1:N-1
    V(i+1) = BSpline(sqrt(Vsq_u)',2,ut(i));
    [~,v_u,a_u,~] = BSplineDrv(P,n,k,ut(i));
    dC = v_u;
    ddC = a_u;
    BB = V(i)*Ts/norm(dC);
    AA = -V(i)^2*Ts^2*dot(dC,ddC)/2/(norm(dC))^4;
    ut(i+1) = min(1,ut(i)+AA+BB);
end
figure()
plot(t,ut)
title('u(t)')
xlabel('time,s')
ylabel('param u')
grid on
end