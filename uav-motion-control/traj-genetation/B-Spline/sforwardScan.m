function udot = sforwardScan(u,udot,v_u,a_u)
for i = 2:length(u)
    [newudot,exitcond] = calcNewUdot(v_u(:,i),a_u(:,i),udot(i-1),u(2)-u(1));
    if exitcond<=0
        [newudot,newlastudot] = BisectionMethod(@(lastudot) calcNewUdot(v_u(:,i),a_u(:,i),lastudot,u(2)-u(1)),0,1,1e-3);
        udot(i-1) = newlastudot;
    end
    if newudot<udot(i)
        udot(i) = newudot;
    end
end
end

function [newu,exitcond] = calcNewUdot(v,a,lastudot,du)
options = optimoptions('fmincon','Display','off');
[newu,~,exitcond,~] = fmincon(@(x) -x,0,[],[],[],[],0,1,@(x) constr(x,v,a,lastudot,du),options);
%     [newu,~,exitcond,~] = ga(@(x) -x,1,[],[],[],[],0,1,@(x) constr(x,v,a,lastudot,du));
end

function uddot = uddot(udot,last_udot,du)
uddot = (udot^2 - last_udot^2)/2/du;
end

function [c,ceq] = constr(udot,v,a,lastudot,du)
vmax = 1;%m/s
amax = 1;
c = zeros(1,4);
for i = 1:3
    c(4*(i-1)+1) = v(i)*udot - vmax;
    c(4*(i-1)+2) = a(i)*udot^2+v(i)*uddot(udot,lastudot,du)-amax;
    c(4*(i-1)+3) = - vmax - v(i)*udot;
    c(4*(i-1)+4) = -amax - a(i)*udot^2+v(i)*uddot(udot,lastudot,du);
end
ceq = [];
end

function [newudot,newlastudot] = BisectionMethod(f,a,b,TOL)
exitflag = 0;
while (b-a)/2 > TOL || exitflag<=0
    c = (a+b)/2;
    [newudot,exitflag] = f(c);
    
    if exitflag<=0
        b = c;
    else
        a = c;
    end
    newlastudot = (a+b)/2;
end
end