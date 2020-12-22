function udot = backwardScan(u,udot,v_u,a_u,cap)
udot(end) = 0;
for i = length(u):-1:2
    check = constrBack(udot(i-1),v_u(:,i-1),a_u(:,i-1),udot(i),u(2)-u(1),cap);
    if max(check)>0
        [newudot,exitcond] = calcNewUdotBack(v_u(:,i-1),a_u(:,i-1),udot(i),u(2)-u(1),cap);
        if exitcond<=0
            [newudot,newlastudot] = BisectionMethod(@(lastudot) calcNewUdotBack(v_u(:,i-1),a_u(:,i-1),lastudot,u(2)-u(1),cap),0,1,1e-3);
            udot(i) = newlastudot;
        end
        if newudot<udot(i-1)
            udot(i-1) = newudot;
        end
    end
end
end

function [newu,exitcond] = calcNewUdotBack(v,a,lastudot,du,cap)
maxVel = cap(1);
maxDec = cap(2);
maxAcc = cap(2);
scale = v'*v;
udotsqr1 = maxVel*maxVel/scale;
curva = norm(v)^3/norm(cross(v,a));
udotsqr2 = maxDec*curva/scale;
udotsqrUp = zeros(3,1);
udotsqrLo = zeros(3,1);
for i = 1:3
    scale4a = v(i)/2/du;
    udotsqrup = (maxAcc-scale4a*lastudot*lastudot)/(a(i)-scale4a);
    udotsqrlo = (-maxDec-scale4a*lastudot*lastudot)/(a(i)-scale4a);
    if udotsqrup>udotsqrlo
        udotsqrUp(i) = udotsqrup;
        udotsqrLo(i) = udotsqrlo;
    else
        udotsqrLo(i) = udotsqrup;
        udotsqrUp(i) = udotsqrlo;
    end
end

upperBnd = min([udotsqr1;udotsqr2;udotsqrUp]);
lowerBnd = max(udotsqrLo);
newu = sqrt(upperBnd);
if upperBnd>=lowerBnd
    exitcond = 1;
else
    exitcond = 0;
end
end

function uddot = uddot(udoti,udotlasti,du)
uddot = (udoti^2 - udotlasti^2)/2/du;
end

function [c,ceq] = constrBack(udot,v,a,lastudot,du,cap)
vmax = cap(1);%m/s
amax = cap(2);
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
while (b-a)/2 > TOL
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