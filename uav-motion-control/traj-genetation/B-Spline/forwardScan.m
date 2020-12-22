function udot = forwardScan(u,udot,v_u,a_u,cap)
for i = 2:length(u)
    [newudot,exitcond] = calcNewUdot(v_u(:,i),a_u(:,i),udot(i-1),u(2)-u(1),cap);
    if exitcond<=0
        [newudot,newlastudot] = BisectionMethod(@(lastudot) calcNewUdot(v_u(:,i),a_u(:,i),lastudot,u(2)-u(1),cap),0,1,1e-3);
        udot(i-1) = newlastudot;
    end
    udot(i) = newudot;
end
end

function [newu,exitcond] = calcNewUdot(v,a,lastudot,du,cap)
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
    udotsqrup = (maxAcc+scale4a*lastudot*lastudot)/(scale4a+a(i));
    udotsqrlo = (-maxDec+scale4a*lastudot*lastudot)/(scale4a+a(i));
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