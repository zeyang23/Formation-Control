function [a,b,c]=cubic_spline(T,R,omega0,alpha0)
    if(nargin<3)
        omega0=zeros(3,1);
        alpha0=zeros(3,1);
    elseif(nargin<4)
        alpha0=zeros(3,1);
    end
    
    n=length(T)-1;
    for i=1:n
        r(:,i)=vex(logm(transpose(R(:,:,i))*R(:,:,i+1)));
    end
    
    c(:,1)=omega0;
    b(:,1)=1/2*alpha0;
    a(:,1)=r(:,1)-b(:,1)-c(:,1);
    
    for i=2:n
        s=r(:,i);
        ns=norm(s);
        t=3*a(:,i-1)+2*b(:,i-1)+c(:,i-1);
        u=6*a(:,i-1)+2*b(:,i-1);
        c(:,i)=(eye(3)-(1-cos(ns)/(ns)^2)*hat(s)+(ns-sin(ns))/(ns)^3*(hat(s))^2)*t;
        b(:,i)=1/2*(u-dot(s,t)/(ns)^4*(2*cos(ns)+ns*sin(ns)-2)*cross(s,t)...
            -(1-cos(ns))/ns^2*cross(s,u)...
            +dot(s,t)/(ns)^5*(3*sin(ns)-ns*cos(ns)-2*ns)*(cross(s,cross(s,t)))...
            +(ns-sin(ns))/ns^3*(cross(t,cross(s,t))+cross(s,cross(s,u))));
        a(:,i)=s-b(:,i)-c(:,i);
    end
end