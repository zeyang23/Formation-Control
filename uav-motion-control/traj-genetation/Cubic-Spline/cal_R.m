function Ri=cal_R(ti,T,R,a,b,c)
    m=length(ti);
    for i=1:m
        t=ti(i);
        for k=1:length(T)
            if(T(k)<=t)
                continue
            else
                break
            end
        end
        tau=(t-T(k-1))/(T(k)-T(k-1));
        Ri(:,:,i)=R(:,:,k-1)*expm(hat(a(:,k-1)*tau^3+b(:,k-1)*tau^2+c(:,k-1)*tau));
    end
end