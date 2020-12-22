function ksi=get_ksi_dot(vels)
    n=size(vels,1);
    ksi_dot=zeros(4*n,1);
    for i=1:n
        ksi(4*i-2,:)=vels(i,1);
        ksi(4*i,:)=vels(i,2);
    end
end