function ksi=get_ksi(targets)
    n=size(targets,1);
    ksi=zeros(4*n,1);
    for i=1:n
        ksi(4*i-3,:)=targets(i,1);
        ksi(4*i-1,:)=targets(i,2);
    end
end