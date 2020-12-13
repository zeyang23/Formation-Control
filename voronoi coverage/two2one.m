function B=two2one(pos)
    n=length(pos);
    nB=2*n;
    B=zeros(nB,1);
    for i=1:n
        B(2*i-1:2*i,:)=pos(i,:);
    end
end