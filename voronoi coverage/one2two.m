function A=one2two(state)
    n=length(state);
    nA=n/2;
    A=zeros(nA,2);
    for i = 1:nA
        A(i,:)=[state(2*i-1),state(2*i)];
    end
end