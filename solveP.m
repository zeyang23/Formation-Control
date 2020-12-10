function P =solveP(K1)
B2 = [0;1];
B1 = [1;0];
A = B2*K1 +B1*B2';
G = -B2*B2';
Q = eye(2);
[P,~,~] = icare(A,[],Q,[],[],[],G);
end