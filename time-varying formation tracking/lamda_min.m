function y = lamda_min(L)
m = size(L,1);
n = size(L,3);
lamda = zeros(m,n);
for i = 1:n
    D = eig(L(:,:,i));
    lamda(:,i) = sort(D);
end
y = min(min(lamda(2:end,:)));
end
