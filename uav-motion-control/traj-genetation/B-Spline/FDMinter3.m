function df = FDMinter3(t,f)
df = zeros(size(f,1),length(t));
for i = 2:length(t)-1%中心差分求插值
    df(:,i) = (f(:,i+1)-f(:,i-1))/(t(i+1)-t(i-1));
end
df(:,1) = (f(:,2)-f(:,1))/(t(2)-t(1));%向后差分求插值初值
df(:,end) = (f(:,end)-f(:,end-1))/(t(end)-t(end-1));%向前差分求插值末值
end