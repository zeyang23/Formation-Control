function L = cal_Lap(connect,n)
% 计算拉普拉斯矩阵
% 输入connect 是 n行两列的矩阵,n是机器人的数量
% connect 每一行代表第几个机器人与第几个机器人相连
m = size(connect,1);
D = zeros(n,m);
for i = 1:m
    D(connect(i,1) ,i) = -1;
    D(connect(i,2) ,i) = 1;
end
    L = D*D';
end