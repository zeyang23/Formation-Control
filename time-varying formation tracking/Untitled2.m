clc
clear all

L = zeros(10,10,4);
connect1 = [1,2;2,3;3,4;4,5;5,6;6,7;7,8;8,9;9,10;10,1];
connect2 = [1,2;2,3;3,4;4,5;3,8;6,7;7,8;8,9;9,10];
connect3 = [1,2;2,3;2,9;4,5;4,7;6,7;7,8;8,9;9,10];
connect4 = [10,1;1,2;2,9;9,8;8,3;3,4;4,7;7,6;6,5];
m = size(connect1,1);
w = diag([1,2,3*ones(1,m-2)]);
[L(:,:,1),Lw] = cal_Lap(connect1,10,w);
% L(:,:,2) = cal_Lap(connect2,10,w);
% L(:,:,3) = cal_Lap(connect3,10,w);
% L(:,:,4) = cal_Lap(connect4,10,w);

function [L,L_w] = cal_Lap(connect,n,w)
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
    L_w =D*w*D'; 
end