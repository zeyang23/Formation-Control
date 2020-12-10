clear all
clc

r = 20;
w = 0.15;
B2 = [0;1];
B1 = [1;0];


K1_single=[-0.36,0];
P_single = solveP(K1_single);

% 升维，此时机器人为二维坐标
K1 = kron(eye(2),K1_single);
P=kron(eye(2),P_single);

L = zeros(10,10,4);
connect1 = [1,2;2,3;3,4;4,5;5,6;6,7;7,8;8,9;9,10;10,1];
connect2 = [1,2;2,3;3,4;4,5;3,8;6,7;7,8;8,9;9,10];
connect3 = [1,2;2,3;2,9;4,5;4,7;6,7;7,8;8,9;9,10];
connect4 = [10,1;1,2;2,9;9,8;8,3;3,4;4,7;7,6;6,5];
L(:,:,1) = cal_Lap(connect1,10);
L(:,:,2) = cal_Lap(connect2,10);
L(:,:,3) = cal_Lap(connect3,10);
L(:,:,4) = cal_Lap(connect4,10);
lamda_min = lamda_min(L);
K2_single = (2*lamda_min)^(-1)*B2'*P_single;

K2 = kron(eye(2),K2_single);

control=@(t,ksi) control_df(t,ksi,L,K1,K2);
tspan = [0,60];
ksi0_1 = [1,2.5,18.3,-0.6]';
ksi0_2 = [12.1,1.9,16.7,-2.2]';
ksi0_3 = [20.3,1.4,6.6,-3.5]';
ksi0_4 = [21.7,-1.3,-6.3,-2.1]';
ksi0_5 = [12.7,-3.4,-14.1,-2.7]';
ksi0_6 = [1.4,-3.5,-20.8,-0.8]';
ksi0_7 = [-8.7,-3.4,-17.4,2.6]';
ksi0_8  = [-14.6,-0.5,-5.2,2]';
ksi0_9 = [-13.9,1.2,8.3,1.8]';
ksi0_10 = [-12.6,1.5,15.6,0.3]';
ksi0 = [ksi0_1;ksi0_2;ksi0_3;ksi0_4;ksi0_5;ksi0_6;...
    ksi0_7;ksi0_8;ksi0_9;ksi0_10];
h =0.005;
[t,y] = RK4(control,tspan,ksi0,h);

center_x =zeros(1,length(t));
center_y = zeros(1,length(t));
for i = 1:4:40
    center_x = center_x+y(i,:);
    center_y = center_y+y(i+2,:);
end

hold on 

for i=1:2:3
    plot(y(2*i-1,:),y(2*i+1,:))
end
axis([-25 25 -25 25]);
axis equal
grid on 

