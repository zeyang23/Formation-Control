clear all
clc

% set the most important parameter: K1
K1_single=[-5,-5];

% define all the Laplacians
L = zeros(10,10,4);
connect1 = [1,2;2,3;3,4;4,5;5,6;6,7;7,8;8,9;9,10;10,1];
connect2 = [1,2;2,3;3,4;4,5;3,8;6,7;7,8;8,9;9,10];
connect3 = [1,2;2,3;2,9;4,5;4,7;6,7;7,8;8,9;9,10];
connect4 = [10,1;1,2;2,9;9,8;8,3;3,4;4,7;7,6;6,5];
L(:,:,1) = cal_Lap(connect1,10);
L(:,:,2) = cal_Lap(connect2,10);
L(:,:,3) = cal_Lap(connect3,10);
L(:,:,4) = cal_Lap(connect4,10);

% define anonymous function for RK4
control=@(t,ksi) controller(t,ksi,K1_single,@cal_h,@cal_hvdot,L);

% define total simulation time
tspan = [0,60];

% define initial state for all agents
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

% define time step in RK4
h =0.01;

% solve the IVP using RK4
[t,state] = RK4(control,tspan,ksi0,h);

plot_pos(t,state)
plot_vel(t,state)

% define desired trajectory
function H=cal_h(t,ksi)

    r = 20;
    w = 0;
    H = zeros(40,1);
    for i =1:10
        hi = [ 25+r*sin(w*t+(i-1)*pi/5);
               r*w*cos(w*t+(i-1)*pi/5);
               15+r*cos(w*t+(i-1)*pi/5);
               -w*r*sin(w*t+(i-1)*pi/5)];   
        H(4*(i-1)+1:4*i,1) = hi;
    end
end

function Hv_dot=cal_hvdot(t,ksi)

    Hv_dot = zeros(20,1);
    
end