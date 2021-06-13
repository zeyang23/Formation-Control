clear all
clc

% set the most important parameter: K1
K1_single=[-1,-0.8];

% define all the Laplacians
L = zeros(4,4,4);
connect1 = [1,2;2,3;3,4];
connect2 = [1,4;4,3;3,2];
connect3 = [1,4;4,2;2,3];
connect4 = [4,1;1,3;3,2];
L(:,:,1) = cal_Lap(connect1,4);
L(:,:,2) = cal_Lap(connect2,4);
L(:,:,3) = cal_Lap(connect3,4);
L(:,:,4) = cal_Lap(connect4,4);

% define anonymous function for RK4
control=@(t,ksi) controller_3d(t,ksi,K1_single,@cal_h,@cal_hvdot,L);

% define total simulation time
tspan = [0,10];

% define initial state for all agents
ksi0_1 = [3,0,0,0,0,0]';
ksi0_2 = [0,0,3,0,0,0]';
ksi0_3 = [-3,0,0,0,0,0]';
ksi0_4 = [0,0,-3,0,0,0]';

ksi0 = [ksi0_1;ksi0_2;ksi0_3;ksi0_4];

% define time step in RK4
h =0.01;

% solve the IVP using RK4
[t,state] = RK4(control,tspan,ksi0,h);
% plot_pos_3d(t,state)
% plot_vel(t,state)

animation(t,state)

% define desired trajectory
function H=cal_h(t,ksi)

    r = 5;
    w = 0.5;
    H = zeros(24,1);
    for i =1:4
        hi = [ r*sin(w*t+(i-1)*pi/2);
               r*w*cos(w*t+(i-1)*pi/2);
               r*cos(w*t+(i-1)*pi/2);
               -w*r*sin(w*t+(i-1)*pi/2);
               4;
               0];   
        H(6*(i-1)+1:6*i,1) = hi;
    end
end

function Hv_dot=cal_hvdot(t,ksi)
    r = 5;
    w = 0.5;
    Hv_dot = zeros(12,1);
    for i =1:4
        hiv_dot = [  -r*w^2*sin(w*t+(i-1)*pi/2);
           -r*w^2*cos(w*t+(i-1)*pi/2);0]; 
        Hv_dot(3*i-2:3*i,1) = hiv_dot;
    end
end