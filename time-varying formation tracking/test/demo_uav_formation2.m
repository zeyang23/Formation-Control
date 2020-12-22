% dynamic formation selection structure
% decentralized case
clear all; close all;

num_uav=10;
noise = 0;
dt = 0.01;
tspan = 60;

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
ksi0 = 10*[ksi0_1;ksi0_2;ksi0_3;ksi0_4;ksi0_5;ksi0_6;...
ksi0_7;ksi0_8;ksi0_9;ksi0_10];

pos0 = get_pos(ksi0);

% define anonymous function for RK4
control=@(t,ksi) controller(t,ksi,K1_single,@cal_h,@cal_hvdot,L);

% simulate on uav dynamics
params = load_params();
quads = cell(1,num_uav);
for i = 1:num_uav
    quads{i} = Quadrotor(params,[pos0(i,1),pos0(i,2),0]);
    % initialization on position
    quads{i}.dt = dt;
end

state = [];
for t = 0:dt:tspan
    ksi = zeros(1,num_uav);
    for i = 1:num_uav
        ksi(4*i-3,1) = quads{i}.position(1);
        ksi(4*i-2,1) = quads{i}.velocity(1);
        ksi(4*i-1,1) = quads{i}.position(2);
        ksi(4*i,1) = quads{i}.velocity(2);
    end
        
	ksi_dot = control(t,ksi);
	state_new = uav_formation_update(ksi_dot,quads,noise);
	state = [state,state_new];
end
plot_pos([0:dt:tspan],state)

% define desired trajectory
function H=cal_h(t,ksi)

    r = 200;
    w = 0.15;
    H = zeros(40,1);
    for i =1:10
        hi = [ r*sin(w*t+(i-1)*pi/5);
               r*w*cos(w*t+(i-1)*pi/5);
               r*cos(w*t+(i-1)*pi/5);
               -w*r*sin(w*t+(i-1)*pi/5)];   
        H(4*(i-1)+1:4*i,1) = hi;
    end
end

function Hv_dot=cal_hvdot(t,ksi)
    r = 200;
    w = 0.15;
    Hv_dot = zeros(20,1);
    for i =1:10
        hiv_dot = [  -r*w^2*sin(w*t+(i-1)*pi/5);
           -r*w^2*cos(w*t+(i-1)*pi/5)]; 
        Hv_dot(2*i-1:2*i,1) = hiv_dot;
    end
end