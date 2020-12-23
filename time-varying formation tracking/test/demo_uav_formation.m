% dynamic formation selection structure
% decentralized case
clear all; close all;

connect2=[1,2;2,3;3,4];
connect1=[1,2;2,3;3,4;4,1];
connects={connect1,connect2};


Target1 = [85,5;85,-5;75,-5;75,5];
Target2 = [90,0;88,0;84,0;82,0];
Targets(:,:,1) = Target1;
Targets(:,:,2) = Target2;

% define total simulation time
num_uav = 4;
noise = 0;
dt = 0.01;
tspan = 30;

% define initial state for all agents
% define initial state for all agents
ksi0_1 = [-50,0,2.5,0]';
ksi0_2 = [-50,0,-2.5,0]';
ksi0_3 = [-45,0,-2.5,0]';
ksi0_4 = [-45,0,2.5,0]';
ksi0 = [ksi0_1;ksi0_2;ksi0_3;ksi0_4];
pos0 = get_pos(ksi0);

F1 = Formations(Targets,connects);
F1.cal_matrices()

% set the most important parameter: K1
K1_single=[-5,-5];

% define anonymous function for simulation
Tconv = 100;
Ttotal = 20;
dfs_control = @(t,ksi) controller_dfs_variant(t,ksi,K1_single,F1,dt,Tconv,Ttotal);

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
        
	ksi_dot = dfs_control(t,ksi);
	state_new = uav_formation_update(ksi_dot,quads,noise);
	state = [state,state_new];
end
t = [0:dt:tspan];
animate_pos(t,state,10000);
hold off;
plot_pos(t,state);