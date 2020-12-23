% dynamic formation selection structure
% decentralized case
clear all; close all;

% define total simulation time
num_uav = 10;
v_x = 3;
num_formation = 3;
accel = 3;
alter_a = 0.3;
dt = 0.005;
tspan = 30;

x_start = -10;
width = 1;
col_size = 1;
col_number = 0;
Targets = zeros(num_uav,2,num_formation);
for j = 1:num_formation
    width = (1-(j-1)/num_formation)*1;
    for i = 1:num_uav
        if mod(i,2) == 1
            Targets(i,:,j) = [x_start + col_number * col_size,width / 2];
            col_number = col_number + 1;
        else
            Targets(i,:,j) = [x_start + col_number * col_size - col_size/2,- width / 2];
        end
    end
%     scatter(Targets(:,1,j),Targets(:,2,j))
%     axis equal
end

connects = cell(1,num_formation);
for k = 1:num_formation
    connect = [];
    for i = 1:num_uav
        for j = i:num_uav
            if norm(Targets(i,:,k)-Targets(j,:,k)) < 3
                connect = [connect;[i,j]];
            end
        end
    end
    connects{k} =connect;
end

% define initial state for all agents
R = 2.5;
for i = 1:num_uav
    r = R*rand();
    theta = 2*pi*rand() - pi;
    pos0(i,1) = r*cos(theta) + x_start;
    pos0(i,2) = r*sin(theta);
end
F1 = Formations(Targets,connects);
F1.cal_matrices()

% set the most important parameter: K1
K1_single=[-5/4,-5]/2;

% define anonymous function for simulation
Tconv = 2000;
dfs_control = @(t,ksi) controller_dfs_variant(t,ksi,K1_single,F1,dt,Tconv,v_x,alter_a);

% simulate on uav dynamics
params = load_params();
quads = cell(1,num_uav);
for i = 1:num_uav
    quads{i} = Quadrotor(params,[pos0(i,1),pos0(i,2),0]);
    % initialization on position
    quads{i}.dt = dt;
end

state0 = [];
for i = 1:num_uav
    state0 = [state0;[pos0(i,1);0;pos0(i,2);0]];
end
% ani = animationPos(state0,"formation control",30);

t = 0:dt:tspan;
state = state0;
for k = 1:size(t,2)-1
    ksi = zeros(4*num_uav,1);
    for i = 1:num_uav
        ksi(4*i-3,1) = quads{i}.position(1);
        ksi(4*i-2,1) = quads{i}.velocity(1);
        ksi(4*i-1,1) = quads{i}.position(2);
        ksi(4*i,1) = quads{i}.velocity(2);
    end
        
	ksi_dot = dfs_control(t(k),ksi);
	state_new = uav_formation_update(ksi_dot,quads,accel);
% 	ani.animation_update(state,state_new,k-1);
    state = [state,state_new];
end
plot_pos(t,state);