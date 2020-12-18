% dynamic formation selection structure
clear
clc

connect2=[1,2;2,3;3,4];
connect1=[1,2;2,3;3,4;4,1];
connects={connect1};

Target1 = [145,5;145,-5;135,-5;135,5];
Targets(:,:,1) = Target1;


% ksi = zeros(16,1);
% ksi=[2;0;2;0;2;0;-2;0;-2;0;-2;0;-2;0;2;0]+4;



% define total simulation time
tspan = [0,30];

% define initial state for all agents
ksi0_1 = [5,0,5,0]';
ksi0_2 = [5,0,-5,0]';
ksi0_3 = [-5,0,-5,0]';
ksi0_4 = [-5,0,5,0]';
ksi0=[ksi0_1;ksi0_2;ksi0_3;ksi0_4];

% define time step in RK4
h =0.01;

F1 = Formations(Targets,connects);
F1.cal_matrices()

% set the most important parameter: K1
K1_single=[-5,-5];

Tconv = 100;

% define anonymous function for RK4
control=@(t,ksi) controller_dfs(t,ksi,K1_single,F1,h,Tconv);


% solve the IVP using RK4
[t,state] = RK4(control,tspan,ksi0,h);

plot_pos(t,state)