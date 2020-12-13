clear
clc

load('pos2.mat');
load('bnd_pnts2.mat');

span = [0 5]; 

h = 0.01;

state0=two2one(pos);

f=@(t,state) control(t,state,bnd_pnts);

[t,state] = RK4(f,span,state0,h); 

[~,vorvx0] = polybnd_voronoi(pos,bnd_pnts);
plot_voronoi_2d(vorvx0,pos,bnd_pnts)
title('start')

state_final=state(:,end);
pos_final=one2two(state_final);
[~,vorvx_final] = polybnd_voronoi(pos_final,bnd_pnts);
plot_voronoi_2d(vorvx_final,pos_final,bnd_pnts)
title('final')