clear
clc
waypts = [
    0,0,0;
    rand(1,3);
    2*rand(1,3);
    3*rand(1,3);
    4*rand(1,3);
    5*rand(1,3);
].';
v0 = [0,0,0];
a0 = [0,0,0];
v1 = [0,0,0];
a1 = [0,0,0];
T = 5;
h=0.01;

min_snap_simple(waypts,v0,a0,v1,a1,T,h)

r=0.2;
min_snap_corridor(waypts,v0,a0,v1,a1,T,h,r)