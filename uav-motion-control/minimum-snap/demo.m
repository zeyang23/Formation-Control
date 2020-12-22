clear
clc
waypts = [0,0,0;
           1,2,2.5;
           2,3,3;
           3,2,2.5;
           4,0,2]';
v0 = [0,0,0];
a0 = [0,0,0];
v1 = [0,0,0];
a1 = [0,0,0];
T = 5;
h=0.01;

min_snap_simple(waypts,v0,a0,v1,a1,T,h)

r=0.2;
min_snap_corridor(waypts,v0,a0,v1,a1,T,h,r)