function params=load_params()
% 机体参数
params.m = 1.3;
params.g = 9.8;
C_T = 0.0048;
params.kd = 0;
params.I = diag([0.081 0.081 0.142]);
params.L = 0.25;
C_Q = 2.3515e-4;
rho = 1.225; %kg/m^3
r = 0.12; %m
A = pi*r^2;
params.b = C_Q*rho*A*r^3;
params.k = C_T*rho*A*r^2;
end
