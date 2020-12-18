clc
clear all

connect1=[1,2;2,3;3,4];
connect2=[1,2;2,3;3,4;4,1];

connects={connect1,connect2};
Target1 = [2,0;1,0;-1,0;-2,0];
Target2 = [2,2;2,-2;-2,-2;-2,2];
Targets(:,:,1) = Target1;
Targets(:,:,2) = Target2;
ksi = zeros(16,1);
F1 = Formations(Targets,connects);
F1.cal_matrices()
global_error= F1.cal_global_error(ksi);
local_error= F1.cal_local_error(ksi);