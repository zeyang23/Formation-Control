clear all
clc

% set the most important parameter: K1
K1_single=[-5,-5];

% define all the Laplacians
L = [];
connect1 = [1,2;2,3;3,4;4,1];
connect2 = [1,2;2,3;3,4;4,1;1,3;2,4];

L(:,:,1) = cal_Lap(connect1,4);
L(:,:,2) = cal_Lap(connect2,4);

% define anonymous function for RK4
control=@(t,ksi) controller(t,ksi,K1_single,@cal_h,@cal_hvdot,L);

% define total simulation time
tspan = [0,30];

% define initial state for all agents
ksi0_1 = [1,0,0,0]';
ksi0_2 = [2,0,0,0]';
ksi0_3 = [3,0,0,0]';
ksi0_4 = [4,0,0,0]';
ksi0=[ksi0_1;ksi0_2;ksi0_3;ksi0_4];

% define time step in RK4
h =0.01;

% solve the IVP using RK4
[t,state] = RK4(control,tspan,ksi0,h);

plot_pos(t,state)
plot_vel(t,state)

% define desired trajectory
function H=cal_h(t,ksi)

    r = 4;
    w = 0;
    
    H1 = zeros(16,1);
    for i =1:4
        hi = [ 90+r*sin(w*t+(i-1)*pi/2);
               r*w*cos(w*t+(i-1)*pi/2);
               r*cos(w*t+(i-1)*pi/2);
               -w*r*sin(w*t+(i-1)*pi/2)];   
        H1(4*(i-1)+1:4*i,1) = hi;
    end
    
    H2 = zeros(16,1);
    for i =1:4
        hi = [ 90+2*(i-1);
               0
               0;
               0];    
        H2(4*(i-1)+1:4*i,1) = hi;
    end
    
%     for i=1:4
%         xi=ksi(4*i-3);
%         yi=ksi(4*i-1);
%         
%         if (xi>30 && xi<60)
%             H=H2;
%             return
%         else
%             H=H1;
%             return
%         end  
%     end
      H=H1;
end

function Hv_dot=cal_hvdot(t,ksi)

    Hv_dot = zeros(8,1);
    
end