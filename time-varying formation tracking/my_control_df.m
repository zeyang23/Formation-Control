function ksi_dot = my_control_df(t,ksi,L,K1,K2)
if t<=15
    index = 1;
elseif t<=30
    index = 2;
elseif t<=45
    index = 3;
else
    index = 4;
end
B2 = kron(eye(2),[0;1]);
B1 = kron(eye(2),[1;0]);
r = 20;
w = 0;

%定义参考队形状态量（reference formation）
H = cal_H(t);

%定义参考队形状态速度量
Hv_dot = cal_Hv_dot(t);

ksi_dot = (kron(eye(10),(B2*K1+B1*B2'))-kron(L(:,:,index),B2*K2))*ksi...
-(kron(eye(10),B2*K1)-kron(L(:,:,index),B2*K2))*H+...
kron(eye(10),B2)*Hv_dot;

end

function H=cal_H(t)
%定义参考队形状态量（reference formation）
r = 5;
w = 0;
H = zeros(40,1);
for i =1:10
    hi = reference(t,i,r,w);
    H(4*(i-1)+1:4*i,1) = hi;
end
end

function Hv_dot=cal_Hv_dot(t)
%定义参考队形状态速度量
r = 5;
w = 0;
Hv_dot = zeros(20,1);
for i =1:10
    hiv_dot = [  -r*w^2*sin(w*t+(i-1)*pi/5);
       -r*w^2*cos(w*t+(i-1)*pi/5)]; 
    Hv_dot(2*i-1:2*i,1) = hiv_dot;
end
end

function hi = reference(t,i,r,w)
if t<40 && t>20
    hi = [ 0.5*t+(i-1)*0.8+8;
       0.5;
       0;
       0];    
elseif t<=20
    hi = [ r*sin(w*t+(i-1)*pi/5)+0.5*t;
       r*w*cos(w*t+(i-1)*pi/5)+0.5;
       r*cos(w*t+(i-1)*pi/5);
       -w*r*sin(w*t+(i-1)*pi/5)];
else
       hi = [ r*sin(w*t+(i-1)*pi/5)+0.5*t+12;
       r*w*cos(w*t+(i-1)*pi/5)+0.5;
       r*cos(w*t+(i-1)*pi/5);
       -w*r*sin(w*t+(i-1)*pi/5)];
end
end