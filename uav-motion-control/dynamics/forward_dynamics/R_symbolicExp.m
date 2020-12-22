clear all; close all; clc

syms x1(t) x2(t) x3(t) psy(t);
syms g;
p = [x1(t), x2(t), x3(t)].';

a = diff(diff(p));
d1 = [cos(psy(t)), sin(psy(t)), 0].';
b3 = (a - g*[0,0,1].'); b3 = b3/tempNorm(b3);
b2 = xev(b3)*d1; b2 = b2/tempNorm(b2);
b1 = xev(b2)*b3;

R = [b1,b2,b3]; R_dot = diff(R);
R

omega = vex(R.'*R_dot).'
D_omega = diff(omega)

function  n=tempNorm(vec)
    n = sqrt(sum(vec.^2));
end