clear;clc;
close all;

n=8;ksi=zeros(4*n,1);

for i=1:n
    ksi(4*i-3:4*i)=[i+3;i*2+2;1;0];
end

rad=3;Obsnum=3;
Obs=zeros(Obsnum,3);
for i=1:Obsnum
    Obs(i,1:3)=[rand(1,2)+n+i,rad];
end

Record=ksi;
dt=0.05;Rorad=2;
for i=1:200
    dksi=(rand(4*n,1)-0.5)*10;
    % dksi=Obsdetect(ksi,Obs,Rorad,dt);
    ksi=ksi+dksi*dt;
    Record=[Record,ksi];
end

Roboplot(Record,Obs);
