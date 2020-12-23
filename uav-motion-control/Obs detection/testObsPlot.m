clear;clc;
close all;

rad=0.5;
fac=[-1 -1;1 -1;1 1;-1 1];fac2=[-1 0;1 0;1 0;-1 0];fac3=[0 -1;0 -1;0 1;0 1];
v1 = [5 0.5; 15 0.5;15 3;5 3];
v1 = fac*rad+v1;
v2 = v1; v2(:,2)=-v2(:,2);
OriObs={v1,v2};

v1 = [5 0.5; 15 0.5;15 3;5 3];
v1 = fac*rad+v1+fac2*1.5+fac3*0;
v2 = v1; v2(:,2)=-v2(:,2);
Obs={v1,v2};

Obsnum=length(Obs);
ObsCenter=zeros(Obsnum,2);
for i=1:Obsnum
    Pointnum=size(Obs{i},1);
    ObsCenter(i,:)=sum(Obs{i})/Pointnum;
end

pos=[1,1];
amp=5;
sigma=[6;1];

dHx=@(x,y) -2*amp*exp(-(x-ObsCenter(:,1)).^2/sigma(1)).*exp(-(y-ObsCenter(:,2)).^2/sigma(2)).*(x-ObsCenter(:,1))/sigma(1);
dHy=@(x,y) -2*amp*exp(-(x-ObsCenter(:,1)).^2/sigma(1)).*exp(-(y-ObsCenter(:,2)).^2/sigma(2)).*(y-ObsCenter(:,2))/sigma(2);
Acl = APF(pos,Obs,amp);

hold on;
for i=1:Obsnum
    H=@(x,y) amp*exp(-(x-ObsCenter(i,1)).^2/sigma(1)).*exp(-(y-ObsCenter(i,2)).^2/sigma(2));
    x=5:0.1:15;y=0:0.1:3.5;y=y*(-1)^(i-1);
    [X,Y] = meshgrid(x,y);
    Z=H(X,Y);
    mesh(X,Y,Z,'FaceAlpha','0.4','EdgeColor','none','FaceColor','flat')
    xlabel('x');ylabel('y');zlabel('z');
    axis equal;
end
