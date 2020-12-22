dt = 0.5;
duration = 10;
tspan = 0:dt:duration;

%input
k = 3;
n = 5;
P(:,1) = zeros(3,1);
for i = 2:n
    P(:,i) = i*rand(3,1);
end
pointNum = size(P,2);
angle = (0:pointNum-1)*pi/pointNum;
R = zeros(3,3,pointNum);
for i=1:pointNum
    R(:,:,i) = rotZ(angle(i))*rotY(angle(i)/3)*rotX(0);
end
[pt,vt,at,Jt] = BSplineC(P,k,tspan);
[Yt,Ydt,Rt,Pt] = OriInter(R,k,tspan);

plot3(pt(1,:),pt(2,:),pt(3,:),'c');
hold on
for i=1:length(tspan)
    Tc(:,:,i) = eye(4);
    Tc(1:3,1:3,i) = rotZ(Yt(i))*rotY(Pt(i))*rotX(Rt(i));
    Tc(1:3,4,i) = pt(:,i);
    trplot(Tc(:,:,i),'rgb','arrow','length',0.3);
end
axis([-1 6 -1 6 -1 6]);
xlabel('x');ylabel('y');zlabel('z')
grid on
title('desired traj')