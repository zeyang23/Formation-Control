function Roboplot(Record,Obs)
% 画无人机运动的轨迹
num=size(Record,1)/4;
Obsnum=size(Obs,1);
sig={'o','+','*','x','s','d','^','v','p','h'};
figure()
hold on;
for i=1:num
    plot(Record(4*i-3,1),Record(4*i-1,1),'Marker',sig{i},'Markersize',12);
    plot(Record(4*i-3,:),Record(4*i-1,:));
    plot(Record(4*i-3,end),Record(4*i-1,end),'Marker',sig{i},'Markersize',12);
end

for i=1:Obsnum
    plot(Obs(i,1),Obs(i,2),'.')
    CirclePlot(Obs(i,1),Obs(i,2),Obs(i,3))
end

axis equal;
xlabel('x');
ylabel('y');
end
