function animate_pos(t,state,fps)
close all;
n =length(t);
m = size(state,1)/4; % number of robots
sig={'o','+','*','x','s','d','^','v','p','h'};

x=zeros(m,n);
y=zeros(m,n);

for k = 1:m
    x(k,:)=state(4*k-3,:);
    y(k,:)=state(4*k-1,:);
end

temp_x=x(:);
temp_y=y(:);

x_min=min(temp_x);
x_max=max(temp_x);

y_min=min(temp_y);
y_max=max(temp_y);

figure
axis([ceil(x_min)-1 ceil(x_max) ceil(y_min)-1 ceil(y_max)])
grid on
axis equal
hold on

% 绘制障碍物
Obsplot();

for i = 1:n
    pics=[];
    pan=[];
    traj=[];
    Radius=0.25;
    for j =1:m
        pics(j)=plot(state(4*j-3,i),state(4*j-1,i),'*','MarkerEdgeColor',[j/m j/m 1-j/m]);
        pan(j)=CirclePlot(state(4*j-3,i),state(4*j-1,i),Radius,j);
        traj(j)=plot(state(4*j-3,1:i),state(4*j-1,1:i),'Color',[j/m 1-j/m j/m]);
    end
    pause(1/fps);
    if i==n
        continue;
    end
    delete(pics);
    delete(pan);
    delete(traj);
end
end