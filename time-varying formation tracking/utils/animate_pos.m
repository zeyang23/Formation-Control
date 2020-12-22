function animate_pos(t,state,fps)
close all;
n =length(t);
m = size(state,1)/4; % number of robots

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

for i = 1:n
    pics=[];
    for j =1:m
        pics(j)=plot(state(4*j-3,i),state(4*j-1,i),'o');
    end
    pause(1/fps);
    delete(pics);
end
end