function my_animate(t,y)
figure;
axis([-5 70 -10 10])
axis equal
hold on
n =length(t);
m = size(y,1)/4;
for i = 1:n
    pics=[];
    for j =1:m
        pics(j)=plot(y(4*j-3,i),y(4*j-1,i),'o');
    end
    pause(0.01);
    delete(pics);
end
end