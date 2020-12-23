function cir=CirclePlot(x,y,Radius,j)

cmap = hsv(5); %// define colors. You could change `hsv` to `jet`, `cool`, ...
alpha = 0.1; %// define level of transparency
t = 0 : .1 : 2 * pi;
Rx = Radius * cos(t) + x;
Ry = Radius * sin(t) + y;
cir=patch(Rx, Ry, cmap(j,:), 'facealpha', alpha, 'edgecolor', 'none'); 

end