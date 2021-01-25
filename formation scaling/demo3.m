% demo3: SCALE UP & DOWN

clear
clc

span = [0 30]; 
h = 0.01;

y0 = [10+5i;
      5+5i;
      5i;
      5;
      5+10i;
      
      1+0.5i;
      1+0.5i;
      0;
      0;
      0];


[t,y] = RK4(@func,span,y0,h); 

x1=real(y(1,:));
y1=imag(y(1,:));
x2=real(y(2,:));
y2=imag(y(2,:));
x3=real(y(3,:));
y3=imag(y(3,:));
x4=real(y(4,:));
y4=imag(y(4,:));
x5=real(y(5,:));
y5=imag(y(5,:));

figure
hold on
plot(x1,y1,'-*','MarkerSize',1)
plot(x2,y2,'-*','MarkerSize',1)
plot(x3,y3,'-*','MarkerSize',1)
plot(x4,y4,'-*','MarkerSize',1)
plot(x5,y5,'-*','MarkerSize',1)
legend("1","2","3","4","5",'Location','NorthWest')
title('agent trajectories')

% figure
% for i = 1:length(t)
%     p1 = plot(y(1,i),'o','MarkerSize',2,'MarkerFaceColor','red');
%     hold on
%     p2 = plot(y(2,i),'o','MarkerSize',2,'MarkerFaceColor','blue');
%     p3 = plot(y(3,i),'o','MarkerSize',2,'MarkerFaceColor','green');
%     p4 = plot(y(4,i),'o','MarkerSize',2,'MarkerFaceColor','yellow');
%     p5 = plot(y(5,i),'o','MarkerSize',2,'MarkerFaceColor','black');
%     
%     axis([0,40,0,40]);
%     
%     pause(0.1);
% %     delete(p1);
% %     delete(p2);
% %     delete(p3);
% %     delete(p4);
% %     delete(p5);
% end

function ydot=func(t,y)
    v0=1+1i;
    if(t<10)
        r12=3;
    elseif(t<20)
        r12=6;
    else
        r12=3;
    end
    leader_dist=norm(y(2,:)-y(1,:));
    
    k=2;
    
    ydot_leader=[v0+k*(y(2,:)-y(1,:))*(leader_dist^2-r12^2);
                 v0+k*(y(1,:)-y(2,:))*(leader_dist^2-r12^2)];
L=[0     0     0  0 0;
   0     0     0  0 0; 
   1     1     -2 0 0;
   0     0 -1i  2i -1i;
   -1+2i -1-2i 0  0 2];

Llf=L(3:5,1:2);
Lff=L(3:5,3:5);


H=[0  0  0 0  0;
   0  0  0 0  0;
   0  -1 2 -1 0;
   -1 -1 0 2  0;
   -1 0  0 -1 2];

Hlf=H(3:5,1:2);
Hff=H(3:5,3:5);

d3=-3;
d4=-0.005-0.25i;
d5=0.25+0.005i;

Df=diag([d3,d4,d5]);

A=[-Df*Lff,   eye(3);
    zeros(3), -Hff];
B=[-Df*Llf,   zeros(3,2);
    zeros(3,2), -Hlf];
    
    ydot_follower=A*[y(3:5,:);y(8:10,:)]+B*[y(1:2,:);[v0;v0]];
    
    ydot=[ydot_leader;ydot_follower(1:3,:);0;0;ydot_follower(4:6,:)];
    
end