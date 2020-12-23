function ksi_dot_add = obstacle(ksi,ksi_dot)
% num为机器人数量
ksi_dot_add=zeros(size(ksi));
pos = get_pos(ksi);
vel = get_vel(ksi);
acl = get_acl(ksi_dot);
num=length(ksi)/4;dt=0.01;
center=sum(pos,1)/num;

rad=0.3;% 无人机半径
%  定义障碍物位置，扩展半径，将机器人可视为质点
fac=[-1 -1;1 -1;1 1;-1 1];fac2=[-1 0;1 0;1 0;-1 0];fac3=[0 -1;0 -1;0 1;0 1];
v1 = [5 0.5; 15 0.5;15 3;5 3];
v1 = fac*rad+v1;
v2 = v1; v2(:,2)=-v2(:,2);
OriObs={v1,v2};

v1 = [5 0.5; 15 0.5;15 3;5 3];
v1 = fac*rad+v1+fac2*1.5+fac3*0;
v2 = v1; v2(:,2)=-v2(:,2);
Obs={v1,v2};

% 碰撞检测与加速度修正
PosNext=pos+vel*dt+acl/2*dt^2;
AclCorrect=zeros(num,2);
gain=50;
for i=1:num
    % Obs为减速区，OriObs为不能碰撞的区域
    flag=Obstect(PosNext(i,:),Obs);
    if sum(flag)~=0
        % 判断是否碰撞
        flagCol=Obstect(PosNext(i,:),OriObs);
        if sum(flagCol)==0
            % 在减速区且不碰撞
            % 向机器人中心汇聚，其实明显不正确
            % correct=gain*(center-pos(i,:))/norm(center-pos(i,:)); 
            % 直接向中心靠拢，其实并不算正确
            correct=(gain*flag'*[-1;1])*[0,1];
            amp=50;
            correct=gain*flag'*APF(pos(i,:),Obs,amp);
            AclCorrect(i,:)=correct;
        else
            % 在减速区且在碰撞区，需要搜索合适的加速度
            % 搜索的精度
            accuracy=12;
            if flagCol(1)==1
                theta=linspace(2*pi,0,accuracy);
            elseif flagCol(2)==1
                theta=linspace(0,2*pi,accuracy);
            else
                continue;
            end
            correct=gain*(center-pos(i,:))/norm(center-pos(i,:));
            R=[];
            % 搜索合适的加速度向量
            for j=1:length(theta)
                R=[cos(theta(j)),-sin(theta(j));
                   sin(theta(j)),cos(theta(j))];
                AclTemp=correct*transpose(R);
                MayPos=pos(i,:)+vel(i,:)*dt+(acl(i,:)+AclTemp)*dt^2;

                iscol=Obstect(MayPos,OriObs);
                if iscol==zeros(2,1)
                    % 修正成功
                    AclCorrect(i,:)=AclTemp;
                    break;
                end
                if j==length(theta)
                    disp('No proper Acceleration Correct find!');
                end
            end
        end
    end

end

% if AclCorrect~=zeros(num,2)
%     disp('Its Correcting!')
% end

% 赋值与修正
for i=1:num
    ksi_dot_add(4*i-2)=AclCorrect(i,1);
    ksi_dot_add(4*i)=AclCorrect(i,2);
end
    
end

function isCollision=Obstect(pos,Obs)
% 检测障碍物Obs与位置pos是否碰撞
% 利用四个角度进行计算，参考https://blog.csdn.net/hhjhh76/article/details/89468653
Obsnum = length(Obs);
isCollision=-ones(Obsnum,1);
rectangle=[];
for i=1:Obsnum
    rectangle=Obs{i};
    flag=[dot(pos-rectangle(4,:),rectangle(3,:)-rectangle(4,:));
          dot(pos-rectangle(4,:),rectangle(1,:)-rectangle(4,:));
          dot(pos-rectangle(2,:),rectangle(3,:)-rectangle(2,:));
          dot(pos-rectangle(2,:),rectangle(1,:)-rectangle(2,:));];
    if (flag>=0)==ones(4,1)
        % 无人机在矩形内部，碰撞
        isCollision(i)=1;
    else
        % 未碰撞
        isCollision(i)=0;
    end
end
end

