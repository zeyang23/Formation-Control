function dksi = Obsdetect(ksi,Obs,Robotrad,dt)
% 计算修正的加速度
num=length(ksi)/4;
Obsnum=size(Obs,1);
ColRad=1;
dksi=zeros(4*num,1);

for i=1:num
    for j=1:Obsnum
        pos=[ksi(4*i-3),ksi(4*i-1)]+dt*[ksi(4*i-2),ksi(4*i)];
        block=Obs(j,1:2);
        radius=Obs(j,3);
        
        distance=norm(pos-block);
        if distance<radius+Robotrad
            % 调整加速度修改的幅值
            gain=10;
            % 距离小于半径之和可能会碰撞
            for theta=pi/2:pi/12:3*pi/2
                relative=pos-block;
                Rotate=[cos(theta),-sin(theta);
                        sin(theta),cos(theta)];
                AfRel=gain*transpose(Rotate*relative');
                tempAcel=[dksi(4*i-2)+AfRel(1),dksi(4*i)+AfRel(2)];
                % 判断修正后是否碰撞
                posnew=pos+tempAcel*dt^2;
                if norm(posnew-block)<ColRad+Robotrad
                    dksi(4*i-2)=dksi(4*i-2)+AfRel(1);
                    dksi(4*i)=dksi(4*i)+AfRel(2);
                    break;
                end                
            end
        end       
    end
    
end

end
