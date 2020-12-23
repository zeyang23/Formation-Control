function Acl = APF(Pos,Obs,amp)
% 人工势场法尝试

Obsnum=length(Obs);
ObsCenter=zeros(Obsnum,2);
for i=1:Obsnum
    Pointnum=size(Obs{i},1);
    ObsCenter(i,:)=sum(Obs{i})/Pointnum;
end
sigma=[6;1];
% H=@(x,y) amp*exp(-(x-ObsCenter(1))^2/sigma(1))*exp(-(y-ObsCenter(2))^2/sigma(2))
dHx=@(x,y) 2*amp*exp(-(x-ObsCenter(:,1)).^2/sigma(1)).*exp(-(y-ObsCenter(:,2)).^2/sigma(2)).*(x-ObsCenter(:,1))/sigma(1);
dHy=@(x,y) 2*amp*exp(-(x-ObsCenter(:,1)).^2/sigma(1)).*exp(-(y-ObsCenter(:,2)).^2/sigma(2)).*(y-ObsCenter(:,2))/sigma(2);

Acl=[dHx(Pos(1),Pos(2)),dHy(Pos(1),Pos(2))];

end

