function Obsplot()

c = [0; 0; 6;6];
v = [5 0.5; 15 0.5;15 3;5 3];
f = [1 2 3 4];
patch('Faces',f,'Vertices',v,'FaceColor',[0.8500 0.3250 0.0980],'EdgeColor','none','FaceAlpha',0.4)

c = [0; 0; 6;6];
v(:,2)=-v(:,2);
f = [1 2 3 4];
patch('Faces',f,'Vertices',v,'FaceColor',[0.9290 0.6940 0.1250],'EdgeColor','none','FaceAlpha',0.4)

end