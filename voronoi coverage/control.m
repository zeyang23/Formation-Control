function vel=control(t,state,bnd_pnts)

pos=one2two(state);
[~,vorvx] = polybnd_voronoi(pos,bnd_pnts);

n=length(vorvx);

% Area=zeros(n,1);
Center=zeros(n,2);

for i=1:n
    Vi=cell2mat(vorvx(1,i));
    xi=Vi(:,1);
    yi=Vi(:,2);
%     Area(i)=polyarea(xi,yi);
    
    Pi=polyshape(xi,yi);
    [xci,yci]=centroid(Pi);
    Center(i,:)=[xci,yci];
end

Center_fixed=two2one(Center);
vel=Center_fixed-state;
vel=5*vel;

end