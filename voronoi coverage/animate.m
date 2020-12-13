function animate(state,bnd_pnts)
    n=length(state);
    figure('position',[0 0 600 600],'Color',[1 1 1]);
    for k=1:n
        
        pos_k=one2two(state(:,k));
        [~,vorvx_k] = polybnd_voronoi(pos_k,bnd_pnts);
        
        for i = 1:size(vorvx_k,2)
            col(i,:)= rand(1,3);
        end
        
        for i = 1:size(pos_k,1)
            plot(vorvx_k{i}(:,1),vorvx_k{i}(:,2),'-r')
            hold on;
        end
        plot(bnd_pnts(:,1),bnd_pnts(:,2),'-');
        hold on;
        plot(pos_k(:,1),pos_k(:,2),'Marker','o','MarkerFaceColor',[0 .75 .75],'MarkerEdgeColor','k','LineStyle','none');
        axis('equal')
        axis([0 1 0 1]);
        set(gca,'xtick',[0 1]);
        set(gca,'ytick',[0 1]);
        
        F=getframe(gcf);
        I=frame2im(F);
        [I,map]=rgb2ind(I,256);
        if k == 1
            imwrite(I,map,'animation.gif','gif', 'Loopcount',inf,'DelayTime',0.2);
        else
            imwrite(I,map,'animation.gif','gif','WriteMode','append','DelayTime',0.2);
        end
        
        pause(0.02);
        clf;
    end