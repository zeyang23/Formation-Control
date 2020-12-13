function plot_voronoi_2d(vorvx,pos,bnd_pnts)
    for i = 1:size(vorvx,2)
        col(i,:)= rand(1,3);
    end

    figure('position',[0 0 600 600],'Color',[1 1 1]);
    for i = 1:size(pos,1)
        plot(vorvx{i}(:,1),vorvx{i}(:,2),'-r')
        hold on;
    end
    plot(bnd_pnts(:,1),bnd_pnts(:,2),'-');
    hold on;
    plot(pos(:,1),pos(:,2),'Marker','o','MarkerFaceColor',[0 .75 .75],'MarkerEdgeColor','k','LineStyle','none');
    axis('equal')
    axis([0 1 0 1]);
    set(gca,'xtick',[0 1]);
    set(gca,'ytick',[0 1]);