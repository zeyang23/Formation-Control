function plot_vel(t,state)
    n =length(t);
    m = size(state,1)/4; % number of robots

    vx=zeros(m,n);
    vy=zeros(m,n);

    for k = 1:m
        vx(k,:)=state(4*k-2,:);
        vy(k,:)=state(4*k,:);
    end

    temp_vx=vx(:);
    temp_vy=vy(:);

    vx_min=min(temp_vx);
    vx_max=max(temp_vx);

    vy_min=min(temp_vy);
    vy_max=max(temp_vy);
    
    figure
    axis([ceil(vx_min)-1 ceil(vx_max) ceil(vy_min)-1 ceil(vy_max)])

    hold on 
    
    center_vx =zeros(1,length(t));
    center_vy =zeros(1,length(t));
    for i = 1:4:4*m
        center_vx = center_vx+state(i+1,:);
        center_vy = center_vy+state(i+3,:);
    end
    center_vx=center_vx/m;
    center_vy=center_vy/m;
    hv=plot(center_vx,center_vy);
    h3=plot(center_vx(1,end),center_vy(1,end),'p','MarkerSize',12);
    set(h3,'MarkerFaceColor',get(hv,'color'));
    h4=plot(center_vx(1,1),center_vy(1,1),'o','MarkerSize',8);
    set(h4,'MarkerFaceColor',get(hv,'color'));
    for i=2:2:2*m
        plot(state(2*i-2,1),state(2*i,1),'o')
    end
    for i=2:2:2*m
        h=plot(state(2*i-2,end),state(2*i,end),'o','MarkerSize',10);
        set(h,'MarkerFaceColor',get(h,'color'));
    end
    for i=2:2:2*m
        plot(state(2*i-2,:),state(2*i,:))
    end

    title('velocity')
    axis equal
    grid on
end