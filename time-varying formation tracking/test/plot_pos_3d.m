function plot_pos(t,state)
    n =length(t);
    m = size(state,1)/6; % number of robots

    x=zeros(m,n);
    y=zeros(m,n);

    for k = 1:m
        x(k,:)=state(6*k-5,:);
        y(k,:)=state(6*k-3,:);
    end

    temp_x=x(:);
    temp_y=y(:);

    x_min=min(temp_x);
    x_max=max(temp_x);

    y_min=min(temp_y);
    y_max=max(temp_y);
    
    figure
    axis([ceil(x_min)-1 ceil(x_max) ceil(y_min)-1 ceil(y_max)])

    hold on 
    center_x =zeros(1,length(t));
    center_y = zeros(1,length(t));
    for i = 1:6:6*m
        center_x = center_x+state(i,:);
        center_y = center_y+state(i+2,:);
    end
    center_x=center_x/m;
    center_y=center_y/m;
    hp=plot(center_x,center_y);
    h1=plot(center_x(1,end),center_y(1,end),'p','MarkerSize',12);
    set(h1,'MarkerFaceColor',get(hp,'color'));
    h2=plot(center_x(1,1),center_y(1,1),'o','MarkerSize',8);
    set(h2,'MarkerFaceColor',get(hp,'color'));
    for i=1:m
        plot(state(6*i-5,1),state(6*i-3,1),'o')
    end
    for i=1:m
        h=plot(state(6*i-5,end),state(6*i-3,end),'o','MarkerSize',10);
        set(h,'MarkerFaceColor',get(h,'color'));
    end
    for i=1:m
        plot(state(6*i-5,:),state(6*i-3,:))
    end
    
    title('position')
    axis equal
    grid on
end