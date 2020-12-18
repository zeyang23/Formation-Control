function ksi_dot_add = obstacle(ksi)
    ksi_dot_add=zeros(size(ksi));
    pos = get_pos(ksi);
    pos_x = pos(:,1);
    pos_y = pos(:,2);
    for i = 1:length(pos_x)
        if pos_x(i)<=85 && pos_x(i)>15
            if pos_y(i)>=2
                ksi_dot_add(4*i) = -50;
            elseif pos_y(i)<=-2
                ksi_dot_add(4*i) = 50;
            end
        end
    end
end