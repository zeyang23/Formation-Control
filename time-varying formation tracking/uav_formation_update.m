function state = uav_formation_update(ksi_dot,quads,accel)
	state = zeros(4,1);
    vel = get_vel(ksi_dot);
	acl = get_acl(ksi_dot);
    vels = [];
    for i = 1:size(quads,2)
        v_d = quads{i}.velocity;%[vel(i,:),0]';
        a_d = [acl(i,:),0]';
        j_d = [0,0,0]';
%         p_d = quads{i}.position + v_d * quads{i}.dt + 1/2 * a_d * quads{i}.dt * quads{i}.dt;
        p_d = quads{i}.position;
        yaw = 0;
        yaw_d = 0;
%         disp(["a_x: " a_d(1) "a_y: " a_d(2)])
    	if norm(a_d)>accel
            a_d = accel*a_d/norm(a_d);
        end
        %[u1,u2] = quads{i}.uav_controller(p_d,v_d,a_d,j_d,yaw,yaw_d,0.5,0,diag([1,1,1.2]),0);    
    	[u1,u2] = quads{i}.uav_controller(p_d,v_d,a_d,j_d,yaw,yaw_d,0.5,0.8,diag([1,1,1.2]),diag([0.1,0.1,1.2]));    
        
        rotorSpeeds = get_rotorspeed(u1,u2,quads{i}.k,quads{i}.L,quads{i}.b);    
    	quads{i}.updateState(rotorSpeeds);
        state(4*i-3,1) = quads{i}.position(1);
        state(4*i-2,1) = quads{i}.velocity(1);
        state(4*i-1,1) = quads{i}.position(2);
        state(4*i,1) = quads{i}.velocity(2);
    end
end