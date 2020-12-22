classdef Quadrotor < handle
    % author: ZRT
    % last edited: SZK
    % Quadrotor class
    % Usage:
    % init a quadrotor
    %   use quad = Quadrotor(params,position)
    %   to create a quadrotor with zero inital state
    %
    %   change the property: position, attitude to DIRECTLY change the position and the attitude

    % simulation interval
    %   the simulation time inteval is by default 0.0005
    %   use quad.dt to modify/read this value

    % update state
    %   use quad.updateState(rotorSpeeds) to update the state of the rotor, by quad.dt

    % get params/state
    %   use quad.getQuadParams, quad.getQuadState to get params/state of the quadrotor
    %   use quad.getQuadTime to get the current time of this quadrotor
    %   use quad.(PROPERTY)_H to get the history of the quadrotor
    %       eg: quad.position_H is the history of the quad position
    
    properties (SetAccess=public)
        % state of a quadrotor
        % attitude: rotation matrix from body to world
        position; velocity; acceleration;
        attitude; Omega; Omega_dot; euler;

        % history of the quadrotor
        position_H; velocity_H; acceleration_H;
        attitude_H; Omega_H; Omega_dot_H; euler_H;
        rotorspeeds;

        % simulation interval
        dt = 0.0005;

    end
    
    properties (SetAccess=private)
        % paramters of a quadrotor
        % explaned below in func params=load_params
        m; g = 9.8; k; kd; I; L; b; 
        current_step = 1;
    end
    
    methods 
        function Quad=Quadrotor(params,pos)
            Quad.m = params.m; Quad.g = params.g; Quad.k = params.k; Quad.kd = params.kd;
            Quad.I = params.I; Quad.L = params.L; Quad.b = params.b;
            if nargin == 1    
                zeroInitState(Quad);
            else
                initStatePos(Quad,pos(1),pos(2),pos(3));
            end
            
        end

        function input2acc(Quad, rotorSpeeds)
            Quad.acceleration = rotor2acc(rotorSpeeds, Quad.attitude, Quad.velocity, Quad.m, Quad.g, Quad.k, Quad.kd);
            Quad.Omega_dot = angular_acceleration(rotorSpeeds, Quad.Omega, Quad.I, Quad.L, Quad.b, Quad.k);
            Quad.current_step = Quad.current_step + 1;

            Quad.acceleration_H(:,Quad.current_step) = Quad.acceleration;
            Quad.Omega_dot_H(:,Quad.current_step) = Quad.Omega_dot;
        end

        function updateState(Quad, rotorSpeeds)
            input2acc(Quad, rotorSpeeds);
            % attitude_dot = Quad.attitude*xev(Quad.Omega_dot);
            % Quad.attitude = attitude_dot*Quad.dt + Quad.attitude;
            
            eulerDot = omega2eulerdot(Quad.Omega, Quad.euler);
            Quad.euler = Quad.euler + Quad.dt*eulerDot;
            Quad.attitude = zyxEuler2rotMat(Quad.euler);
            Quad.Omega = Quad.Omega + Quad.Omega_dot*Quad.dt;
            
            Quad.position = Quad.position + Quad.velocity*Quad.dt;
            Quad.velocity = Quad.velocity + Quad.acceleration*Quad.dt;

            Quad.position_H(:,Quad.current_step) = Quad.position;
            Quad.velocity_H(:,Quad.current_step) = Quad.velocity;
            Quad.attitude_H(:,:,Quad.current_step) = Quad.attitude;
            Quad.Omega_H(:,Quad.current_step) = Quad.Omega;
            Quad.euler_H(:,Quad.current_step) = Quad.euler;
            Quad.rotorspeeds(:,Quad.current_step) = rotorSpeeds;
        end

        function params=getQuadParams(Quad)
            params.m = Quad.m;
            params.g = Quad.g;
            params.k = Quad.k;
            params.kd = Quad.kd;
            params.I = Quad.I;
            params.L = Quad.L;
            params.b = Quad.b;
        end

        function state=getQuadState(Quad, noise)
            if nargin == 1
                noise = 0;
            end
            state.position = Quad.position;
            state.velocity = Quad.velocity;
            state.acceleration = Quad.acceleration;
            state.attitude = Quad.attitude;
            state.Omega = Quad.Omega;
            state.Omega_dot = Quad.Omega_dot;
        end
        
        function t=getQuadTime(Quad)
            t = Quad.dt*Quad.current_step;
        end

        function zeroInitState(Quad)
            Quad.position = [0,0,0].';
            Quad.velocity = [0,0,0].';
            Quad.acceleration = [0,0,0].';
            Quad.attitude = eye(3);
            Quad.Omega = [0,0,0].';
            Quad.Omega_dot = [0,0,0].';
            Quad.euler = [0,0,0].';

            Quad.position_H = [0,0,0].';
            Quad.velocity_H = [0,0,0].';
            Quad.acceleration_H = [0,0,0].';
            Quad.attitude_H = eye(3);
            Quad.Omega_H = [0,0,0].';
            Quad.Omega_dot_H = [0,0,0].';
            Quad.euler_H = [0,0,0].';
        end

        function initStatePos(Quad,x,y,z)
            Quad.position = [x,y,z].';
            Quad.velocity = [0,0,0].';
            Quad.acceleration = [0,0,0].';
            Quad.attitude = eye(3);
            Quad.Omega = [0,0,0].';
            Quad.Omega_dot = [0,0,0].';
            Quad.euler = [0,0,0].';

            Quad.position_H = [0,0,0].';
            Quad.velocity_H = [0,0,0].';
            Quad.acceleration_H = [0,0,0].';
            Quad.attitude_H = eye(3);
            Quad.Omega_H = [0,0,0].';
            Quad.Omega_dot_H = [0,0,0].';
            Quad.euler_H = [0,0,0].';
        end

        function [u1,u2]=uav_controller(Quad,ksi_desired, ksi_desired_dot, ksi_desired_dot2, ksi_desired_dot3, psi_desired, psi_desired_dot, Kp_ksi, Kd_ksi, Kp_omega, Kd_omega)
        
            e_ksi=Quad.position-ksi_desired;
            e_ksi_dot=Quad.velocity-ksi_desired_dot;
            F_desired = -Kp_ksi*e_ksi-Kd_ksi*e_ksi_dot+Quad.m*Quad.g*[0;0;1]+Quad.m*ksi_desired_dot2;
        
            u1=dot(F_desired,Quad.attitude(:,3));
        
            zB_desired=F_desired/norm(F_desired);
            xC_desired=[cos(psi_desired);sin(psi_desired);0];
            yB_desired=cross(zB_desired,xC_desired)/norm(cross(zB_desired,xC_desired));
            xB_desired=cross(yB_desired,zB_desired);
        
            R_desired=[xB_desired,yB_desired,zB_desired];
        
            e_R=1/2*vex((R_desired.')*Quad.attitude-(Quad.attitude.')*R_desired);
            e_R = e_R.';
        
            % omega_desired 到底该怎么算？ 还没想完全清楚。
            h=Quad.m/u1*(ksi_desired_dot3-dot(zB_desired,ksi_desired_dot3)*zB_desired);
            p_desired=-dot(h,yB_desired);
            q_desired=dot(h,xB_desired);
            r_desired=psi_desired_dot*dot([0;0;1],zB_desired);
            omega_desired=[p_desired;q_desired;r_desired];
        
            e_omega=Quad.Omega-omega_desired;
            u2=-Kd_omega*e_omega-Kp_omega*e_R;
        end
    end
end