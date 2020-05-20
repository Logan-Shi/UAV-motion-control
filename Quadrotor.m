classdef Quadrotor < handle
    % author: ZRT
    % last edited: ZRT
    % Quadrotor class
    % Usage:
    % init a quadrotor
    %   use quad = Quadrotor(params) or quad = Quadrotor(m,g,k,kd,I,L,b)
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
        % simulation interval
        dt = 0.0005;
        
        % quad noise:
        % dynamics noise: noise in the dynamics model
        % sensor noise: noise in the sensor
        dynamics_noise = [];
        sensor_noise = [];
        
        % quad noise type: currently set to normal distribution
        dynamics_noise_type = @(t)(t.*randn(size(t)));
        sensor_noise_type = @(t)(t.*randn(size(t)));
        
    end
    
    properties (SetAccess=private)
        % private access means read-only access

        % history of the quadrotor
        % these value have NO sensor noises, in order to track the actual position
        position_H; velocity_H; acceleration_H;
        attitude_H; Omega_H; Omega_dot_H; euler_H;

        % state of a quadrotor
        % attitude: rotation matrix from body to world
        % set this to private to add noise whenever try to get them.
        position; velocity; acceleration;
        attitude; Omega; Omega_dot; euler;
        
        % paramters of a quadrotor
        % explaned below in func params=load_params
        m; g = 9.8; k; kd; I; L; b; 
        current_step = 1;
    end
    
    methods 
        function Quad=Quadrotor(m,g,k,kd,I,L,b)
            if nargin == 1
                Quad.m = m.m; Quad.g = m.g; Quad.k = m.k; Quad.kd = m.kd;
                Quad.I = m.I; Quad.L = m.L; Quad.b = m.b;
            else
                Quad.m = m; Quad.g = g; Quad.k = k; Quad.kd = kd;
                Quad.I = I; Quad.L = L; Quad.b = b;
            end
            zeroInitState(Quad);
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
    end
end

% function params=load_params()
%     % 机体参数
%     params.m = 1;
%     params.g = 9.8;
%     params.k = 1;
%     params.kd = 0;
%     params.I = diag([0.04 0.04 0.008]);
%     params.L = 0.25;
%     params.b = 0.007;
% end
