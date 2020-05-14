clear all; close all; clc

params = load_params();
quad_a = Quadrotor(params);

dt = 0.01;
quad_a.dt = dt;

start_t = 0; end_t = 10;
tspan = 0:dt:10;

% waypts = [0,0,0;
%            1,2,2.5;
%            2,3,3;
%            3,2,2.5;
%            4,0,2]';
waypts = [
    0,0,0;
    rand(1,3);
    2*rand(1,3);
    3*rand(1,3);
    4*rand(1,3);
    5*rand(1,3);
].';
v0 = [0,0,0];
a0 = [0,0,0];
v1 = [0,0,0];
a1 = [0,0,0];

[pt,vt,at,Jt] = min_snap_simple_fcn(waypts,v0,a0,v1,a1,end_t,tspan);

for k = 1:length(tspan)
    t = tspan(k);
    p = pt(:,k); v = vt(:,k); a = at(:,k); J = Jt(:,k);
    
    noise = 1;
    p_c = quad_a.position + randn(1)*noise;
    v_c = quad_a.velocity + randn(1)*noise;
    omg_c = quad_a.Omega + randn(1)*noise;

    [u1,u2] = controller(p,v,a,J,0,0,p_c, v_c, quad_a.attitude, omg_c, quad_a.m, quad_a.g, 1,1,1,1);
    
    spd = get_rotorspeed(u1,u2,quad_a.k,quad_a.L,quad_a.b);
    
    quad_a.updateState(spd)

    traj_d(:,k) = p;
    % static_quadrotor_plot(quad_a.position, quad_a.attitude);
    % hold on
    % hold off
    % pause(dt)
end

traj = quad_a.position_H;
figure()
plot3(traj(1,:), traj(2,:), traj(3,:), 'b')
hold on; grid on;
plot3(traj_d(1,:), traj_d(2,:), traj_d(3,:), 'r')
view(45,45)
title('actual curve')

figure()
plot3(traj_d(1,:), traj_d(2,:), traj_d(3,:))
view(45,45)
title('desired curve')

traj(:,1) = [];
err = norm(traj - traj_d);
fprintf('Error: %.8f\n', err);
