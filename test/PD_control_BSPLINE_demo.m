clear all; close all; clc

params = load_params();
quad_a = Quadrotor(params);

dt = 0.01;
quad_a.dt = dt;

duration = 10;
tspan = 0:dt:duration;

%input
k = 4;
P = [0,9.036145, 21.084337, 37.607573, 51.893287, 61.187608;
    0,51.779661, 70.084746, 50.254237, 69.745763, 49.576271;
        0,-10         10           -20        30       -40];
pointNum = size(P,2);
angle = (1:pointNum)*pi/pointNum;
R = zeros(3,3,pointNum);
for i=1:pointNum
    R(:,:,i) = rotZ(angle(i))*rotY(angle(i))*rotX(angle(i));
end
[pt,vt,at,Jt] = BSplineC(P,k,tspan);
[Rt,Rdt] = OriInter(R,k,tspan);

for k = 1:length(tspan)
    t = tspan(k);
    p = pt(:,k); v = vt(:,k); a = at(:,k); J = Jt(:,k); yaw = Rt(2,k); yd = Rdt(2,k);
    
    noise = 1;
    p_c = quad_a.position + randn(1)*noise;
    v_c = quad_a.velocity + randn(1)*noise;
    omg_c = quad_a.Omega + randn(1)*noise;

    [u1,u2] = controller(p,v,a,J,yaw,yd,p_c, v_c, quad_a.attitude, omg_c, quad_a.m, quad_a.g, 1,1,1,1);
    
    spd = get_rotorspeed(u1,u2,quad_a.k,quad_a.L,quad_a.b);
    
    quad_a.updateState(spd)

    traj_d(:,k) = p;
    yaw_d(:,k) = yaw;
    % static_quadrotor_plot(quad_a.position, quad_a.attitude);
    % hold on
    % hold off
    % pause(dt)
end

traj = quad_a.position_H;
subplot(2,2,1)
plot3(traj(1,:), traj(2,:), traj(3,:), 'b')
hold on; grid on;
plot3(traj_d(1,:), traj_d(2,:), traj_d(3,:), 'r')
view(45,45)
legend('act','des')
title('actual curve')
subplot(2,2,2)
plot3(traj_d(1,:), traj_d(2,:), traj_d(3,:))
view(45,45)
title('desired curve')
subplot(2,2,3)
plot(tspan,yaw_d,tspan,quad_a.Omega_H(2,2:end))
legend('des','act')
title('yaw')
traj(:,1) = [];
err = norm(traj - traj_d);
fprintf('Error: %.8f\n', err);