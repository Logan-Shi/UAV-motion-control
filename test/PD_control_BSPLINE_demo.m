clear all; close all; clc

params = load_params();
quad_a = Quadrotor(params);

dt = 0.01;
quad_a.dt = dt;

duration = 10;
tspan = 0:dt:duration;

%input
k = 4;
n = 6;
P(:,1) = zeros(3,1);
for i = 1:n
    P(1,i) = cos((i-1)/n*2*pi)-1;
    P(2,i) = sin((i-1)/n*2*pi);
    P(3,i) = 1;
end
P = [[0;0;0],P];
load waypts;
P = waypts;
pointNum = size(P,2);
angle = (0:pointNum-1)*pi/pointNum;
R = zeros(3,3,pointNum);
for i=1:pointNum
    R(:,:,i) = rotZ(angle(i)/4)*rotY(0)*rotX(0);
end
[pt,vt,at,Jt] = BSplineC(P,k,tspan,1,1);

%%
for k = 1:length(tspan)
    t = tspan(k);
    p = pt(:,k); v = vt(:,k); a = at(:,k); J = Jt(:,k); yaw = 0; yd = 0;
    
    noise = 0;
    p_c = quad_a.position + randn(1)*noise;
    v_c = quad_a.velocity + randn(1)*noise;
    omg_c = quad_a.Omega + randn(1)*noise;

    [u1,u2] = controller(p,v,a,J,yaw,yd,p_c, v_c, quad_a.attitude, omg_c, quad_a.m, quad_a.g,0.5,0.5,diag([1,1.5,1.2]),diag([0.1,0.1,1.2]));
    
    spd = get_rotorspeed(u1,u2,quad_a.k,quad_a.L,quad_a.b);
    
    quad_a.updateState(spd)

    traj_d(:,k) = p;
    yaw_d(:,k) = yaw;
    yawd_d(:,k) = yd;
    % static_quadrotor_plot(quad_a.position, quad_a.attitude);
    % hold on
    % hold off
    % pause(dt)
end

figure()
traj = quad_a.position_H;
subplot(2,2,1)
plot3(traj(1,:), traj(2,:), traj(3,:), 'b')
hold on; grid on;
plot3(traj_d(1,:), traj_d(2,:), traj_d(3,:), 'r')
view(45,45)
legend('act','des')
title('actual curve')

subplot(2,2,3)
plot(tspan,yaw_d,tspan,quad_a.euler_H(3,2:end))
legend('des','act')
title('yaw')
subplot(2,2,4)
plot(tspan,yawd_d,tspan,yaw_d)
grid on
legend('yawd','yaw')
title('yaw desired')
traj(:,1) = [];
err = norm(traj - traj_d);
fprintf('Error: %.8f\n', err);