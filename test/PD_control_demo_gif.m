clear all; close all;

% addpath(genpath('..'))

params = load_params();
quad_a = Quadrotor(params); save_gif = false; use_bspline = true;

dt = 0.01; quad_a.dt = dt; end_t = 10; tspan = 0:dt:end_t;

% waypts = [0,0,0;
%            1,2,2.5;
%            2,3,3;
%            3,2,2.5;
%            4,0,2]';
% waypts = [
%     0,0,0;
%     rand(1,3);
%     2*rand(1,3);
%     3*rand(1,3);
%     4*rand(1,3);
%     5*rand(1,3);
% ].';

load waypts;

v0 = [0,0,0]; a0 = [0,0,0]; v1 = [0,0,0]; a1 = [0,0,0];

if use_bspline
    [pt,vt,at,Jt] = BSplineC(waypts,3,tspan);
    % Kp1 = 1.5; Kd1 = 3.3; Kp2 = 1.5; Kd2 = 3.5;
else
    [pt,vt,at,Jt] = min_snap_simple_fcn(waypts,v0,a0,v1,a1,end_t,tspan);
end
Kp1 = 0.6; Kd1 = 2.5; Kp2 = diag([1,1.8,1.8]); Kd2 = diag([1.5,1.5,1.5]);



[yaw_t,yaw_dot_t]= jtraj(0,pi/6,tspan);

if save_gif
    figure()
    plot3(pt(1,:), pt(2,:), pt(3,:), 'b');
    hold on; grid on; 
    plot3(waypts(1,:), waypts(2,:), waypts(3,:), 'ro') ;
    view(45,45); title('real time result');
    ax = [-1.5,4.5,-0.5,4.5,-2,6];
    axis(ax);
end

for k = 1:length(tspan)
    t = tspan(k);
    p = pt(:,k); v = vt(:,k); a = at(:,k); J = Jt(:,k);
    yaw = yaw_t(k); yaw_dot = yaw_dot_t(k);
    
    noise = 0;
    p_c = quad_a.position + randn(1)*noise;
    v_c = quad_a.velocity + randn(1)*noise;
    omg_c = quad_a.Omega + randn(1)*noise;

    % 0,0: yaw, yaw_dot
    [u1,u2] = controller(p,v,a,J,yaw,yaw_dot,p_c, v_c, quad_a.attitude, omg_c, quad_a.m, quad_a.g, Kp1,Kd1,Kp2,Kd2);
    
    rotorSpeeds = get_rotorspeed(u1,u2,quad_a.k,quad_a.L,quad_a.b);
    
    quad_a.updateState(rotorSpeeds);
    
    if save_gif
        static_quadrotor_plot(quad_a.position, quad_a.attitude, quad_a.L, 1);
        hold on
        plot3(pt(1,:), pt(2,:), pt(3,:), 'b');
        plot3(quad_a.position_H(1,:), quad_a.position_H(2,:), quad_a.position_H(3,:),'r');
        title('real time result');
        hold off
        
        axis(ax);

        img = frame2im(getframe(gcf));
        [img, map] = rgb2ind(img, 256);
        if mod(k,10) == 2
            if k == 2
                imwrite(img, map, 'test.gif', 'gif', 'Loopcount', inf, 'DelayTime', dt);
            else
                imwrite(img, map, 'test.gif', 'gif', 'WriteMode', 'append', 'DelayTime', dt);
            end
        end
    end

end

traj = quad_a.position_H;
traj(:,1) = [];

figure()
plot3(traj(1,:), traj(2,:), traj(3,:), 'b')
hold on; grid on; view(45,45)
plot3(pt(1,:), pt(2,:), pt(3,:), 'r')
plot3(waypts(1,:), waypts(2,:), waypts(3,:), 'ro') ;
title('Simulation result'); legend('actual trajectory', 'desired trajectory', 'way points')


figure()
plot3(pt(1,:), pt(2,:), pt(3,:))
view(45,45); grid on; hold on;
plot3(waypts(1,:), waypts(2,:), waypts(3,:), 'ro') ;
title('Desired trajectory')
legend('desired trajectory', 'way points')

figure()
subplot(2,2,1)
plot(tspan, traj(1,:), tspan, pt(1,:))
xlabel('t'); ylabel('x'); legend('actual', 'desired')
title('x coordinates');
subplot(2,2,2)
plot(tspan, traj(2,:), tspan, pt(2,:))
xlabel('t'); ylabel('y'); legend('actual', 'desired')
title('y coordinates');
subplot(2,2,3)
plot(tspan, traj(3,:), tspan, pt(3,:))
xlabel('t'); ylabel('z'); legend('actual', 'desired')
title('z coordinates');
subplot(2,2,4)
plot(tspan, (quad_a.euler_H(3,2:end)), tspan, yaw_t)
xlabel('t'); ylabel('yaw'); legend('actual', 'desired')
title('yaw angle');


err = norm(traj - pt);
x_err = norm(traj(1,:) - pt(1,:));
y_err = norm(traj(2,:) - pt(2,:));
z_err = norm(traj(3,:) - pt(3,:));
yaw_err = norm(quad_a.euler_H(3,2:end) - yaw_t.');
fprintf('Error: %.8f\n', err);
fprintf('X Error: %.8f\n', x_err);
fprintf('Y Error: %.8f\n', y_err);
fprintf('Z Error: %.8f\n', z_err);
fprintf('Yaw Error: %.8f\n', yaw_err);

% rmpath(genpath('..'))
