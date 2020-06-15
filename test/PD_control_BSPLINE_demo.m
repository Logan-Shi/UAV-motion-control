clear all; close all; clc
addpath(genpath('..'));

params = load_params();
quad_a = Quadrotor(params);

dt = 0.1;
quad_a.dt = dt;

duration = 10;
tspan = 0:dt:duration;

%input
k = 5;
n = k+1;
P(:,1) = zeros(3,1);
for i = 2:n
    P(:,i) = i*ones(3,1);
end
% load waypts;
% P =  waypts;
pointNum = size(P,2);
angle = (0:pointNum-1)*pi/pointNum;
R = zeros(3,3,pointNum);
for i=1:pointNum
    R(:,:,i) = rotZ(angle(i)/4)*rotY(0)*rotX(0);
end
[pt,vt,at,Jt] = BSplineC(P,k,tspan,1,0);
% [Yt,Ydt,Rt,Pt] = OriInter(R,k,tspan);

subplot(2,2,2)
plot3(pt(1,:),pt(2,:),pt(3,:),'c');
hold on
% for i=1:length(tspan)
%     Tc(:,:,i) = eye(4);
%     Tc(1:3,1:3,i) = rotZ(Yt(i))*rotY(Pt(i))*rotX(Rt(i));
%     Tc(1:3,4,i) = pt(:,i);
%     trplot(Tc(:,:,i),'rgb','arrow','length',0.3);
% end
axis([-1 6 -1 6 -1 6]);
grid on
title('desired traj')

%%
for k = 1:length(tspan)
    t = tspan(k);
    p = pt(:,k); v = vt(:,k); a = at(:,k); J = Jt(:,k); yaw = 0; yd = 0;
    
    noise = 0;
    p_c = quad_a.position + randn(1)*noise;
    v_c = quad_a.velocity + randn(1)*noise;
    omg_c = quad_a.Omega + randn(1)*noise;

    [u1,u2] = controller(p,v,a,J,yaw,yd,p_c, v_c, quad_a.attitude, omg_c, quad_a.m, quad_a.g,0.8,0.1,0.1,0.2);
    
    spd = get_rotorspeed(u1,u2,quad_a.k,quad_a.L,quad_a.b);
    
    quad_a.updateState(spd)

    traj_d(:,k) = p;
    yaw_d(:,k) = yaw;
    yawd_d(:,k) = yd;
    plot3(pt(1,:),pt(2,:),pt(3,:),'c');
    hold on
    static_quadrotor_plot(quad_a.position, quad_a.attitude);
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

rmpath(genpath('..'));