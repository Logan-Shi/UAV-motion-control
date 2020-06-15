clear
clc
addpath(genpath('..'));

%���ز���
params=load_params();  %���ز���
m = params.m;
g = params.g;
k = params.k;
kd = params.kd;
I = params.I;
L = params.L;
b = params.b;

% ģ��ʱ��
start_time = 0;
end_time = 10;
dt = 0.005;
times = start_time:dt:end_time;

% ��ʼ״̬
r = [0; 0; 0];
rdot = zeros(3, 1);
euler = [pi/3;pi/3;pi/3];
eulerdot = 0.5 * ones(3,1);
omega=eulerdot2omega(eulerdot,euler);
rotorspeeds=2*[1 1 1 1];

trajectories(:,1)=r;
attitudes(:,1)=euler;

for i=1:length(times)
    
    % �����߼��ٶȺͽǼ��ٶ�
    R = zyxEuler2rotMat(euler);
    a = rotor2acc(rotorspeeds, R, rdot, m, g, k, kd);
    omegadot = angular_acceleration(rotorspeeds, omega, I, L, b, k);
    
    % ���²���
    eulerdot = omega2eulerdot(omega, euler);
    euler = euler + dt * eulerdot;
    omega = omega + dt * omegadot;
    
    r = r + dt * rdot;
    rdot = rdot + dt * a;
    % ��¼��ǰʱ�̵�λ�ú���̬
    trajectories(:,i)=r;
    attitudes(:,i)=euler;
    
    % todo ���ݿ���������rotorspeeds
    
    
    % todo ���ӻ�
    % [~, fig]=static_quadrotor_plot(r,euler);
    % hold on
    % plot3(trajectories(1,:),trajectories(2,:),trajectories(3,:))
    % hold off
    
    % pause(0.002);
end

plot3(trajectories(1,:),trajectories(2,:),trajectories(3,:))
view(45,45)
rmpath(genpath('..'));