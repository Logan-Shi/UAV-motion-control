clear all; close all; clc

n = 20;
expCO = rand(3,n);

R = zeros(3,3,n);
for k = 1:n
    R(:,:,k) = exp2R(expCO(:,k));
end

T = linspace(0,5,n);
omega0=[0.6,0.1,-0.3];

ti=linspace(0,3,10001);

%**************************************************
%********************exp interp*******************
%**************************************************

ri = CubicExpInterp(T,R,ti,omega0);

% 三点中心差分计算r的导数
ri_dot=zeros(3,length(ti));
ri_dot(:,1)=(ri(:,2)-ri(:,1))/(ti(2)-ti(1));
ri_dot(:,end)=(ri(:,end)-ri(:,end-1))/(ti(end)-ti(end-1));
for j=2:length(ti)-1
    ri_dot(:,j)=(ri(:,j+1)-ri(:,j-1))/(ti(j+1)-ti(j-1));
end

% 根据rdot计算角速度
omegai_exp=zeros(3,length(ti));
for j=1:length(ti)
    % omegai_exp(:,j)=rdot2omega(ri(:,j),ri_dot(:,j));
    omegai_exp(:,j)=expCdot2omega(ri(:,j),ri_dot(:,j));
end

figure(1)
hold on
plot(ti,ri(1,:))
plot(ti,ri(2,:))
plot(ti,ri(3,:))
title('expC')

figure(2)
hold on
plot(ti,ri_dot(1,:))
plot(ti,ri_dot(2,:))
plot(ti,ri_dot(3,:))
title('expCdot')

figure(3)
hold on
plot(ti,omegai_exp(1,:))
plot(ti,omegai_exp(2,:))
plot(ti,omegai_exp(3,:))
title('omega exp')

%**************************************************
%********************quat interp*******************
%**************************************************

ri = CubicQuatInterp(T,R,ti,omega0);

% 三点中心差分计算r的导数
ri_dot=zeros(4,length(ri));
ri_dot(:,1)=(ri(:,2)-ri(:,1))/(ti(2)-ti(1));
ri_dot(:,end)=(ri(:,end)-ri(:,end-1))/(ti(end)-ti(end-1));
for j=2:length(ri)-1
    ri_dot(:,j)=(ri(:,j+1)-ri(:,j-1))/(ti(j+1)-ti(j-1));
end

% 根据rdot计算角速度
omegai_quat=zeros(3,length(ri));
for j=1:length(ri)
    % omegai_quat(:,j)=rdot2omega(ri(:,j),ri_dot(:,j));
    omegai_quat(:,j)=qdot2omega(ri(:,j),ri_dot(:,j));
end

figure(4)
hold on
plot(ti,ri(1,:))
plot(ti,ri(2,:))
plot(ti,ri(3,:))
plot(ti,ri(4,:))
title('quat')

figure(5)
hold on
plot(ti,ri_dot(1,:))
plot(ti,ri_dot(2,:))
plot(ti,ri_dot(3,:))
plot(ti,ri_dot(4,:))
title('qdot')

figure(6)
hold on
plot(ti,omegai_quat(1,:))
plot(ti,omegai_quat(2,:))
plot(ti,omegai_quat(3,:))
title('omega quat')

omegai_quat_std = std(omegai_quat')
omegai_exp_std = std(omegai_exp')
