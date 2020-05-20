clear
clc
addpath('../minimum-snap')

waypts = [0,0,-3;
          0.4,0.5,-3.2;
          0.8,0.9,-3.4;
          0.5,1.2,-3.2;
          0.2,1,-3]';

v0 = [0,0,0];
a0 = [0,0,0];
v1 = [0,0,0];
a1 = [0,0,0];
T = 10;
h=0.005;

[xx,yy,zz,tt]=min_snap_simple(waypts,v0,a0,v1,a1,T,h);

XT=[transpose(tt),transpose(xx)];
% XT=[transpose(tt),transpose(ones(size(xx)))];
YT=[transpose(tt),transpose(yy)];
ZT=[transpose(tt),transpose(zz)];

mdl_quadrotor;

sim('./sl_quadrotor',10)
figure
plot(xx,yy)
rmpath('../minimum-snap')