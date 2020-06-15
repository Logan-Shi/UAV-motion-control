clear
clc
waypts = [0,0,0;
           1,2,2.5;
           2,3,3;
           3,2,2.5;
           4,0,2]';
v0 = [0,0,0];
a0 = [0,0,0];
v1 = [0,0,0];
a1 = [0,0,0];
T = 5;
h=0.01;

min_snap_simple(waypts,v0,a0,v1,a1,T,h)
hold on
min_energy_simple(waypts,v0,a0,v1,a1,T,h)
legend('keyframes','min snap','min energy')

% r=0.2;
% min_snap_corridor(waypts,v0,a0,v1,a1,T,h,r)

psi_pts=linspace(0,pi/6,length(waypts));
psi_dot_0 = 0;
psi_dot2_0 = 0;
psi_dot_1 = 0;
psi_dot2_1 = 0;
min_psi_dot2(psi_pts,psi_dot_0,psi_dot2_0,psi_dot_1,psi_dot2_1,T,h)
title('psi')
legend('keyframes','psi')