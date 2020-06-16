clear
clc

waypts = [0,0,0;
           1,2,2.5;
           2,3,3;
           3,2,2.5;
           4,0,2]';
       
psi_pts=linspace(0,pi/6,length(waypts));

T=20;

[t_series,way_traj,psi_traj]=min_energy_traj_series(waypts,psi_pts,T);

figure
plot3(way_traj(1,:),way_traj(2,:),way_traj(3,:))
title('trajectory')

figure
plot(t_series,psi_traj)
title('psi traj')