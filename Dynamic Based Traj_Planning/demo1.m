clear all
close all
clc

rs=[0;0;0];
re=[4;5;6];

ts=0;
te=20;

psi_s=0;
psi_e=pi/4;

out=min_energy_traj(ts,te,rs,re,psi_s,psi_e);

draw_output()