clear all; close all; clc
%input
k = 4;
n = 6;
P(:,1) = zeros(3,1);
for i = 1:n
    P(1,i) = cos((i-1)/n*2*pi)-1;
    P(2,i) = sin((i-1)/n*2*pi);
    P(3,i) = 1;
    P(4,i) = (i-1)/n*2*pi;
end
P = [[0;0;0;0],P];

% function demo
t = linspace(0,10,200);
capibility = [3,4,3];
%Vmax,Amax,Jmax
isOnPts = 1;
isGraph = 1;
[p_u,v_u,a_u,j_u] = BSplineC(P(1:3,:),4,t,capibility,isOnPts,isGraph);