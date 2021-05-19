clear all; close all; clc
% input
k = 4; % rank of BSpline, at least 4 for jerk constraint
n = 6; % number of waypts
t = linspace(0,10,1000);
max = [3,5,5]; % Vmax,Amax,Jmax
isOnPts = 1; % is trajectory need to pass waypts
isGraph = 1; % is graphing, may slow down demo
type = 1; % 1 for time-scaling,2 for Linear Programming

% circle path
P(:,1) = zeros(3,1);
for i = 1:n
    P(1,i) = cos((i-1)/n*2*pi)-1;
    P(2,i) = sin((i-1)/n*2*pi);
    P(3,i) = 1;
    P(4,i) = (i-1)/n*2*pi;
end
P = [[0;0;0;0],P];

% scan path
n = 5;
m = 5;
count = 1;
signal = 1;
for i = 1:n
    for j = 1:m
        P(1,count) = signal*(j-1)+(n-1)*(1-signal)/2;
        P(2,count) = i;
        P(3,count) = 1;
        P(4,count) = (1-signal)/2*pi;
        count = count + 1;
    end
    signal = -signal;
end
% P = [[0;0;0;0],P];

% ohter waypts
% load waypts
% P = waypts;
% P(3,:) = P(3,:)+1;
% P = [[0;0;0],P];

% function demo
[p_u,v_u,a_u,j_u] = BSplineC(P(1:3,:),k,t,max,isOnPts,isGraph,type);
