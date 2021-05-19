clear all; close all; clc
surfix = './circle/';
% obj = 'car.mat';
% filepath = strcat(surfix,obj);
% load(filepath,'wayPts');
point1 = [100,0];
point2 = [200,0];
wayPts = generate_wayPts(point1,point2,50);
% input
k = 3; % rank of BSpline, at least 4 for jerk constraint
n = 6; % number of waypts
t = linspace(0,10,650);
max = [300,200,500]; % Vmax,Amax,Jmax
isOnPts = 1; % is trajectory need to pass waypts
isGraph = 1; % is graphing, may slow down demo
type = 3; % 1 for time-scaling,2 for Linear Programming
P = wayPts;
[p_u,v_u,a_u,j_u] = BSplineC(P(1:3,:),k,t,max,isOnPts,isGraph,type);
p = p_u;
v = v_u;
ftable = fopen(strcat(surfix,'path_car.txt'),'w');
for j = 1:size(p,2)
    fprintf(ftable,'CGeoPoint:new_local(%d,%d),\n',p(1,j),p(2,j));
end
fprintf(ftable,'\n');
for j = 1:size(p,2)
    fprintf(ftable,'CGeoPoint:new_local(%d,%d),\n',v(1,j),v(2,j));
end
fclose(ftable);