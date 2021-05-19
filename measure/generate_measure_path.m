clear all; close all; clc
surfix = './measure/';
obj = 'car.mat';
filepath = strcat(surfix,obj);
load(filepath,'wayPts');

% input
k = 1; % rank of BSpline, at least 4 for jerk constraint
n = 6; % number of waypts
t = linspace(0,10,1000);
max = [3,5,5]; % Vmax,Amax,Jmax
isOnPts = 1; % is trajectory need to pass waypts
isGraph = 1; % is graphing, may slow down demo
type = 1; % 1 for time-scaling,2 for Linear Programming
P = wayPts;
u = linspace(0,1,24);
% function demo
[P,P2] = on_way_pts(P,k);
[p,~] = BSpline(P,k,u);

figure()
hold on
plot3(P2(1, :), P2(2, :),P2(3, :),...
    'o','LineWidth',1,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor','g',...
    'MarkerSize',6);
%     plot3(p_sample(1, :), p_sample(2, :),p_sample(3, :),...
%         'o','LineWidth',1,...
%         'MarkerEdgeColor','k',...
%         'MarkerFaceColor','b',...
%         'MarkerSize',8);
plot3(p(1,:), p(2,:), p(3,:), 'Marker','.','LineStyle','-', 'Color',[.3 .6 .9]);
grid on;axis equal
legend('control point','p')
title('B-Spline Demo')
xlabel('x');ylabel('y');zlabel('z')
view(45,45)

ftable = fopen(strcat(surfix,'path_car.txt'),'w');
for j = 1:size(p,2)
    fprintf(ftable,' %d %d %d\n',p(1,j),p(2,j),p(3,j));
end
fclose(ftable);

