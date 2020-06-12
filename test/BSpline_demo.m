clear all; close all; clc
%input
k = 4;
n = 6;
P(:,1) = zeros(3,1);
for i = 1:n
    P(1,i) = cos((i-1)/n*2*pi)-1;
    P(2,i) = sin((i-1)/n*2*pi);
    P(3,i) = 1;
end
P = [[0;0;0],P];

% figure()
% p_u = BSpline(P,k,linspace(0,1,n));
% plot3(P(1,:),P(2,:),P(3,:),...
%                         'o','LineWidth',1,...
%                         'MarkerEdgeColor','k',...
%                         'MarkerFaceColor','g',...
%                         'MarkerSize',6);
% hold on
% plot3(p_u(1,:),p_u(2,:),p_u(3,:));

% load waypts
% P = waypts;

% function demo
t = linspace(0,3,1000);
isOnPts = 1;
isGraph = 1;
[p_u,v_u,a_u,j_u] = BSplineC(P,k,t,isOnPts,isGraph);