function [L,U]=binary(u,udot)

%input
k = 4;
n = 6;
% P(:,1) = zeros(3,1);
% for i = 1:n
%     P(1,i) = cos((i-1)/n*2*pi)-1;
%     P(2,i) = sin((i-/n*2*pi);
%     P(3,i) = 1;
%     P(4,i) = (i-1)/n*2*pi;
% end
% P = [[0;0;0;0],P];

P=[0,0,0,0;
   0.5,0.8,0.3,0;
   0.3,1.0,0.2,0;
   0.2,1.2,0.5,0;
   0.6,1.5,0.3,0;
   1.0,2.1,0.9,0];
P=P';

% function demo
t = linspace(0,1,200);
capibility = [1.5,0.08,3];
%Vmax,Amax,Jmax
isOnPts = 0;
isGraph = 1;
[L,U] = BSplineC(P(1:3,:),4,t,capibility,isOnPts,isGraph,u,udot);