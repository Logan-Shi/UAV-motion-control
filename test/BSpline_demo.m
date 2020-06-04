clear all;
%input
k = 5;
n = 6;
P(:,1) = zeros(3,1);
for i = 1:n
    P(:,i) = (i-1)*ones(3,1);
end

load waypts
P = waypts;

% function demo
t = linspace(0,10,100);
isOnPts = 1;
isGraph = 1;
[p_u,v_u,a_u,j_u] = BSplineC(P,k,t,isOnPts,isGraph);