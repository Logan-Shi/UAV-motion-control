clear all;
%input
k = 3;
n = 4;
P(:,1) = zeros(3,1);
for i = 2:n
    P(:,i) = i*rand(3,1);
end

load waypts
P = waypts;

% function demo
t = linspace(0,10,100);
[p_u,v_u,a_u,j_u] = BSplineC(P,k,t);

% graphing
subplot(2,1,1)
hold on
plot3(P(1, :), P(2, :),P(3, :),...
                'o','LineWidth',1,...
                'MarkerEdgeColor','k',...
                'MarkerFaceColor','g',...
                'MarkerSize',6);
line(P(1, :), P(2, :), P(3, :));
plot3(p_u(1,:), p_u(2,:), p_u(3,:), 'Marker','.','LineStyle','-', 'Color',[.3 .6 .9]);
quiver3(p_u(1,:), p_u(2,:), p_u(3,:),v_u(1,:), v_u(2,:), v_u(3,:), 3)
grid on;axis equal
title('B-Spline Demo')
xlabel('x');ylabel('y');zlabel('z')

subplot(2,1,2)
plot(t,p_u(1,:))
hold on
plot(t,v_u(1,:))
plot(t,a_u(1,:))
plot(t,j_u(1,:))
legend('pos','vel','acc','jer')
