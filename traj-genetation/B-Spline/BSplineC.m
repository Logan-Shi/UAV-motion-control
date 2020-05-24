function [p_u,v_u,a_u,j_u] = BSplineC(P,k,t,OnPts,Graph)
n = size(P,2)-1;
NodeVector = U_quasi_uniform(n,k); % 准均匀B样条的节点矢量
if OnPts % 经过way points，控制点修正
    [P,P2] = on_way_pts(P,n,k,NodeVector);
end

[Q,Q2,Q3,v_u,a_u] = BSplineDrv(P,n,k,t,NodeVector);

% 速度规划
Vsq_u = VelPlan(t,v_u,a_u);

N = length(t);
% Ts = t(2)-t(1);
% ut = zeros(1,N);
% for i = 2:N
%     V = sqrt(Vsq_u(i-1));
%     dC = BSpline(Q,k-1,ut(i-1));
%     ddC = BSpline(Q2,k-2,ut(i-1));
%     ut(i) = ut(i-1)+V*Ts/norm(dC)-V^2*Ts^2*dot(dC,ddC)/2/(norm(dC)^4);
% end
p_u = BSpline(P,k,t);
v_u = BSpline(Q,k-1,t);
V = zeros(1,N);
for i = 1:N
    V(i) = sqrt(Vsq_u(i));
    v_u(:,i) = v_u(:,i)/norm(v_u(:,i))*V(i);
end
A = FDMinter(t,V);
a_u = BSpline(Q2,k-2,t);
for i = 1:N
    a_u(:,i) = a_u(:,i)/norm(a_u(:,i))*A(i);
end
J = FDMinter(t,A);
j_u = BSpline(Q3,k-3,t);
for i = 1:N
    j_u(:,i) = j_u(:,i)/norm(j_u(:,i))*J(i);
end

if Graph
    % graphing
    subplot(1,2,1)
    hold on
    plot3(P2(1, :), P2(2, :),P2(3, :),...
        'o','LineWidth',1,...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor','g',...
        'MarkerSize',6);
    line(P2(1, :), P2(2, :), P2(3, :));
    plot3(p_u(1,:), p_u(2,:), p_u(3,:), 'Marker','.','LineStyle','-', 'Color',[.3 .6 .9]);
    quiver3(p_u(1,:), p_u(2,:), p_u(3,:),v_u(1,:), v_u(2,:), v_u(3,:), 3)
%     plot3(P(1, :), P(2, :),P(3, :),...
%         'o','LineWidth',1,...
%         'MarkerEdgeColor','k',...
%         'MarkerFaceColor','r',...
%         'MarkerSize',6);
    grid on;axis equal
    legend('control point','control polygon','p','v')
    title('B-Spline Demo')
    xlabel('x');ylabel('y');zlabel('z')
    
    subplot(1,2,2)
    plot(t,p_u(1,:))
    hold on
    plot(t,v_u(1,:))
    plot(t,a_u(1,:))
    plot(t,j_u(1,:))
    legend('pos','vel','acc','jer')
    xlabel('x');ylabel('y');
    title('higher derivatives')
end
end