function [p_u,v_u,a_u,j_u] = BSplineC(P,k,t,OnPts,Graph)
n = size(P,2)-1;
NodeVector = U_quasi_uniform(n,k); % 准均匀B样条的节点矢量
% 经过way points，控制点修正
if OnPts
    [P,P2] = on_way_pts(P,k,NodeVector);
    p_tmp = BSpline(P,k,linspace(0,1,n+1));
    disp(['off way points by: ' num2str(norm(p_tmp-P2))]);
end

% 导矢计算
[p_u,v_u,a_u] = BSplineDrv(P,n,k,t);

% 速度规划
Vsq_u = VelPlan(t,p_u,v_u,a_u);

% 轨迹插补
ut = trajInter(t,Vsq_u,v_u,a_u);

p_u = BSpline(P,k,ut);
p_sample = BSpline(P,k,ut(end));
v_u = FDMinter3(t,p_u);
a_u = FDMinter3(t,v_u);
j_u = FDMinter3(t,a_u);
figure()
if Graph
    % graphing
    subplot(1,2,1)
    hold on
    plot3(P2(1, :), P2(2, :),P2(3, :),...
        'o','LineWidth',1,...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor','g',...
        'MarkerSize',6);
    plot3(p_sample(1, :), p_sample(2, :),p_sample(3, :),...
        'o','LineWidth',1,...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor','b',...
        'MarkerSize',8);
    plot3(p_u(1,:), p_u(2,:), p_u(3,:), 'Marker','.','LineStyle','-', 'Color',[.3 .6 .9]);
    quiver3(p_u(1,:), p_u(2,:), p_u(3,:),v_u(1,:), v_u(2,:), v_u(3,:), 3)
    quiver3(p_u(1,:), p_u(2,:), p_u(3,:),a_u(1,:), a_u(2,:), a_u(3,:), 3)
    grid on;axis equal
    legend('control point','end point','p','v','a')
    title('B-Spline Demo')
    xlabel('x');ylabel('y');zlabel('z')
    
    subplot(1,2,2)
    plot(t,p_u(1,:))
    hold on
    plot(t,v_u(1,:))
    plot(t,a_u(1,:))
%     plot(t,j_u(1,:))
    legend('pos','vel','acc')
    xlabel('x');ylabel('y');
    title('higher derivatives')
end
end