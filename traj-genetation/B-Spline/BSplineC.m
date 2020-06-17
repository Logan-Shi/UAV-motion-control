function [p_t,v_t,a_t,j_t] = BSplineC(P,k,t,max,OnPts,Graph)
n = size(P,2)-1;
NodeVector = U_quasi_uniform(n,k); % 准均匀B样条的节点矢量
% 经过way points，控制点修正
if OnPts
    [P,P2] = on_way_pts(P,k,NodeVector);
    p_tmp = BSpline(P,k,linspace(0,1,n+1));
    disp(['off way points by: ' num2str(norm(p_tmp-P2))]);
end

% 导矢计算
u = linspace(0,1,length(t));
[p_u,v_u,a_u,j_u] = BSplineDrv(P,n,k,u);

% 速度规划
[Vsq_u,kapsq] = VelPlan(t,p_u,v_u,a_u,max(1),max(2));
Vsq_jerk = JerkMaxed(Vsq_u,v_u,a_u,j_u,t,kapsq,max(3));
% 轨迹插补
[ut,Vt] = trajInter(t,P,n,k,Vsq_jerk);
p_sample = BSpline(P,k,ut(end));
% % for i = 1:length(t)
% %     V(i) = norm(v_u(:,i));
% % end
% % plot(t,V)
[p_t,v_u,a_u,j_u] = BSplineDrv(P,n,k,ut);
v_t = zeros(3,length(t));
a_t = zeros(3,length(t));
j_t = zeros(3,length(t));
for i = 1:length(t)
    v_t(:,i) = v_u(:,i)*Vt(i)/norm(v_u(:,i));
end
for i = 1:length(t)
    a_t(:,i) = a_u(:,i)*(Vt(i)/norm(v_u(:,i)))^2-v_u(:,i)*Vt(i)^2*dot(v_u(:,i),a_u(:,i))/norm(v_u(:,i))^4;
end
j_t = FDMinter3(t,a_t);
% p_td = BSpline(P,k,ut);
% v_td = FDMinter3(t,p_t);
% a_td = FDMinter3(t,v_t);
% j_td = FDMinter3(t,a_t);
% figure()
% plot(t,a_td(1,:))
% hold on
% plot(t,a_t(1,:))
% legend('fdm','calced')

if Graph
    % graphing
    figure()
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
    plot3(p_t(1,:), p_t(2,:), p_t(3,:), 'Marker','.','LineStyle','-', 'Color',[.3 .6 .9]);
    quiver3(p_t(1,:), p_t(2,:), p_t(3,:),v_t(1,:), v_t(2,:), v_t(3,:), 3)
    quiver3(p_t(1,:), p_t(2,:), p_t(3,:),a_u(1,:), a_t(2,:), a_t(3,:), 3)
    grid on;axis equal
    legend('control point','end point','p','v','a')
    title('B-Spline Demo')
    xlabel('x');ylabel('y');zlabel('z')
    view(45,45)
    
    figure()
    subplot(3,1,1)
    plot(t,p_t(1,:))
    hold on
    plot(t,p_t(2,:))
    plot(t,p_t(3,:))
    legend('x','y','z')
    xlabel('time,s');ylabel('m');
    title('x y z position')
    
    subplot(3,1,2)
    plot(t,v_t(1,:))
    hold on
    plot(t,v_t(2,:))
    plot(t,v_t(3,:))
    legend('xdot','ydot','zdot')
    xlabel('time,s');ylabel('m/s');
    title('x y z velocity')
    
    subplot(3,1,3)
    plot(t,a_t(1,:))
    hold on
    plot(t,a_t(2,:))
    plot(t,a_t(3,:))
    legend('xddot','yddot','zddot')
    xlabel('time,s');ylabel('m/s^2');
    title('x y z accelaration')
    
    figure()
    plot(t,j_t(1,:))
    hold on
    plot(t,j_t(2,:))
    plot(t,j_t(3,:))
    legend('xdddot','ydddot','zdddot')
    xlabel('time,s');ylabel('m/s^3');
    title('x y z jerk')
end
end