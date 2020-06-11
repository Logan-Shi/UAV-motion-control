function [p_u,v_u,a_u,j_u] = BSplineC(P,k,t,OnPts,Graph)
n = size(P,2)-1;
NodeVector = U_quasi_uniform(n,k); % 准均匀B样条的节点矢量
if OnPts % 经过way points，控制点修正
    [P,P2] = on_way_pts(P,n,k,NodeVector);
end

[p_u,v_u,a_u] = BSplineDrv(P,n,k,t,NodeVector);

% 速度规划
Vsq_u = VelPlan(t,p_u,v_u,a_u);
N = length(t);
u = linspace(0,1,N);
figure()
plot(u,Vsq_u);
title('V(u)')
xlabel('u');
ylabel('V(u)');
ut = zeros(1,N);
for i = 1:N-1
    if i == 1
        V = sqrt(Vsq_u(2));
        dC = v_u(:,2);
        ddC = a_u(:,2);
        AA = V^2*dot(dC,ddC)/2/(norm(dC)^4);
        BB = V/norm(dC);
        CC = u(2)-u(1);
        Ts1 = -(-BB+sqrt(BB^2-4*AA*CC))/2/AA;
        Ts2 = -(-BB-sqrt(BB^2-4*AA*CC))/2/AA;
        Ts = min(Ts1,Ts2);
    else
        V = sqrt(Vsq_u(i));
        dC = v_u(:,i);
        ddC = a_u(:,i);
        AA = V^2*dot(dC,ddC)/2/(norm(dC)^4);
        BB = -V/norm(dC);
        CC = u(2)-u(1);
        Ts1 = (-BB+sqrt(BB^2-4*AA*CC))/2/AA;
        Ts2 = (-BB-sqrt(BB^2-4*AA*CC))/2/AA;
        Ts = min(Ts1,Ts2);
    end
    if AA<0
        Ts = max(Ts1,Ts2);
    end
    ut(i+1) = ut(i)+Ts;
end
figure()
plot(ut,u)
u_appliedt = pchip(ut,u,t);
p_u = BSpline(P,k,u_appliedt);
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
    line(P2(1, :), P2(2, :), P2(3, :));
    plot3(p_u(1,:), p_u(2,:), p_u(3,:), 'Marker','.','LineStyle','-', 'Color',[.3 .6 .9]);
    quiver3(p_u(1,:), p_u(2,:), p_u(3,:),v_u(1,:), v_u(2,:), v_u(3,:), 3)
%     P3 = BSpline(P,k,10);
%     plot3(P3(1, :), P3(2, :),P3(3, :),...
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
%     plot(t,j_u(1,:))
    legend('pos','vel','acc','jer')
    xlabel('x');ylabel('y');
    title('higher derivatives')
end
end