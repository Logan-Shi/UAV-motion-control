function [p_u,v_u,a_u,j_u] = BSplineC(P,k,t,OnPts,Graph)
n = size(P,2)-1;
if OnPts % 经过way points，控制点修正
    NodeVector = U_quasi_uniform(n,k); % 准均匀B样条的节点矢量
    
    u = linspace(0,1,n+1);
    phi = zeros(n+1);
    for i = 0:n
        for j = 0:n
            phi(i+1,j+1) = BaseFunction(j,k,u(i+1),NodeVector);
        end
    end
    
    P2 = (transpose(phi)*phi)\(transpose(phi)*transpose(P));
    P2 = transpose(P2);
    P = P2;
end

% 曲线导矢计算
for i = 0:n-1
    Q(:,i+1) = k/(NodeVector(i+k+1)-NodeVector(i+1))*(P(:,i+2)-P(:,i+1));
end
[v_u,NodeVector] = BSpline(Q,k-1,t);

for i = 0:n-2
    Q2(:,i+1) = (k-1)/(NodeVector(i+k)-NodeVector(i+1))*(Q(:,i+2)-Q(:,i+1));
end
a_u = BSpline(Q2,k-2,t);

for i = 0:n-3
    Q3(:,i+1) = (k-2)/(NodeVector(i+k-1)-NodeVector(i+1))*(Q2(:,i+2)-Q2(:,i+1));
end

% 速度规划
Vsq_max = 4;%m^2/s^2
Track_err_max = 0.1;%m
Acc_x_max = 3;%m/s^2
Acc_y_max = 3;%m/s^2
Acc_z_max = 3;%m/s^2
N = length(t);
Max_target = ones(1,N-2);
lb = zeros(1,N-2);
ub = calc_vel_ub(Track_err_max,t,Vsq_max,v_u,a_u);
b = [Acc_x_max*ones(1,N-2),Acc_y_max*ones(1,N-2),Acc_z_max*ones(1,N-2)];
A = calc_acc_cstrts(v_u,a_u,t);
res = linprog(Max_target,A,b,[],[],lb,ub);
Vsq_u = [0;res;0];
ut = zeros(1:N);
for i = 2:N
    V = sqrt(Vsq_u(i-1));
    dC = BSpline(Q,k-1,ut(i-1));
    ddC = BSpline(Q2,k-2,ut(i-1));
    ut(i) = ut(i-1)+V*Ts/norm(dC)-V^2*Ts^2*dot(dC,ddC)/2/(norm(dC)^4);
end
p_u = BSpline(P,k,ut);
v_u = BSpline(Q,k-1,ut);
for i = 1:N
    v_u(:,i) = v_u(:,i)/norm(v_u(:,i))*sqrt(Vsq_u(i));
end
a_u = BSpline(Q2,k-2,ut);
j_u = BSpline(Q3,k-3,ut);

if Graph
    % graphing
    subplot(1,2,1)
    hold on
    plot3(P(1, :), P(2, :),P(3, :),...
        'o','LineWidth',1,...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor','g',...
        'MarkerSize',6);
    plot3(P2(1, :), P2(2, :),P2(3, :),...
        'o','LineWidth',1,...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor','r',...
        'MarkerSize',6);
    line(P(1, :), P(2, :), P(3, :));
    plot3(p_u(1,:), p_u(2,:), p_u(3,:), 'Marker','.','LineStyle','-', 'Color',[.3 .6 .9]);
    quiver3(p_u(1,:), p_u(2,:), p_u(3,:),v_u(1,:), v_u(2,:), v_u(3,:), 3)
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