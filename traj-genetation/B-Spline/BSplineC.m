function [p_t,v_t,a_t,j_t] = BSplineC(P,k,t,cap,OnPts,Graph,type)

if nargin<4
    cap = [3,4,5];
    OnPts = 1;
    Graph = 0;
    type = 1;
else
    if nargin<7
        type = 1;
    end
end

% 经过way points，控制点修正
if OnPts
    [P,P2] = on_way_pts(P,k);
else
    P2 = P;
end

sample_size = length(t);
u = linspace(0,1,sample_size);
[~,v_u,a_u,j_u] = BSplineDrv(P,k,u);
tic
%轨迹规划
if type == 1
    udot = 0.2*ones(1,length(u));
    udot_for = forwardScan(u,udot,v_u,a_u,cap);
    udot_back = backwardScan(u,udot_for,v_u,a_u,cap,0.2);
    udot_jerk = JerkMaxed(udot_back,v_u,a_u,j_u,cap(3));
    disp("spent time calculating")
    disp(toc)
    if Graph
        figure()
        plot(u,udot_for)
        hold on
        plot(u,udot_back)
        plot(u,udot_jerk)
        legend("forward","backward","jerk-constrained")
        legend("forward","jerk-constrained")
        xlabel("u");ylabel("du/dt")
        grid on
    end
else
    if type == 2
        udot = VelPlan(v_u,a_u,cap(1),cap(2));
        udot_jerk = JerkMaxed(udot,v_u,a_u,j_u,cap(3));
        disp("spent time calculating")
        disp(toc)
        if Graph
            figure()
            plot(u,udot)
            hold on
            plot(u,udot_jerk)
            legend("without-jerk","jerk-constrained")
            xlabel("u");ylabel("du/dt")
            grid on
        end
else
    if type == 3
        udot = 0.2*ones(1,length(u));
        udot_for = forwardScan(u,udot,v_u,a_u,cap);
        udot_back = backwardScan(u,udot_for,v_u,a_u,cap,0.2);
%         udot_jerk = JerkMaxed(udot_back,v_u,a_u,j_u,cap(3));
        disp("spent time calculating")
        disp(toc)
        if Graph
            figure()
            plot(u,udot_for)
            hold on
            plot(u,udot_back)
%             plot(u,udot_jerk)
            legend("forward","backward")
%             legend("forward","jerk-constrained")
            xlabel("u");ylabel("du/dt")
            grid on
        end
    end
end

%轨迹插值
[ut,udott,uddott] = calcUt(t,u,udot_back);
[p_t,v_u,a_u,~] = BSplineDrv(P,k,ut);

%分析
path = 0;
v_t = zeros(3,length(t));
a_t = zeros(3,length(t));
Vt = zeros(1,length(t));
for i = 1:length(t)
    v_t(:,i) = v_u(:,i)*udott(i);
    a_t(:,i) = a_u(:,i)*(udott(i))^2 + v_u(:,i)*uddott(i);
    Vt(i) = norm(v_u(:,i)*udott(i));
    path = path + Vt(i)*(t(2)-t(1));
end
j_t = FDMinter3(t,a_t);
disp(['total length ' num2str(path)])
if (ut(end)>1-1e-5)
    disp(['total time ' num2str(min(t(ut>1-1e-5)))])
else
    disp('cannot finish path under given constraint')
end

if Graph
    % graphing
    figure()
    plot(t,ut)
    hold on
    plot(t,udott)
    plot(t,uddott)
    legend("u","udot","uddot")
    title("time-u")
    xlabel("time,s")
    ylabel("u")
    
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
    plot3(p_t(1,:), p_t(2,:), p_t(3,:), 'Marker','.','LineStyle','-', 'Color',[.3 .6 .9]);
    quiver3(p_t(1,:), p_t(2,:), p_t(3,:),v_t(1,:), v_t(2,:), v_t(3,:), 3)
    quiver3(p_t(1,:), p_t(2,:), p_t(3,:),a_t(1,:), a_t(2,:), a_t(3,:), 3)
    grid on;axis equal
    legend('control point','p','v','a')
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
    grid on
    
    subplot(3,1,2)
    plot(t,v_t(1,:))
    hold on
    plot(t,v_t(2,:))
    plot(t,v_t(3,:))
    legend('xdot','ydot','zdot')
    xlabel('time,s');ylabel('m/s');
    title('x y z velocity')
    grid on
    
    subplot(3,1,3)
    plot(t,a_t(1,:))
    hold on
    plot(t,a_t(2,:))
    plot(t,a_t(3,:))
    legend('xddot','yddot','zddot')
    xlabel('time,s');ylabel('m/s^2');
    title('x y z accelaration')
    grid on
    
    figure()
    plot(t,j_t(1,:))
    hold on
    plot(t,j_t(2,:))
    plot(t,j_t(3,:))
    legend('xdddot','ydddot','zdddot')
    xlabel('time,s');ylabel('m/s^3');
    title('x y z jerk')
    grid on
end
end