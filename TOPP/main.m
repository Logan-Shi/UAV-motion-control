clear
clc

addpath(genpath('.'));

K = 4;
n = 6;
P(:,1) = zeros(3,1);
for i = 1:n
    P(1,i) = cos((i-1)/n*2*pi)-1;
    P(2,i) = sin((i-1)/n*2*pi);
    P(3,i) = 1;
    P(4,i) = (i-1)/n*2*pi;
end
P = [[0;0;0;0],P];

% P=[0,0,0,0;
%    0.5,0.8,0.3,0;
%    0.3,1.0,0.2,0;
%    0.2,1.2,0.5,0;
%    0.6,1.5,0.3,0;
%    1.0,2.1,0.9,0];
% P=P';

% function demo
t = linspace(0,10,200);
capibility = [1.5,5,3];
%Vmax,Amax,Jmax
isOnPts = 0;
isGraph = 1;

Lower=@(u,udot) BSplineC_L(P(1:3,:),4,t,capibility,isOnPts,isGraph,u,udot);
Upper=@(u,udot) BSplineC_U(P(1:3,:),4,t,capibility,isOnPts,isGraph,u,udot);



s_start=0;
s_end=1;

dt=0.01;

TOL=0.005;

% 初始化
S=[0,0];
i=0;

si=0;
si_dot=0;

sdot_max=10;

% 生成曲线F
F=[s_end,0];
j=1;
while(true)
    L=Lower(F(j,1),F(j,2));
    U=Upper(F(j,1),F(j,2));
    Lseries(j)=L;
    Useries(j)=U;
    if L>U 
        break;
    end
    [F(j+1,1),F(j+1,2)]=back_int(F(j,1),F(j,2),Lower,dt);
    j=j+1;
end

hit_F=0;
% 主循环
while(true)
    si=S(end,1);
    si_dot=S(end,2);
    A=[si,si_dot];
    
    k=1;
    % 前向积分的循环
    while(true)
        % 如果与F相交，则任务完成，跳出主循环
        [intersected,iA,iF]=check(A,F,TOL);
        if intersected == 1
            hit_F=1;
            break;
        end
        
        % 碰撞MVC，则跳转到二分搜索
        if(Lower(A(k,1),A(k,2)) > Upper(A(k,1),A(k,2))) || norm(A(k,2))>sdot_max
            slim=A(k,1);
            slimdot=A(k,2);
            break;
        end
        
        [A(k+1,1),A(k+1,2)]=forward_int(A(k,1),A(k,2),Upper,dt);
        k=k+1;
    end
    
    if hit_F==1
        S(i+1,:)=A(end,:);
        i=i+1;
        break;
    end
    
    % 执行二分搜索，得到相切点
    [stan,standot]=bisection(slim,slimdot,Lower,Upper,dt,TOL);
    
    %从相切点后向积分
    n=1;
    Tan=[stan,standot];
    while(true)
        if check(A,Tan,TOL)==1
            S(i+1,:)=Tan(end,:);
            i=i+1;
            break;
        end
        [Tan(n+1,1),Tan(n+1,2)]=back_int(Tan(j,1),Tan(j,2),Lower,dt);
        n=n+1;
    end
    S(i+1,:)=[stan,standot];
    i=i+1;
end

plot(A(:,1),A(:,2));
hold on
plot(F(:,1),F(:,2));

figure
plot(Lseries)
hold on
plot(Useries)


As=A(:,1)';
t_AS=linspace(0,(iA-1)*dt,iA);

Fs=F(:,1)';
Fs=fliplr(Fs(1:iF));
t_FS=linspace(t_AS(end)+dt,t_AS(end)+iF*dt,iF);

T=[t_AS,t_FS];
UT=[As,Fs];

ut=spline(T,UT,t);


[p_t,v_u,a_u,j_u] = BSplineDrv(P,n,K,ut);

v_t = zeros(3,length(t));
a_t = zeros(3,length(t));
j_t = zeros(3,length(t));
path = 0;
for i = 1:length(t)
    v_t(:,i) = v_u(:,i)*Vt(i)/norm(v_u(:,i));
    path = path + Vt(i)*(t(2)-t(1));
end
disp(['total length ' num2str(path)])
if (ut(end)==1)
    disp(['total time ' num2str(min(t(ut==ut(end))))])
else
    disp('cannot finish path under given constraint')
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
Graph=1;
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
    quiver3(p_t(1,:), p_t(2,:), p_t(3,:),a_t(1,:), a_t(2,:), a_t(3,:), 3)
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

rmpath(genpath('.'));