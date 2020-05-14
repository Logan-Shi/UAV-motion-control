function [p_u,NodeVector] = BSpline(P,k,t)
%控制顶点P,样条次数k
% %clamped
% for i = 1:k+1
%     P = [P(:,1),P,P(:,end)];
% end
n = size(P,2)-1;
NodeVector = linspace(0, 1, n+k+2); % 均匀B样条的节点矢量

% 绘制样条曲线
Nik = zeros(n+1,1);
u = linspace(k/(n+k+1),(n+1)/(n+k+1),length(t));
p_u = zeros(size(P,1),length(u));
for j = 1:length(u)
    for i = 0:n
        Nik(i+1,1) = BaseFunction(i,k,u(j),NodeVector);
    end
    p_u(:,j) = P*Nik;
end
end