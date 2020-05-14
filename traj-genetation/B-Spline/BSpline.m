function [p_u,NodeVector] = BSpline(P,k,t)
%控制顶点P,样条次数k
n = size(P,2)-1;
NodeVector = U_quasi_uniform(n,k); % 准均匀B样条的节点矢量

% 绘制样条曲线
Nik = zeros(n+1,1);
u = linspace(0,1,length(t));
p_u = zeros(size(P,1),length(u));
for j = 1:length(u)
    for i = 0:n
        Nik(i+1,1) = BaseFunction(i,k,u(j),NodeVector);
    end
    p_u(:,j) = P*Nik;
end
p_u(:,end) = P(:,end);
NodeVector = NodeVector(2:end-1);
end