function [p_u,NodeVector] = BSpline(P,k,u)
%���ƶ���P,��������k
n = size(P,2)-1;
NodeVector = U_quasi_uniform(n,k); % ׼����B�����Ľڵ�ʸ��

Nik = zeros(n+1,1);

if length(u)>1
    p_u = zeros(size(P,1),length(u));
    for j = 1:length(u)
        for i = 0:n
            Nik(i+1,1) = BaseFunction(i,k,u(j),NodeVector);
        end
        p_u(:,j) = P*Nik;
    end
%     p_u(:,end) = P(:,end);
else
    for i = 0:n
        Nik(i+1,1) = BaseFunction(i,k,u,NodeVector);
    end
    p_u = P*Nik;
end
NodeVector = NodeVector(2:end-1);
end