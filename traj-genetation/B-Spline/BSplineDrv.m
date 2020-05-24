function [Q,Q2,Q3,v_u,a_u] = BSplineDrv(P,n,k,t,NodeVector)
NodeVector4v = NodeVector(2:end-1);
% ÇúÏßµ¼Ê¸¼ÆËã
for i = 0:n-1
    Q(:,i+1) = k/(NodeVector4v(i+k+1)-NodeVector4v(i+1))*(P(:,i+2)-P(:,i+1));
end
[v_u,NodeVector4a] = BSpline(Q,k-1,t);

for i = 0:n-2
    Q2(:,i+1) = (k-1)/(NodeVector4a(i+k)-NodeVector4a(i+1))*(Q(:,i+2)-Q(:,i+1));
end
[a_u,NodeVector4j] = BSpline(Q2,k-2,t);

for i = 0:n-3
    Q3(:,i+1) = (k-2)/(NodeVector4j(i+k-1)-NodeVector4j(i+1))*(Q2(:,i+2)-Q2(:,i+1));
end
end