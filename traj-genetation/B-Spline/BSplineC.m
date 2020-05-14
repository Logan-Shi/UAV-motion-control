function [p_u,v_u,a_u,j_u] = BSplineC(P,k,t)
n = size(P,2)-1;
[p_u,NodeVector] = BSpline(P,k,t);

for i = 0:n-1
    Q(:,i+1) = k/(NodeVector(i+k+1)-NodeVector(i+1))*(P(:,i+2)-P(:,i+1));
end
[v_u,NodeVector] = BSpline(Q,k-1,t);
v_u = v_u / t(end);
for i = 0:n-2
    Q2(:,i+1) = k/(NodeVector(i+k+1)-NodeVector(i+1))*(Q(:,i+2)-Q(:,i+1));
end
[a_u,NodeVector] = BSpline(Q2,k-2,t);
a_u = a_u / t(end);
for i = 0:n-3
    Q3(:,i+1) = k/(NodeVector(i+k+1)-NodeVector(i+1))*(Q2(:,i+2)-Q2(:,i+1));
end
j_u = BSpline(Q3,k-3,t);
j_u = j_u / t(end);
end