function [L,U] = BSplineC(P,k,t,capibility,OnPts,Graph,u,udot)
n = size(P,2)-1;
NodeVector = U_quasi_uniform(n,k); % ׼����B�����Ľڵ�ʸ��
% ����way points�����Ƶ�����
if OnPts
    [P,P2] = on_way_pts(P,k,NodeVector);
    p_tmp = BSpline(P,k,linspace(0,1,n+1));
    disp(['off way points by: ' num2str(norm(p_tmp-P2))]);
else
    P2 = P;
end

% ��ʸ����
u = linspace(0,1,length(t));
[p_u,v_u,a_u,j_u] = BSplineDrv(P,n,k,u);

[L,U] = feasibiltyCheck(u,udot,v_u,a_u,capibility);
p_t = 0;
v_t = 0;
a_t = 0;
j_t = 0;
end