function [P,P2] = on_way_pts(P,k)
    n = size(P,2)-1;
    NodeVector = U_quasi_uniform(n,k); % 准均匀B样条的节点矢量
    
    P2 = P;
    n = size(P,2)-1;
    u = linspace(0,1,n+1);
    phi = zeros(n+1);
    for i = 0:n
        for j = 0:n
            phi(i+1,j+1) = BaseFunction(j,k,u(i+1),NodeVector);
        end
    end
    phi(end,end) = 1;
    
    P = (transpose(phi)*phi)\(transpose(phi)*transpose(P));
    P = transpose(P);

    p_tmp = BSpline(P,k,linspace(0,1,n+1));
    disp(['off way points by: ' num2str(norm(p_tmp-P2))]);
end