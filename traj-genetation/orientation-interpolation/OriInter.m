function [R_u,Rd_u] = OriInter(R,k,t)
    [r,p,y] = R2rpy(R);

    [p_u,v_u] = BSplineC([r;p;y],k,t);

    for i = 1:size(p_u,2)
        R_u(:,:,i) = rotZ(p_u(3,i))*rotY(p_u(2,i))*rotX(p_u(1,i));
        Rd_u(:,:,i) = rotZ(v_u(3,i))*rotY(v_u(2,i))*rotX(v_u(1,i));
    end
end