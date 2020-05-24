function [y_u,yd_u,r_u,p_u] = OriInter(R,k,t)

[r,p,y] = R2rpy(R);

[y_u,yd_u] = BSplineC(y,k,t,1,0);
r_u = BSpline(r,k,t);
p_u = BSpline(p,k,t);

%     for i = 1:size(p_u,2)
%         R_u(:,:,i) = rotZ(p_u(3,i))*rotY(p_u(2,i))*rotX(p_u(1,i));
%         Rd_u(:,:,i) = rotZ(v_u(3,i))*rotY(v_u(2,i))*rotX(v_u(1,i));
%     end
end