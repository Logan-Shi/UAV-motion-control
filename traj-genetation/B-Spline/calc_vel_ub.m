function ub = calc_vel_ub(Track_err_max,u,Vsq_max,v_u,a_u)
    N = length(u)-2;
    ub = zeros(1,N);
    den = u(2) - u(1);
    for i = 1:N
        rho = norm(v_u(:,i+1))^3/norm(cross(v_u(:,i+1),a_u(:,i+1)));
        ub(i) = min(Vsq_max,8*rho*Track_err_max/den/den);
    end
end