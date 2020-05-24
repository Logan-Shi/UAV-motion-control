function ub = calc_vel_ub(Track_err_max,t,Vsq_max,v_u,a_u)
    N = length(t);
    ub = zeros(1,N);
    Ts = t(2) - t(1);
    for i = 2:N-1
        rho = norm(v_u(:,i))^3/norm(cross(v_u(:,i),a_u(:,i)));
        ub(i) = min(Vsq_max,8*rho*Track_err_max/Ts/Ts);
    end
end