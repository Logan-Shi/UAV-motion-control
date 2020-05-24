function ub = calc_vel_ub(Track_err_max,t,Vsq_max,v_u,a_u)
    N = length(t)-2;
    ub = zeros(1,N);
    Ts = t(2) - t(1);
    for i = 1:N
        rho = norm(v_u(:,i+1))^3/norm(cross(v_u(:,i+1),a_u(:,i+1)));
        ub(i) = min(Vsq_max,8*rho*Track_err_max/Ts/Ts);
    end
end