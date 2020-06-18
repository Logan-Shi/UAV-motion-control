function q2=CubicQuatInterp(ts, Rs, newt, omega0, omega_n)
    if nargin <=4
        omega_n=[0,0,0]';
        if nargin == 3
            omega0 = [0,0,0]';
        end
    end
    n = length(ts);
    qs = zeros(4,n);
    for k = 1:n
        qs(:,k) = R2q(Rs(:,:,k));
    end

    q_dot_0 = omega2qdot(qs(:,1),omega0);
    q_dot_n = omega2qdot(qs(:,n),omega_n);

    q2 = zeros(4,length(newt));
    for k = 1:4
        q2(k,:) = clamped_cubic_spline(ts,qs(k,:),q_dot_0(k),q_dot_n(k),newt);
    end
    
end