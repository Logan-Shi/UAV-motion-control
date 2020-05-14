function [R, omega, omega_dot]=calc_inputs(ppath, t)
    g = 9.8;
    p = ppath.position(t); v = ppath.velocity(t);
    a = ppath.acceleration(t); J = ppath.jerk(t); s = ppath.snap(t);
    psy = ppath.yaw(t); psy_dot = ppath.yaw_dot(t); psy_dot = ppath.yaw_ddot(t);

    d1 = [cos(psy), sin(psy), 0].';
    b3 = (a - g*[0,0,1].'); b3 = b3/tempNorm(b3);
    b2 = xev(b3)*d1; b2 = b2/tempNorm(b2);
    b1 = xev(b2)*b3;
    R = [b1,b2,b3];
    R_dot = derivative(R);

    omega = vex(R.'*R_dot);

    omega_dot = derivative(omega);

end

% all inputs

% % four inputs
% zeta_p = ppath.position(t);
% D_zeta_p = ppath.velocity(t);
% DD_zeta_p = ppath.acceleration(t);
% psy_p = ppath.yaw(t);

% % and the angular velocity
% omega_p = ppath.omega(t);
% d_omega_p = ppath.omega_dot(t);