function rotorspeeds=controller_book(ppath, t, quad_state, quad_params, con_params)
    % controller for quadrotor, output the rotor speed
    % input:
    % ppath: planned path(output by path planner)
    % t: current time
    % quad_state: current quadrotor state;
    % quad_params: quadrotor parameters
    % con_params: controller parameters (contains Kd, Kp, etc.)

    % output:
    % rotorspeed: speed of the rotor()

    % four inputs
    zeta_p = ppath.position(t);
    D_zeta_p = ppath.velocity(t);
    DD_zeta_p = ppath.acceleration(t);
    psy_p = ppath.yaw(t);
    
    % and the angular velocity
    omega_p = ppath.omega(t);
    d_omega_p = ppath.omega_dot(t);

    zeta_c = quad_state.position;
    D_zeta_c = quad_state.velocity;
    % DD_zeta_c = quad_state.acceleration;
    R_c = quad_state.attitude;
    omega_c = quad_state.omega;
    % d_omega_c = quad_state.omega_dot;

    m = quad_params.m;
    I = quad_params.I;
    k = quad_params.k;
    L = quad_params.L;
    b = quad_params.b;

    trajKd = con_params.tKd; trajKp = con_params.tKp;
    [R_d, u1] = trajTrackControl(zeta_p, D_zeta_p, DD_zeta_p, psy_p, zeta_c, D_zeta_c, R_c, trajKd, trajKp, m);

    atKd = con_params.aKd; atKp = con_params.aKp;
    tau = attitudePdControl(R_d, omega_p, d_omega_p, R_c, omega_c, I, atKp, atKd);

    rotorspeeds = get_rotorspeed(u1, tau, k, L, b);

end

% function params=load_params()
%     % 机体参数
%     params.m = 1;
%     params.g = 9.8;
%     params.k = 1;
%     params.kd = 0;
%     params.I = diag([0.04 0.04 0.008]);
%     params.L = 0.25;
%     params.b = 0.007;
% end
