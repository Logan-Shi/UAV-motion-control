function [R_d, u1]=trajTrackControl(zeta_d, D_zeta_d, DD_zeta_d, psy_p, zeta_c, D_zeta_c, R_c, K_d, K_p, m)
    % author: zrt
    % last edited by: zrt

    % track the PLANNED trajectory
    % the path planning alg will specify:
    %   1. the trajectory of the centroid over time.
    %   2. the yaw angle 
    %   3. the full anguler velocity

    % input:
    %  zeta_d: desired trajectory of the quadrotor centroid
    %  D_zeta_d, DD_zeta_d
    %  psy_p: PLANNED yaw angle
    %  zeta_c: current position on the trajectory
    %  D_zeta_c, DD_zeta_c
    %  R_c: current atitude

    %  K_d, K_p: PD params
    %  m: mass of the quadrotor
    
    % output:
    %  R_d: desired atitude, used as input for atitude control
    %  u1: the heave(total thurst) T, used as input for motor control
    g = 9.8;
    euler_ZXY_c = rotMat2zxyEuler(R_c);
    psy_c = euler_ZXY_c(1);

    phi_d = 1/g*(DD_zeta_d(1)*sin(psy_c)-DD_zeta_d(2)*cos(psy_c)); % desired x euler angle
    theta_d = 1/g*(DD_zeta_d(1)*cos(psy_c)+DD_zeta_d*sin(psy_c)); % desired y euler angle
    psy_d = psy_p; % desired z euler angle

    R_d = zxyEuler2rotMat([psy_d,phi_d,theta_d]);

    % b3 is the direction of the z-axis of the quadrotor, used for non-linear control
    b3 = R_c(:,3);

    u1 = m*b3.'*(DD_zeta_d + K_d*(D_zeta_d - D_zeta_c) + K_p*(zeta_d - zeta_c) + g*[0,0,1].');

end
