function [tau, u2]=attitudePdControl(R_d, Omega_d, D_Omega_d, R_c, Omega_c, I, kp, kd)
    % author: zrt
    % last edit by: zrt

    % [tau, u2]=attitudePdControl(R_d, Omega_d, D_Omega_d, R_c, Omega_c, I, kp, kd)
    % calculate the total moment and moment control value
    % to send to the motor controller

    % R_d: a 3x3 matrix tells the desired atitude 
    % Omega_d: a 3x1 vector tells the desired angular velocity
    % D_Omega_d: a 3x1 vector tells the derivative of the Omega_d
    % R_c: a 3x3 matrix tells the current atitude
    % Omega_c: a 3x1 vector tells the current angular velocity
    % I: the inertia matrix of the airframe, expressed in the BODY FIXED frame
    % kp, kd: controller parameters

    % error terms:
    R_e = R_d.'*R_c;
    Omega_e = Omega_c - Omega_d;

    % moment shift u2
    PR = 0.5*(R_e-R_e.');
    u2 = -kp*vex(PR) - kd*Omega_e;

    % total moment
    tau_d = I*D_Omega_d + xev(Omega_d)*I*Omega_c;
    tau = tau_d + u2;

end
