function [u1,u2]=control3(R_desired,omega_desired,z_desired,z_desired_dot,R,omega,z,z_dot,z_dot2,I,m,g,K_R,K_omega,K_z,K_zdot)
    e_R=1/2*vex(transpose(R_desired)*R-transpose(R)*R_desired);
    e_omega=omega-transpose(R)*R_desired*omega_desired;
    u2=-K_R*e_R'-K_omega*e_omega+cross(omega,I*omega);
    
    % z的控制方程不确定是否正确
    e_z=z-z_desired;
    e_zdot=z_dot-z_desired_dot;
%     u1=dot([0,0,(-K_z*e_z-K_zdot*e_zdot+m*(g+z_dot2))],R(:,3));
    u1=-K_z*e_z-K_zdot*e_zdot+m*(g+z_dot2);
end