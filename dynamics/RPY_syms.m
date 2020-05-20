clear all
clc
% RPY
% Roll phi
% Pitch theta
% Yaw psi
% Rotation Matrix: R=Rz(psi)*Ry(theta)*Rx(phi)
syms phi theta psi;
R=[cos(psi)*cos(theta),cos(psi)*sin(phi)*sin(theta)-cos(phi)*sin(psi),sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);
   cos(theta)*sin(psi),cos(psi)*cos(phi)+sin(phi)*sin(psi)*sin(theta),cos(phi)*sin(psi)*sin(theta)-cos(psi)*sin(phi);
   -sin(theta),cos(theta)*sin(phi),cos(phi)*cos(theta)];
% simplify(det(R));
JA=[cos(psi)*cos(theta),-sin(psi),0;sin(psi)*cos(theta),cos(psi),0;-sin(theta),0,1];
JB=[cos(psi)*cos(theta),-sin(psi),0;cos(theta)*sin(psi),cos(psi),0;-sin(theta),0,1];