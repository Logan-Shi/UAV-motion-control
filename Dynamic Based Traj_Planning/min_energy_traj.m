function out=min_energy_traj(ts,te,rs,re,psi_s,psi_e)
BEGIN_ACADO;

acadoSet('problemname', 'quadcopter');

DifferentialState x1;
DifferentialState x2;
DifferentialState x3;
DifferentialState x4;
DifferentialState x5;
DifferentialState x6;
DifferentialState x7;
DifferentialState x8;
DifferentialState x9;
DifferentialState x10;
DifferentialState x11;
DifferentialState x12;
DifferentialState x13;
DifferentialState x14;
DifferentialState x15;
DifferentialState x16;

Control u1;
Control u2;
Control u3;
Control u4;

K_V = 920; %rpm/V
nB = 2;
rho = 1.225; %kg/m^3
mB = 0.0055; %kg
m = 1.3; %kg
Tf = 4e-2; %N*m
r = 0.12; %m
l = 0.175; %m
Df = 2e-4; %N*m*s/rad
epsilon = 0.004; %m
Ix = 0.081; %kg*m^2
Iy = 0.081; %kg*m^2
Iz = 0.142; %kg*m^2
R = 0.2; % Ohm
C_T = 0.0048;
Jm = 4.9e-6; %kg*m^2
C_Q = 2.3515e-4;
omega_max = 1047.197; %rad/s
% r_rot = 0.014; %m
% m_rot = 0.025; %kg

%Calculated constants
K_E = 9.5493/K_V; %V*s/rad
K_T = K_E; %per page 3.
J_L = 1/4*nB*mB*(r-epsilon)^2;
A = pi*r^2;
kb = C_T*rho*A*r^2;
kt = C_Q*rho*A*r^3;

J = Jm + J_L;  % total moment of inertia
g = 9.8066;

c1 = R*Tf^2/K_T^2;
c2 = Tf/K_T*(2*R*Df/K_T+K_E);
c3 = Df/K_T*(R*Df/K_T+K_E)+2*R*Tf*kt/K_T^2;
c4 = kt/K_T*(2*R*Df/K_T+K_E);
c5 = R*kt^2/K_T^2;
% c6 = 2*R*J*Tf/K_T^2;
c7 = R*J^2/K_T^2;
% c8 = J/K_T*(2*R*Df/K_T+K_E);
% c9 = 2*R*J*kt/K_T^2;

omega_h = 912.109; %rad/s



% Differential Equation
f = acado.DifferentialEquation(); % Set the differential equation object
% f.linkMatlabODE('quadcopter_ode'); % Link to a Matlab ODE
% f.add(dot(s) == v);
    f.add(dot(x1)==x2);
    f.add(dot(x2)==kb/m*(x7*sin(x11)+cos(x11)*x9)*(x13^2+x14^2+x15^2+x16^2));
    f.add(dot(x3)==x4);
    f.add(dot(x4)==kb/m*(x9*sin(x11)-cos(x11)*x7)*(x13^2+x14^2+x15^2+x16^2));
    f.add(dot(x5)==x6);
    f.add(dot(x6)==kb/m*(x13^2+x14^2+x15^2+x16^2)-g);
    f.add(dot(x7)==x8);
    f.add(dot(x8)==(Iy-Iz)/Ix*x10*x12+l*kb/Ix*(x14^2-x16^2)-J/Ix*x10*(x13-x14+x15-x16));
    f.add(dot(x9)==x10);
    f.add(dot(x10)==(Iz-Ix)/Iy*x8*x12+l*kb/Iy*(x15^2-x13^2)+J/Iy*x8*(x13-x14+x15-x16));
    f.add(dot(x11)==x12);
    f.add(dot(x12)==(Ix-Iy)/Iz*x8*x10+kt/Iz*(x13^2-x14^2+x15^2-x16^2));
    f.add(dot(x13)==u1);
    f.add(dot(x14)==u2);
    f.add(dot(x15)==u3);
    f.add(dot(x16)==u4);


%%%% HERE COMES THE OPTIMIZATION PROBLEM %%%%
ocp = acado.OCP(ts,te,101);

% Energy=c1+c2*x13+c3*x13^2+c4*x13^3+c5*x13^4 ...
%       +c1+c2*x14+c3*x14^2+c4*x14^3+c5*x14^4 ...
%       +c1+c2*x15+c3*x15^2+c4*x15^3+c5*x15^4 ...
%       +c1+c2*x16+c3*x16^2+c4*x16^3+c5*x16^4 ...
%       +c7*(u1^2+u2^2+u3^2+u4^2);

ocp.minimizeLagrangeTerm(c1+c2*x13+c3*x13^2+c4*x13^3+c5*x13^4 ...
      +c1+c2*x14+c3*x14^2+c4*x14^3+c5*x14^4 ...
      +c1+c2*x15+c3*x15^2+c4*x15^3+c5*x15^4 ...
      +c1+c2*x16+c3*x16^2+c4*x16^3+c5*x16^4 ...
      +c7*(u1^2+u2^2+u3^2+u4^2));

ocp.subjectTo(f);
ocp.subjectTo('AT_START', x1 == rs(1));
ocp.subjectTo('AT_START', x2 == 0.0);
ocp.subjectTo('AT_START', x3 == rs(2));
ocp.subjectTo('AT_START', x4 == 0.0);
ocp.subjectTo('AT_START', x5 == rs(3));
ocp.subjectTo('AT_START', x6 == 0.0);
ocp.subjectTo('AT_START', x7 == 0.0);
ocp.subjectTo('AT_START', x8 == 0.0);
ocp.subjectTo('AT_START', x9 == 0.0);
ocp.subjectTo('AT_START', x10 == 0.0);
ocp.subjectTo('AT_START', x11 == psi_s);
ocp.subjectTo('AT_START', x12 == 0.0);
ocp.subjectTo('AT_START', x13 == omega_h);
ocp.subjectTo('AT_START', x14 == omega_h);
ocp.subjectTo('AT_START', x15 == omega_h);
ocp.subjectTo('AT_START', x16 == omega_h);

ocp.subjectTo('AT_END', x1 == re(1));
ocp.subjectTo('AT_END', x2 == 0.0);
ocp.subjectTo('AT_END', x3 == re(2));
ocp.subjectTo('AT_END', x4 == 0.0);
ocp.subjectTo('AT_END', x5 == re(3));
ocp.subjectTo('AT_END', x6 == 0.0);
ocp.subjectTo('AT_END', x7 == 0.0);
ocp.subjectTo('AT_END', x8 == 0.0);
ocp.subjectTo('AT_END', x9 == 0.0);
ocp.subjectTo('AT_END', x10 == 0.0);
ocp.subjectTo('AT_END', x11 == psi_e);
ocp.subjectTo('AT_END', x12 == 0.0);
ocp.subjectTo('AT_END', x13 == omega_h);
ocp.subjectTo('AT_END', x14 == omega_h);
ocp.subjectTo('AT_END', x15 == omega_h);
ocp.subjectTo('AT_END', x16 == omega_h);

ocp.subjectTo( 0.01 <= x13 <= omega_max );
ocp.subjectTo( 0.01 <= x14 <= omega_max );
ocp.subjectTo( 0.01 <= x15 <= omega_max );
ocp.subjectTo( 0.01 <= x16 <= omega_max );

% ocp.subjectTo( 0.01 <= x13 & x13 <= omega_max);
% ocp.subjectTo( 0.01 <= x14 & x14 <= omega_max);
% ocp.subjectTo( 0.01 <= x15 & x15 <= omega_max);
% ocp.subjectTo( 0.01 <= x16 & x16 <= omega_max);

algo =acado.OptimizationAlgorithm(ocp);
algo.set( 'KKT_TOLERANCE',1e-5);
algo.set('MAX_NUM_ITERATIONS',40);
END_ACADO; % End with "END ACADO" to compile.

out = quadcopter_RUN(); % Run the test.

end