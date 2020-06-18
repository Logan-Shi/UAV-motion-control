clear;
clc
BEGIN_ACADO;

acadoSet('problemname', 'SuspendedPayload2');

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
DifferentialState x17;
DifferentialState x18;
DifferentialState x19;
DifferentialState x20;
DifferentialState x21;
DifferentialState x22;
DifferentialState x23;
DifferentialState x24;
DifferentialState x25;
DifferentialState x26;


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
r_rot = 0.014; %m
m_rot = 0.025; %kg

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
c6 = 2*R*J*Tf/K_T^2;
c7 = R*J^2/K_T^2;
% c8 = J/K_T*(2*R*Df/K_T+K_E);
% c9 = 2*R*J*kt/K_T^2;

omega_h = 912.109; %rad/s

xf=4;
yf=5;
zf=6;

tf=20;



mp=0.2;
L=1;
den1=m*mp*L^2+Ix*(mp+m);
den2=m*mp*L^2+Iy*(mp+m);


% Differential Equation
f = acado.DifferentialEquation(); % Set the differential equation object
% f.linkMatlabODE('quadcopter_ode'); % Link to a Matlab ODE
% f.add(dot(s) == v);

    f.add(dot(x1)==x12);
    f.add(dot(x2)==x13);
    f.add(dot(x3)==x14);
    f.add(dot(x4)==x15);
    f.add(dot(x5)==x16);
    f.add(dot(x6)==x17);
    f.add(dot(x7)==x18);
    f.add(dot(x8)==x19);
    f.add(dot(x9)==x20);
    f.add(dot(x10)==x21);
    f.add(dot(x11)==x22);
    
    f.add(dot(x12)==-g*L^2*mp^2/den1*x10+L*mp/den1*x24);
    f.add(dot(x13)==g*L^2*mp^2/den2*x11-L*mp/den1*x25);
    f.add(dot(x14)==x23/(m+mp));
    f.add(dot(x15)==x24/Ix);
    f.add(dot(x16)==x25/Iy);
    f.add(dot(x17)==x26/Iz);
    f.add(dot(x18)==-g*L^2*mp/den2*x10-L*m/den1*x24);
    f.add(dot(x19)==-g*L^2*mp/den1*x11+L*m/den2*x25);
    f.add(dot(x20)==x23/(m+mp));
    f.add(dot(x21)==g*L^2*mp*(mp+m)/den1*x10+(m+mp)*x24/den1);
    f.add(dot(x22)==g*L^2*mp*(mp+m)/den2*x11+(m+mp)*x25/den2);
    
    f.add(dot(x23)==u1);
    f.add(dot(x24)==u2);
    f.add(dot(x25)==u3);
    f.add(dot(x26)==u4);

    
    


%%%% HERE COMES THE OPTIMIZATION PROBLEM %%%%
ocp = acado.OCP(0, tf,20);

ocp.minimizeLagrangeTerm((x10^2+x11^2)*L^2+x7^2+x8^2+x9^2);

ocp.subjectTo(f);
ocp.subjectTo('AT_START', x1 == 0.0);
ocp.subjectTo('AT_START', x2 == 0.0);
ocp.subjectTo('AT_START', x3 == 2.0);
ocp.subjectTo('AT_START', x4 == 0.0);
ocp.subjectTo('AT_START', x5 == 0.0);
ocp.subjectTo('AT_START', x6 == 0.0);
ocp.subjectTo('AT_START', x7 == 0.0);
ocp.subjectTo('AT_START', x8 == 0.0);
ocp.subjectTo('AT_START', x9 == 1.0);
ocp.subjectTo('AT_START', x10 == 0.0);
ocp.subjectTo('AT_START', x11 == 0.0);
ocp.subjectTo('AT_START', x12 == 0.0);
ocp.subjectTo('AT_START', x13 == 0.0);
ocp.subjectTo('AT_START', x14 == 0.0);
ocp.subjectTo('AT_START', x15 == 0.0);
ocp.subjectTo('AT_START', x16 == 0.0);
ocp.subjectTo('AT_START', x17 == 0.0);
ocp.subjectTo('AT_START', x18 == 0.0);
ocp.subjectTo('AT_START', x19 == 0.0);
ocp.subjectTo('AT_START', x20 == 0.0);
ocp.subjectTo('AT_START', x21 == 0.0);
ocp.subjectTo('AT_START', x22 == 0.0);

ocp.subjectTo('AT_START', x23 == (m+mp)*g);
ocp.subjectTo('AT_START', x24 == 0);
ocp.subjectTo('AT_START', x25 == 0);
ocp.subjectTo('AT_START', x26 == 0);

ocp.subjectTo('AT_END', x1 == xf);
ocp.subjectTo('AT_END', x2 == yf);
ocp.subjectTo('AT_END', x3 == zf);
ocp.subjectTo('AT_END', x4 == 0.0);
ocp.subjectTo('AT_END', x5 == 0.0);
ocp.subjectTo('AT_END', x6 == pi/4);
ocp.subjectTo('AT_END', x7 == xf);
ocp.subjectTo('AT_END', x8 == yf);
ocp.subjectTo('AT_END', x9 == zf-1);
ocp.subjectTo('AT_END', x10 == 0.0);
ocp.subjectTo('AT_END', x11 == 0.0);
ocp.subjectTo('AT_END', x12 == 0.0);
ocp.subjectTo('AT_END', x13 == 0.0);
ocp.subjectTo('AT_END', x14 == 0.0);
ocp.subjectTo('AT_END', x15 == 0.0);
ocp.subjectTo('AT_END', x16 == 0.0);
ocp.subjectTo('AT_END', x17 == 0.0);
ocp.subjectTo('AT_END', x18 == 0.0);
ocp.subjectTo('AT_END', x19 == 0.0);
ocp.subjectTo('AT_END', x20 == 0.0);
ocp.subjectTo('AT_END', x21 == 0.0);
ocp.subjectTo('AT_END', x22 == 0.0);

ocp.subjectTo('AT_END', x23 == (m+mp)*g);
ocp.subjectTo('AT_END', x24 == 0);
ocp.subjectTo('AT_END', x25 == 0);
ocp.subjectTo('AT_END', x26 == 0);


% ocp.subjectTo( -100 <= x23 <= 100 );
% ocp.subjectTo( -100 <= x24 <= 100 );
% ocp.subjectTo( -100 <= x25 <= 100 );
% ocp.subjectTo( -100 <= x26 <= 100 );

% ocp.subjectTo( 0.01 <= x13 & x13 <= omega_max);
% ocp.subjectTo( 0.01 <= x14 & x14 <= omega_max);
% ocp.subjectTo( 0.01 <= x15 & x15 <= omega_max);
% ocp.subjectTo( 0.01 <= x16 & x16 <= omega_max);

algo =acado.OptimizationAlgorithm(ocp);
algo.set( 'KKT_TOLERANCE',1e-3);
algo.set('MAX_NUM_ITERATIONS',20);
END_ACADO; % End with "END ACADO" to compile.

out = SuspendedPayload2_RUN(); % Run the test.