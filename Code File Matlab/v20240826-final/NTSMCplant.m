function [sys,x0,str,ts]=NTSMCplant(t,x,u,flag)

switch flag,
case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
case 1,
    sys=mdlDerivatives(t,x,u);
case 3,
    sys=mdlOutputs(t,x,u);
case {2, 4, 9 }
    sys = [];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

%-----------------------------------Initialization-----------------------------------%
function [sys,x0,str,ts]=mdlInitializeSizes
global m1 m2 m3 l1 l2 l3 g tol

sizes = simsizes;
sizes.NumContStates  = 6; % q1, dq1, q2, dq2, q3, dq3
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 6; % q1, dq1, q2, dq2, q3, dq3
sizes.NumInputs      = 7; % u1, u2, u3, e1, e2, e3, en_norm
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 0;
sys=simsizes(sizes);
x0=[0 0 0 0 0 0];
str=[];
ts=[];

%-------------------------------------Derivative-------------------------------------%
function sys=mdlDerivatives(t,x,u)
global m1 m2 m3 l1 l2 l3 g tol

% Robot's Model Parameter
m1 = 0.0865; m2 = m1; m3 = m1;             % kg
l1 = 0.0975; l2 = 0.14665; l3 = 0.18997;   % m
g = 9.81;                                  % m/s^2
tol = 1e-9;

% State variables
q1 = x(1); dq1 = x(2);
q2 = x(3); dq2 = x(4);
q3 = x(5); dq3 = x(6);

q = [q1; q2; q3]; 
dq = [dq1; dq2; dq3]; 

% Inertia Matrix I
Ic1 = [1243271.12*tol,   116023.89*tol,   118509.08*tol;
        116023.89*tol,   669264.09*tol,   559757.74*tol;
        118509.08*tol,   559757.74*tol,   634170.06*tol];

Ic2 = [2302070.69*tol,   199555.95*tol,    93639.68*tol;
        199555.95*tol,   450604.70*tol,   869745.15*tol;
         93639.68*tol,   869745.15*tol,  1926391.30*tol];

Ic3 = [8034547.48*tol,    382202.33*tol,     99611.10*tol;
        382202.33*tol,    548948.42*tol,   1948062.81*tol;
         99611.10*tol,   1948062.81*tol,  7556521.21*tol];

Ia11 = Ic1(1,1);  Ia12 = Ic1(1,2);  Ia13 = Ic1(1,3);
Ia21 = Ic1(2,1);  Ia22 = Ic1(2,2);  Ia23 = Ic1(2,3);
Ia31 = Ic1(3,1);  Ia32 = Ic1(3,2);  Ia33 = Ic1(3,3);

Ib11 = Ic2(1,1);  Ib12 = Ic2(1,2);  Ib13 = Ic2(1,3);
Ib21 = Ic2(2,1);  Ib22 = Ic2(2,2);  Ib23 = Ic2(2,3);
Ib31 = Ic2(3,1);  Ib32 = Ic2(3,2);  Ib33 = Ic2(3,3);

Ic11 = Ic3(1,1);  Ic12 = Ic3(1,2);  Ic13 = Ic3(1,3);
Ic21 = Ic3(2,1);  Ic22 = Ic3(2,2);  Ic23 = Ic3(2,3);
Ic31 = Ic3(3,1);  Ic32 = Ic3(3,2);  Ic33 = Ic3(3,3);

% Mass Matrix M(q)
M11 = Ia22 + cos(x(3) + x(5))*(Ic22*cos(x(3) + x(5)) + Ic12*sin(x(3) + x(5))) + sin(x(3) + x(5))*(Ic21*cos(x(3) + x(5)) + Ic11*sin(x(3) + x(5))) + m3*(l3*cos(x(3) + x(5)) + l2*cos(x(3)))^2 + cos(x(3))*(Ib22*cos(x(3)) + Ib12*sin(x(3))) + sin(x(3))*(Ib21*cos(x(3)) + Ib11*sin(x(3))) - l2^2*m2*(sin(x(3))^2 - 1);
M12 = Ic23*cos(x(3) + x(5)) + Ic13*sin(x(3) + x(5)) + Ib23*cos(x(3)) + Ib13*sin(x(3));
M13 = Ic23*cos(x(3) + x(5)) + Ic13*sin(x(3) + x(5));
M21 = Ic32*cos(x(3) + x(5)) + Ic31*sin(x(3) + x(5)) + Ib32*cos(x(3)) + Ib31*sin(x(3));
M22 = Ib33 + Ic33 + l2^2*m2 + l2^2*m3 + l3^2*m3 + 2*l2*l3*m3*cos(x(5));
M23 = m3*l3^2 + l2*m3*cos(x(5))*l3 + Ic33;
M31 = Ic32*cos(x(3) + x(5)) + Ic31*sin(x(3) + x(5));
M32 = m3*l3^2 + l2*m3*cos(x(5))*l3 + Ic33;
M33 = m3*l3^2 + Ic33;

M = [M11 M12 M13; 
     M21 M22 M23; 
     M31 M32 M33];

% Centrifugal and Coriolis Matrix C(q,dq)
C11 = (Ib12*x(4)*cos(2*x(3)))/2 + (Ib21*x(4)*cos(2*x(3)))/2 + (Ib11*x(4)*sin(2*x(3)))/2 - (Ib22*x(4)*sin(2*x(3)))/2 + (Ic12*x(4)*cos(2*x(3) + 2*x(5)))/2 + (Ic12*x(6)*cos(2*x(3) + 2*x(5)))/2 + (Ic21*x(4)*cos(2*x(3) + 2*x(5)))/2 + (Ic21*x(6)*cos(2*x(3) + 2*x(5)))/2 + (Ic11*x(4)*sin(2*x(3) + 2*x(5)))/2 + (Ic11*x(6)*sin(2*x(3) + 2*x(5)))/2 - (Ic22*x(4)*sin(2*x(3) + 2*x(5)))/2 - (Ic22*x(6)*sin(2*x(3) + 2*x(5)))/2 - (x(4)*l2^2*m2*sin(2*x(3)))/2 - (x(4)*l2^2*m3*sin(2*x(3)))/2 - (x(4)*l3^2*m3*sin(2*x(3) + 2*x(5)))/2 - (x(6)*l3^2*m3*sin(2*x(3) + 2*x(5)))/2 - x(4)*l2*l3*m3*sin(2*x(3) + x(5)) - (x(6)*l2*l3*m3*sin(2*x(3) + x(5)))/2 - (x(6)*l2*l3*m3*sin(x(5)))/2;
C12 = (Ib12*x(2)*cos(2*x(3)))/2 + (Ib21*x(2)*cos(2*x(3)))/2 + (Ib11*x(2)*sin(2*x(3)))/2 - (Ib22*x(2)*sin(2*x(3)))/2 + (Ic12*x(2)*cos(2*x(3) + 2*x(5)))/2 + (Ic21*x(2)*cos(2*x(3) + 2*x(5)))/2 + (Ic11*x(2)*sin(2*x(3) + 2*x(5)))/2 - (Ic22*x(2)*sin(2*x(3) + 2*x(5)))/2 + Ic13*x(4)*cos(x(3) + x(5)) + Ic13*x(6)*cos(x(3) + x(5)) - Ic23*x(4)*sin(x(3) + x(5)) - Ic23*x(6)*sin(x(3) + x(5)) + Ib13*x(4)*cos(x(3)) - Ib23*x(4)*sin(x(3)) - (x(2)*l2^2*m2*sin(2*x(3)))/2 - (x(2)*l2^2*m3*sin(2*x(3)))/2 - (x(2)*l3^2*m3*sin(2*x(3) + 2*x(5)))/2 - x(2)*l2*l3*m3*sin(2*x(3) + x(5));
C13 = (Ic12*x(2)*cos(2*x(3) + 2*x(5)))/2 + (Ic21*x(2)*cos(2*x(3) + 2*x(5)))/2 + (Ic11*x(2)*sin(2*x(3) + 2*x(5)))/2 - (Ic22*x(2)*sin(2*x(3) + 2*x(5)))/2 + Ic13*x(4)*cos(x(3) + x(5)) + Ic13*x(6)*cos(x(3) + x(5)) - Ic23*x(4)*sin(x(3) + x(5)) - Ic23*x(6)*sin(x(3) + x(5)) - (x(2)*l3^2*m3*sin(2*x(3) + 2*x(5)))/2 - (x(2)*l2*l3*m3*sin(2*x(3) + x(5)))/2 - (x(2)*l2*l3*m3*sin(x(5)))/2;
C21 = -x(2)*((Ic12*cos(2*x(3) + 2*x(5)))/2 + (Ic21*cos(2*x(3) + 2*x(5)))/2 + (Ic11*sin(2*x(3) + 2*x(5)))/2 - (Ic22*sin(2*x(3) + 2*x(5)))/2 + (Ib12*cos(2*x(3)))/2 + (Ib21*cos(2*x(3)))/2 + (Ib11*sin(2*x(3)))/2 - (Ib22*sin(2*x(3)))/2 - (l3^2*m3*sin(2*x(3) + 2*x(5)))/2 - (l2^2*m2*sin(2*x(3)))/2 - (l2^2*m3*sin(2*x(3)))/2 - l2*l3*m3*sin(2*x(3) + x(5)));
C22 = - x(2)*((Ic13*cos(x(3) + x(5)))/2 - (Ic31*cos(x(3) + x(5)))/2 - (Ic23*sin(x(3) + x(5)))/2 + (Ic32*sin(x(3) + x(5)))/2 + (Ib13*cos(x(3)))/2 - (Ib31*cos(x(3)))/2 - (Ib23*sin(x(3)))/2 + (Ib32*sin(x(3)))/2) - x(6)*l2*l3*m3*sin(x(5));
C23 = - x(2)*((Ic13*cos(x(3) + x(5)))/2 - (Ic31*cos(x(3) + x(5)))/2 - (Ic23*sin(x(3) + x(5)))/2 + (Ic32*sin(x(3) + x(5)))/2) - x(4)*l2*l3*m3*sin(x(5)) - x(6)*l2*l3*m3*sin(x(5));
C31 = x(2)*((Ic22*sin(2*x(3) + 2*x(5)))/2 - (Ic21*cos(2*x(3) + 2*x(5)))/2 - (Ic11*sin(2*x(3) + 2*x(5)))/2 - (Ic12*cos(2*x(3) + 2*x(5)))/2 + (l3^2*m3*sin(2*x(3) + 2*x(5)))/2 + (l2*l3*m3*sin(x(5)))/2 + (l2*l3*m3*sin(2*x(3) + x(5)))/2);
C32 = x(4)*l2*l3*m3*sin(x(5)) - x(2)*((Ic13*cos(x(3) + x(5)))/2 - (Ic31*cos(x(3) + x(5)))/2 - (Ic23*sin(x(3) + x(5)))/2 + (Ic32*sin(x(3) + x(5)))/2);
C33 = -x(2)*((Ic13*cos(x(3) + x(5)))/2 - (Ic31*cos(x(3) + x(5)))/2 - (Ic23*sin(x(3) + x(5)))/2 + (Ic32*sin(x(3) + x(5)))/2);

C = [C11 C12 C13; 
     C21 C22 C23; 
     C31 C32 C33];

% Gravity Vector G(q)
G1 = - g*m3*cos(x(1))*(l3*sin(x(3) + x(5)) + l2*sin(x(3))) - g*l2*m2*cos(x(1))*sin(x(3));
G2 = - g*m3*sin(x(1))*(l3*sin(x(3) + x(5)) + l2*sin(x(3))) - g*l2*m2*sin(x(1))*sin(x(3));
G3 =   g*m3*(l3*cos(x(3) + x(5)) + l2*cos(x(3))) + g*l2*m2*cos(x(3));

G = [G1; 
     G2; 
     G3];

% Friction
% F = 0.01*sign(dq);        % for sine + square wave
F = 0.025*sign(dq); 
% F = 0.05*ones(3,1);               % for const wave

% External Disturbance for sine wave
% told = [0.01*sin(t);
%         0.01*sin(t);
%         0.01*sin(t)];   
told = [0.025*sin(t);
        0.025*sin(t);
        0.025*sin(t)]; 
% told = [0.01*sin(2*pi*t);
%         0.01*cos(2*pi*t);
%         0.01*sin(2*pi*t)]; 
% told = [0.005 * sin(t) + 0.003 * sin(2*t) + 0.002 * sin(0.5*t) + 0.001 * randn(1, length(t));
%         0.005 * sin(t) + 0.003 * sin(2*t) + 0.002 * sin(0.5*t) + 0.001 * randn(1, length(t));
%         0.005 * sin(t) + 0.003 * sin(2*t) + 0.002 * sin(0.5*t) + 0.001 * randn(1, length(t))];

% External Disturbance for Square Wave
% told = [0.01 * square(t);   
%         0.01 * square(t);  
%         0.01 * square(t)]; 
% told = [0.025 * square(t);   
%         0.025 * square(t);  
%         0.025 * square(t)]; 
% told = [0.05;
%         0.05;
%         0.05];            % for const wave

% Control Input for corresponding 3DOF
ut = u(1:3);       % u(1), u(2), u(3)

% Dynamics
% S = inv(M)*(ut-C-G);
% S = inv(M)*(ut-C-G-F);
% S = inv(M)*(ut-C-G-told);
S = inv(M)*(ut-C-G-F-told);

% Find 1st and 2nd derivatives
sys(1)=x(2); % dq1
sys(2)=S(1); % ddq1
sys(3)=x(4); % dq2
sys(4)=S(2); % ddq2
sys(5)=x(6); % dq3
sys(6)=S(3); % ddq3

%---------------------------------------Output---------------------------------------%
function sys=mdlOutputs(t,x,u)
global m1 m2 m3 l1 l2 l3 g tol

% Robot's Model Parameters
m1 = 0.0865; m2 = m1; m3 = m1;            % kg
l1 = 0.0975; l2 = 0.14665; l3 = 0.18997;  % m
g = 9.81;                                 % m/s^2
tol = 1e-9;

% State variables
q1 = x(1); dq1 = x(2);
q2 = x(3); dq2 = x(4);
q3 = x(5); dq3 = x(6);

q = [q1; q2; q3]; 
dq = [dq1; dq2; dq3]; 

% Inertia Matrix I
Ic1 = [1243271.12*tol,   116023.89*tol,   118509.08*tol;
        116023.89*tol,   669264.09*tol,   559757.74*tol;
        118509.08*tol,   559757.74*tol,   634170.06*tol];

Ic2 = [2302070.69*tol,   199555.95*tol,    93639.68*tol;
        199555.95*tol,   450604.70*tol,   869745.15*tol;
         93639.68*tol,   869745.15*tol,  1926391.30*tol];

Ic3 = [8034547.48*tol,    382202.33*tol,     99611.10*tol;
        382202.33*tol,    548948.42*tol,   1948062.81*tol;
         99611.10*tol,   1948062.81*tol,  7556521.21*tol];

Ia11 = Ic1(1,1);  Ia12 = Ic1(1,2);  Ia13 = Ic1(1,3);
Ia21 = Ic1(2,1);  Ia22 = Ic1(2,2);  Ia23 = Ic1(2,3);
Ia31 = Ic1(3,1);  Ia32 = Ic1(3,2);  Ia33 = Ic1(3,3);

Ib11 = Ic2(1,1);  Ib12 = Ic2(1,2);  Ib13 = Ic2(1,3);
Ib21 = Ic2(2,1);  Ib22 = Ic2(2,2);  Ib23 = Ic2(2,3);
Ib31 = Ic2(3,1);  Ib32 = Ic2(3,2);  Ib33 = Ic2(3,3);

Ic11 = Ic3(1,1);  Ic12 = Ic3(1,2);  Ic13 = Ic3(1,3);
Ic21 = Ic3(2,1);  Ic22 = Ic3(2,2);  Ic23 = Ic3(2,3);
Ic31 = Ic3(3,1);  Ic32 = Ic3(3,2);  Ic33 = Ic3(3,3);

% Mass Matrix M(q)
M11 = Ia22 + cos(x(3) + x(5))*(Ic22*cos(x(3) + x(5)) + Ic12*sin(x(3) + x(5))) + sin(x(3) + x(5))*(Ic21*cos(x(3) + x(5)) + Ic11*sin(x(3) + x(5))) + m3*(l3*cos(x(3) + x(5)) + l2*cos(x(3)))^2 + cos(x(3))*(Ib22*cos(x(3)) + Ib12*sin(x(3))) + sin(x(3))*(Ib21*cos(x(3)) + Ib11*sin(x(3))) - l2^2*m2*(sin(x(3))^2 - 1);
M12 = Ic23*cos(x(3) + x(5)) + Ic13*sin(x(3) + x(5)) + Ib23*cos(x(3)) + Ib13*sin(x(3));
M13 = Ic23*cos(x(3) + x(5)) + Ic13*sin(x(3) + x(5));
M21 = Ic32*cos(x(3) + x(5)) + Ic31*sin(x(3) + x(5)) + Ib32*cos(x(3)) + Ib31*sin(x(3));
M22 = Ib33 + Ic33 + l2^2*m2 + l2^2*m3 + l3^2*m3 + 2*l2*l3*m3*cos(x(5));
M23 = m3*l3^2 + l2*m3*cos(x(5))*l3 + Ic33;
M31 = Ic32*cos(x(3) + x(5)) + Ic31*sin(x(3) + x(5));
M32 = m3*l3^2 + l2*m3*cos(x(5))*l3 + Ic33;
M33 = m3*l3^2 + Ic33;

M = [M11 M12 M13; 
     M21 M22 M23; 
     M31 M32 M33];

% Centrifugal and Coriolis Matrix C(q,dq)
C11 = (Ib12*x(4)*cos(2*x(3)))/2 + (Ib21*x(4)*cos(2*x(3)))/2 + (Ib11*x(4)*sin(2*x(3)))/2 - (Ib22*x(4)*sin(2*x(3)))/2 + (Ic12*x(4)*cos(2*x(3) + 2*x(5)))/2 + (Ic12*x(6)*cos(2*x(3) + 2*x(5)))/2 + (Ic21*x(4)*cos(2*x(3) + 2*x(5)))/2 + (Ic21*x(6)*cos(2*x(3) + 2*x(5)))/2 + (Ic11*x(4)*sin(2*x(3) + 2*x(5)))/2 + (Ic11*x(6)*sin(2*x(3) + 2*x(5)))/2 - (Ic22*x(4)*sin(2*x(3) + 2*x(5)))/2 - (Ic22*x(6)*sin(2*x(3) + 2*x(5)))/2 - (x(4)*l2^2*m2*sin(2*x(3)))/2 - (x(4)*l2^2*m3*sin(2*x(3)))/2 - (x(4)*l3^2*m3*sin(2*x(3) + 2*x(5)))/2 - (x(6)*l3^2*m3*sin(2*x(3) + 2*x(5)))/2 - x(4)*l2*l3*m3*sin(2*x(3) + x(5)) - (x(6)*l2*l3*m3*sin(2*x(3) + x(5)))/2 - (x(6)*l2*l3*m3*sin(x(5)))/2;
C12 = (Ib12*x(2)*cos(2*x(3)))/2 + (Ib21*x(2)*cos(2*x(3)))/2 + (Ib11*x(2)*sin(2*x(3)))/2 - (Ib22*x(2)*sin(2*x(3)))/2 + (Ic12*x(2)*cos(2*x(3) + 2*x(5)))/2 + (Ic21*x(2)*cos(2*x(3) + 2*x(5)))/2 + (Ic11*x(2)*sin(2*x(3) + 2*x(5)))/2 - (Ic22*x(2)*sin(2*x(3) + 2*x(5)))/2 + Ic13*x(4)*cos(x(3) + x(5)) + Ic13*x(6)*cos(x(3) + x(5)) - Ic23*x(4)*sin(x(3) + x(5)) - Ic23*x(6)*sin(x(3) + x(5)) + Ib13*x(4)*cos(x(3)) - Ib23*x(4)*sin(x(3)) - (x(2)*l2^2*m2*sin(2*x(3)))/2 - (x(2)*l2^2*m3*sin(2*x(3)))/2 - (x(2)*l3^2*m3*sin(2*x(3) + 2*x(5)))/2 - x(2)*l2*l3*m3*sin(2*x(3) + x(5));
C13 = (Ic12*x(2)*cos(2*x(3) + 2*x(5)))/2 + (Ic21*x(2)*cos(2*x(3) + 2*x(5)))/2 + (Ic11*x(2)*sin(2*x(3) + 2*x(5)))/2 - (Ic22*x(2)*sin(2*x(3) + 2*x(5)))/2 + Ic13*x(4)*cos(x(3) + x(5)) + Ic13*x(6)*cos(x(3) + x(5)) - Ic23*x(4)*sin(x(3) + x(5)) - Ic23*x(6)*sin(x(3) + x(5)) - (x(2)*l3^2*m3*sin(2*x(3) + 2*x(5)))/2 - (x(2)*l2*l3*m3*sin(2*x(3) + x(5)))/2 - (x(2)*l2*l3*m3*sin(x(5)))/2;
C21 = -x(2)*((Ic12*cos(2*x(3) + 2*x(5)))/2 + (Ic21*cos(2*x(3) + 2*x(5)))/2 + (Ic11*sin(2*x(3) + 2*x(5)))/2 - (Ic22*sin(2*x(3) + 2*x(5)))/2 + (Ib12*cos(2*x(3)))/2 + (Ib21*cos(2*x(3)))/2 + (Ib11*sin(2*x(3)))/2 - (Ib22*sin(2*x(3)))/2 - (l3^2*m3*sin(2*x(3) + 2*x(5)))/2 - (l2^2*m2*sin(2*x(3)))/2 - (l2^2*m3*sin(2*x(3)))/2 - l2*l3*m3*sin(2*x(3) + x(5)));
C22 = - x(2)*((Ic13*cos(x(3) + x(5)))/2 - (Ic31*cos(x(3) + x(5)))/2 - (Ic23*sin(x(3) + x(5)))/2 + (Ic32*sin(x(3) + x(5)))/2 + (Ib13*cos(x(3)))/2 - (Ib31*cos(x(3)))/2 - (Ib23*sin(x(3)))/2 + (Ib32*sin(x(3)))/2) - x(6)*l2*l3*m3*sin(x(5));
C23 = - x(2)*((Ic13*cos(x(3) + x(5)))/2 - (Ic31*cos(x(3) + x(5)))/2 - (Ic23*sin(x(3) + x(5)))/2 + (Ic32*sin(x(3) + x(5)))/2) - x(4)*l2*l3*m3*sin(x(5)) - x(6)*l2*l3*m3*sin(x(5));
C31 = x(2)*((Ic22*sin(2*x(3) + 2*x(5)))/2 - (Ic21*cos(2*x(3) + 2*x(5)))/2 - (Ic11*sin(2*x(3) + 2*x(5)))/2 - (Ic12*cos(2*x(3) + 2*x(5)))/2 + (l3^2*m3*sin(2*x(3) + 2*x(5)))/2 + (l2*l3*m3*sin(x(5)))/2 + (l2*l3*m3*sin(2*x(3) + x(5)))/2);
C32 = x(4)*l2*l3*m3*sin(x(5)) - x(2)*((Ic13*cos(x(3) + x(5)))/2 - (Ic31*cos(x(3) + x(5)))/2 - (Ic23*sin(x(3) + x(5)))/2 + (Ic32*sin(x(3) + x(5)))/2);
C33 = -x(2)*((Ic13*cos(x(3) + x(5)))/2 - (Ic31*cos(x(3) + x(5)))/2 - (Ic23*sin(x(3) + x(5)))/2 + (Ic32*sin(x(3) + x(5)))/2);

C = [C11 C12 C13; 
     C21 C22 C23; 
     C31 C32 C33];

% Gravity G(q)
G1 = - g*m3*cos(x(1))*(l3*sin(x(3) + x(5)) + l2*sin(x(3))) - g*l2*m2*cos(x(1))*sin(x(3));
G2 = - g*m3*sin(x(1))*(l3*sin(x(3) + x(5)) + l2*sin(x(3))) - g*l2*m2*sin(x(1))*sin(x(3));
G3 =   g*m3*(l3*cos(x(3) + x(5)) + l2*cos(x(3))) + g*l2*m2*cos(x(3));

G = [G1; 
     G2; 
     G3];

% Friction
% F = 0.01*sign(dq); 
F = 0.025*sign(dq); % for sine + square wave
% F = 0.05*ones(3,1);

% External Disturbance for sine wave
% told = [0.01*sin(t);
%         0.01*sin(t);
%         0.01*sin(t)]; 
told = [0.025*sin(t);
        0.025*sin(t);
        0.025*sin(t)]; 
% told = [0.01*sin(2*pi*t);
%         0.01*cos(2*pi*t);
%         0.01*sin(2*pi*t)]; 
% told = [0.005 * sin(t) + 0.003 * sin(2*t) + 0.002 * sin(0.5*t) + 0.001 * randn(1, length(t));
%         0.005 * sin(t) + 0.003 * sin(2*t) + 0.002 * sin(0.5*t) + 0.001 * randn(1, length(t));
%         0.005 * sin(t) + 0.003 * sin(2*t) + 0.002 * sin(0.5*t) + 0.001 * randn(1, length(t))];

% External Disturbance for Square Wave
% told = [0.01 * square(t);  
%         0.01 * square(t);   
%         0.01 * square(t)];  
% told = [0.025 * square(t);  
%         0.025 * square(t);   
%         0.025 * square(t)]; 

% told = [0.05;
%         0.05;
%         0.05];          % for const wave


% Desired Trajectory for 3DOF
%--------------------------------CONST WAVE--------------------------------%
% qd1 = 1;    dqd1 = 0;  ddqd1 = 0;
% qd2 = 1.5;  dqd2 = 0;  ddqd2 = 0;
% qd3 = 2;    dqd3 = 0;  ddqd3 = 0;
%--------------------------------SINE WAVE--------------------------------%
% qd1 = 0.1*sin(t); dqd1 = 0.1*cos(t); ddqd1 = -0.1*sin(t);
% qd2 = 0.1*sin(t); dqd2 = 0.1*cos(t); ddqd2 = -0.1*sin(t);
% qd3 = 0.1*sin(t); dqd3 = 0.1*cos(t); ddqd3 = -0.1*sin(t);
% qd1 = 0.5*sin(t); dqd1 = 0.5*cos(t); ddqd1 = -0.5*sin(t);
% qd2 = 0.5*sin(t); dqd2 = 0.5*cos(t); ddqd2 = -0.5*sin(t);
% qd3 = 0.5*sin(t); dqd3 = 0.5*cos(t); ddqd3 = -0.5*sin(t);
%--------------------------------SQUARE WAVE--------------------------------%
% qd1 = 0.1 * square(t);  dqd1 = 0;  ddqd1 = 0;              
% qd2 = 0.1 * square(t);  dqd2 = 0;  ddqd2 = 0;
% qd3 = 0.1 * square(t);  dqd3 = 0;  ddqd3 = 0;   
qd1 = 0.5 * square(t);  dqd1 = 0;  ddqd1 = 0;              
qd2 = 0.5 * square(t);  dqd2 = 0;  ddqd2 = 0;
qd3 = 0.5 * square(t);  dqd3 = 0;  ddqd3 = 0;   

dqd = [dqd1; dqd2; dqd3];
ddqd = [ddqd1; ddqd2; ddqd3];

% Tracking Errors
e1 = qd1-q1; de1 = dqd1-dq1;
e2 = qd2-q2; de2 = dqd2-dq2;
e3 = qd3-q3; de3 = dqd3-dq3;

e = [e1; e2; e3]; 
de = [de1; de2; de3];

% Real f
Fai = 10*eye(3); 
dqr = dqd + Fai.*e;
ddqr = ddqd + Fai.*de;

% f = M.*ddqr + C.*dqr + G;
% f = M.*ddqr + C.*dqr + G + F;
% f = M.*ddqr + C.*dqr + G + told;
f = M.*ddqr + C.*dqr + G + F + told;

f_norm = norm(f);   
e_norm = norm(e);

% Output
sys(1) = x(1); % q1
sys(2) = x(2); % dq1
sys(3) = x(3); % q2
sys(4) = x(4); % dq2
sys(5) = x(5); % q3
sys(6) = x(6); % dq3