function [sys,x0,str,ts]=s_function(t,x,u,flag)

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
function [sys,x0,str,ts]=mdlInitializeSizes

% If 3DOF
global m1 m2 m3 L1 L2 L3 R g
sizes = simsizes;
sizes.NumContStates  = 6; % q1, dq1, q2, dq2, q3, dq3
sizes.NumDiscStates  = 0;
% sizes.NumOutputs     = 7; % q1, dq1, q2, dq2, q3, dq3, f_n
% sizes.NumInputs      = 4; % u1, u2, u3, f_hat_n  % controller corresponding to joints
sizes.NumOutputs     = 6; % q1, dq1, q2, dq2, q3, dq3
sizes.NumInputs      = 7; % u1, u2, u3, es1, es2, es3, rmses_norm, qds, qs

sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 0;
sys=simsizes(sizes);
% x0=[0 0 0 0]; % initial values

% If 3DOF 
x0=[0 0 0 0 0 0]; % initial values, add 2 more elements

str=[];
ts=[];

function sys=mdlDerivatives(t,x,u)
global m1 m2 m3 L1 L2 L3 R g
m1=15; m2=6; m3=4; L1=0.25; L2=0.24; L3=0.36; R=0.06; g = 9.81;

% State variables
q1 = x(1); dq1 = x(2);
q2 = x(3); dq2 = x(4);
q3 = x(5); dq3 = x(6);

q = [q1; q2; q3]; 
dq = [dq1; dq2; dq3]; 

% global m1 m2 m3 l1 l2 l3 g
% % Robot's Model Parameter
% m1 = 5;    m2 = 5;     m3 = 5;        % kg
% l1 = 0.60;  l2 = 0.80;  l3 = 1.20;     % m
% g = 9.81;                              % m/s^2
% tol = 1;
% 
% % State variables
% q1 = x(1); dq1 = x(2);
% q2 = x(3); dq2 = x(4);
% q3 = x(5); dq3 = x(6);
% 
% q = [q1; q2; q3]; 
% dq = [dq1; dq2; dq3]; 
% 
% % Inertia Matrix I
% Ic1 = [0.85277*tol, -0.06561*tol, 0.26963*tol;
%        -0.06561*tol, 1.37516*tol, -0.09551*tol;
%         0.26963*tol, -0.09551*tol, 0.96321*tol];
% 
% Ic2 = Ic1;
% 
% Ic3 = [0.11546*tol, -0.00738*tol, 0.00193*tol;
%        -0.00738*tol, 0.26568*tol, -0.00874*tol;
%         0.00193*tol, -0.00874*tol, 0.26790*tol];
% 
% Ia11 = Ic1(1,1);  Ia12 = Ic1(1,2);  Ia13 = Ic1(1,3);
% Ia21 = Ic1(2,1);  Ia22 = Ic1(2,2);  Ia23 = Ic1(2,3);
% Ia31 = Ic1(3,1);  Ia32 = Ic1(3,2);  Ia33 = Ic1(3,3);
% 
% Ib11 = Ic2(1,1);  Ib12 = Ic2(1,2);  Ib13 = Ic2(1,3);
% Ib21 = Ic2(2,1);  Ib22 = Ic2(2,2);  Ib23 = Ic2(2,3);
% Ib31 = Ic2(3,1);  Ib32 = Ic2(3,2);  Ib33 = Ic2(3,3);
% 
% Ic11 = Ic3(1,1);  Ic12 = Ic3(1,2);  Ic13 = Ic3(1,3);
% Ic21 = Ic3(2,1);  Ic22 = Ic3(2,2);  Ic23 = Ic3(2,3);
% Ic31 = Ic3(3,1);  Ic32 = Ic3(3,2);  Ic33 = Ic3(3,3);
% 
% % Mass Matrix M(q)
% M11 = Ia22 + cos(x(3) + x(5))*(Ic22*cos(x(3) + x(5)) + Ic12*sin(x(3) + x(5))) + sin(x(3) + x(5))*(Ic21*cos(x(3) + x(5)) + Ic11*sin(x(3) + x(5))) + m3*(l3*cos(x(3) + x(5)) + l2*cos(x(3)))^2 + cos(x(3))*(Ib22*cos(x(3)) + Ib12*sin(x(3))) + sin(x(3))*(Ib21*cos(x(3)) + Ib11*sin(x(3))) - l2^2*m2*(sin(x(3))^2 - 1);
% M12 = Ic23*cos(x(3) + x(5)) + Ic13*sin(x(3) + x(5)) + Ib23*cos(x(3)) + Ib13*sin(x(3));
% M13 = Ic23*cos(x(3) + x(5)) + Ic13*sin(x(3) + x(5));
% M21 = Ic32*cos(x(3) + x(5)) + Ic31*sin(x(3) + x(5)) + Ib32*cos(x(3)) + Ib31*sin(x(3));
% M22 = Ib33 + Ic33 + l2^2*m2 + l2^2*m3 + l3^2*m3 + 2*l2*l3*m3*cos(x(5));
% M23 = m3*l3^2 + l2*m3*cos(x(5))*l3 + Ic33;
% M31 = Ic32*cos(x(3) + x(5)) + Ic31*sin(x(3) + x(5));
% M32 = m3*l3^2 + l2*m3*cos(x(5))*l3 + Ic33;
% M33 = m3*l3^2 + Ic33;
% 
% M = [M11 M12 M13; 
%      M21 M22 M23; 
%      M31 M32 M33];
% 
% % Centrifugal and Coriolis Matrix C(q,dq)
% C11 = (Ib12*x(4)*cos(2*x(3)))/2 + (Ib21*x(4)*cos(2*x(3)))/2 + (Ib11*x(4)*sin(2*x(3)))/2 - (Ib22*x(4)*sin(2*x(3)))/2 + (Ic12*x(4)*cos(2*x(3) + 2*x(5)))/2 + (Ic12*x(6)*cos(2*x(3) + 2*x(5)))/2 + (Ic21*x(4)*cos(2*x(3) + 2*x(5)))/2 + (Ic21*x(6)*cos(2*x(3) + 2*x(5)))/2 + (Ic11*x(4)*sin(2*x(3) + 2*x(5)))/2 + (Ic11*x(6)*sin(2*x(3) + 2*x(5)))/2 - (Ic22*x(4)*sin(2*x(3) + 2*x(5)))/2 - (Ic22*x(6)*sin(2*x(3) + 2*x(5)))/2 - (x(4)*l2^2*m2*sin(2*x(3)))/2 - (x(4)*l2^2*m3*sin(2*x(3)))/2 - (x(4)*l3^2*m3*sin(2*x(3) + 2*x(5)))/2 - (x(6)*l3^2*m3*sin(2*x(3) + 2*x(5)))/2 - x(4)*l2*l3*m3*sin(2*x(3) + x(5)) - (x(6)*l2*l3*m3*sin(2*x(3) + x(5)))/2 - (x(6)*l2*l3*m3*sin(x(5)))/2;
% C12 = (Ib12*x(2)*cos(2*x(3)))/2 + (Ib21*x(2)*cos(2*x(3)))/2 + (Ib11*x(2)*sin(2*x(3)))/2 - (Ib22*x(2)*sin(2*x(3)))/2 + (Ic12*x(2)*cos(2*x(3) + 2*x(5)))/2 + (Ic21*x(2)*cos(2*x(3) + 2*x(5)))/2 + (Ic11*x(2)*sin(2*x(3) + 2*x(5)))/2 - (Ic22*x(2)*sin(2*x(3) + 2*x(5)))/2 + Ic13*x(4)*cos(x(3) + x(5)) + Ic13*x(6)*cos(x(3) + x(5)) - Ic23*x(4)*sin(x(3) + x(5)) - Ic23*x(6)*sin(x(3) + x(5)) + Ib13*x(4)*cos(x(3)) - Ib23*x(4)*sin(x(3)) - (x(2)*l2^2*m2*sin(2*x(3)))/2 - (x(2)*l2^2*m3*sin(2*x(3)))/2 - (x(2)*l3^2*m3*sin(2*x(3) + 2*x(5)))/2 - x(2)*l2*l3*m3*sin(2*x(3) + x(5));
% C13 = (Ic12*x(2)*cos(2*x(3) + 2*x(5)))/2 + (Ic21*x(2)*cos(2*x(3) + 2*x(5)))/2 + (Ic11*x(2)*sin(2*x(3) + 2*x(5)))/2 - (Ic22*x(2)*sin(2*x(3) + 2*x(5)))/2 + Ic13*x(4)*cos(x(3) + x(5)) + Ic13*x(6)*cos(x(3) + x(5)) - Ic23*x(4)*sin(x(3) + x(5)) - Ic23*x(6)*sin(x(3) + x(5)) - (x(2)*l3^2*m3*sin(2*x(3) + 2*x(5)))/2 - (x(2)*l2*l3*m3*sin(2*x(3) + x(5)))/2 - (x(2)*l2*l3*m3*sin(x(5)))/2;
% C21 = -x(2)*((Ic12*cos(2*x(3) + 2*x(5)))/2 + (Ic21*cos(2*x(3) + 2*x(5)))/2 + (Ic11*sin(2*x(3) + 2*x(5)))/2 - (Ic22*sin(2*x(3) + 2*x(5)))/2 + (Ib12*cos(2*x(3)))/2 + (Ib21*cos(2*x(3)))/2 + (Ib11*sin(2*x(3)))/2 - (Ib22*sin(2*x(3)))/2 - (l3^2*m3*sin(2*x(3) + 2*x(5)))/2 - (l2^2*m2*sin(2*x(3)))/2 - (l2^2*m3*sin(2*x(3)))/2 - l2*l3*m3*sin(2*x(3) + x(5)));
% C22 = - x(2)*((Ic13*cos(x(3) + x(5)))/2 - (Ic31*cos(x(3) + x(5)))/2 - (Ic23*sin(x(3) + x(5)))/2 + (Ic32*sin(x(3) + x(5)))/2 + (Ib13*cos(x(3)))/2 - (Ib31*cos(x(3)))/2 - (Ib23*sin(x(3)))/2 + (Ib32*sin(x(3)))/2) - x(6)*l2*l3*m3*sin(x(5));
% C23 = - x(2)*((Ic13*cos(x(3) + x(5)))/2 - (Ic31*cos(x(3) + x(5)))/2 - (Ic23*sin(x(3) + x(5)))/2 + (Ic32*sin(x(3) + x(5)))/2) - x(4)*l2*l3*m3*sin(x(5)) - x(6)*l2*l3*m3*sin(x(5));
% C31 = x(2)*((Ic22*sin(2*x(3) + 2*x(5)))/2 - (Ic21*cos(2*x(3) + 2*x(5)))/2 - (Ic11*sin(2*x(3) + 2*x(5)))/2 - (Ic12*cos(2*x(3) + 2*x(5)))/2 + (l3^2*m3*sin(2*x(3) + 2*x(5)))/2 + (l2*l3*m3*sin(x(5)))/2 + (l2*l3*m3*sin(2*x(3) + x(5)))/2);
% C32 = x(4)*l2*l3*m3*sin(x(5)) - x(2)*((Ic13*cos(x(3) + x(5)))/2 - (Ic31*cos(x(3) + x(5)))/2 - (Ic23*sin(x(3) + x(5)))/2 + (Ic32*sin(x(3) + x(5)))/2);
% C33 = -x(2)*((Ic13*cos(x(3) + x(5)))/2 - (Ic31*cos(x(3) + x(5)))/2 - (Ic23*sin(x(3) + x(5)))/2 + (Ic32*sin(x(3) + x(5)))/2);
% 
% C = [C11 C12 C13; 
%      C21 C22 C23; 
%      C31 C32 C33];
% 
% % Gravity Vector G(q)
% G1 = - g*m3*cos(x(1))*(l3*sin(x(3) + x(5)) + l2*sin(x(3))) - g*l2*m2*cos(x(1))*sin(x(3));
% G2 = - g*m3*sin(x(1))*(l3*sin(x(3) + x(5)) + l2*sin(x(3))) - g*l2*m2*sin(x(1))*sin(x(3));
% G3 =   g*m3*(l3*cos(x(3) + x(5)) + l2*cos(x(3))) + g*l2*m2*cos(x(3));
% 
% G = [G1; 
%      G2; 
%      G3];

% Matrix Initialization
M11 = m1*R^2 + m3*L3^2.*cos(x(3)+x(5)).^2 + 2*m3*L2*L3.*cos(x(3)).*cos(x(3)+x(5)) + (m2+m3)*L2^2.*cos(x(3)).^2;
M22 = (m2+m3)*L2^2 + 2*m3*L2*L3.*cos(x(5)) + m3*L3^2;
M23 = m3*L3^2 + m3*L2*L3.*cos(x(5));
M32 = M23;
M33 = m3*L3^2;
M = [M11 0 0; 0 M22 M23; 0 M32 M33];

C11 = -(m2+m3)*L2^2.*sin(x(3)).*cos(x(3)).*x(4) - m3*L3^2.*sin(x(3)+x(5)).*cos(x(3)+x(5)).*(x(4)+x(6)) - m3*L2*L3.*sin(x(3)).*cos(x(3)+x(5)).*x(4) - m3*L2*L3.*cos(x(3)).*sin(x(3)+x(5)).*(x(4)+x(6));
C12 = -(m2+m3)*L2^2.*sin(x(3)).*cos(x(3)).*x(2) - m3*L3^2.*sin(x(3)+x(5)).*cos(x(3)+x(5)).*x(2) - m3*L2*L3.*sin(x(3)).*cos(x(3)+x(5)).*x(2) - m3*L2*L3.*cos(x(3)).*sin(x(3)+x(5)).*x(2);
C13 = -m3*L3^2.*sin(x(3)+x(5)).*cos(x(3)+x(5)).*x(2) - m3*L2*L3.*cos(x(3)).*sin(x(3)+x(5)).*x(2);
C21 = (m2+m3)*L2^2.*sin(x(3)).*cos(x(3)).*x(2) + m3*L3^2.*sin(x(3)+x(5)).*cos(x(3)+x(5)).*x(2) + m3*L2*L3.*sin(x(3)).*cos(x(3)+x(5)).*x(2) + m3*L2*L3.*cos(x(3)).*sin(x(3)+x(5)).*x(2);
C22 = -m3*L2*L3.*sin(x(5)).*x(6);
C23 = -m3*L2*L3.*sin(x(5)).*x(6) - m3*L2*L3.*sin(x(5)).*x(4);
C31 = m3*L3^2.*sin(x(3)+x(5)).*cos(x(3)+x(5)).*x(2) + m3*L2*L3.*cos(x(3)).*sin(x(3)+x(5)).*x(2);
C32 = m3*L2*L3.*sin(x(5)).*x(4);
C33 = 0;
C = [C11 C12 C13; C21 C22 C23; C31 C32 C33];

G1 = 0;
G2 = (m2+m3)*g*L2.*cos(x(3)) + m3*g*L3.*cos(x(3)+x(5));
G3 = m3*g*L3.*cos(x(3)+x(5));
G = [G1; G2; G3];

% Friction
% F = 0.001*sign(dq);      % for sine + square wave
F = 0.025*sign(dq);
% F = 0.05*ones(3,1);               % for const wave

% External Disturbance for sine wave
% told = [0.01*sin(t);
%         0.01*sin(t);
%         0.01*sin(t)];
% told = [0.025*sin(t);
%         0.025*sin(t);
%         0.025*sin(t)];
% told = [0.005 * sin(t) + 0.003 * sin(2*t) + 0.002 * sin(0.5*t) + 0.001 * randn(1, length(t));
%         0.005 * sin(t) + 0.003 * sin(2*t) + 0.002 * sin(0.5*t) + 0.001 * randn(1, length(t));
%         0.005 * sin(t) + 0.003 * sin(2*t) + 0.002 * sin(0.5*t) + 0.001 * randn(1, length(t))];

% External Disturbance for Square Wave
% told = [0.001 * square(t);  
%         0.001 * square(t);   
%         0.001 * square(t)];  
told = [0.025 * square(t);  
        0.025 * square(t);   
        0.025 * square(t)];
% told = [0.05;
%         0.05;
%         0.05];          % for const wave

% Control Input
ut = u(1:3); % u(1), u(2), u(3)

% Dynamics
% S = inv(M)*(ut-C-G);
% S = inv(M)*(ut-C-G-F);
% S = inv(M)*(ut-C-G-told);
S = inv(M)*(ut-C-G-F-told);

% If 3DOF
sys(1)=x(2); % dq1
sys(2)=S(1); % ddq1
sys(3)=x(4); % dq2
sys(4)=S(2); % ddq2
sys(5)=x(6); % dq3
sys(6)=S(3); % ddq3

function sys=mdlOutputs(t,x,u)
global m1 m2 m3 L1 L2 L3 R g
m1=15; m2=6; m3=4; L1=0.25; L2=0.24; L3=0.36; R=0.06; g = 9.81;

% State variables
q1 = x(1); dq1 = x(2);
q2 = x(3); dq2 = x(4);
q3 = x(5); dq3 = x(6);

q = [q1; q2; q3]; 
dq = [dq1; dq2; dq3];

% Matrix Initialization
M11 = m1*R^2 + m3*L3^2.*cos(x(3)+x(5)).^2 + 2*m3*L2*L3.*cos(x(3)).*cos(x(3)+x(5)) + (m2+m3)*L2^2.*cos(x(3)).^2;
M22 = (m2+m3)*L2^2 + 2*m3*L2*L3.*cos(x(5)) + m3*L3^2;
M23 = m3*L3^2 + m3*L2*L3.*cos(x(5));
M32 = M23;
M33 = m3*L3^2;
M = [M11 0 0; 0 M22 M23; 0 M32 M33];

C11 = -(m2+m3)*L2^2.*sin(x(3)).*cos(x(3)).*x(4) - m3*L3^2.*sin(x(3)+x(5)).*cos(x(3)+x(5)).*(x(4)+x(6)) - m3*L2*L3.*sin(x(3)).*cos(x(3)+x(5)).*x(4) - m3*L2*L3.*cos(x(3)).*sin(x(3)+x(5)).*(x(4)+x(6));
C12 = -(m2+m3)*L2^2.*sin(x(3)).*cos(x(3)).*x(2) - m3*L3^2.*sin(x(3)+x(5)).*cos(x(3)+x(5)).*x(2) - m3*L2*L3.*sin(x(3)).*cos(x(3)+x(5)).*x(2) - m3*L2*L3.*cos(x(3)).*sin(x(3)+x(5)).*x(2);
C13 = -m3*L3^2.*sin(x(3)+x(5)).*cos(x(3)+x(5)).*x(2) - m3*L2*L3.*cos(x(3)).*sin(x(3)+x(5)).*x(2);
C21 = (m2+m3)*L2^2.*sin(x(3)).*cos(x(3)).*x(2) + m3*L3^2.*sin(x(3)+x(5)).*cos(x(3)+x(5)).*x(2) + m3*L2*L3.*sin(x(3)).*cos(x(3)+x(5)).*x(2) + m3*L2*L3.*cos(x(3)).*sin(x(3)+x(5)).*x(2);
C22 = -m3*L2*L3.*sin(x(5)).*x(6);
C23 = -m3*L2*L3.*sin(x(5)).*x(6) - m3*L2*L3.*sin(x(5)).*x(4);
C31 = m3*L3^2.*sin(x(3)+x(5)).*cos(x(3)+x(5)).*x(2) + m3*L2*L3.*cos(x(3)).*sin(x(3)+x(5)).*x(2);
C32 = m3*L2*L3.*sin(x(5)).*x(4);
C33 = 0;
C = [C11 C12 C13; C21 C22 C23; C31 C32 C33];

G1 = 0;
G2 = (m2+m3)*g*L2.*cos(x(3)) + m3*g*L3.*cos(x(3)+x(5));
G3 = m3*g*L3.*cos(x(3)+x(5));
G = [G1; G2; G3];

% global m1 m2 m3 l1 l2 l3 R g
% % Robot's Model Parameter
% m1 = 5;    m2 = 5;     m3 = 5;        % kg
% l1 = 0.60;  l2 = 0.80;  l3 = 1.20;     % m
% g = 9.81;                              % m/s^2
% tol = 1;
% 
% % State variables
% q1 = x(1); dq1 = x(2);
% q2 = x(3); dq2 = x(4);
% q3 = x(5); dq3 = x(6);
% 
% q = [q1; q2; q3]; 
% dq = [dq1; dq2; dq3]; 
% 
% % Inertia Matrix I
% Ic1 = [0.85277*tol, -0.06561*tol, 0.26963*tol;
%        -0.06561*tol, 1.37516*tol, -0.09551*tol;
%         0.26963*tol, -0.09551*tol, 0.96321*tol];
% 
% Ic2 = Ic1;
% 
% Ic3 = [0.11546*tol, -0.00738*tol, 0.00193*tol;
%        -0.00738*tol, 0.26568*tol, -0.00874*tol;
%         0.00193*tol, -0.00874*tol, 0.26790*tol];
% 
% Ia11 = Ic1(1,1);  Ia12 = Ic1(1,2);  Ia13 = Ic1(1,3);
% Ia21 = Ic1(2,1);  Ia22 = Ic1(2,2);  Ia23 = Ic1(2,3);
% Ia31 = Ic1(3,1);  Ia32 = Ic1(3,2);  Ia33 = Ic1(3,3);
% 
% Ib11 = Ic2(1,1);  Ib12 = Ic2(1,2);  Ib13 = Ic2(1,3);
% Ib21 = Ic2(2,1);  Ib22 = Ic2(2,2);  Ib23 = Ic2(2,3);
% Ib31 = Ic2(3,1);  Ib32 = Ic2(3,2);  Ib33 = Ic2(3,3);
% 
% Ic11 = Ic3(1,1);  Ic12 = Ic3(1,2);  Ic13 = Ic3(1,3);
% Ic21 = Ic3(2,1);  Ic22 = Ic3(2,2);  Ic23 = Ic3(2,3);
% Ic31 = Ic3(3,1);  Ic32 = Ic3(3,2);  Ic33 = Ic3(3,3);
% 
% % Mass Matrix M(q)
% M11 = Ia22 + cos(x(3) + x(5))*(Ic22*cos(x(3) + x(5)) + Ic12*sin(x(3) + x(5))) + sin(x(3) + x(5))*(Ic21*cos(x(3) + x(5)) + Ic11*sin(x(3) + x(5))) + m3*(l3*cos(x(3) + x(5)) + l2*cos(x(3)))^2 + cos(x(3))*(Ib22*cos(x(3)) + Ib12*sin(x(3))) + sin(x(3))*(Ib21*cos(x(3)) + Ib11*sin(x(3))) - l2^2*m2*(sin(x(3))^2 - 1);
% M12 = Ic23*cos(x(3) + x(5)) + Ic13*sin(x(3) + x(5)) + Ib23*cos(x(3)) + Ib13*sin(x(3));
% M13 = Ic23*cos(x(3) + x(5)) + Ic13*sin(x(3) + x(5));
% M21 = Ic32*cos(x(3) + x(5)) + Ic31*sin(x(3) + x(5)) + Ib32*cos(x(3)) + Ib31*sin(x(3));
% M22 = Ib33 + Ic33 + l2^2*m2 + l2^2*m3 + l3^2*m3 + 2*l2*l3*m3*cos(x(5));
% M23 = m3*l3^2 + l2*m3*cos(x(5))*l3 + Ic33;
% M31 = Ic32*cos(x(3) + x(5)) + Ic31*sin(x(3) + x(5));
% M32 = m3*l3^2 + l2*m3*cos(x(5))*l3 + Ic33;
% M33 = m3*l3^2 + Ic33;
% 
% M = [M11 M12 M13; 
%      M21 M22 M23; 
%      M31 M32 M33];
% 
% % Centrifugal and Coriolis Matrix C(q,dq)
% C11 = (Ib12*x(4)*cos(2*x(3)))/2 + (Ib21*x(4)*cos(2*x(3)))/2 + (Ib11*x(4)*sin(2*x(3)))/2 - (Ib22*x(4)*sin(2*x(3)))/2 + (Ic12*x(4)*cos(2*x(3) + 2*x(5)))/2 + (Ic12*x(6)*cos(2*x(3) + 2*x(5)))/2 + (Ic21*x(4)*cos(2*x(3) + 2*x(5)))/2 + (Ic21*x(6)*cos(2*x(3) + 2*x(5)))/2 + (Ic11*x(4)*sin(2*x(3) + 2*x(5)))/2 + (Ic11*x(6)*sin(2*x(3) + 2*x(5)))/2 - (Ic22*x(4)*sin(2*x(3) + 2*x(5)))/2 - (Ic22*x(6)*sin(2*x(3) + 2*x(5)))/2 - (x(4)*l2^2*m2*sin(2*x(3)))/2 - (x(4)*l2^2*m3*sin(2*x(3)))/2 - (x(4)*l3^2*m3*sin(2*x(3) + 2*x(5)))/2 - (x(6)*l3^2*m3*sin(2*x(3) + 2*x(5)))/2 - x(4)*l2*l3*m3*sin(2*x(3) + x(5)) - (x(6)*l2*l3*m3*sin(2*x(3) + x(5)))/2 - (x(6)*l2*l3*m3*sin(x(5)))/2;
% C12 = (Ib12*x(2)*cos(2*x(3)))/2 + (Ib21*x(2)*cos(2*x(3)))/2 + (Ib11*x(2)*sin(2*x(3)))/2 - (Ib22*x(2)*sin(2*x(3)))/2 + (Ic12*x(2)*cos(2*x(3) + 2*x(5)))/2 + (Ic21*x(2)*cos(2*x(3) + 2*x(5)))/2 + (Ic11*x(2)*sin(2*x(3) + 2*x(5)))/2 - (Ic22*x(2)*sin(2*x(3) + 2*x(5)))/2 + Ic13*x(4)*cos(x(3) + x(5)) + Ic13*x(6)*cos(x(3) + x(5)) - Ic23*x(4)*sin(x(3) + x(5)) - Ic23*x(6)*sin(x(3) + x(5)) + Ib13*x(4)*cos(x(3)) - Ib23*x(4)*sin(x(3)) - (x(2)*l2^2*m2*sin(2*x(3)))/2 - (x(2)*l2^2*m3*sin(2*x(3)))/2 - (x(2)*l3^2*m3*sin(2*x(3) + 2*x(5)))/2 - x(2)*l2*l3*m3*sin(2*x(3) + x(5));
% C13 = (Ic12*x(2)*cos(2*x(3) + 2*x(5)))/2 + (Ic21*x(2)*cos(2*x(3) + 2*x(5)))/2 + (Ic11*x(2)*sin(2*x(3) + 2*x(5)))/2 - (Ic22*x(2)*sin(2*x(3) + 2*x(5)))/2 + Ic13*x(4)*cos(x(3) + x(5)) + Ic13*x(6)*cos(x(3) + x(5)) - Ic23*x(4)*sin(x(3) + x(5)) - Ic23*x(6)*sin(x(3) + x(5)) - (x(2)*l3^2*m3*sin(2*x(3) + 2*x(5)))/2 - (x(2)*l2*l3*m3*sin(2*x(3) + x(5)))/2 - (x(2)*l2*l3*m3*sin(x(5)))/2;
% C21 = -x(2)*((Ic12*cos(2*x(3) + 2*x(5)))/2 + (Ic21*cos(2*x(3) + 2*x(5)))/2 + (Ic11*sin(2*x(3) + 2*x(5)))/2 - (Ic22*sin(2*x(3) + 2*x(5)))/2 + (Ib12*cos(2*x(3)))/2 + (Ib21*cos(2*x(3)))/2 + (Ib11*sin(2*x(3)))/2 - (Ib22*sin(2*x(3)))/2 - (l3^2*m3*sin(2*x(3) + 2*x(5)))/2 - (l2^2*m2*sin(2*x(3)))/2 - (l2^2*m3*sin(2*x(3)))/2 - l2*l3*m3*sin(2*x(3) + x(5)));
% C22 = - x(2)*((Ic13*cos(x(3) + x(5)))/2 - (Ic31*cos(x(3) + x(5)))/2 - (Ic23*sin(x(3) + x(5)))/2 + (Ic32*sin(x(3) + x(5)))/2 + (Ib13*cos(x(3)))/2 - (Ib31*cos(x(3)))/2 - (Ib23*sin(x(3)))/2 + (Ib32*sin(x(3)))/2) - x(6)*l2*l3*m3*sin(x(5));
% C23 = - x(2)*((Ic13*cos(x(3) + x(5)))/2 - (Ic31*cos(x(3) + x(5)))/2 - (Ic23*sin(x(3) + x(5)))/2 + (Ic32*sin(x(3) + x(5)))/2) - x(4)*l2*l3*m3*sin(x(5)) - x(6)*l2*l3*m3*sin(x(5));
% C31 = x(2)*((Ic22*sin(2*x(3) + 2*x(5)))/2 - (Ic21*cos(2*x(3) + 2*x(5)))/2 - (Ic11*sin(2*x(3) + 2*x(5)))/2 - (Ic12*cos(2*x(3) + 2*x(5)))/2 + (l3^2*m3*sin(2*x(3) + 2*x(5)))/2 + (l2*l3*m3*sin(x(5)))/2 + (l2*l3*m3*sin(2*x(3) + x(5)))/2);
% C32 = x(4)*l2*l3*m3*sin(x(5)) - x(2)*((Ic13*cos(x(3) + x(5)))/2 - (Ic31*cos(x(3) + x(5)))/2 - (Ic23*sin(x(3) + x(5)))/2 + (Ic32*sin(x(3) + x(5)))/2);
% C33 = -x(2)*((Ic13*cos(x(3) + x(5)))/2 - (Ic31*cos(x(3) + x(5)))/2 - (Ic23*sin(x(3) + x(5)))/2 + (Ic32*sin(x(3) + x(5)))/2);
% 
% C = [C11 C12 C13; 
%      C21 C22 C23; 
%      C31 C32 C33];
% 
% % Gravity Vector G(q)
% G1 = - g*m3*cos(x(1))*(l3*sin(x(3) + x(5)) + l2*sin(x(3))) - g*l2*m2*cos(x(1))*sin(x(3));
% G2 = - g*m3*sin(x(1))*(l3*sin(x(3) + x(5)) + l2*sin(x(3))) - g*l2*m2*sin(x(1))*sin(x(3));
% G3 =   g*m3*(l3*cos(x(3) + x(5)) + l2*cos(x(3))) + g*l2*m2*cos(x(3));
% 
% G = [G1; 
%      G2; 
%      G3];

% Friction
% F = 0.001*sign(dq);        % for sine + square wave
F = 0.25*sign(dq);
% F = 0.05*ones(3,1);               % for const wave

% External Disturbance for sine wave
% told = [0.01*sin(t);
%         0.01*sin(t);
%         0.01*sin(t)];
% told = [0.025*sin(t);
%         0.025*sin(t);
%         0.025*sin(t)];
% told = [0.005 * sin(t) + 0.003 * sin(2*t) + 0.002 * sin(0.5*t) + 0.001 * randn(1, length(t));
%         0.005 * sin(t) + 0.003 * sin(2*t) + 0.002 * sin(0.5*t) + 0.001 * randn(1, length(t));
%         0.005 * sin(t) + 0.003 * sin(2*t) + 0.002 * sin(0.5*t) + 0.001 * randn(1, length(t))];

% External Disturbance for Square Wave
% told = [0.001 * square(t);   
%         0.001 * square(t);  
%         0.001 * square(t)]; 
told = [0.25 * square(t);   
        0.25 * square(t);  
        0.25 * square(t)];
% told = [0.05;
%         0.05;
%         0.05];            % for const wave

% State variables for 3DOF
q = [x(1); x(3); x(5)]; % add
dq = [x(2); x(4); x(6)]; % add
q1 = q(1);
dq1 = dq(1);
q2 = q(2);
dq2 = dq(2); 
q3 = q(3); % add
dq3 = dq(3); % add

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

dqd = [dqd1; dqd2; dqd3]; % add
ddqd = [ddqd1; ddqd2; ddqd3]; % add

% Errors 3DOF
e1 = qd1-q1;
e2 = qd2-q2;
e3 = qd3-q3; % add
de1 = dqd1-dq1;
de2 = dqd2-dq2;
de3 = dqd3-dq3; % add
e = [e1; e2; e3]; % add
de = [de1; de2; de3]; % add

% Real f
Fai = 15*eye(3); % change from eye(2) to eye(3) 

dqr = dqd + Fai.*e; % Formula 9.20 in book Intelligent Control
ddqr = ddqd + Fai.*de;


% f = M.*ddqr + C.*dqr + G;
% f = M.*ddqr + C.*dqr + G + F;
% f = M.*ddqr + C.*dqr + G + told;
f = M.*ddqr + C.*dqr + G + F + told;

f_norm = norm(f);

% Output
sys(1)=x(1); % q1
sys(2)=x(2); % dq1
sys(3)=x(3); % q2
sys(4)=x(4); % dq2
sys(5)=x(5); % q3
sys(6)=x(6); % dq3
% sys(7)=qs;
% sys(7)=f_norm;