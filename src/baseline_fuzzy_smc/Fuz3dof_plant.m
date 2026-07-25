function [sys,x0,str,ts]=MIMO_Tong_plant(t,x,u,flag)
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

%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%

function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates = 6;   % q1, dq1, q2, dq2, q3, dq3
sizes.NumDiscStates = 0;
sizes.NumOutputs = 9;      % q1, dq1, q2, dq2, q3, dq3, Fr1, Fr2, Fr3
sizes.NumInputs = 9;       % tol1, tol2, tol3, Fp1, Fp2, Fp3, e1, e2, e3
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 0;
sys=simsizes(sizes);
x0 = [0 0 0 0 0 0];
str=[];
ts=[];

%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%

function sys = mdlDerivatives(t,x,u)

%-----------Dynamics Model-----------%
% Constants
l1 = 0.5; l2 = 0.5; l3 = 0.5; % Link lengths (m)
ml1 = 10; ml2 = 10; ml3 = 10; % Link masses (kg)
mm1 = 1; mm2 = 1; mm3 = 1;    % Motor masses (kg)

Il1 = 1.0; Il2 = 1.0; Il3 = 1.0; % Link moments of inertia (kg·m^2)
Im1 = 0.01; Im2 = 0.01; Im3 = 0.01; % Motor moments of inertia (kg·m^2)
kr1 = 1; kr2 = 1; kr3 = 1; % Gear ratio constants
a1 = l1 / 2; a2 = l2 / 2; a3 = l3 / 2; % Center of mass distances
g = 9.81;

% Joint angles and their cosines
c1 = cos(x(1));  s1 = sin(x(1));
c2 = cos(x(3));  s2 = sin(x(3));
c3 = cos(x(5));  s3 = sin(x(5));

c12 = cos(x(1) + x(3)); 
s23 = sin(x(3) + x(5));
c23 = cos(x(3) + x(5));

c123 = cos(x(1) + x(3) + x(5));

% Inertia matrix components
b11 = Il1 + ml1 * l1^2 + kr1^2 * Im1 + ...
      Il2 + mm2 * a1^2 + Im2 + ml2 * (a1^2 + l2^2 + 2 * a1 * l2 * c2) + ...
      Il3 + Im3 + mm3 * (a1^2 + a2^2 + 2 * a1 * a2 * c2) + ...
      ml3 * (a1^2 + a2^2 + l3^2 + 2 * a1 * a2 * c2 + 2 * a1 * l3 * c23 + 2 * a2 * l3 * c3);

b12 = Il2 + Il3 + kr2^2 * Im2 + Im3 + ...
      mm3 * (a2^2 + a1 * a2 * c2) + ml2 * (l2^2 + a1 * l2 * c2) + ...
      ml3 * (a2^2 + l3^2 + a1 * a2 * c2 + a1 * l3 * c23 + 2 * a2 * l3 * c3);

b13 = Il3 + kr3^2 * Im3 + ml3 * l3^2;

b22 = Il2 + Il3 + kr2^2 * Im2 + Im3 + ...
      mm3 * a2^2 + ml2 * l2^2 + ...
      ml3 * (a2^2 + l3^2 + 2 * a2 * l3 * c3);

b23 = Il3 + kr3^2 * Im3 + ml3 * l3^2;

b33 = Il3 + kr3^2 * Im3 + ml3 * l3^2;

% Symmetric terms
b21 = b12;
b31 = b13;
b32 = b23;

% Inertia matrix
H = [b11, b12, b13;
     b21, b22, b23;
     b31, b32, b33];

% Coriolis and centrifugal torque components
c11 = -ml3 * a1 * a2 * s2 * x(2) - (ml2 * a1 * l2 * s2 + ml3 * a1 * a2 * s2 + ml3 * a1 * l3 * s23) * x(4) ...
      - (ml3 * a1 * l3 * s23 + ml3 * a2 * l3 * s3) * x(6);

c12 = -(ml2 * a1 * l2 * s2 + ml3 * a1 * a2 * s2 + ml3 * a1 * l3 * s23) * x(2) ...
      - (ml3 * a1 * a2 * s2 + ml3 * a1 * l3 * s23 + ml2 * a1 * l2 * s2) * x(4) ...
      - (ml3 * a1 * l3 * s23 + ml3 * a2 * l3 * s3) * x(6);

c13 = -(ml3 * a1 * l3 * s23 + ml3 * a2 * l3 * s3) * x(2) ...
      - (ml3 * a1 * l3 * s23 + ml3 * a2 * l3 * s3) * x(4) ...
      - (ml3 * a2 * l3 * s3 + ml3 * a1 * l3 * s23) * x(6);

c21 = (ml2 * a1 * l2 * s2 + ml3 * a1 * a2 * s2 + ml3 * a1 * l3 * s23) * x(2) ...
      - (ml3 * a2 * l3 * s3) * x(6);

c22 = -ml3 * a2 * l3 * s3 * x(6);

c23 = -(ml3 * a2 * l3 * s3) * x(2) - ml3 * a2 * l3 * s3 * x(4) + ml3 * a2 * l3 * s3 * x(6);

c31 = (ml3 * a1 * l3 * s23 + ml3 * a2 * l3 * s3) * x(2) ...
      + (ml3 * a2 * l3 * s3) * x(4);

c32 = (ml3 * a2 * l3 * s3) * x(2) + (ml3 * a2 * l3 * s3) * x(4) ...
      - (ml3 * a2 * l3 * s3) * x(6);

c33 = ml3 * a2 * l3 * s3 * x(6);

% Coriolis and centrifugal torque matrix
C = [c11, c12, c13;
     c21, c22, c23;
     c31, c32, c33];

% Gravity matrix components
g1 = (ml1 * l1 + ml2 * a1 + mm2 * a1 + ml3 * a1 + mm3 * a1) * g * c1 ...
     + (ml2 * l2 + ml3 * a2 + mm3 * a2) * g * c12 ...
     + ml3 * l3 * g * c123;

g2 = (ml2 * l2 + ml3 * a2 + mm3 * a2) * g * c12 ...
     + ml3 * l3 * g * c123;

g3 = ml3 * l3 * g * c123;

% Gravity matrix
G = [g1;
     g2;
     g3];

Fr = [15*x(2)+6*sign(x(2));
      15*x(4)+6*sign(x(4));
      15*x(6)+6*sign(x(6))];

told = [0.25 * square(t);  
        0.25 * square(t);   
        0.25 * square(t)];

tol=[u(1) u(2) u(3)]';

S = inv(H)*(tol-C*[x(2);x(4);x(6)]-G-Fr);
% S = inv(H)*(tol-C*[x(2);x(4);x(6)]-G-Fr+told);

sys(1) = x(2);
sys(2) = S(1);
sys(3) = x(4);
sys(4) = S(2);
sys(5) = x(6);
sys(6) = S(3);

%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%

function sys = mdlOutputs(t,x,u)

Fr = [15*x(2)+6*sign(x(2));
      15*x(4)+6*sign(x(4));
      15*x(6)+6*sign(x(6))];

sys(1)=x(1);
sys(2)=x(2);
sys(3)=x(3);
sys(4)=x(4);
sys(5)=x(5);
sys(6)=x(6);
sys(7)=Fr(1);
sys(8)=Fr(2);
sys(9)=Fr(3);
