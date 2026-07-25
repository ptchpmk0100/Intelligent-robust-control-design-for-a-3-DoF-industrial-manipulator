function [sys, x0, str, ts] = MIMO_Tong_s(t, x, u, flag)
switch flag
    case 0
        [sys, x0, str, ts] = mdlInitializeSizes;
    case 1
        sys = mdlDerivatives(t, x, u);
    case 3
        sys = mdlOutputs(t, x, u);
    case {2, 4, 9}
        sys = [];
    otherwise
        error(['Unhandled flag = ', num2str(flag)]);
end

%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%

function [sys, x0, str, ts] = mdlInitializeSizes
global Fai

Fai = 10*eye(3);

sizes = simsizes;
sizes.NumContStates = 15;     % 15 States for 3 DOF fuzzy approx (5 per joint)
sizes.NumDiscStates = 0;
sizes.NumOutputs = 9;         % tol1, tol2, tol3, Fp1, Fp2, Fp3, e1, e2, e3
sizes.NumInputs = 18;         % qd1, qd2, qd3, q1, dq1, q2, dq2, q3, dq3, 
                              % q1, dq1, q2, dq2, q3, dq3, Fr1, Fr2, Fr3
sizes.DirFeedthrough = 1;     % Plant feedback
sizes.NumSampleTimes = 0;

sys = simsizes(sizes);
x0 = 0.1 * ones(15, 1);       % Init 15 states (5 per DOF)
str = [];
ts = [];

%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%

function sys = mdlDerivatives(t, x, u)
global Fai

% Desired Trajectories
qd1 = u(1); dqd1 = u(2); ddqd1 = u(3);
qd2 = u(4); dqd2 = u(5); ddqd2 = u(6);
qd3 = u(7); dqd3 = u(8); ddqd3 = u(9);

qd = [qd1; qd2; qd3];
dqd = [dqd1; dqd2; dqd3]; 
ddqd = [ddqd1; ddqd2; ddqd3]; 

% State Variables
q1 = u(10);  dq1 = u(11); 
q2 = u(12);  dq2 = u(13);
q3 = u(14);  dq3 = u(15);

q = [q1; q2; q3];
dq = [dq1; dq2; dq3]; 

% Tracking Errors
e1 = qd1-q1;  de1 = dqd1-dq1;
e2 = qd2-q2;  de2 = dqd2-dq2;
e3 = qd3-q3;  de3 = dqd3-dq3;

e = [e1; e2; e3]; 
de = [de1; de2; de3];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fsd1 = 0;
for l1 = 1:1:5
    gs1 = -((dq1+pi/6-(l1-1)*pi/12)/(pi/24))^2;
    u1(l1) = exp(gs1);
end

fsd2 = 0;
for l2 = 1:1:5
    gs2 = -((dq2+pi/6-(l2-1)*pi/12)/(pi/24))^2;
    u2(l2) = exp(gs2);
end

fsd3 = 0;
for l3 = 1:1:5
    gs3 = -((dq3+pi/6-(l3-1)*pi/12)/(pi/24))^2;
    u3(l3) = exp(gs3);
end

for l1 = 1:1:5
    fsu1(l1) = u1(l1);
    fsd1 = fsd1 + u1(l1);
end

for l2 = 1:1:5
    fsu2(l2) = u2(l2);
    fsd2 = fsd2 + u2(l2);
end

for l3 = 1:1:5
    fsu3(l3) = u3(l3);
    fsd3 = fsd3 + u3(l3);
end

fs1 = fsu1/(fsd1+0.001);
fs2 = fsu2/(fsd2+0.001);
fs3 = fsu3/(fsd3+0.001);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
e1 = q1-qd1;   de1 = dq1-dqd1;
e2 = q2-qd2;   de2 = dq2-dqd2;
e3 = q3-qd3;   de3 = dq3-dqd3;

e = [e1 e2 e3]'; 
de = [de1 de2 de3]';

s = de + Fai*e;

Gama = 0.0005;

S1 = -1/Gama*s(1)*fs1;
S2 = -1/Gama*s(2)*fs2;
S3 = -1/Gama*s(3)*fs3;

for i = 1:1:5
    sys(i) = S1(i);
end

for j = 6:1:10
    sys(j) = S2(j-5);
end

for j = 11:1:15
    sys(j) = S2(j-10);
end

%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%

function sys=mdlOutputs(t,x,u)
global Fai

% State Variables
q1 = u(10); dq1 = u(11); 
q2 = u(12); dq2 = u(13);
q3 = u(14); dq3 = u(15);

q = [q1; q2; q3];
dq = [dq1; dq2; dq3]; 

%---------- Dynamic Model ----------%
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
c1 = cos(q1);    s1 = sin(q1);
c2 = cos(q2);    s2 = sin(q2);
c3 = cos(q3);    s3 = sin(q3);

c12 = cos(q1 + q2); 
s23 = sin(q2 + q3);
c23 = cos(q2 + q3);
c123 = cos(q1 + q2 + q3);

% Inertia matrix
b11 = Il1 + ml1 * l1^2 + kr1^2 * Im1 + ...
      Il2 + mm2 * a1^2 + Im2 + ml2 * (a1^2 + l2^2 + 2 * a1 * l2 * c2) + ...
      Il3 + Im3 + mm3 * (a1^2 + a2^2 + 2 * a1 * a2 * c2) + ...
      ml3 * (a1^2 + a2^2 + l3^2 + 2 * a1 * a2 * c2 + 2 * a1 * l3 * c23 + 2 * a2 * l3 * c3);
b12 = Il2 + Il3 + kr2^2 * Im2 + Im3 + ...
      mm3 * (a2^2 + a1 * a2 * c2) + ml2 * (l2^2 + a1 * l2 * c2) + ...
      ml3 * (a2^2 + l3^2 + a1 * a2 * c2 + a1 * l3 * c23 + 2 * a2 * l3 * c3);
b13 = Il3 + kr3^2 * Im3 + ml3 * l3^2;
b21 = b12;
b22 = Il2 + Il3 + kr2^2 * Im2 + Im3 + ...
      mm3 * a2^2 + ml2 * l2^2 + ...
      ml3 * (a2^2 + l3^2 + 2 * a2 * l3 * c3);
b23 = Il3 + kr3^2 * Im3 + ml3 * l3^2;
b31 = b13;
b32 = b23;
b33 = Il3 + kr3^2 * Im3 + ml3 * l3^2;

H = [b11, b12, b13;
     b21, b22, b23;
     b31, b32, b33];

% Coriolis and centrifugal Matrix
c11 = -ml3 * a1 * a2 * s2 * dq1 - (ml2 * a1 * l2 * s2 + ml3 * a1 * a2 * s2 + ml3 * a1 * l3 * s23) * dq2 ...
      - (ml3 * a1 * l3 * s23 + ml3 * a2 * l3 * s3) * dq3;
c12 = -(ml2 * a1 * l2 * s2 + ml3 * a1 * a2 * s2 + ml3 * a1 * l3 * s23) * dq1 ...
      - (ml3 * a1 * a2 * s2 + ml3 * a1 * l3 * s23 + ml2 * a1 * l2 * s2) * dq2 ...
      - (ml3 * a1 * l3 * s23 + ml3 * a2 * l3 * s3) * dq3;
c13 = -(ml3 * a1 * l3 * s23 + ml3 * a2 * l3 * s3) * dq1 ...
      - (ml3 * a1 * l3 * s23 + ml3 * a2 * l3 * s3) * dq2 ...
      - (ml3 * a2 * l3 * s3 + ml3 * a1 * l3 * s23) * dq3;
c21 = (ml2 * a1 * l2 * s2 + ml3 * a1 * a2 * s2 + ml3 * a1 * l3 * s23) * dq1 ...
      - (ml3 * a2 * l3 * s3) * dq3;
c22 = -ml3 * a2 * l3 * s3 * dq3;
c23 = -(ml3 * a2 * l3 * s3) * dq1 - ml3 * a2 * l3 * s3 * dq2 + ml3 * a2 * l3 * s3 * dq3;
c31 = (ml3 * a1 * l3 * s23 + ml3 * a2 * l3 * s3) * dq1 ...
      + (ml3 * a2 * l3 * s3) * dq2;
c32 = (ml3 * a2 * l3 * s3) * dq1 + (ml3 * a2 * l3 * s3) * dq2 ...
      - (ml3 * a2 * l3 * s3) * dq3;
c33 = ml3 * a2 * l3 * s3 * dq3;

C = [c11, c12, c13;
     c21, c22, c23;
     c31, c32, c33];

% Gravity matrix
g1 = (ml1 * l1 + ml2 * a1 + mm2 * a1 + ml3 * a1 + mm3 * a1) * g * c1 ...
     + (ml2 * l2 + ml3 * a2 + mm3 * a2) * g * c12 ...
     + ml3 * l3 * g * c123;
g2 = (ml2 * l2 + ml3 * a2 + mm3 * a2) * g * c12 ...
     + ml3 * l3 * g * c123;
g3 = ml3 * l3 * g * c123;

G = [g1;
     g2;
     g3];

% Desired trajectories and derivatives
qd1 = u(1); dqd1 = u(2); ddqd1 = u(3);
qd2 = u(4); dqd2 = u(5); ddqd2 = u(6);
qd3 = u(7); dqd3 = u(8); ddqd3 = u(9);

qd = [qd1; qd2; qd3];
dqd = [dqd1; dqd2; dqd3]; 
ddqd = [ddqd1; ddqd2; ddqd3]; 

e1 = q1-qd1;   de1 = dq1-dqd1;
e2 = q2-qd2;   de2 = dq2-dqd2;
e3 = q3-qd3;   de3 = dq3-dqd3;

e = [e1 e2 e3]'; 
de = [de1 de2 de3]';

s = de+Fai*e;

dqr = dqd-Fai*e;
ddqr = ddqd-Fai*de;

for i = 1:1:5
    thta1(i,1) = x(i);
end

for i = 1:1:5
    thta2(i,1) = x(i+5);
end

for i = 1:1:5
    thta3(i,1) = x(i+10);
end

fsd1 = 0;
for l1 = 1:1:5
    gs1 = -[(dq1+pi/6-(l1-1)*pi/12)/(pi/24)]^2;
    u1(l1) = exp(gs1);
end

fsd2 = 0;
for l2=1:1:5
    gs2 = -[(dq2+pi/6-(l2-1)*pi/12)/(pi/24)]^2;
    u2(l2) = exp(gs2);
end

fsd3 = 0;
for l3=1:1:5
    gs3 = -[(dq3+pi/6-(l3-1)*pi/12)/(pi/24)]^2;
    u3(l3) = exp(gs3);
end

for l1=1:1:5
    fsu1(l1) = u1(l1);
    fsd1 = fsd1+u1(l1);
end

for l2 = 1:1:5
    fsu2(l2) = u2(l2);
    fsd2 = fsd2+u2(l2);
end

for l3 = 1:1:5
    fsu3(l3) = u3(l3);
    fsd3 = fsd3+u3(l3);
end

fs1 = fsu1/(fsd1+0.001);
fs2 = fsu2/(fsd2+0.001);
fs3 = fsu3/(fsd3+0.001);

Fp(1) = thta1'*fs1';
Fp(2) = thta2'*fs2';
Fp(3) = thta3'*fs3';

epN = 0.20;
bd = 0.1;
v = -(epN+bd)*sign(s);

% KD = 0.01*eye(3);   % small val => smaller error but slows conver + fluc
% KD = 200*eye(3);   % optimal val
% W = 50*eye(3);     % optimal val
% KD = 100*eye(3);   % disturbed
% W = 200*eye(3);    
KD = 2000*eye(3);   % Ideal
W = 200*eye(3); 

% Friction
% v_i = 0.025;
% k_d = 0.025;
v_i = 25;
k_d = 25;
Fv = v_i*sign(dq);
Fd = k_d*diag(v_i)*sign(dq);
F = Fv + Fd;  

tol = H*ddqr + C*dqr + G + F + 1*Fp' - KD*s - v - W*sign(s); %(4.134)
% tol = H*ddqr + C*dqr + G + 1*Fp' - KD*s - W*tanh(s); 
% tol = H*ddqr + C*dqr + G + F + 1*Fp' - KD*s - W*tanh(s); 

sys(1) = tol(1);
sys(2) = tol(2);
sys(3) = tol(3);
sys(4) = Fp(1);
sys(5) = Fp(2);
sys(6) = Fp(3);
% sys(7) = abs(e1);
% sys(8) = abs(e2);
% sys(9) = abs(e3);
sys(7) = e1;
sys(8) = e2;
sys(9) = e3;