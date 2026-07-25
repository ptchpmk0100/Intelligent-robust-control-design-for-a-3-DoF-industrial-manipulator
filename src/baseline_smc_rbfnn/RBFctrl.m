function [sys,x0,str,ts] = spacemodel(t,x,u,flag)
switch flag,
case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
case 1,
    sys=mdlDerivatives(t,x,u);
case 3,
    sys=mdlOutputs(t,x,u);
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts]=mdlInitializeSizes
global neuron c b Fai
% RBF structure 2-7-1

% RBFNN Param Initialization
neuron = 7; % number of nerons in hidden layer
% c = [-6 -6;
%      -4 -4;
%      -2 -2;
%       0  0;
%       2 2;
%       4 4;
%       6 6]; % # of rows = # of inputs
c = [0.014 0.91;
     0.03 0.53;
     0.03 0.13;
     0.03 -0.3;
     0.01 -0.63;
     -0.01 0.26;
     -0.01 0.01];
% c = [-80 -80;
%      -40 -40;
%      -20 -20;
%       0  0;
%       20 20;
%       40 40;
%       80 80];

b = 10; % Spread parameter, assume same for 7 neurons
% b = 30;

% Controller Param Initialization
% Fai = 15*eye(3);   % change to matrix 3x3 
Fai = 5*eye(3);

sizes = simsizes;
sizes.NumContStates  = 3*neuron; % 7 Neurons => 21 Continuous States
sizes.NumDiscStates  = 0;
% sizes.NumOutputs     = 4;  % u1, u2, u3 and f_hat_n
% If 3DOF => % qd1, dqd1, ddqd1, qd2, dqd2, ddqd2, qd3, dqd3, ddqd3, q1, dq1, q2, dq2, q3, dq3, f_n
% sizes.NumInputs = 16;
sizes.NumOutputs     = 7;  % u1, u2, u3, es1, es2, es3, rmses_norm, qds, qs
% If 3DOF => % qd1, dqd1, ddqd1, qd2, dqd2, ddqd2, qd3, dqd3, ddqd3, 
             % q1, dq1, q2, dq2, q3, dq3
sizes.NumInputs = 15;

sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;
sys = simsizes(sizes);
x0  = zeros(1,3*neuron);  % change from 2*neuron to 3*neuron
str = [];
ts  = [];

function sys=mdlDerivatives(t,x,u)
global neuron c b Fai

%If 3DOF
qd1 = u(1); dqd1 = u(2); ddqd1 = u(3);
qd2 = u(4); dqd2 = u(5); ddqd2 = u(6);
qd3 = u(7); dqd3 = u(8); ddqd3 = u(9); % add

qd = [qd1; qd2; qd3]; % add
dqd = [dqd1; dqd2; dqd3]; % add
ddqd = [ddqd1; ddqd2; ddqd3]; % add

% State Variables
q1 = u(10); dq1 = u(11); 
q2 = u(12); dq2 = u(13);
q3 = u(14); dq3 = u(15); % add

q = [q1; q2; q3]; % add
dq = [dq1; dq2; dq3]; % add

% Errors
es1 = qd1-q1; des1 = dqd1-dq1;
es2 = qd2-q2; des2 = dqd2-dq2;
es3 = qd3-q3; des3 = dqd3-dq3; % add
es = [es1; es2; es3]; des = [des1; des2; des3]; % add

% Tracking Error
r = des + Fai*es; % Formula 9.18 book Intelligent Control

% "Virtual State variables"
dqr = dqd + Fai*es;
ddqr = ddqd + Fai*des;

% Input Vectors
% Different from page 201 book Intelligent Control
% Author Code Suggests:
z1=[es(1); des(1)]; 
z2=[es(2); des(2)];
z3=[es(3); des(3)]; % add

% % Author Book Suggests:
% z1=[e(1); e(2); e(3)]; 
% z2=[de(1); de(2); de(3)];

% Output of Hidden Layer

for j = 1:neuron % j from 1 to neuron, step=1
    h1(j) = exp(-norm(z1' - c(j,:))^2 / (b * b)); % output of hidden layer
    h2(j) = exp(-norm(z2' - c(j,:))^2 / (b * b));
    h3(j) = exp(-norm(z3' - c(j,:))^2 / (b * b));
end

F=15*eye(neuron); % not sure here
for i=1:1:neuron   % not sure here
    sys(i)=15*h1(i)*r(1);  % r is tracking error
    sys(i+neuron)=15*h2(i)*r(2);
    sys(i+2*neuron)=15*h3(i)*r(3);
end
% for i=1:1:neuron   % not sure here
%     sys(i)=30*h1(i)*r(1);  % r is tracking error
%     sys(i+neuron)=30*h2(i)*r(2);
%     sys(i+2*neuron)=30*h3(i)*r(3);
% end

function sys=mdlOutputs(t,x,u)
global neuron c b Fai

%If 3DOF
qd1 = u(1); dqd1 = u(2); ddqd1 = u(3);
qd2 = u(4); dqd2 = u(5); ddqd2 = u(6);
qd3 = u(7); dqd3 = u(8); ddqd3 = u(9); % add

qd = [qd1; qd2; qd3]; % add
dqd = [dqd1; dqd2; dqd3]; % add
ddqd = [ddqd1; ddqd2; ddqd3]; % add

% State Variables
q1 = u(10); dq1 = u(11); 
q2 = u(12); dq2 = u(13);
q3 = u(14); dq3 = u(15); % add

q = [q1; q2; q3]; % add
dq = [dq1; dq2; dq3];  % add

% Errors
es1 = qd1-q1; des1 = dqd1-dq1;
es2 = qd2-q2; des2 = dqd2-dq2;
es3 = qd3-q3; des3 = dqd3-dq3; % add
es = [es1; es2; es3]; des = [des1; des2; des3]; % add

% Tracking Error
r = des + Fai*es; % Formula 9.18 book Intelligent Control (3x1)

% "Virtual State variables"
dqr = dqd + Fai*es;
ddqr = ddqd + Fai*des;

% RBF Weights Initialization
W_f1=[x(1:neuron)]'; % x is input from block input ( 1x7 )
W_f2=[x(neuron+1:neuron*2)]'; % not sure here
W_f3=[x(neuron*2+1:neuron*3)]'; % take the last 7th element

% Input Vectors
% Different from page 201 book Intelligent Control
% Author Code Suggests:
z1=[es(1); des(1)]; 
z2=[es(2); des(2)];
z3=[es(3); des(3)]; % add

% Author Book Suggests:
% z1=[e(1); e(2); e(3)]; 
% z2=[de(1); de(2); de(3)];

for j = 1:neuron % j from 1 to neuron, step=1
    h1(j) = exp(-norm(z1' - c(j,:))^2 / (b * b)); % output of hidden layer
    h2(j) = exp(-norm(z2' - c(j,:))^2 / (b * b)); % 1x7
    h3(j) = exp(-norm(z3' - c(j,:))^2 / (b * b));
end

fn=[W_f1*h1';
    W_f2*h2'; % not sure here
    W_f3*h3'];

%-------------Include Dynamics-------------%
% m1 = 5;    m2 = 5;     m3 = 5;        % kg
% l1 = 0.60;  l2 = 0.80;  l3 = 1.20;     % m
% g = 9.81;                              % m/s^2
% tol = 1;
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

% v_i = 0,25;
% k_d = 0.25;
v_i = 2;
k_d = 2;
Fv = v_i*sign(dq);
Fd = k_d*diag(v_i)*sign(dq);
F = Fv + Fd;                         % for sine + square wave

Kv=20*eye(3);
% Kv=100*eye(3);

% epN = 0.20; bd = 0.1;
epN = 5; bd = 1;
% epN = 50; bd = 30;
v=-(epN+bd)*tanh(r);
% tol = fn + Kv*r - v; 
tol = fn + Kv*r + F - v;
% tol1 = M*ddqr + C*dqr + G;
% tol2 = fn +Kv*r +v;
% tol = tol1 + tol2;

fn_norm = norm(fn);
qds = qd;
qs = q;
es_norm = norm(es);
mses_norm = (1/3)*es_norm.^2;
rmses_norm = sqrt(mses_norm);
sys(1)=tol(1);
sys(2)=tol(2);
sys(3)=tol(3);
% sys(4)=fn_norm;
sys(4) = es1;
sys(5) = es2;
sys(6) = es3;
% sys(7) = es_norm;
% sys(7) = mses_norm;
sys(7) = rmses_norm;
% sys(8) = qds;
% sys(9) = qs;