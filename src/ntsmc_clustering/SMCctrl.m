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

%-----------------------------------Initialization-----------------------------------%
function [sys,x0,str,ts]=mdlInitializeSizes
global neuron c b

% RBFNN Param Initialization
neuron = 7;                % # of nodes in NN model
c = [-3 -2 -1 0 1 2 3;     % center coordinates
     -3 -2 -1 0 1 2 3];
b = 10;                    % width (spread)

% Controller Param Initialization
sizes = simsizes;
sizes.NumContStates  = 3*neuron; 
sizes.NumDiscStates  = 0;
% sizes.NumOutputs     = 4;  % u1, u2, u3 and f_n (change f_n to position tracking error e)
% sizes.NumInputs      = 16; % qd1, dqd1, ddqd1, qd2, dqd2, ddqd2, qd3, dqd3, ddqd3, 
                           % q1, dq1, q2, dq2, q3, dq3, 
                           % f_n
sizes.NumOutputs     = 7;  % u1, u2, u3, en_norm and e1, e2 e3
sizes.NumInputs      = 15; % qd1, dqd1, ddqd1, qd2, dqd2, ddqd2, qd3, dqd3, ddqd3, 
                           % q1, dq1, q2, dq2, q3, dq3 (no f_n or e_n from
                           % block plant)

sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;
sys = simsizes(sizes);
x0  = zeros(1,3*neuron);
str = [];
ts  = [];


%-------------------------------SMC---------------------------------%
%----------Derivative----------%
function sys=mdlDerivatives(t,x,u)
global neuron c b

qds1 = u(1); dqds1 = u(2); ddqds1 = u(3);
qds2 = u(4); dqds2 = u(5); ddqds2 = u(6);
qds3 = u(7); dqds3 = u(8); ddqds3 = u(9);

qds = [qds1; qds2; qds3];
dqds = [dqds1; dqds2; dqds3]; 
ddqds = [ddqds1; ddqds2; ddqds3]; 

% State Variables
qs1 = u(10); dqs1 = u(11); 
qs2 = u(12); dqs2 = u(13);
qs3 = u(14); dqs3 = u(15);

qs = [qs1; qs2; qs3];
dqs = [dqs1; dqs2; dqs3]; 

% Tracking Errors
es1 = qds1-qs1; des1 = dqds1-dqs1;
es2 = qds2-qs2; des2 = dqds2-dqs2;
es3 = qds3-qs3; des3 = dqds3-dqs3;

es = [es1; es2; es3]; 
des = [des1; des2; des3];

%--Traditional Sliding Surface (SMC)--%
Fai = 15*eye(3);
s_old = des + Fai*es;

% Input Vectors
zs1 = [es(1); des(1)];
zs2 = [es(2); des(2)];
zs3 = [es(3); des(3)]; 

% Output of Hidden Layer
for i = 1:neuron 
    hs1(i) = exp(-norm(zs1 - c(:,i))^2 / (b * b)); 
    hs2(i) = exp(-norm(zs2 - c(:,i))^2 / (b * b));
    hs3(i) = exp(-norm(zs3 - c(:,i))^2 / (b * b));
end

%--Traditional Adaptive Rule--%
for i=1:1:neuron   
    sys(i) = 15*hs1(i)*s_old(1);
    sys(i+neuron) = 15*hs2(i)*s_old(2);
    sys(i+2*neuron) = 15*hs3(i)*s_old(3);
end

%---------------------------------------Output---------------------------------------%
function sys=mdlOutputs(t,x,u)
global neuron c b  

qds1 = u(1); dqds1 = u(2); ddqds1 = u(3);
qds2 = u(4); dqds2 = u(5); ddqds2 = u(6);
qds3 = u(7); dqds3 = u(8); ddqds3 = u(9);

qds = [qds1; qds2; qds3];
dqds = [dqds1; dqds2; dqds3]; 
ddqds = [ddqds1; ddqds2; ddqds3]; 

% State Variables
qs1 = u(10); dqs1 = u(11); 
qs2 = u(12); dqs2 = u(13);
qs3 = u(14); dqs3 = u(15);

qs = [qs1; qs2; qs3];
dqs = [dqs1; dqs2; dqs3]; 

% Tracking Errors
es1 = qds1-qs1; des1 = dqds1-dqs1;
es2 = qds2-qs2; des2 = dqds2-dqs2;
es3 = qds3-qs3; des3 = dqds3-dqs3;

es = [es1; es2; es3]; 
des = [des1; des2; des3];

%--Traditional Sliding Surface (SMC)--%
Fai = 15*eye(3);
s_old = des + Fai*es;

% RBF Weights Initialization
Ws_f1 = [x(1:neuron)]';
Ws_f2 = [x(neuron+1:neuron*2)]';
Ws_f3 = [x(neuron*2+1:neuron*3)]';

% Input Vectors
zs1 = [es(1); des(1)]; 
zs2 = [es(2); des(2)];
zs3 = [es(3); des(3)];

for i = 1:neuron
    hs1(i) = exp(-norm(zs1 - c(:,i))^2 / (b * b));
    hs2(i) = exp(-norm(zs2 - c(:,i))^2 / (b * b));
    hs3(i) = exp(-norm(zs3 - c(:,i))^2 / (b * b));
end

fn = [Ws_f1*hs1';
      Ws_f2*hs2';
      Ws_f3*hs3']; 

Kv = 20*eye(3);
epN = 0.20;
bd = 0.1;
v = -(epN+bd)*sign(s_old);

tols = fn + Kv*s_old - v;

ens_norm = norm(es);
sys(1)=tols(1);
sys(2)=tols(2);
sys(3)=tols(3);
%sys(4)=fn_norm;
sys(4)=es1;
sys(5)=es2;
sys(6)=es3;
sys(7)=ens_norm;
