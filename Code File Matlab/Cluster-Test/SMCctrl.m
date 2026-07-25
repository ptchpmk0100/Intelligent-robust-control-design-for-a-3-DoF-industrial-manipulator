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

%---------------------------------NTSMC---------------------------------%
% %--------------------Derivative--------------------%
% function sys=mdlDerivatives(t,x,u)
% global neuron c b
% 
% qd1 = u(1); dqd1 = u(2); ddqd1 = u(3);
% qd2 = u(4); dqd2 = u(5); ddqd2 = u(6);
% qd3 = u(7); dqd3 = u(8); ddqd3 = u(9);
% 
% qd = [qd1; qd2; qd3];
% dqd = [dqd1; dqd2; dqd3]; 
% ddqd = [ddqd1; ddqd2; ddqd3]; 
% 
% % State Variables
% q1 = u(10); dq1 = u(11); 
% q2 = u(12); dq2 = u(13);
% q3 = u(14); dq3 = u(15);
% 
% q = [q1; q2; q3];
% dq = [dq1; dq2; dq3]; 
% 
% % Tracking Errors
% e1 = qd1-q1; de1 = dqd1-dq1;
% e2 = qd2-q2; de2 = dqd2-dq2;
% e3 = qd3-q3; de3 = dqd3-dq3;
% 
% e = [e1; e2; e3]; 
% de = [de1; de2; de3];
% 
% %--New Sliding Surface (NTSMC)--%
% gamma = 2;                % optimal value
% sigma = 3;                % optimal value
% lambda = 0.01;            % optimal value for sine wave
% % lambda = 0.5;           % optimal value for const wave
% 
% s = de + lambda*(sign(e).*abs(e).^(gamma/sigma));
% 
% % Input Vectors
% z1 = [e(1); de(1)];
% z2 = [e(2); de(2)];
% z3 = [e(3); de(3)]; 
% 
% % Output of Hidden Layer
% for i = 1:neuron 
%     h1(i) = exp(-norm(z1 - c(:,i))^2 / (b * b)); 
%     h2(i) = exp(-norm(z2 - c(:,i))^2 / (b * b));
%     h3(i) = exp(-norm(z3 - c(:,i))^2 / (b * b));
% end
% 
% %------------- New Adaptive Law -------------%
% % Robot's Model Parameter
% m1 = 0.0865; m2 = m1; m3 = m1;               % kg 
% l1 = 0.0975; l2 = 0.14665; l3 = 0.18997;     % m
% g = 9.81;                                    % m/s^2
% tol = 1e-9;
% 
% % Inertia Matrix I
% Ic1 = [1243271.12*tol,   116023.89*tol,   118509.08*tol;
%         116023.89*tol,   669264.09*tol,   559757.74*tol;
%         118509.08*tol,   559757.74*tol,   634170.06*tol];
% 
% Ic2 = [2302070.69*tol,   199555.95*tol,    93639.68*tol;
%         199555.95*tol,   450604.70*tol,   869745.15*tol;
%          93639.68*tol,   869745.15*tol,  1926391.30*tol];
% 
% Ic3 = [8034547.48*tol,    382202.33*tol,     99611.10*tol;
%         382202.33*tol,    548948.42*tol,   1948062.81*tol;
%          99611.10*tol,   1948062.81*tol,  7556521.21*tol];
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
% M11 = Ia22 + cos(q2 + q3)*(Ic22*cos(q2 + q3) + Ic12*sin(q2 + q3)) + sin(q2 + q3)*(Ic21*cos(q2 + q3) + Ic11*sin(q2 + q3)) + m3*(l3*cos(q2 + q3) + l2*cos(q2))^2 + cos(q2)*(Ib22*cos(q2) + Ib12*sin(q2)) + sin(q2)*(Ib21*cos(q2) + Ib11*sin(q2)) - l2^2*m2*(sin(q2)^2 - 1);
% M12 = Ic23*cos(q2 + q3) + Ic13*sin(q2 + q3) + Ib23*cos(q2) + Ib13*sin(q2);
% M13 = Ic23*cos(q2 + q3) + Ic13*sin(q2 + q3);
% M21 = Ic32*cos(q2 + q3) + Ic31*sin(q2 + q3) + Ib32*cos(q2) + Ib31*sin(q2);
% M22 = Ib33 + Ic33 + l2^2*m2 + l2^2*m3 + l3^2*m3 + 2*l2*l3*m3*cos(q3);
% M23 = m3*l3^2 + l2*m3*cos(q3)*l3 + Ic33;
% M31 = Ic32*cos(q2 + q3) + Ic31*sin(q2 + q3);
% M32 = m3*l3^2 + l2*m3*cos(q3)*l3 + Ic33;
% M33 = m3*l3^2 + Ic33;
% 
% M = [M11 M12 M13; 
%      M21 M22 M23; 
%      M31 M32 M33];
% 
% c1 = 0.01;              % optimal value for both wave
% 
% e_1 = qd - q;
% alpha = dqd - c1*e_1;
% e_2 = dq - alpha;
% 
% Theta = (gamma/sigma)*norm(s)*norm(c1*diag(sign(e_2).*abs(e_2).^(gamma/sigma-1))*inv(M));
% 
% eta = 0.1;              % optimal value for sine wave
% % eta = 0.5;            % optimal value for const wave
% 
% %--New Adaptive Rule--%
% for i=1:1:neuron   
%     sys(i) = eta*h1(i)*Theta;
%     sys(i+neuron) = eta*h2(i)*Theta;
%     sys(i+2*neuron) = eta*h3(i)*Theta;
% end
% 
% %------------------Output------------------%
% function sys=mdlOutputs(t,x,u)
% global neuron c b  
% 
% qd1 = u(1); dqd1 = u(2); ddqd1 = u(3);
% qd2 = u(4); dqd2 = u(5); ddqd2 = u(6);
% qd3 = u(7); dqd3 = u(8); ddqd3 = u(9);
% 
% qd = [qd1; qd2; qd3];
% dqd = [dqd1; dqd2; dqd3];
% ddqd = [ddqd1; ddqd2; ddqd3];
% 
% % State Variables
% q1 = u(10); dq1 = u(11); 
% q2 = u(12); dq2 = u(13);
% q3 = u(14); dq3 = u(15);
% 
% q = [q1; q2; q3]; 
% dq = [dq1; dq2; dq3]; 
% 
% % Errors
% e1 = qd1-q1; de1 = dqd1-dq1;
% e2 = qd2-q2; de2 = dqd2-dq2;
% e3 = qd3-q3; de3 = dqd3-dq3;
% 
% e = [e1; e2; e3]; 
% de = [de1; de2; de3];
% 
% %--New Sliding Surface (NTSMC)--%
% gamma = 2;                % optimal value
% sigma = 3;                % optimal value
% lambda = 0.01;            % optimal value for sine wave
% % lambda = 0.5;           % optimal value for const wave
% 
% s = de + lambda*(sign(e).*abs(e).^(gamma/sigma));
% 
% % RBF Weights Initialization
% W_f1 = [x(1:neuron)]';
% W_f2 = [x(neuron+1:neuron*2)]';
% W_f3 = [x(neuron*2+1:neuron*3)]';
% 
% % Input Vectors
% z1 = [e(1); de(1)]; 
% z2 = [e(2); de(2)];
% z3 = [e(3); de(3)];
% 
% for i = 1:neuron
%     h1(i) = exp(-norm(z1 - c(:,i))^2 / (b * b));
%     h2(i) = exp(-norm(z2 - c(:,i))^2 / (b * b));
%     h3(i) = exp(-norm(z3 - c(:,i))^2 / (b * b));
% end
% 
% fn = [W_f1*h1';
%       W_f2*h2';
%       W_f3*h3']; 
% 
% %------------Dynamics------------%
% % Robot's Model Parameter
% m1 = 0.0865; m2 = m1; m3 = m1;               % kg 
% l1 = 0.0975; l2 = 0.14665; l3 = 0.18997;     % m
% g = 9.81;                                    % m/s^2
% tol = 1e-9;
% 
% % Inertia Matrix I
% Ic1 = [1243271.12*tol,   116023.89*tol,   118509.08*tol;
%         116023.89*tol,   669264.09*tol,   559757.74*tol;
%         118509.08*tol,   559757.74*tol,   634170.06*tol];
% 
% Ic2 = [2302070.69*tol,   199555.95*tol,    93639.68*tol;
%         199555.95*tol,   450604.70*tol,   869745.15*tol;
%          93639.68*tol,   869745.15*tol,  1926391.30*tol];
% 
% Ic3 = [8034547.48*tol,    382202.33*tol,     99611.10*tol;
%         382202.33*tol,    548948.42*tol,   1948062.81*tol;
%          99611.10*tol,   1948062.81*tol,  7556521.21*tol];
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
% M11 = Ia22 + cos(q2 + q3)*(Ic22*cos(q2 + q3) + Ic12*sin(q2 + q3)) + sin(q2 + q3)*(Ic21*cos(q2 + q3) + Ic11*sin(q2 + q3)) + m3*(l3*cos(q2 + q3) + l2*cos(q2))^2 + cos(q2)*(Ib22*cos(q2) + Ib12*sin(q2)) + sin(q2)*(Ib21*cos(q2) + Ib11*sin(q2)) - l2^2*m2*(sin(q2)^2 - 1);
% M12 = Ic23*cos(q2 + q3) + Ic13*sin(q2 + q3) + Ib23*cos(q2) + Ib13*sin(q2);
% M13 = Ic23*cos(q2 + q3) + Ic13*sin(q2 + q3);
% M21 = Ic32*cos(q2 + q3) + Ic31*sin(q2 + q3) + Ib32*cos(q2) + Ib31*sin(q2);
% M22 = Ib33 + Ic33 + l2^2*m2 + l2^2*m3 + l3^2*m3 + 2*l2*l3*m3*cos(q3);
% M23 = m3*l3^2 + l2*m3*cos(q3)*l3 + Ic33;
% M31 = Ic32*cos(q2 + q3) + Ic31*sin(q2 + q3);
% M32 = m3*l3^2 + l2*m3*cos(q3)*l3 + Ic33;
% M33 = m3*l3^2 + Ic33;
% 
% M = [M11 M12 M13; 
%      M21 M22 M23; 
%      M31 M32 M33];
% 
% % Centrifugal and Coriolis Matrix C(q,dq)
% C11 = (Ib12*dq2*cos(2*q2))/2 + (Ib21*dq2*cos(2*q2))/2 + (Ib11*dq2*sin(2*q2))/2 - (Ib22*dq2*sin(2*q2))/2 + (Ic12*dq2*cos(2*q2 + 2*q3))/2 + (Ic12*dq3*cos(2*q2 + 2*q3))/2 + (Ic21*dq2*cos(2*q2 + 2*q3))/2 + (Ic21*dq3*cos(2*q2 + 2*q3))/2 + (Ic11*dq2*sin(2*q2 + 2*q3))/2 + (Ic11*dq3*sin(2*q2 + 2*q3))/2 - (Ic22*dq2*sin(2*q2 + 2*q3))/2 - (Ic22*dq3*sin(2*q2 + 2*q3))/2 - (dq2*l2^2*m2*sin(2*q2))/2 - (dq2*l2^2*m3*sin(2*q2))/2 - (dq2*l3^2*m3*sin(2*q2 + 2*q3))/2 - (dq3*l3^2*m3*sin(2*q2 + 2*q3))/2 - dq2*l2*l3*m3*sin(2*q2 + q3) - (dq3*l2*l3*m3*sin(2*q2 + q3))/2 - (dq3*l2*l3*m3*sin(q3))/2;
% C12 = (Ib12*dq1*cos(2*q2))/2 + (Ib21*dq1*cos(2*q2))/2 + (Ib11*dq1*sin(2*q2))/2 - (Ib22*dq1*sin(2*q2))/2 + (Ic12*dq1*cos(2*q2 + 2*q3))/2 + (Ic21*dq1*cos(2*q2 + 2*q3))/2 + (Ic11*dq1*sin(2*q2 + 2*q3))/2 - (Ic22*dq1*sin(2*q2 + 2*q3))/2 + Ic13*dq2*cos(q2 + q3) + Ic13*dq3*cos(q2 + q3) - Ic23*dq2*sin(q2 + q3) - Ic23*dq3*sin(q2 + q3) + Ib13*dq2*cos(q2) - Ib23*dq2*sin(q2) - (dq1*l2^2*m2*sin(2*q2))/2 - (dq1*l2^2*m3*sin(2*q2))/2 - (dq1*l3^2*m3*sin(2*q2 + 2*q3))/2 - dq1*l2*l3*m3*sin(2*q2 + q3);
% C13 = (Ic12*dq1*cos(2*q2 + 2*q3))/2 + (Ic21*dq1*cos(2*q2 + 2*q3))/2 + (Ic11*dq1*sin(2*q2 + 2*q3))/2 - (Ic22*dq1*sin(2*q2 + 2*q3))/2 + Ic13*dq2*cos(q2 + q3) + Ic13*dq3*cos(q2 + q3) - Ic23*dq2*sin(q2 + q3) - Ic23*dq3*sin(q2 + q3) - (dq1*l3^2*m3*sin(2*q2 + 2*q3))/2 - (dq1*l2*l3*m3*sin(2*q2 + q3))/2 - (dq1*l2*l3*m3*sin(q3))/2;
% C21 = -dq1*((Ic12*cos(2*q2 + 2*q3))/2 + (Ic21*cos(2*q2 + 2*q3))/2 + (Ic11*sin(2*q2 + 2*q3))/2 - (Ic22*sin(2*q2 + 2*q3))/2 + (Ib12*cos(2*q2))/2 + (Ib21*cos(2*q2))/2 + (Ib11*sin(2*q2))/2 - (Ib22*sin(2*q2))/2 - (l3^2*m3*sin(2*q2 + 2*q3))/2 - (l2^2*m2*sin(2*q2))/2 - (l2^2*m3*sin(2*q2))/2 - l2*l3*m3*sin(2*q2 + q3));
% C22 = - dq1*((Ic13*cos(q2 + q3))/2 - (Ic31*cos(q2 + q3))/2 - (Ic23*sin(q2 + q3))/2 + (Ic32*sin(q2 + q3))/2 + (Ib13*cos(q2))/2 - (Ib31*cos(q2))/2 - (Ib23*sin(q2))/2 + (Ib32*sin(q2))/2) - dq3*l2*l3*m3*sin(q3);
% C23 = - dq1*((Ic13*cos(q2 + q3))/2 - (Ic31*cos(q2 + q3))/2 - (Ic23*sin(q2 + q3))/2 + (Ic32*sin(q2 + q3))/2) - dq2*l2*l3*m3*sin(q3) - dq3*l2*l3*m3*sin(q3);
% C31 = dq1*((Ic22*sin(2*q2 + 2*q3))/2 - (Ic21*cos(2*q2 + 2*q3))/2 - (Ic11*sin(2*q2 + 2*q3))/2 - (Ic12*cos(2*q2 + 2*q3))/2 + (l3^2*m3*sin(2*q2 + 2*q3))/2 + (l2*l3*m3*sin(q3))/2 + (l2*l3*m3*sin(2*q2 + q3))/2);
% C32 = dq2*l2*l3*m3*sin(q3) - dq1*((Ic13*cos(q2 + q3))/2 - (Ic31*cos(q2 + q3))/2 - (Ic23*sin(q2 + q3))/2 + (Ic32*sin(q2 + q3))/2);
% C33 = -dq1*((Ic13*cos(q2 + q3))/2 - (Ic31*cos(q2 + q3))/2 - (Ic23*sin(q2 + q3))/2 + (Ic32*sin(q2 + q3))/2);
% 
% C = [C11 C12 C13; 
%      C21 C22 C23; 
%      C31 C32 C33];
% 
% % Gravity G(q)
% G1 = - g*m3*cos(q1)*(l3*sin(q2 + q3) + l2*sin(q2)) - g*l2*m2*cos(q1)*sin(q2);
% G2 = - g*m3*sin(q1)*(l3*sin(q2 + q3) + l2*sin(q2)) - g*l2*m2*sin(q1)*sin(q2);
% G3 =   g*m3*(l3*cos(q2 + q3) + l2*cos(q2)) + g*l2*m2*cos(q2);
% 
% G = [G1; 
%      G2; 
%      G3];
% 
% % Friction F(dq)
% % v_i = 0.01;
% % k_d = 0.01;
% % Fv = v_i*sign(dq);
% % Fd = k_d*diag(v_i)*sign(dq);
% % F = Fv + Fd;                         % for sine wave
% F = 0.001;                         % for const wave
% 
% % Controller Design
% c1 = 0.01;                           % optimal value for both wave
% 
% e_1 = qd - q;
% de_1 = dqd - dq;
% alpha = dqd - c1.*e_1;
% e_2 = dq - alpha;
% dalpha = ddqd - c1.*de_1;
% 
% %--New Controller--%
% K = 50;                               % optimal value for sine wave
% % K = 1;                              % optimal value for const wave
% Kd = -K*tanh(s);
% 
% % tol_eq = M*dalpha + C*dq + G;
% tol_eq = M*dalpha + C*dq + G + F;
% tol1 = -M*(sigma/gamma)*(1/lambda)*diag(sign(e_2).*abs(e_2).^(1-gamma/sigma))*(e_1+((pinv(s))*(e_1'*e_2))');
% tol2 = -fn.*Kd;
% tol = tol_eq + tol1 + tol2;
% 
% fn_norm=norm(fn);
% en_norm = norm(e);
% sys(1)=tol(1);
% sys(2)=tol(2);
% sys(3)=tol(3);
% %sys(4)=fn_norm;
% sys(4)=e1;
% sys(5)=e2;
% sys(6)=e3;
% sys(7)=en_norm;


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