function [sys,x0,str,ts] = NTSMCinput(t,x,u,flag)
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
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 9;  % qd1, dqd1, ddqd1, qd2, dqd2, ddqd2, qd3, dqd3, ddqd3
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];

%---------------------------------------Output---------------------------------------%
function sys=mdlOutputs(t,x,u)
%-----------------CONST WAVE-----------------%
% sys(1) = 1;
% sys(2) = 0;
% sys(3) = 0;
% sys(4) = 1.5;
% sys(5) = 0;
% sys(6) = 0;
% sys(7) = 2;
% sys(8) = 0;
% sys(9) = 0;
%-----------------SINE WAVE------------------%
% for AC motors
% qd1=0.1*sin(t);
% dqd1=0.1*cos(t);
% ddqd1=-0.1*sin(t);
% qd2=0.1*sin(t);
% dqd2=0.1*cos(t);
% ddqd2=-0.1*sin(t);
% qd3=0.1*sin(t);
% dqd3=0.1*cos(t);
% ddqd3=-0.1*sin(t);

qd1=1*sin(t);
dqd1=1*cos(t);
ddqd1=-1*sin(t);
qd2=1*sin(t);
dqd2=1*cos(t);
ddqd2=-1*sin(t);
qd3=1*sin(t);
dqd3=1*cos(t);
ddqd3=-1*sin(t);


sys(1)=qd1;
sys(2)=dqd1;
sys(3)=ddqd1;
sys(4)=qd2;
sys(5)=dqd2;
sys(6)=ddqd2;
sys(7)=qd3;
sys(8)=dqd3;
sys(9)=ddqd3;

% -----------------SQUARE WAVE------------------%
% for stepper motors
% Generating a square wave with period T=2*pi, amplitude = 0.1
% qd1 = 0.1 * square(t);   % Square wave signal
% dqd1 = 0;                % Derivative of square wave is zero except at transitions
% ddqd1 = 0;               % Second derivative is zero except at transitions
% 
% qd2 = 0.1 * square(t);   % Square wave signal
% dqd2 = 0;                % Derivative of square wave is zero except at transitions
% ddqd2 = 0;               % Second derivative is zero except at transitions
% 
% qd3 = 0.1 * square(t);   % Square wave signal
% dqd3 = 0;                % Derivative of square wave is zero except at transitions
% ddqd3 = 0;               % Second derivative is zero except at transitions
% 
% % Store values in the sys array
% sys(1) = qd1;
% sys(2) = dqd1;
% sys(3) = ddqd1;
% sys(4) = qd2;
% sys(5) = dqd2;
% sys(6) = ddqd2;
% sys(7) = qd3;
% sys(8) = dqd3;
% sys(9) = ddqd3;
